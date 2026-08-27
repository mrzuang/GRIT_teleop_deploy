"""GRIT policy inference and reference-motion integration."""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from common.joint_mapper import JointMapper
from common.utils import DictToClass
from runtime.motion_sources import (
    LocalNpzMotionSource,
    MotionSourceBase,
    remap_joint_array_by_names,
)
from runtime.observation import Observation
from runtime.policy import ONNXRuntimeModel, ReferenceTrackingPolicy
from runtime.grit_observation import (
    GRIT_CONTEXT_FRAMES,
    GRIT_FEATURE_DIM,
    GritProprioHistory,
    GritReferenceContext,
    GRIT_HISTORY_LENGTH,
    GRIT_PROPRIO_FRAME_DIM,
    build_grit_reference_features,
    build_grit_streaming_reference_features,
    grit_context_offsets,
    grit_default_joint_pos_by_name,
)

# The exported policy action order matches the reference qpos order.
GRIT_ACTION_JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

# G1_ACTION_SCALE per actuator regex group (0.25 * effort_limit / stiffness).
GRIT_ACTION_SCALE_BY_NAME = {
    ".*_elbow_joint": 0.43857731392336724,
    ".*_shoulder_pitch_joint": 0.43857731392336724,
    ".*_shoulder_roll_joint": 0.43857731392336724,
    ".*_shoulder_yaw_joint": 0.43857731392336724,
    ".*_wrist_roll_joint": 0.43857731392336724,
    ".*_hip_pitch_joint": 0.5475464629911068,
    ".*_hip_yaw_joint": 0.5475464629911068,
    "waist_yaw_joint": 0.5475464629911068,
    ".*_hip_roll_joint": 0.35066146637882434,
    ".*_knee_joint": 0.35066146637882434,
    ".*_wrist_pitch_joint": 0.07450087032950714,
    ".*_wrist_yaw_joint": 0.07450087032950714,
    "waist_pitch_joint": 0.43857731392336724,
    "waist_roll_joint": 0.43857731392336724,
    ".*_ankle_pitch_joint": 0.43857731392336724,
    ".*_ankle_roll_joint": 0.43857731392336724,
}


def _resolve_by_name(
    values: dict[str, float], names: list[str]
) -> np.ndarray:
    import re

    out = np.zeros(len(names), dtype=np.float32)
    for i, name in enumerate(names):
        for pat, v in values.items():
            if re.fullmatch(pat, name):
                out[i] = v
    return out


class GritONNXModel(ONNXRuntimeModel):
    """Adapter for the deployment-only two-input ONNX contract."""

    DEPLOY_INPUTS = ("reference_context", "proprio_history")

    def __init__(self, path: str):
        super().__init__(path)
        input_names = tuple(inp.name for inp in self.ort_session.get_inputs())
        if input_names != self.DEPLOY_INPUTS:
            raise ValueError(
                f"Unsupported deployment ONNX inputs {input_names}; expected "
                f"{self.DEPLOY_INPUTS}"
            )
        self.mode = "deploy_multi_input"
        self.policy_input_dim = (
            GRIT_CONTEXT_FRAMES * GRIT_FEATURE_DIM
            + GRIT_HISTORY_LENGTH * GRIT_PROPRIO_FRAME_DIM
        )
        output_names = tuple(out.name for out in self.ort_session.get_outputs())
        if output_names != ("actions",):
            raise ValueError(
                f"Unsupported grit ONNX outputs {output_names}; expected ('actions',)"
            )
        print(
            f"[GRITONNX] mode={self.mode}, runtime_obs={self.policy_input_dim}, "
            f"graph_inputs={input_names}"
        )

    def __call__(self, input: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        if all(name in input for name in self.DEPLOY_INPUTS):
            args = {
                name: np.asarray(input[name], dtype=np.float32)
                for name in self.DEPLOY_INPUTS
            }
        else:
            packed = np.asarray(input["policy"], dtype=np.float32)
            if packed.ndim != 2 or packed.shape[1] != self.policy_input_dim:
                raise ValueError(
                    f"grit compact packed input shape {packed.shape} != "
                    f"[batch, {self.policy_input_dim}]"
                )
            reference_width = GRIT_CONTEXT_FRAMES * GRIT_FEATURE_DIM
            args = {
                "reference_context": packed[:, :reference_width].reshape(
                    packed.shape[0], GRIT_CONTEXT_FRAMES, GRIT_FEATURE_DIM
                ),
                "proprio_history": packed[:, reference_width:],
            }
        outputs = self.ort_session.run(None, args)
        return {k: v for k, v in zip(self.out_keys, outputs)}


class GritPolicy(ReferenceTrackingPolicy):
    """Convert GRIT observations and ONNX actions into G1 joint targets."""

    def __init__(self, name: str, policy_cfg: DictToClass, controller):
        self.controller = controller

        self.reference_horizon = int(
            grit_context_offsets(
                reference_fps=float(getattr(policy_cfg, "reference_fps", 50.0))
            )[-1]
        )
        if not hasattr(policy_cfg, "reference_joint_names"):
            raise ValueError("[GritPolicy] reference_joint_names required")
        if not hasattr(policy_cfg, "action_joint_names"):
            policy_cfg.action_joint_names = list(GRIT_ACTION_JOINT_NAMES)
        if list(policy_cfg.action_joint_names) != list(policy_cfg.reference_joint_names):
            raise ValueError(
                "[GritPolicy] action_joint_names must match "
                "reference_joint_names"
            )
        if not hasattr(policy_cfg, "action_scale"):
            policy_cfg.action_scale = _resolve_by_name(
                GRIT_ACTION_SCALE_BY_NAME, policy_cfg.action_joint_names
            ).tolist()
        if len(policy_cfg.action_scale) != len(policy_cfg.action_joint_names):
            raise ValueError("[GritPolicy] action_scale must match action_joint_names")

        super().__init__(name, policy_cfg, controller)

        # grit reference feature cache (T, 70), maintained in parallel with
        # the parent's ref_joint_pos/ref_root_quat/ref_root_pos arrays.
        self.grit_ref_features: Optional[np.ndarray] = None

        # Action conversion tables (grit ACTION order).
        self.grit_action_scale = np.array(
            policy_cfg.action_scale, dtype=np.float32
        )
        self.grit_default_pos_action = _resolve_by_name(
            grit_default_joint_pos_by_name(), list(self.action_joint_names)
        )
        encoder_offset = getattr(policy_cfg, "encoder_offset", None)
        if encoder_offset is not None:
            self.grit_encoder_offset = np.array(
                encoder_offset, dtype=np.float32
            )
            assert self.grit_encoder_offset.shape == (29,)
        else:
            self.grit_encoder_offset = np.zeros(29, dtype=np.float32)

        print(
            f"[GritPolicy] action order = {list(self.action_joint_names)[:5]}... "
            f"default keyframe nonzero = {int(np.count_nonzero(self.grit_default_pos_action))}"
        )

    def _create_motion_source(self, policy_cfg: DictToClass) -> MotionSourceBase:
        if self.motion_source == "npz":
            return GritNpzMotionSource(self, policy_cfg)
        return super()._create_motion_source(policy_cfg)

    # ---- reference feature cache -----------------------------------------

    def _build_obs_modules(self):
        self.module = GritONNXModel(self.policy_path)
        self.obs_modules = [GritReferenceContext(self), GritProprioHistory(self)]
        self.num_obs = sum(m.size for m in self.obs_modules)
        assert self.num_obs == self.module.policy_input_dim, (
            f"grit obs dim {self.num_obs} != {self.module.policy_input_dim}"
        )

    def append_ref_frames(self, frames: Dict[str, np.ndarray]) -> None:
        n_new = int(np.asarray(frames["joint_pos"]).shape[0])

        # Extend the feature cache FIRST so the parent's append/trim flow
        # keeps cache and standard arrays frame-aligned (the parent may trim
        # leading frames from both on the same call).
        precomputed = frames.get("grit_features", None)
        if precomputed is not None:
            feats = np.asarray(precomputed, dtype=np.float32).reshape(n_new, -1)
        else:
            jp_obs = np.asarray(frames["joint_pos"], dtype=np.float32)
            jp = remap_joint_array_by_names(
                jp_obs, self.obs_joint_names, self.reference_joint_names
            )
            root_pos = np.asarray(frames["root_pos"], dtype=np.float64)
            root_quat = np.asarray(frames["root_quat"], dtype=np.float64)
            have_previous = (
                self.ref_len > 0
                and self.ref_joint_pos is not None
                and self.ref_root_pos is not None
                and self.ref_root_quat is not None
                and self.grit_ref_features is not None
                and self.grit_ref_features.shape[0] > 0
            )
            previous_joint = None
            previous_pos = None
            previous_quat = None
            if have_previous:
                n_previous = min(2, int(self.ref_len))
                previous_joint = remap_joint_array_by_names(
                    self.ref_joint_pos[-n_previous:].astype(np.float32),
                    self.obs_joint_names,
                    self.reference_joint_names,
                )
                previous_pos = self.ref_root_pos[-n_previous:]
                previous_quat = self.ref_root_quat[-n_previous:]

            feats, previous_feature = build_grit_streaming_reference_features(
                jp,
                root_pos,
                root_quat,
                fps=float(getattr(self.config, "reference_fps", 50.0)),
                previous_joint_pos=previous_joint,
                previous_root_pos=previous_pos,
                previous_root_quat_wxyz=previous_quat,
            )
            if previous_feature is not None:
                self.grit_ref_features[-previous_feature.shape[0] :] = (
                    previous_feature
                )
        assert feats.shape[1] == GRIT_FEATURE_DIM

        if self.grit_ref_features is None or self.grit_ref_features.shape[0] == 0:
            self.grit_ref_features = feats
        else:
            self.grit_ref_features = np.concatenate(
                [self.grit_ref_features, feats], axis=0
            )

        super().append_ref_frames(frames)

        # Safety align: the parent's trim already shrinks the cache via the
        # _trim_ref_prefix override; enforce exact frame-alignment regardless.
        if (
            self.grit_ref_features is not None
            and self.grit_ref_features.shape[0] > self.ref_len
        ):
            drop = self.grit_ref_features.shape[0] - self.ref_len
            self.grit_ref_features = self.grit_ref_features[drop:]

    def _trim_ref_prefix(self) -> None:
        # Trim the feature cache by exactly the number of frames the parent
        # drops from the standard arrays.
        old_len = self.ref_len if self.ref_len > 0 else 0
        super()._trim_ref_prefix()
        drop = old_len - self.ref_len
        if drop > 0 and self.grit_ref_features is not None:
            self.grit_ref_features = self.grit_ref_features[drop:]

    def deactivate(self):
        super().deactivate()
        self.grit_ref_features = None

    # ---- inference --------------------------------------------------------

    def compute_action(self) -> np.ndarray:
        try:
            out = self.module(self.policy_input)
        except Exception as e:
            print(f"[GritPolicy] ONNX forward failed: {e}")
            return np.zeros(self.controller.dof_size, dtype=np.float32)

        raw = out["actions"][0].astype(np.float32)
        self.last_action[:] = raw

        # Absolute joint-position target in grit action order
        # (raw * G1_ACTION_SCALE + knees-bent keyframe - encoder offset).
        target = (
            raw * self.grit_action_scale
            + self.grit_default_pos_action
            - self.grit_encoder_offset
        )
        # deploy.Controller._apply_action computes
        #     desired = default_qpos + action_delta
        # so return the DELTA against the controller's default stance; the
        # controller adds it back and the final target is exactly `target`.
        # Both orders agree only after mapping to the controller joint order.
        target_mapped = self.mapper_action.map_action_from_to(target)
        default_qpos = np.asarray(
            getattr(self.controller, "default_qpos", np.zeros(29)),
            dtype=np.float32,
        )
        return target_mapped - default_qpos


class GritNpzMotionSource(LocalNpzMotionSource):
    """Preserve exported velocity channels for local GRIT motion clips."""

    def append_motion_from_tail(self, name: str) -> bool:
        if name not in self.motions:
            print(f"[GritNpzMotionSource] Unknown motion '{name}'")
            return False

        anchor = self.policy.read_ref_tail_state()
        aligned = self._align_motion_to_anchor(self.motions[name], anchor)

        tgt_first = {
            "joint_pos": aligned["joint_pos"][0],
            "root_quat": aligned["root_quat"][0],
            "root_pos": aligned["root_pos"][0],
        }
        prefix = self._build_transition_prefix(anchor, tgt_first)
        n_prefix = int(prefix["joint_pos"].shape[0])

        segment = {
            "joint_pos": np.concatenate([prefix["joint_pos"], aligned["joint_pos"]], 0),
            "root_quat": np.concatenate([prefix["root_quat"], aligned["root_quat"]], 0),
            "root_pos": np.concatenate([prefix["root_pos"], aligned["root_pos"]], 0),
        }

        # Preserve the exported velocity channels for the motion segment;
        # constant-yaw alignment leaves root-frame velocities invariant.
        # finite-difference fallback for the transition prefix. Motions
        # without extras (e.g. 1-frame default clips) fall back entirely.
        extras = self._grit_extras.get(name)
        jp_seg = remap_joint_array_by_names(
            segment["joint_pos"],
            self.policy.obs_joint_names,
            self.policy.reference_joint_names,
        )
        if extras is not None and n_prefix < segment["joint_pos"].shape[0]:
            feats_motion = build_grit_reference_features(
                extras["joint_pos"],  # reference order, unchanged by alignment
                aligned["root_pos"],
                aligned["root_quat"],
                fps=float(getattr(self.config, "reference_fps", 50.0)),
                joint_vel=remap_joint_array_by_names(
                    extras["joint_vel"],
                    self.policy.obs_joint_names,
                    self.policy.reference_joint_names,
                ),
                root_lin_vel_b=extras["root_lin_vel_b"],
                root_ang_vel_b=extras["root_ang_vel_b"],
            )
            feats_prefix = (
                build_grit_reference_features(
                    jp_seg[:n_prefix],
                    prefix["root_pos"],
                    prefix["root_quat"],
                    fps=float(getattr(self.config, "reference_fps", 50.0)),
                )
                if n_prefix > 0
                else np.zeros((0, GRIT_FEATURE_DIM), dtype=np.float32)
            )
            segment["grit_features"] = np.concatenate(
                [feats_prefix, feats_motion], 0
            )
        else:
            segment["grit_features"] = build_grit_reference_features(
                jp_seg,
                segment["root_pos"],
                segment["root_quat"],
                fps=float(getattr(self.config, "reference_fps", 50.0)),
            )
        self.policy.append_ref_frames(segment)

        self.policy.current_name = name
        self.policy.current_done = (
            self.policy.ref_idx >= self.policy.ref_len - 1
        )
        print(
            f"[GritNpzMotionSource] Append '{name}' | appended={segment['joint_pos'].shape[0]}, "
            f"ref_len={self.policy.ref_len}, transition={n_prefix}"
        )
        return True
