"""Observation builders for the current 70D, future-nine deployment policy.

The deployment ONNX has exactly two inputs:

    reference_context [B, 9, 70]  current plus 8 future frames at 30 Hz
    proprio_history   [B, 990]    six 10-frame histories in term-major order

Semantics are pinned to the exported GRIT model contract:
  * gravity is the UNIT (0,0,-1) vector rotated into the pelvis frame;
  * rot6d = first two COLUMNS of the rotation matrix, flattened row-major;
  * reference joint_pos is raw (no default-pose subtraction) while the
    proprioceptive joint_pos subtracts the knees-bent default keyframe;
  * every one of the nine 70D reference frames passes through the shared
    graph encoder. At a 50 Hz motion rate, 30 Hz model offsets [0..8] map to
    motion indices [0, 2, 3, 5, 7, 8, 10, 12, 13].
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from runtime.observation import Observation

GRIT_REF_FPS = 50.0
GRIT_MODEL_FPS = 30.0
GRIT_CONTEXT_FRAMES = 9
GRIT_FEATURE_DIM = 70
GRIT_PROPRIO_TERM_DIMS = (3, 3, 6, 29, 29, 29)
GRIT_PROPRIO_FRAME_DIM = sum(GRIT_PROPRIO_TERM_DIMS)  # 99
GRIT_HISTORY_LENGTH = 10


def grit_context_offsets(
    reference_fps: float = GRIT_REF_FPS,
    model_fps: float = GRIT_MODEL_FPS,
) -> np.ndarray:
    """Map model-rate offsets [0..8] to indices in the reference stream."""
    if reference_fps <= 0 or model_fps <= 0:
        raise ValueError("reference_fps and model_fps must be positive")
    offsets = np.arange(GRIT_CONTEXT_FRAMES, dtype=np.float64)
    return np.round(offsets * (reference_fps / model_fps)).astype(np.int64)


def grit_default_joint_pos_by_name() -> dict[str, float]:
    """Knees-bent default keyframe (mjlab g1_constants.KNEES_BENT_KEYFRAME)."""
    d = {
        ".*_hip_pitch_joint": -0.312,
        ".*_knee_joint": 0.669,
        ".*_ankle_pitch_joint": -0.363,
        ".*_elbow_joint": 0.6,
        "left_shoulder_roll_joint": 0.2,
        "left_shoulder_pitch_joint": 0.2,
        "right_shoulder_roll_joint": -0.2,
        "right_shoulder_pitch_joint": 0.2,
    }
    return d


def _resolve_keyframe(values: dict[str, float], names: list[str]) -> np.ndarray:
    """Expand a regex keyframe dict against a name list (last match wins)."""
    out = np.zeros(len(names), dtype=np.float32)
    import re

    for i, name in enumerate(names):
        for pat, v in values.items():
            if re.fullmatch(pat, name):
                out[i] = v
    return out


def _r_from_quat_wxyz(quat_wxyz: np.ndarray):
    """scipy Rotation from wxyz quaternion(s).

    Converts wxyz -> xyzw explicitly and avoids the ``scalar_first`` keyword:
    some scipy builds (e.g. certain conda 1.9 wheels) expose ``from_quat``
    as a C builtin that rejects keyword arguments.
    ``quat_wxyz`` may be (4,) or (N, 4); a single quat returns a Rotation.
    """
    quat = np.asarray(quat_wxyz, dtype=np.float64)
    single = quat.ndim == 1
    if single:
        quat = quat[None, :]
    xyzw = np.concatenate([quat[:, 1:], quat[:, :1]], axis=-1)
    r = R.from_quat(xyzw)
    return r[0] if single else r


def _rot6d_cols(quat_wxyz: np.ndarray) -> np.ndarray:
    """rot6d = first two columns of the rotation matrix, row-major flatten.

    Matches mjlab ``matrix_from_quat(q)[..., :2].reshape(-1, 6)``.
    ``quat_wxyz`` may be (4,) or (N, 4).
    """
    quat = np.asarray(quat_wxyz, dtype=np.float64)
    single = quat.ndim == 1
    if single:
        quat = quat[None, :]
    mat = _r_from_quat_wxyz(quat).as_matrix()  # (N,3,3)
    rot6d = mat[:, :, :2].reshape(-1, 6).astype(np.float32)
    return rot6d[0] if single else rot6d


def build_grit_reference_features(
    joint_pos: np.ndarray,
    root_pos: np.ndarray,
    root_quat_wxyz: np.ndarray,
    fps: float = GRIT_REF_FPS,
    joint_vel: np.ndarray | None = None,
    root_lin_vel_b: np.ndarray | None = None,
    root_ang_vel_b: np.ndarray | None = None,
) -> np.ndarray:
    """Build per-frame 70D reference features for an aligned segment.

    Args:
        joint_pos: (N, 29) absolute joint angles in qpos order.
        root_pos: (N, 3) world root (pelvis) position.
        root_quat_wxyz: (N, 4) world root quaternion, wxyz.
        fps: reference frame rate (50).
        joint_vel / root_lin_vel_b / root_ang_vel_b:
            optional precomputed channels (e.g. from a mjlab-format npz).
            Missing channels fall back to finite differences / analytic FK.
    """
    n = joint_pos.shape[0]
    if n < 1:
        return np.zeros((0, GRIT_FEATURE_DIM), dtype=np.float32)

    quat = np.asarray(root_quat_wxyz, dtype=np.float64)
    Rm = _r_from_quat_wxyz(quat).as_matrix()  # (N,3,3)

    if joint_vel is None:
        jv = np.zeros_like(joint_pos, dtype=np.float32)
        if n > 1:
            jv[0] = (joint_pos[1] - joint_pos[0]) * fps
            jv[-1] = (joint_pos[-1] - joint_pos[-2]) * fps
        if n > 2:
            jv[1:-1] = (joint_pos[2:] - joint_pos[:-2]) * (0.5 * fps)
    else:
        jv = np.asarray(joint_vel, dtype=np.float32)

    if root_lin_vel_b is None:
        v_w = np.zeros((n, 3), dtype=np.float64)
        if n > 1:
            v_w[0] = (root_pos[1] - root_pos[0]) * fps
            v_w[-1] = (root_pos[-1] - root_pos[-2]) * fps
        if n > 2:
            v_w[1:-1] = (root_pos[2:] - root_pos[:-2]) * (0.5 * fps)
        v_b = np.einsum("nij,nj->ni", np.transpose(Rm, (0, 2, 1)), v_w)
    else:
        v_b = np.asarray(root_lin_vel_b, dtype=np.float32)

    if root_ang_vel_b is None:
        w_b = np.zeros((n, 3), dtype=np.float64)
        rotations = R.from_matrix(Rm)
        if n > 1:
            w_b[0] = (rotations[0].inv() * rotations[1]).as_rotvec() * fps
            w_b[-1] = (
                rotations[-2].inv() * rotations[-1]
            ).as_rotvec() * fps
        for i in range(1, n - 1):
            forward = (rotations[i].inv() * rotations[i + 1]).as_rotvec()
            backward = (rotations[i].inv() * rotations[i - 1]).as_rotvec()
            w_b[i] = (forward - backward) * (0.5 * fps)
    else:
        w_b = np.asarray(root_ang_vel_b, dtype=np.float32)

    rot6d = _rot6d_cols(quat)  # (N,6)
    return np.concatenate(
        [
            joint_pos.astype(np.float32),
            jv,
            rot6d,
            v_b.astype(np.float32),
            w_b.astype(np.float32),
        ],
        axis=-1,
    )


def build_grit_streaming_reference_features(
    joint_pos: np.ndarray,
    root_pos: np.ndarray,
    root_quat_wxyz: np.ndarray,
    *,
    fps: float = GRIT_REF_FPS,
    previous_joint_pos: np.ndarray | None = None,
    previous_root_pos: np.ndarray | None = None,
    previous_root_quat_wxyz: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray | None]:
    """Build finite-difference features for frames arriving in small batches.

    A one-frame teleop packet has no internal temporal difference. Prepending
    the existing reference tail recovers the velocity channels and also lets
    the caller replace the formerly provisional tail feature.
    """
    previous = (
        previous_joint_pos,
        previous_root_pos,
        previous_root_quat_wxyz,
    )
    if all(value is None for value in previous):
        return (
            build_grit_reference_features(
                joint_pos,
                root_pos,
                root_quat_wxyz,
                fps=fps,
            ),
            None,
        )
    if any(value is None for value in previous):
        raise ValueError("Previous streaming reference state must be complete")

    previous_joint = np.asarray(previous_joint_pos).reshape(-1, joint_pos.shape[-1])
    previous_position = np.asarray(previous_root_pos).reshape(-1, 3)
    previous_quaternion = np.asarray(previous_root_quat_wxyz).reshape(-1, 4)
    if not (
        previous_joint.shape[0]
        == previous_position.shape[0]
        == previous_quaternion.shape[0]
    ):
        raise ValueError("Previous streaming reference arrays must have equal length")

    joint = np.concatenate([previous_joint, np.asarray(joint_pos)], axis=0)
    position = np.concatenate([previous_position, np.asarray(root_pos)], axis=0)
    quaternion = np.concatenate(
        [previous_quaternion, np.asarray(root_quat_wxyz)],
        axis=0,
    )
    combined = build_grit_reference_features(
        joint,
        position,
        quaternion,
        fps=fps,
    )
    n_previous = previous_joint.shape[0]
    return combined[n_previous:], combined[:n_previous].copy()


class GritReferenceContext(Observation):
    """630D current-plus-future window sampled at the 30 Hz model rate."""

    def __init__(self, policy):
        self.policy = policy
        reference_fps = float(getattr(policy.config, "reference_fps", GRIT_REF_FPS))
        self.offsets = grit_context_offsets(reference_fps=reference_fps)

    @property
    def size(self) -> int:
        return GRIT_CONTEXT_FRAMES * GRIT_FEATURE_DIM

    def reset(self):
        pass

    def update(self):
        pass

    def compute(self) -> np.ndarray:
        feats = self.policy.grit_ref_features
        if feats is None or feats.shape[0] == 0:
            raise ValueError("grit reference features not available yet.")
        n = feats.shape[0]
        idx = np.clip(self.policy.ref_idx + self.offsets, 0, n - 1)
        return feats[idx].reshape(-1).astype(np.float32)


class GritProprioHistory(Observation):
    """990D term-major history of the 99D proprioceptive frame."""

    # term order must match GRIT_PROPRIO_TERM_DIMS (gyro, gravity, ori_b,
    # joint_pos, joint_vel, last_action).
    max_step = GRIT_HISTORY_LENGTH - 1  # drives the base warmup loop

    def __init__(self, policy):
        self.policy = policy
        self.ctrl = policy.controller
        dims = GRIT_PROPRIO_TERM_DIMS
        self.hists = [
            np.zeros((GRIT_HISTORY_LENGTH, d), dtype=np.float32) for d in dims
        ]
        # Remap controller order (policy_joint_names, paired) -> qpos order.
        from common.joint_mapper import JointMapper

        self.mapper_state = JointMapper(
            list(self.ctrl.config.policy_joint_names),
            list(self.policy.reference_joint_names),
        )
        self.default_qpos = _resolve_keyframe(
            grit_default_joint_pos_by_name(), list(self.policy.reference_joint_names)
        )

    @property
    def size(self) -> int:
        return GRIT_HISTORY_LENGTH * GRIT_PROPRIO_FRAME_DIM

    def reset(self):
        for h in self.hists:
            h[:] = 0.0

    def update(self):
        gyro = self.ctrl.gyro.astype(np.float32)  # pelvis frame

        # Unit gravity vector in the pelvis frame (mjlab gravity_vec_w is
        # [0, 0, -1.0], NOT the physical 9.81).
        g_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        grav = (
            _r_from_quat_wxyz(self.ctrl.quat).inv().apply(g_world).astype(np.float32)
        )

        # Reference root orientation relative to the robot pelvis, rot6d.
        if self.policy.ref_root_quat is not None and self.policy.ref_len > 0:
            q_ref = self.policy.ref_root_quat[
                min(self.policy.ref_idx, self.policy.ref_len - 1)
            ]
        else:
            q_ref = self.ctrl.quat
        r_rel = _r_from_quat_wxyz(self.ctrl.quat).inv() * _r_from_quat_wxyz(q_ref)
        ori_b = r_rel.as_matrix()[:, :2].reshape(-1).astype(np.float32)  # (6,)

        # Joint state in qpos order; joint_pos relative to the default keyframe
        # (measured encoder values already contain the real encoder offset).
        # NOTE: JointMapper.map_state_to_from maps TO->FROM (reverse of
        # map_action_from_to); the controller state is in the FROM space here.
        q = self.mapper_state.map_action_from_to(
            self.ctrl.qj.astype(np.float32)
        ).astype(np.float32)
        dq = self.mapper_state.map_action_from_to(
            self.ctrl.dqj.astype(np.float32)
        ).astype(np.float32)
        jpos = q - self.default_qpos

        # Previous raw policy output in grit action order.
        last_action = self.policy.last_action.astype(np.float32)

        frames = [gyro, grav, ori_b, jpos, dq, last_action]
        assert len(self.hists) == len(frames)
        # Newest frame at the END: matches the mjlab CircularBuffer flattening
        # (chronological oldest -> newest per term); the network's frame-major
        # conversion takes the LAST frame as the current state.
        for hist, frame in zip(self.hists, frames):
            hist[:] = np.roll(hist, -1, axis=0)
            hist[-1] = frame

    def compute(self) -> np.ndarray:
        # Term-major: [A_t0..A_t9 | B_t0..B_t9 | ...], t0 = oldest, t9 = newest.
        return np.concatenate([h.reshape(-1) for h in self.hists], axis=-1)
