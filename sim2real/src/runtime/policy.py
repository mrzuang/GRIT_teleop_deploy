import json
import os
import statistics
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import onnxruntime as ort
from common.joint_mapper import JointMapper
from common.utils import DictToClass
from runtime.motion_sources import (
    LocalNpzMotionSource,
    MotionSourceBase,
    UDPMotionSource,
    VRMotionSource,
)

def benchmark_onnx(module, sample_input, runs=100, warmup=10, desc=""):
    for _ in range(warmup):
        _ = module(sample_input)

    ts = []
    for _ in range(runs):
        t0 = time.perf_counter()
        _ = module(sample_input)
        t1 = time.perf_counter()
        ts.append((t1 - t0) * 1000.0)

    mean = statistics.mean(ts)
    stdev = statistics.pstdev(ts)
    p50 = np.percentile(ts, 50)
    p90 = np.percentile(ts, 90)
    p95 = np.percentile(ts, 95)
    p99 = np.percentile(ts, 99)

    print(f"[{desc}] runs={runs}, warmup={warmup}")
    print(f"mean={mean:.3f} ms, stdev={stdev:.3f} ms")
    print(f"p50={p50:.3f} ms, p90={p90:.3f} ms, p95={p95:.3f} ms, p99={p99:.3f} ms")
    return {"mean": mean, "stdev": stdev, "p50": p50, "p90": p90, "p95": p95, "p99": p99}


class ONNXRuntimeModel:
    CPU_AFFINITY = (4, 5, 6, 7)
    CPU_THREADS = len(CPU_AFFINITY)

    @classmethod
    def _bind_process_to_first_cpus(cls) -> None:
        if not hasattr(os, "sched_getaffinity") or not hasattr(os, "sched_setaffinity"):
            return
        try:
            allowed = sorted(os.sched_getaffinity(0))
            target = [cpu for cpu in cls.CPU_AFFINITY if cpu in allowed]
            if len(target) != len(cls.CPU_AFFINITY):
                print(
                    f"[ONNXRuntimeModel] Requested CPUs {list(cls.CPU_AFFINITY)} but only "
                    f"{target} are available in current affinity mask {allowed}"
                )
            if len(target) >= 1:
                os.sched_setaffinity(0, set(target))
                print(f"[ONNXRuntimeModel] Bound process affinity to CPUs {target}")
        except Exception as e:
            print(f"[ONNXRuntimeModel] Failed to set process affinity: {e}")

    def __init__(self, path: str):
        self._bind_process_to_first_cpus()
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = self.CPU_THREADS
        sess_options.inter_op_num_threads = 1
        # sess_options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        self.ort_session = ort.InferenceSession(
            path,
            sess_options=sess_options,
            providers=["CPUExecutionProvider"],
        )
        meta_path = path.replace(".onnx", ".json")
        with open(meta_path, "r") as f:
            self.meta = json.load(f)
        self.in_keys = [k if isinstance(k, str) else tuple(k) for k in self.meta["in_keys"]]
        self.out_keys = [k if isinstance(k, str) else tuple(k) for k in self.meta["out_keys"]]

    def __call__(self, input: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        args = {
            inp.name: input[key]
            for inp, key in zip(self.ort_session.get_inputs(), self.in_keys)
            if key in input
        }
        outputs = self.ort_session.run(None, args)
        outputs = {k: v for k, v in zip(self.out_keys, outputs)}
        return outputs

# =========================================
# Policy Base
# =========================================
class Policy:
    def __init__(self, name: str, policy_cfg: DictToClass, controller):
        self.name = name
        self.controller = controller

        self.config = policy_cfg

        # Resolve the model relative to the tracking configuration.
        p = Path(policy_cfg.policy_path)
        cfg_dir = Path(getattr(policy_cfg, "_config_dir"))
        self.policy_path = str(p if p.is_absolute() else (cfg_dir / p))
        self.action_joint_names = list(policy_cfg.action_joint_names)

        self.mapper_action = JointMapper(
            self.action_joint_names,
            self.controller.config.policy_joint_names
        )
        map_info = self.mapper_action.get_mapping_info()
        print(f"[Policy:{self.name}] Action mapping: {map_info['mapped_joints']}/{map_info['from_space_size']} mapped")
        if map_info['unmapped_from_joints']:
            print(f"[Policy:{self.name}] Unmapped policy action joints: {map_info['unmapped_from_joints']}")
        if map_info['unmapped_to_joints']:
            print(f"[Policy:{self.name}] Unmapped controller joints: {map_info['unmapped_to_joints']}")

        self.policy_input: Optional[Dict[str, np.ndarray]] = None
        self.last_action = np.zeros(len(self.action_joint_names), dtype=np.float32)

        self.obs_modules = []
        self.num_obs = 0
        self._build_obs_modules()

        self.policy_input = {
            "policy": np.zeros((1, self.num_obs), dtype=np.float32)
        }
        input_shape = self.module.ort_session.get_inputs()[0].shape
        expected_obs_dim = getattr(self.module, "policy_input_dim", None)
        if expected_obs_dim is None:
            expected_obs_dim = input_shape[-1] if len(input_shape) > 0 else None
        if isinstance(expected_obs_dim, int) and expected_obs_dim != self.num_obs:
            raise ValueError(
                f"[Policy:{self.name}] Observation dim mismatch: built={self.num_obs}, "
                f"onnx expects {expected_obs_dim}. Please align tracking.yaml observation settings."
            )
        benchmark_onnx(self.module, self.policy_input, runs=100, warmup=200, desc="model@cpu")

    # -------- lifecycle ----------
    def fade_in(self):
        self.reset()
        print(f"[Policy:{self.name}] fade_in()")

    def deactivate(self):
        print(f"[Policy:{self.name}] deactivated")

    # -------- abstract hooks ----------
    def _build_obs_modules(self):
        raise NotImplementedError

    def _reset_obs_modules(self):
        for m in self.obs_modules:
            if hasattr(m, "reset") and callable(m.reset):
                m.reset()
        # Fill temporal observations with the current state before inference.
        warmup = max(
            (m.max_step for m in self.obs_modules if hasattr(m, "max_step")),
            default=0,
        )
        for _ in range(warmup + 1):
            for m in self.obs_modules:
                if hasattr(m, "max_step") and hasattr(m, "update"):
                    m.update()

    def update_obs(self):
        obs_list = []
        for m in self.obs_modules:
            m.update()
            val = m.compute()
            obs_list.append(val)
        if self.policy_input is None:
            self.policy_input = {
                "policy": np.zeros((1, self.num_obs), dtype=np.float32),
            }
        self.policy_input["policy"][0, :] = np.concatenate(obs_list, axis=0)

    def compute_action(self) -> np.ndarray:
        raise NotImplementedError

    def reset(self):
        self.policy_input = None
        self.last_action[:] = 0.0
        self._reset_obs_modules()

    def post_step(self):
        """Hook called once after each policy inference/application step."""
        return

# =========================================
# Policy Subclasses
# =========================================
class ReferenceTrackingPolicy(Policy):
    def __init__(self, name: str, policy_cfg: DictToClass, controller):
        self.controller = controller
        # ---- Config ---------------------------------------------------------
        self.transition_steps = int(getattr(policy_cfg, "transition_steps", 100))
        self.switch_tail_keep_steps = int(
            getattr(policy_cfg, "switch_tail_keep_steps", 8)
        )
        motion_source_cfg = getattr(policy_cfg, "motion_source")
        self.motion_source = str(motion_source_cfg["type"]).strip().lower()
        if self.motion_source not in ("udp", "vr", "npz"):
            raise ValueError(
                f"[ReferenceTrackingPolicy] motion_source must be 'udp', 'vr' or 'npz', got '{self.motion_source}'"
            )
        self.ref_max_len = int(getattr(policy_cfg, "ref_max_len", 2048))
        self.reference_joint_names = list(getattr(policy_cfg, "reference_joint_names", []))
        if len(self.reference_joint_names) == 0:
            raise ValueError(
                "[ReferenceTrackingPolicy] reference_joint_names must be provided in tracking.yaml."
            )
        self.obs_joint_names = controller.config.policy_joint_names
        self.n_joints = len(self.obs_joint_names)

        # ---- Reference stream ----------------------------------------------
        self.ref_joint_pos: Optional[np.ndarray] = None  # (T_ref, J)
        self.ref_root_quat: Optional[np.ndarray] = None  # (T_ref, 4)
        self.ref_root_pos: Optional[np.ndarray] = None   # (T_ref, 3)

        # ---- Playback state ------------------------------------------------
        self.ref_idx: int = 0
        self.ref_len: int = 0
        self.current_name: str = "default"
        self.current_done: bool = True  # boot: default done

        self.source = self._create_motion_source(policy_cfg)
        self.motions = self.source.motions

        super().__init__(name, policy_cfg, controller)

    def _create_motion_source(self, policy_cfg: DictToClass) -> MotionSourceBase:
        if self.motion_source == "udp":
            return UDPMotionSource(self, policy_cfg)
        if self.motion_source == "vr":
            return VRMotionSource(self, policy_cfg)
        if self.motion_source == "npz":
            return LocalNpzMotionSource(self, policy_cfg)
        raise RuntimeError(f"Unsupported motion source: {self.motion_source}")

    def fade_in(self):
        super().fade_in()
        self.source.on_fade_in()

    def deactivate(self):
        self.source.deactivate()
        self.ref_root_pos = None
        self.ref_root_quat = None
        self.ref_joint_pos = None
        super().deactivate()

    def update_obs(self):
        if self.ref_len > 0 and self.ref_idx < self.ref_len - 1:
            self.ref_idx += 1
            if self.ref_idx == self.ref_len - 1:
                self.current_done = True
        super().update_obs()

    def read_current_state(self) -> Dict[str, np.ndarray]:
        q_policy = self.controller.qj.copy().astype(np.float32)

        if self.ref_root_pos is not None:
            root_pos = self.ref_root_pos[self.ref_idx]
            root_quat = self.ref_root_quat[self.ref_idx]
        else:
            if hasattr(self, "motions") and "default" in self.motions:
                root_pos = self.motions["default"]["root_pos"][0].astype(np.float32, copy=True)
            else:
                root_pos = np.array([0.0, 0.0, 0.78], dtype=np.float32)
            root_quat = self.controller.quat.copy()
        return {
            "joint_pos": q_policy,
            "root_pos": root_pos,
            "root_quat": root_quat,
        }

    def read_ref_tail_state(self) -> Dict[str, np.ndarray]:
        if (
            self.ref_joint_pos is not None
            and self.ref_root_quat is not None
            and self.ref_root_pos is not None
            and self.ref_len > 0
        ):
            return {
                "joint_pos": self.ref_joint_pos[self.ref_len - 1].astype(np.float32, copy=True),
                "root_pos": self.ref_root_pos[self.ref_len - 1].astype(np.float32, copy=True),
                "root_quat": self.ref_root_quat[self.ref_len - 1].astype(np.float32, copy=True),
            }
        return self.read_current_state()

    def append_ref_frames(self, frames: Dict[str, np.ndarray]) -> None:
        if frames is None:
            return

        j = np.asarray(frames["joint_pos"], dtype=np.float32)
        q = np.asarray(frames["root_quat"], dtype=np.float32)
        p = np.asarray(frames["root_pos"], dtype=np.float32)

        if j.ndim == 1:
            j = j.reshape(1, -1)
        if q.ndim == 1:
            q = q.reshape(1, -1)
        if p.ndim == 1:
            p = p.reshape(1, -1)

        if j.shape[0] != q.shape[0] or j.shape[0] != p.shape[0]:
            raise ValueError(f"Frame length mismatch: joint={j.shape}, quat={q.shape}, pos={p.shape}")
        if j.shape[0] == 0:
            return
        if j.shape[1] != self.n_joints:
            raise ValueError(f"Joint dim mismatch: got={j.shape[1]}, expected={self.n_joints}")
        if q.shape[1] != 4 or p.shape[1] != 3:
            raise ValueError(f"Root dim mismatch: quat={q.shape[1]}, pos={p.shape[1]}")

        if self.ref_joint_pos is None or self.ref_root_quat is None or self.ref_root_pos is None or self.ref_len <= 0:
            self.ref_joint_pos = j.copy()
            self.ref_root_quat = q.copy()
            self.ref_root_pos = p.copy()
        else:
            self.ref_joint_pos = np.concatenate([self.ref_joint_pos, j], axis=0)
            self.ref_root_quat = np.concatenate([self.ref_root_quat, q], axis=0)
            self.ref_root_pos = np.concatenate([self.ref_root_pos, p], axis=0)

        self.ref_len = int(self.ref_joint_pos.shape[0])
        self.current_done = (self.ref_idx >= self.ref_len - 1)
        self._trim_ref_prefix()

    def _trim_ref_prefix(self) -> None:
        if (
            self.ref_joint_pos is None
            or self.ref_root_quat is None
            or self.ref_root_pos is None
            or self.ref_len <= 0
        ):
            return

        keep_hist = self.switch_tail_keep_steps + 2
        drop = max(0, int(self.ref_idx) - int(keep_hist))
        if self.ref_max_len > 0:
            overflow = max(0, int(self.ref_len) - int(self.ref_max_len))
            drop = max(drop, min(overflow, max(0, int(self.ref_idx) - int(keep_hist))))
        if drop <= 0:
            return

        self.ref_joint_pos = self.ref_joint_pos[drop:]
        self.ref_root_quat = self.ref_root_quat[drop:]
        self.ref_root_pos = self.ref_root_pos[drop:]
        self.ref_idx -= drop
        self.ref_len = int(self.ref_joint_pos.shape[0])
        self.current_done = (self.ref_idx >= self.ref_len - 1)

    def post_step(self):
        self.source.post_step()
