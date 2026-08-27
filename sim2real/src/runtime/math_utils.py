import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


def _normalize_quat_wxyz(quat: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    quat = np.asarray(quat)
    norm = np.linalg.norm(quat, axis=-1, keepdims=True)
    return quat / np.maximum(norm, eps)


def yaw_quat_np(quat: np.ndarray) -> np.ndarray:
    """Extract the yaw-only component of wxyz quaternion(s)."""
    quat = np.asarray(quat)
    if quat.shape[-1] != 4:
        raise ValueError("quat shape must be (..., 4) in wxyz order")
    original_shape = quat.shape
    flat = quat.reshape(-1, 4)
    w, x, y, z = flat.T
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    half_yaw = 0.5 * yaw

    yaw_quat = np.zeros_like(flat, dtype=np.result_type(quat, np.float32))
    yaw_quat[:, 0] = np.cos(half_yaw)
    yaw_quat[:, 3] = np.sin(half_yaw)
    return _normalize_quat_wxyz(yaw_quat).reshape(original_shape)


def _slerp(q0: np.ndarray, q1: np.ndarray, steps: int) -> np.ndarray:
    """Interpolate wxyz quaternions, excluding both endpoints."""
    if steps <= 0:
        return np.zeros((0, 4), dtype=np.float32)
    quaternions_wxyz = np.stack([q0, q1], axis=0)
    quaternions_xyzw = np.concatenate(
        [quaternions_wxyz[:, 1:], quaternions_wxyz[:, :1]], axis=-1
    )
    rotations = R.from_quat(quaternions_xyzw)
    fractions = np.linspace(0.0, 1.0, steps + 2)[1:-1]
    interpolated_xyzw = Slerp([0.0, 1.0], rotations)(fractions).as_quat()
    return np.concatenate(
        [interpolated_xyzw[:, 3:4], interpolated_xyzw[:, :3]], axis=-1
    ).astype(np.float32)


def _linspace_rows(a: np.ndarray, b: np.ndarray, steps: int) -> np.ndarray:
    """Linearly interpolate vectors, excluding both endpoints."""
    if steps <= 0:
        return np.zeros((0, a.shape[-1]), dtype=np.float32)
    fractions = np.linspace(0.0, 1.0, steps + 2)[1:-1, None]
    return (a[None, :] * (1.0 - fractions) + b[None, :] * fractions).astype(
        np.float32
    )


def _yaw_component_wxyz(quat: np.ndarray) -> np.ndarray:
    return yaw_quat_np(quat).astype(np.float32, copy=False)
