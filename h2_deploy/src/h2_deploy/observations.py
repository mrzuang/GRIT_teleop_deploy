"""H2 policy inputs: reference window [9, 74] and state history [1050] (10 frames x 105)."""
import numpy as np

JOINTS = 31
REFERENCE_FRAMES = 9
FEATURE_DIM = 74
HISTORY_LENGTH = 10
TERM_DIMS = (3, 3, 6, 31, 31, 31)
MODEL_FRAME_OFFSETS = np.arange(9)


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if q.shape[-1] != 4 or np.any(norm < 1e-8) or not np.isfinite(q).all():
        raise ValueError('Expected finite nonzero wxyz quaternion')
    w, x, y, z = np.moveaxis(q / norm, -1, 0)
    return np.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                     2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=-1).reshape(q.shape[:-1]+(3,3))


def reference_features(joint_pos, joint_vel, root_quat, linear_velocity_world, angular_velocity_world):
    rotation = rotation_matrix(root_quat)
    local_linear = np.einsum('...ji,...j->...i', rotation, linear_velocity_world)
    local_angular = np.einsum('...ji,...j->...i', rotation, angular_velocity_world)
    result = np.concatenate((joint_pos, joint_vel, rotation[..., :2].reshape(-1,6),
                             local_linear, local_angular), axis=-1).astype(np.float32)
    if result.ndim != 2 or result.shape[1] != FEATURE_DIM or not np.isfinite(result).all():
        raise ValueError('Invalid H2 reference features')
    return result


def reference_offsets(fps=50.):
    return np.rint(MODEL_FRAME_OFFSETS * float(fps)/30.).astype(np.int64)


def proprio_frame(gyro, root_quat, reference_root_quat, joint_pos, joint_vel, previous_action, default_joint_pos):
    rotation = rotation_matrix(root_quat)
    reference = rotation_matrix(reference_root_quat)
    gravity = rotation.T @ np.array([0.,0.,-1.])
    orientation = (rotation.T @ reference)[:, :2].reshape(6)
    result = np.concatenate((gyro,gravity,orientation,
                             np.asarray(joint_pos)-default_joint_pos,joint_vel,previous_action)).astype(np.float32)
    if result.shape != (105,) or not np.isfinite(result).all():
        raise ValueError('Invalid H2 proprioceptive state')
    return result


class ProprioHistory:
    """Oldest-to-newest within each term; first observation fills all slots."""
    def __init__(self):
        self.frames = np.zeros((HISTORY_LENGTH, sum(TERM_DIMS)), np.float32)
        self.initialized = False

    def reset(self):
        self.frames.fill(0)
        self.initialized = False

    def append(self, frame):
        frame = np.asarray(frame,dtype=np.float32)
        if frame.shape != (105,) or not np.isfinite(frame).all():
            raise ValueError('History requires one finite 105D frame')
        if not self.initialized:
            self.frames[:] = frame
            self.initialized = True
        else:
            self.frames[:-1] = self.frames[1:]
            self.frames[-1] = frame
        return self.packed()

    def packed(self):
        return np.concatenate([part.reshape(-1) for part in
                               np.split(self.frames,np.cumsum(TERM_DIMS)[:-1],axis=1)])
