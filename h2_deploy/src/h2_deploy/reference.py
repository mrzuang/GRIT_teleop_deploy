"""Reference trajectory utilities: heading alignment and clip cutting.

The policy reads the reference root orientation in the world frame and the
robot/reference relative orientation. On hardware the IMU heading is
arbitrary, so the reference must be rotated about world z to start at the
robot's current heading. Body-local velocities are unchanged by this rotation.
"""
from pathlib import Path

import numpy as np

from .motion import MotionClip
from .observations import reference_features

MOTION_FIELDS = ('joint_pos', 'joint_vel', 'body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w')


def yaw_of(quat_wxyz):
    w, x, y, z = np.moveaxis(np.asarray(quat_wxyz, dtype=np.float64), -1, 0)
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def yaw_quat(yaw):
    return np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])


def quat_mul(a, b):
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack((aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw), axis=-1)


def rotate_arrays(arrays, yaw, xy_offset=(0., 0.)):
    """Rotate world-frame motion arrays by yaw about z, then translate in XY."""
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    out = dict(arrays)
    out['body_pos_w'] = arrays['body_pos_w'] @ rotation.T + np.array([xy_offset[0], xy_offset[1], 0.])
    out['body_quat_w'] = quat_mul(np.broadcast_to(yaw_quat(yaw), arrays['body_quat_w'].shape), arrays['body_quat_w'])
    out['body_lin_vel_w'] = arrays['body_lin_vel_w'] @ rotation.T
    out['body_ang_vel_w'] = arrays['body_ang_vel_w'] @ rotation.T
    return out


def align_clip_heading(clip: MotionClip, robot_root_quat, frame=0):
    """Rotate a loaded clip in place so its root heading at `frame` matches the robot."""
    delta = float(yaw_of(robot_root_quat) - yaw_of(clip.body_quat_w[frame, clip.root]))
    arrays = rotate_arrays({k: getattr(clip, k) for k in MOTION_FIELDS}, delta)
    for key in ('body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w'):
        setattr(clip, key, arrays[key])
    clip.features = reference_features(clip.joint_pos, clip.joint_vel, clip.body_quat_w[:, clip.root],
                                       clip.body_lin_vel_w[:, clip.root], clip.body_ang_vel_w[:, clip.root])
    return delta


def cut_clip(source, output, start_s, seconds, joint_names, body_names):
    """Cut [start, start+seconds) and normalise the first frame to the origin, heading +x."""
    with np.load(source, allow_pickle=False) as archive:
        fps = float(archive['fps'].reshape(-1)[0])
        a, b = round(start_s*fps), round((start_s+seconds)*fps)
        if not 0 <= a < b <= len(archive['joint_pos']):
            raise ValueError('Window lies outside the source motion')
        arrays = {k: np.asarray(archive[k][a:b], dtype=np.float64) for k in MOTION_FIELDS}
    root = list(body_names).index('pelvis')
    yaw = -float(yaw_of(arrays['body_quat_w'][0, root]))
    arrays = rotate_arrays(arrays, yaw)
    arrays['body_pos_w'][..., :2] -= arrays['body_pos_w'][0, root, :2]
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, fps=np.array([fps], np.float32),
                        joint_names=np.array(joint_names), body_names=np.array(body_names),
                        **{k: v.astype(np.float32) for k, v in arrays.items()})
    return b - a
