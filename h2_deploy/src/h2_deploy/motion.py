"""Load an H2 reference motion NPZ (50 Hz), preserving its recorded velocities."""
from pathlib import Path
import numpy as np
from .observations import reference_features, reference_offsets


class MotionClip:
    def __init__(self, path, joint_names, body_names):
        self.path = Path(path)
        with np.load(self.path, allow_pickle=False) as archive:
            self.fps = float(archive['fps'].reshape(-1)[0])
            if not np.isclose(self.fps,50.):
                raise ValueError('This policy requires an H2 motion sampled at 50 Hz')
            order = list(joint_names)
            stored = archive['joint_names'].tolist() if 'joint_names' in archive else order
            if len(stored)!=31 or set(stored)!=set(order):
                raise ValueError('Motion must contain all 31 H2 joints exactly once')
            remap = [stored.index(n) for n in order]
            self.joint_pos = archive['joint_pos'][:,remap].astype(np.float64)
            self.joint_vel = archive['joint_vel'][:,remap].astype(np.float64)
            self.body_names = archive['body_names'].tolist() if 'body_names' in archive else list(body_names)
            for field in ('body_pos_w','body_quat_w','body_lin_vel_w','body_ang_vel_w'):
                setattr(self,field,np.asarray(archive[field],dtype=np.float64))
        self.length = len(self.joint_pos)
        if self.length < 2:
            raise ValueError('Motion needs at least two frames')
        for field,tail in (('joint_pos',(31,)),('joint_vel',(31,)),
                           ('body_pos_w',(len(self.body_names),3)),('body_quat_w',(len(self.body_names),4)),
                           ('body_lin_vel_w',(len(self.body_names),3)),('body_ang_vel_w',(len(self.body_names),3))):
            values=getattr(self,field)
            if values.shape!=(self.length,*tail) or not np.isfinite(values).all():
                raise ValueError(f'Invalid motion array {field}: {values.shape}')
        self.root = self.body_names.index('pelvis')
        self.anchor = self.body_names.index('torso_link')
        self.features = reference_features(self.joint_pos,self.joint_vel,
            self.body_quat_w[:,self.root],self.body_lin_vel_w[:,self.root],self.body_ang_vel_w[:,self.root])
        self.offsets = reference_offsets(self.fps)

    def context(self, frame):
        indices = np.clip(int(frame)+self.offsets,0,self.length-1)
        return self.features[indices]
