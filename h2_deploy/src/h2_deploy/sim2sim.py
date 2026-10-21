"""Run the H2 ONNX policy in standalone CPU MuJoCo."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import mujoco as mj
import numpy as np
from .motion import MotionClip
from .observations import rotation_matrix
from .policy import H2Policy

ROOT=Path(__file__).resolve().parents[2]


class Simulator:
    def __init__(self,config,policy_path,motion_path,start_seconds=0.,action_delay=0):
        self.config=config
        self.scene=ROOT/config['scene']
        if hashlib.sha256(self.scene.read_bytes()).hexdigest()!=config['scene_sha256']:
            raise ValueError('MuJoCo scene differs from the packaged deployment configuration')
        self.model=mj.MjModel.from_xml_path(str(self.scene))
        self.data=mj.MjData(self.model)
        self.policy=H2Policy(policy_path,config)
        self.motion=MotionClip(motion_path,config['joint_names'],config['body_names'])
        self.joint_ids=np.array([self.model.joint(n).id for n in config['joint_names']])
        self.qpos_ids=self.model.jnt_qposadr[self.joint_ids]
        self.qvel_ids=self.model.jnt_dofadr[self.joint_ids]
        self.actuator_ids=np.array([int(np.flatnonzero(self.model.actuator_trnid[:,0]==jid)[0]) for jid in self.joint_ids])
        self.pelvis=self.model.body('pelvis').id
        self.torso=self.model.body('torso_link').id
        self.gyro_id=self.model.sensor('imu_ang_vel').id
        self.gyro_adr=self.model.sensor_adr[self.gyro_id]
        self.start_frame=round(start_seconds*self.motion.fps)
        if not 0<=self.start_frame<self.motion.length-1:raise ValueError('Start time lies outside the motion')
        if not 0<=action_delay<=2:raise ValueError('Action delay must be 0, 1 or 2 physics steps')
        self.delay=action_delay
        self.reset_count=0
        self.reset()

    def reset(self):
        mj.mj_resetData(self.model,self.data)
        self.frame=self.start_frame
        m=self.motion;f=self.frame
        self.data.qpos[:3]=m.body_pos_w[f,m.root]
        self.data.qpos[3:7]=m.body_quat_w[f,m.root]
        limits=np.asarray(self.config['reset_soft_joint_limits'])
        self.data.qpos[self.qpos_ids]=np.clip(m.joint_pos[f],limits[:,0],limits[:,1])
        self.data.qvel[:3]=m.body_lin_vel_w[f,m.root]
        self.data.qvel[3:6]=rotation_matrix(m.body_quat_w[f,m.root]).T@m.body_ang_vel_w[f,m.root]
        self.data.qvel[self.qvel_ids]=m.joint_vel[f]
        self.data.ctrl[self.actuator_ids]=self.data.qpos[self.qpos_ids]
        mj.mj_forward(self.model,self.data)
        self.policy.reset()
        self.pending=deque([np.zeros(31) for _ in range(self.delay)])
        self.position_errors=[];self.root_violation_steps=0;self.first_root_failure=None

    def step(self):
        if self.frame>=self.motion.length-1:return False
        m=self.motion;f=self.frame
        self.step_reference(m.context(f),m.body_quat_w[f,m.root])
        self.frame+=1
        error=float(np.linalg.norm(self.data.xpos[self.torso]-m.body_pos_w[self.frame,m.anchor]))
        self.position_errors.append(error)
        self.root_violation_steps=self.root_violation_steps+1 if error>.6 else 0
        if self.root_violation_steps>=15 and self.first_root_failure is None:
            self.first_root_failure=(self.frame-self.start_frame)*self.config['control_dt']
            print(f'Root tracking error exceeded 0.6 m for 15 frames at {self.first_root_failure:.2f}s; continuous playback.',flush=True)
        return True

    def step_reference(self,context,reference_root_quat):
        target=self.policy.step(context,reference_root_quat,
            gyro=self.data.sensordata[self.gyro_adr:self.gyro_adr+3].copy(),
            root_quat=self.data.xquat[self.pelvis].copy(),
            joint_pos=self.data.qpos[self.qpos_ids]+self.policy.encoder_offset,
            joint_vel=self.data.qvel[self.qvel_ids].copy())
        self.step_position_target(target)

    def step_position_target(self,target):
        target=np.asarray(target)
        if target.shape!=(31,) or not np.isfinite(target).all():raise ValueError('Expected finite H2 position targets')
        for _ in range(self.config['decimation']):
            self.pending.append(target)
            self.data.ctrl[self.actuator_ids]=self.pending.popleft()
            mj.mj_step(self.model,self.data)
        mj.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError('Non-finite MuJoCo state')

    def set_preparation_pose(self,root_pose,joint_targets):
        """Match the previous bridge's default_pos gantry before policy start."""
        self.data.qpos[:7]=root_pose
        self.data.qpos[self.qpos_ids]=joint_targets
        self.data.qvel[:]=0.
        self.data.ctrl[self.actuator_ids]=joint_targets
        mj.mj_forward(self.model,self.data)

    def report(self):
        return {'policy':self.policy.path.name,
                'motion':self.motion.path.resolve().relative_to(ROOT).as_posix() if self.motion.path.resolve().is_relative_to(ROOT) else self.motion.path.name,'start_frame':self.start_frame,'last_frame':self.frame,
                'simulation_seconds':(self.frame-self.start_frame)*self.config['control_dt'],
                'action_delay_physics_steps':self.delay,'resets':self.reset_count,
                'state_finite':bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()),
                'root_error_mean_m':float(np.mean(self.position_errors)) if self.position_errors else 0.,
                'root_error_final_m':self.position_errors[-1] if self.position_errors else 0.,
                'first_root_tracking_failure_s':self.first_root_failure,
                'mujoco_warning_counts':[int(w.number) for w in self.data.warning]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/h2.json')
    parser.add_argument('--policy',type=Path)
    parser.add_argument('--motion',type=Path)
    parser.add_argument('--start-seconds',type=float,default=0.)
    parser.add_argument('--seconds',type=float,help='Stop after this simulated duration; default: whole motion')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--paused',action='store_true')
    parser.add_argument('--action-delay',type=int,default=0,choices=(0,1,2),help='Delay in 1 ms physics steps')
    parser.add_argument('--report',type=Path,default=ROOT/'reports/sim2sim_latest.json')
    args=parser.parse_args()
    config=json.loads(args.config.read_text())
    sim=Simulator(config,args.policy or ROOT/config['policy'],args.motion or ROOT/config['motion'],args.start_seconds,args.action_delay)
    if args.seconds is not None and args.seconds<=0:parser.error('--seconds must be positive')
    steps=sim.motion.length-1-sim.start_frame
    if args.seconds is not None:steps=min(steps,round(args.seconds/config['control_dt']))
    print(f'H2 | {sim.motion.path.name} | 50 Hz policy / 1000 Hz physics',flush=True)
    if args.headless:
        for _ in range(steps):sim.step()
    else:
        import mujoco.viewer
        controls={'paused':args.paused,'reset':False,'ghost':True,'collision':False,'follow':True}
        def key(keycode):
            if keycode==32:controls['paused']=not controls['paused']
            elif keycode==ord('R'):controls['reset']=True
            elif keycode==ord('G'):controls['ghost']=not controls['ghost']
            elif keycode==ord('C'):controls['collision']=not controls['collision']
            elif keycode==ord('F'):controls['follow']=not controls['follow']
        ghost_model=mj.MjModel.from_xml_path(str(sim.scene));ghost_data=mj.MjData(ghost_model)
        ghost_model.geom_contype[:]=0;ghost_model.geom_conaffinity[:]=0
        ghost_model.geom_rgba[:]=[.25,.9,.35,.25]
        ghost_model.geom_matid[:]=-1
        ghost_opt=mj.MjvOption();ghost_opt.geomgroup[:]=0;ghost_opt.geomgroup[2]=1
        print('Space: pause | R: reset | G: reference | C: collisions | F: follow camera',flush=True)
        with mujoco.viewer.launch_passive(sim.model,sim.data,key_callback=key,show_left_ui=False,show_right_ui=False) as viewer:
            viewer.cam.distance=3.8;viewer.cam.azimuth=135;viewer.cam.elevation=-15
            viewer.cam.lookat[:]=sim.data.xpos[sim.pelvis]
            while viewer.is_running():
                tick=time.perf_counter()
                if controls['reset']:
                    sim.reset_count+=1;sim.reset();controls['reset']=False
                if not controls['paused'] and sim.frame-sim.start_frame<steps:sim.step()
                with viewer.lock():
                    viewer.opt.geomgroup[2]=not controls['collision']
                    viewer.opt.geomgroup[3]=controls['collision']
                    if controls['follow']:
                        viewer.cam.lookat[:]=(sim.data.xpos[sim.pelvis,0],sim.data.xpos[sim.pelvis,1],.95)
                    viewer.user_scn.ngeom=0
                    if controls['ghost'] and not controls['collision']:
                        m=sim.motion;f=sim.frame
                        ghost_data.qpos[:3]=m.body_pos_w[f,m.root]
                        ghost_data.qpos[3:7]=m.body_quat_w[f,m.root]
                        ghost_data.qpos[sim.qpos_ids]=m.joint_pos[f]
                        mj.mj_forward(ghost_model,ghost_data)
                        mj.mjv_addGeoms(ghost_model,ghost_data,ghost_opt,mj.MjvPerturb(),mj.mjtCatBit.mjCAT_DYNAMIC,viewer.user_scn)
                viewer.sync()
                time.sleep(max(0.,config['control_dt']-(time.perf_counter()-tick)))
    report=sim.report()
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
