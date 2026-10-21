"""H2 sim2real policy runtime: native UDP bridge (real robot) or MuJoCo stand-in.

Phases, driven by bridge buttons (bridge terminal keys s / a / x, or the
wireless remote A / stop):
  zero     enable=0, Kp=Kd=0. The first such command triggers the bridge's
           low-level handoff. Keep the robot on its gantry.        -> s
  prepare  interpolate from the measured pose to the default pose with the
           configured PD gains over `prepare_seconds`.
  hold     hold the default pose; lower the robot onto its feet.   -> A
  track    heading-align the reference to the IMU, reset the policy history
           and track the clip at 50 Hz; after the last frame the final
           (standing) reference frame is held.
  stop/x, tilt, joint-limit or invalid state -> damping (Kp=0, Kd=damping_kd), exit.
"""
import argparse
import datetime
import json
from pathlib import Path
import signal
import time

import numpy as np

from .motion import MotionClip
from .observations import rotation_matrix
from .policy import H2Policy
from .reference import align_clip_heading
from .robot_io import BUTTONS, RobotClient

ROOT = Path(__file__).resolve().parents[2]


class SafetyStop(RuntimeError):
    pass


class Controller:
    def __init__(self, config, deploy, policy_path, motion_path, log_path=None):
        self.config, self.deploy = config, deploy
        self.policy = H2Policy(policy_path, config)
        self.motion_path = Path(motion_path)
        MotionClip(self.motion_path, config['joint_names'], config['body_names'])  # validate before connecting
        self.kp = np.asarray(config['kp'], np.float32)
        self.kd = np.asarray(config['kd'], np.float32)
        self.default = np.asarray(config['default_joint_pos'], np.float32)
        soft = np.asarray(config['reset_soft_joint_limits'])
        center, half = soft.mean(1), np.diff(soft, axis=1).ravel()/2/.9
        margin = float(deploy['safety']['joint_limit_margin_rad'])
        self.joint_low, self.joint_high = center-half-margin, center+half+margin
        self.min_up = float(np.cos(np.deg2rad(deploy['safety']['max_tilt_deg'])))
        self.client = RobotClient(deploy['udp'])
        self.seq = None
        self.buttons = {k: False for k in BUTTONS}
        self.rise = dict(self.buttons)
        self.log_path = log_path
        self.log = []

    # ---- state / command I/O -------------------------------------------------
    def read_state(self):
        timeout = float(self.deploy['safety']['state_timeout_s'])
        packet = self.client.read_next_state(self.seq, timeout)
        if packet is None:
            raise SafetyStop(f'No bridge state for {timeout:.2f}s')
        self.seq = packet.seq
        d = packet.data
        self.q = np.asarray(d['q'], np.float64)
        self.dq = np.asarray(d['dq'], np.float64)
        self.quat = np.asarray(d['quat_wxyz'], np.float64)
        self.gyro = np.asarray(d['gyro'], np.float64)
        self.state_time_ns = d.get('state_receive_time_ns')
        if self.q.shape != (31,) or self.dq.shape != (31,) or self.quat.shape != (4,) or self.gyro.shape != (3,):
            raise SafetyStop('Bridge state has unexpected shapes; check policy_joint_names in the bridge config')
        if not all(np.isfinite(x).all() for x in (self.q, self.dq, self.quat, self.gyro)):
            raise SafetyStop('Non-finite bridge state')
        now = {k: bool(d['buttons'].get(k, False)) for k in BUTTONS}
        self.rise = {k: now[k] and not self.buttons[k] for k in BUTTONS}
        self.buttons = now
        if self.rise['stop']:
            raise SafetyStop('Stop button')

    def send(self, q_des, kp, kd, enable):
        self.client.send_command(q_des=q_des, kp=kp, kd=kd, enable=enable, state_receive_time_ns=self.state_time_ns)

    def check_pose(self, check_tilt):
        if check_tilt and rotation_matrix(self.quat)[2, 2] < self.min_up:
            raise SafetyStop(f'Pelvis tilt exceeds {self.deploy["safety"]["max_tilt_deg"]} deg')
        outside = np.flatnonzero((self.q < self.joint_low) | (self.q > self.joint_high))
        if len(outside):
            names = [self.config['joint_names'][i] for i in outside]
            raise SafetyStop(f'Joint outside hard limit + margin: {names}')

    # ---- phases ---------------------------------------------------------------
    def zero(self):
        print('[zero] Kp=Kd=0, robot limp: keep it on the gantry. Press s in the bridge terminal to prepare.', flush=True)
        while True:
            self.read_state()
            self.send(np.zeros(31), np.zeros(31), np.zeros(31), 0)
            if self.rise['start']:
                return

    def prepare(self):
        steps = max(1, round(float(self.deploy['prepare_seconds'])/self.config['control_dt']))
        start = self.q.copy()
        print(f'[prepare] moving to the default pose over {steps*self.config["control_dt"]:.1f}s', flush=True)
        for i in range(steps):
            self.read_state()
            self.check_pose(check_tilt=False)
            alpha = (i+1)/steps
            self.send((1-alpha)*start+alpha*self.default, self.kp, self.kd, 0)

    def hold(self):
        print('[hold] holding the default pose. Lower the robot onto its feet, then press A to start tracking.', flush=True)
        while True:
            self.read_state()
            self.check_pose(check_tilt=False)
            self.send(self.default, self.kp, self.kd, 0)
            if self.rise['A']:
                return

    def track(self):
        clip = MotionClip(self.motion_path, self.config['joint_names'], self.config['body_names'])
        delta = align_clip_heading(clip, self.quat)
        self.policy.reset()
        offset = self.policy.encoder_offset
        print(f'[track] {clip.path.name}: {clip.length/clip.fps:.1f}s, reference heading rotated by '
              f'{np.rad2deg(delta):+.1f} deg to match the IMU. x / stop = damping.', flush=True)
        frame, announced = 0, False
        while True:
            target = self.policy.step(clip.context(frame), clip.body_quat_w[frame, clip.root],
                                      gyro=self.gyro, root_quat=self.quat,
                                      joint_pos=self.q+offset, joint_vel=self.dq)
            self.send(target, self.kp, self.kd, 1)
            self.log.append(np.concatenate(([time.time(), frame], self.q, self.dq, self.quat, self.gyro, target)))
            if frame < clip.length-1:
                frame += 1
            elif not announced:
                print('[track] reference finished; holding its final standing frame. Press x to stop.', flush=True)
                announced = True
            self.read_state()
            self.check_pose(check_tilt=True)

    def damp(self, reason):
        print(f'[damping] {reason}', flush=True)
        kd = np.full(31, float(self.deploy['damping_kd']), np.float32)
        for _ in range(max(1, round(float(self.deploy['damping_seconds'])/self.config['control_dt']))):
            self.send(np.zeros(31), np.zeros(31), kd, 0)
            time.sleep(self.config['control_dt'])

    def run(self):
        try:
            print('[connect] waiting for bridge state ...', flush=True)
            self.read_state()
            print('[connect] bridge state received', flush=True)
            self.zero(); self.prepare(); self.hold(); self.track()
        except SafetyStop as stop:
            self.damp(str(stop))
        except KeyboardInterrupt:
            self.damp('Ctrl+C')
        finally:
            self.client.close()
            if self.log_path and self.log:
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(self.log_path, log=np.asarray(self.log),
                                    columns=np.array(['time', 'frame']+[f'q{i}' for i in range(31)]+[f'dq{i}' for i in range(31)]
                                                     +['qw', 'qx', 'qy', 'qz', 'gx', 'gy', 'gz']+[f'target{i}' for i in range(31)]))
                print(f'[log] {self.log_path}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=ROOT/'config/h2.json')
    parser.add_argument('--deploy', type=Path, default=ROOT/'config/deploy.json')
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--motion', type=Path)
    parser.add_argument('--log', type=Path, default=ROOT/'reports'/f'robot_run_{datetime.datetime.now():%Y%m%d_%H%M%S}.npz')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    deploy = json.loads(args.deploy.read_text())
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    Controller(config, deploy, args.policy or ROOT/config['policy'], args.motion or ROOT/config['motion'], args.log).run()


if __name__ == '__main__':
    main()
