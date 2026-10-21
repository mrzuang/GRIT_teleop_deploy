"""MuJoCo stand-in for the H2 native bridge, for closed-loop tests of the real runtime.

Speaks the same UDP protocol and safety rules as native_bridge/h2_udp_bridge:
  * before handoff only enable=0, Kp=0 commands are accepted;
  * after handoff, a command stream older than command_timeout_s -> damping;
  * keys s / a / x (or --keys) pulse start / A / stop for 0.2 s.
Each command sets the scene's affine PD actuators (Kp, Kd, q_des);
physics runs at 1 kHz, state is sent at 50 Hz in real time.
While no enable=1 command has arrived the pelvis is held as on a gantry,
at a random-ish heading (--yaw) to exercise reference heading alignment.
"""
import argparse
import contextlib
import json
from pathlib import Path
import select
import sys
import termios
import time
import tty

import mujoco as mj
import numpy as np

from .robot_io import BridgeServer

ROOT = Path(__file__).resolve().parents[2]
PULSE_S = .2


class StandInBridge:
    def __init__(self, config, deploy, yaw, keys=()):
        self.config, self.deploy = config, deploy
        self.model = mj.MjModel.from_xml_path(str(ROOT/config['scene']))
        self.data = mj.MjData(self.model)
        joints = [self.model.joint(n).id for n in config['joint_names']]
        self.qpos_ids = self.model.jnt_qposadr[joints]
        self.qvel_ids = self.model.jnt_dofadr[joints]
        self.act = np.array([int(np.flatnonzero(self.model.actuator_trnid[:, 0] == j)[0]) for j in joints])
        self.pelvis = self.model.body('pelvis').id
        gyro, acc = self.model.sensor('imu_ang_vel'), self.model.sensor('imu_lin_acc')
        self.gyro_adr, self.acc_adr = self.model.sensor_adr[gyro.id], self.model.sensor_adr[acc.id]
        root = np.asarray(config['home_root_qpos'], np.float64)
        root[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
        self.data.qpos[:7] = root
        self.data.qpos[self.qpos_ids] = config['default_joint_pos']
        mj.mj_forward(self.model, self.data)
        # Gantry height: default-pose soles 2 mm above the floor (robot lowered onto its feet).
        soles = []
        for side in ('left', 'right'):
            g = self.model.geom(side+'_foot_collision').id
            m = self.model.geom_dataid[g]
            v = self.model.mesh_vert[self.model.mesh_vertadr[m]:self.model.mesh_vertadr[m]+self.model.mesh_vertnum[m]]
            soles.append((v@self.data.geom_xmat[g].reshape(3, 3).T+self.data.geom_xpos[g])[:, 2].min())
        root[2] += .002-min(soles)
        self.gantry_root = root
        self.data.qpos[:7] = root
        mj.mj_forward(self.model, self.data)
        self.set_pd(np.asarray(config['default_joint_pos']), np.zeros(31), np.zeros(31))
        self.server = BridgeServer({'state_host': '127.0.0.1', 'state_port': deploy['udp']['state_port'],
                                    'cmd_bind_host': '127.0.0.1', 'cmd_port': deploy['udp']['cmd_port']})
        self.handoff = False
        self.gantry = True
        self.last_command_time = None
        self.last_seq = None
        self.pulses = {}
        self.keys = sorted(keys, key=lambda k: k[1])
        self.timeout = float(deploy['bridge_command_timeout_s'])
        self.damping_kd = float(deploy['damping_kd'])

    def set_pd(self, q_des, kp, kd):
        self.model.actuator_gainprm[self.act, 0] = kp
        self.model.actuator_biasprm[self.act, 1] = -kp
        self.model.actuator_biasprm[self.act, 2] = -kd
        self.data.ctrl[self.act] = q_des

    def pulse(self, key, now):
        name = {'s': 'start', 'a': 'A', 'x': 'stop'}.get(key.lower())
        if name:
            self.pulses[name] = now+PULSE_S
            print(f'[stand-in] button {name}', flush=True)

    def apply_commands(self, now):
        packet = self.server.read_latest_command()
        if packet is not None and packet.seq != self.last_seq:
            self.last_seq = packet.seq
            c = packet.data
            q, kp, kd = (np.asarray(c[k], np.float64) for k in ('q_des', 'kp', 'kd'))
            valid = all(x.shape == (31,) and np.isfinite(x).all() for x in (q, kp, kd)) and (kp >= 0).all() and (kd >= 0).all()
            if valid and (self.handoff or (c['enable'] == 0 and not kp.any())):
                if not self.handoff:
                    print('[stand-in] low-level handoff', flush=True)
                self.handoff = True
                self.last_command_time = now
                self.set_pd(q, kp, kd)
                if c['enable'] == 1 and self.gantry:
                    print('[stand-in] enable=1: releasing the gantry', flush=True)
                    self.gantry = False
        if self.handoff and now-self.last_command_time > self.timeout:
            self.set_pd(np.zeros(31), np.zeros(31), np.full(31, self.damping_kd))

    def step(self):
        for _ in range(self.config['decimation']):
            if self.gantry:
                self.data.qpos[:7] = self.gantry_root
                self.data.qvel[:6] = 0
            mj.mj_step(self.model, self.data)

    def send_state(self, now):
        d = self.data
        buttons = {k: until > now for k, until in self.pulses.items()}
        self.server.send_state(q=d.qpos[self.qpos_ids], dq=d.qvel[self.qvel_ids], quat_wxyz=d.xquat[self.pelvis],
                               gyro=d.sensordata[self.gyro_adr:self.gyro_adr+3],
                               linacc=d.sensordata[self.acc_adr:self.acc_adr+3], buttons=buttons)

    def run(self, duration, viewer=False):
        dt = self.config['control_dt']
        start = time.monotonic()
        interactive = sys.stdin.isatty() and not self.keys
        old = termios.tcgetattr(sys.stdin) if interactive else None
        if interactive:
            tty.setcbreak(sys.stdin.fileno())
            print('[stand-in] keys: s=start  a=A  x=stop  (Ctrl+C quits)', flush=True)
        if viewer:
            import mujoco.viewer
            window = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False,
                key_callback=lambda k: self.pulse(chr(k), time.monotonic()-start) if 0 < k < 256 else None)
        else:
            window = contextlib.nullcontext()
        tick = 0
        try:
            with window as handle:
                if handle is not None:
                    handle.cam.distance, handle.cam.azimuth, handle.cam.elevation = 3.8, 135, -15
                while duration is None or tick*dt < duration:
                    now = time.monotonic()-start
                    while self.keys and self.keys[0][1] <= now:
                        self.pulse(self.keys.pop(0)[0], now)
                    if interactive and select.select([sys.stdin], [], [], 0)[0]:
                        for ch in sys.stdin.read(1):
                            self.pulse(ch, now)
                    # The render thread reads the model: change gains and step under its lock.
                    with handle.lock() if handle is not None else contextlib.nullcontext():
                        self.apply_commands(now)
                        self.step()
                        if handle is not None:
                            handle.cam.lookat[:] = (self.data.xpos[self.pelvis, 0], self.data.xpos[self.pelvis, 1], .95)
                    self.send_state(now)
                    if handle is not None:
                        if not handle.is_running():
                            break
                        handle.sync()
                    tick += 1
                    time.sleep(max(0., start+tick*dt-time.monotonic()))
        except KeyboardInterrupt:
            pass
        finally:
            if old is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            self.server.close()
        up = self.data.xmat[self.pelvis].reshape(3, 3)[2, 2]
        return {'seconds': tick*dt, 'handoff': self.handoff, 'gantry_released': not self.gantry,
                'pelvis_xy_m': self.data.xpos[self.pelvis, :2].round(3).tolist(),
                'pelvis_height_m': round(float(self.data.xpos[self.pelvis, 2]), 3), 'pelvis_up_z': round(float(up), 4)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=ROOT/'config/h2.json')
    parser.add_argument('--deploy', type=Path, default=ROOT/'config/deploy.json')
    parser.add_argument('--yaw', type=float, default=.7, help='Initial pelvis heading in rad (IMU frame is arbitrary on hardware)')
    parser.add_argument('--keys', default='', help='Scripted buttons, e.g. "s@1,a@6,x@24" (seconds after start)')
    parser.add_argument('--seconds', type=float)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    keys = [(k.split('@')[0], float(k.split('@')[1])) for k in args.keys.split(',') if k]
    bridge = StandInBridge(json.loads(args.config.read_text()), json.loads(args.deploy.read_text()), args.yaw, keys)
    report = bridge.run(args.seconds, args.viewer)
    print('[stand-in] '+json.dumps(report), flush=True)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
