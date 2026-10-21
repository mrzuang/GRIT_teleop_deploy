"""Hardware-free end-to-end test of native_bridge/h2_udp_bridge joint mapping.

dds_loopback_robot publishes LowState with motor_state[i].q = 0.01*(i+1) in DDS
motor order. Through the real bridge, the policy-order state must equal the
DDS value of each policy joint's motor; a policy-order command must arrive at
the matching DDS motor slot (unused slots 31-34 stay zero).
Run: ./scripts/python.sh scripts/test_bridge_loopback.py [--net lo]
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from h2_deploy.robot_io import RobotClient  # noqa: E402

BRIDGE = ROOT/'native_bridge'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--net', default='lo')
    args = parser.parse_args()
    config = json.loads((ROOT/'config/h2.json').read_text())
    deploy = json.loads((ROOT/'config/deploy.json').read_text())
    for target in ('h2_udp_bridge', 'dds_loopback_robot'):
        subprocess.run(['cmake', '--build', str(BRIDGE/'build'), '--target', target, '-j4'], check=True, stdout=subprocess.DEVNULL)
    arch = {'x86_64': 'x86_64', 'aarch64': 'aarch64', 'arm64': 'aarch64'}[os.uname().machine]
    env = dict(os.environ, LD_LIBRARY_PATH=str(BRIDGE/f'third_party/unitree_sdk2/thirdparty/lib/{arch}'))
    env.pop('LD_PRELOAD', None)
    text = (BRIDGE/'config/h2_bridge.yaml').read_text().replace('release_motion_service: true', 'release_motion_service: false')
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as handle:
        handle.write(text)
    robot = subprocess.Popen([str(BRIDGE/'build/dds_loopback_robot'), args.net, '15'], env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(.5)
    bridge = subprocess.Popen([str(BRIDGE/'build/h2_udp_bridge'), '--net', args.net, '--config', handle.name], env=env,
                              stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    client = RobotClient(deploy['udp'])
    try:
        packet = client.read_next_state(None, 8.)
        if packet is None:
            raise RuntimeError('No state from the bridge; is DDS discovery working on --net?')
        q = np.asarray(packet.data['q'])
        dds_index = [config['dds_joint_names'].index(n) for n in config['joint_names']]
        expected_q = .01*(np.asarray(dds_index)+1)
        np.testing.assert_allclose(q, expected_q, atol=1e-6, err_msg='LowState -> policy mapping')
        np.testing.assert_allclose(np.asarray(packet.data['dq']), -expected_q, atol=1e-6)
        for _ in range(10):  # safe handoff commands
            client.send_command(q_des=np.zeros(31), kp=np.zeros(31), kd=np.zeros(31), enable=0)
            time.sleep(.02)
        q_des = .1*np.arange(1, 32)
        kp = np.arange(1, 32, dtype=float)
        deadline = time.time()+6
        line = None
        while time.time() < deadline and line is None:
            client.send_command(q_des=q_des, kp=kp, kd=.5*kp, enable=1)
            time.sleep(.02)
            if robot.poll() is not None:
                break
        out, _ = robot.communicate(timeout=20)
        line = next((l for l in out.splitlines() if l.startswith('LOWCMD ')), None)
        if line is None:
            raise RuntimeError('Loopback robot received no active LowCmd:\n'+out[-2000:])
        cmd = json.loads(line[7:])
        policy_index = [config['joint_names'].index(n) for n in config['dds_joint_names']]
        np.testing.assert_allclose(cmd['q'][:31], q_des[policy_index], atol=1e-6, err_msg='policy -> LowCmd q mapping')
        np.testing.assert_allclose(cmd['kp'][:31], kp[policy_index], atol=1e-6)
        np.testing.assert_allclose(cmd['kd'][:31], .5*kp[policy_index], atol=1e-6)
        assert not any(cmd['q'][31:]+cmd['kp'][31:]+cmd['kd'][31:]), 'unused motor slots must stay zero'
        report = {'status': 'passed', 'net': args.net, 'state_mapping_joints': 31, 'command_mapping_joints': 31,
                  'unused_slots_zero': True,
                  'example': {n: {'dds_slot': config['dds_joint_names'].index(n)} for n in ('waist_pitch_joint', 'head_pitch_joint', 'left_shoulder_pitch_joint')}}
        (ROOT/'reports').mkdir(exist_ok=True)
        (ROOT/'reports/bridge_loopback_test.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
    finally:
        client.close()
        for p in (bridge, robot):
            if p.poll() is None:
                p.send_signal(2)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
        os.unlink(handle.name)


if __name__ == '__main__':
    main()
