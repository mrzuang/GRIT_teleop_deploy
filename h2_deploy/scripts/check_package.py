"""Package self-check: static contracts, sim2sim, and the full real-robot flow on the stand-in.

  ./scripts/python.sh scripts/check_package.py            # contracts + sim2sim + closed loop
  ./scripts/python.sh scripts/check_package.py --bridge   # also DDS loopback through h2_udp_bridge
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from h2_deploy.motion import MotionClip  # noqa: E402
from h2_deploy.observations import rotation_matrix  # noqa: E402
from h2_deploy.policy import H2Policy  # noqa: E402

PY = [sys.executable]


def static_checks(config, deploy):
    policy = ROOT/config['policy']
    meta = json.loads(policy.with_suffix('.json').read_text())
    assert hashlib.sha256(policy.read_bytes()).hexdigest() == meta['onnx_sha256'], 'ONNX checksum'
    assert meta['joint_names'] == config['joint_names'], 'policy joint order'
    assert hashlib.sha256((ROOT/config['scene']).read_bytes()).hexdigest() == config['scene_sha256'], 'scene checksum'
    assert [config['joint_names'][i] for i in config['dds_from_policy_index']] == config['dds_joint_names']
    assert [config['dds_joint_names'][i] for i in config['policy_from_dds_index']] == config['joint_names']
    bridge = yaml.safe_load((ROOT/'native_bridge/config/h2_bridge.yaml').read_text())
    assert bridge['policy_joint_names'] == config['joint_names'], 'bridge policy_joint_names'
    assert bridge['real_joint_names'] == config['dds_joint_names'], 'bridge real_joint_names (DDS order)'
    assert bridge['udp']['state_port'] == deploy['udp']['state_port'] and bridge['udp']['cmd_port'] == deploy['udp']['cmd_port']
    assert bridge['freq']['physical_hz']/bridge['freq']['state_decimation'] == 1/config['control_dt'], 'bridge state rate'
    assert bridge['low_level']['command_timeout_s'] == deploy['bridge_command_timeout_s']
    for key in ('kp', 'kd', 'default_joint_pos', 'action_scale', 'encoder_offset'):
        assert len(config[key]) == 31, key
    H2Policy(policy, config)
    motions = {}
    for path in sorted((ROOT/'motions').glob('*.npz')):
        clip = MotionClip(path, config['joint_names'], config['body_names'])
        speed = np.linalg.norm(clip.body_lin_vel_w[:, clip.root, :2], axis=1)
        motions[path.name] = {'seconds': clip.length/clip.fps, 'peak_speed_m_s': round(float(speed.max()), 2),
                              'distance_m': round(float(np.linalg.norm(clip.body_pos_w[-1, clip.root, :2]-clip.body_pos_w[0, clip.root, :2])), 2)}
    return {'onnx_sha256': meta['onnx_sha256'], 'motions': motions}


def sim2sim(config):
    results = {}
    for path in sorted((ROOT/'motions').glob('*.npz')):
        report = ROOT/'reports'/f'sim2sim_{path.stem}.json'
        subprocess.run(PY+['-m', 'h2_deploy.sim2sim', '--motion', str(path), '--headless', '--report', str(report)],
                       cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
        r = json.loads(report.read_text())
        assert r['state_finite'] and r['first_root_tracking_failure_s'] is None, (path.name, r)
        results[path.name] = {k: round(r[k], 3) for k in ('root_error_mean_m', 'root_error_final_m')}
    return results


def closed_loop(config):
    """Controller + stand-in: zero -> s -> prepare -> hold -> A -> track 12 s -> hold -> x."""
    motion = ROOT/config['motion']
    seconds = MotionClip(motion, config['joint_names'], config['body_names']).length/50
    a_time = 6.
    stop_time = a_time+seconds+2.
    standin = subprocess.Popen(PY+['-m', 'h2_deploy.mujoco_robot', '--keys', f's@1,a@{a_time},x@{stop_time}',
                                   '--seconds', str(stop_time+1.5), '--yaw', '0.7',
                                   '--report', str(ROOT/'reports/closed_loop_standin.json')],
                               cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(.5)
    log = ROOT/'reports/closed_loop_controller_run.npz'
    out = subprocess.run(PY+['-m', 'h2_deploy.controller', '--log', str(log)], cwd=ROOT, capture_output=True, text=True, timeout=stop_time+30)
    standin.wait(timeout=30)
    text = out.stdout
    for phase in ('[zero]', '[prepare]', '[hold]', '[track]', 'reference finished', '[damping] Stop button'):
        assert phase in text, f'controller phase missing: {phase}\n{text}\n{out.stderr[-2000:]}'
    data = np.load(log)['log']
    up = np.array([rotation_matrix(q)[2, 2] for q in data[:, 64:68]])
    steps = np.diff(data[:, 0])
    tracked = data[:, 1].astype(int)
    assert up.min() > .9, f'robot tilted during tracking: min up_z {up.min():.3f}'
    return {'policy_steps': len(data), 'reference_frames_tracked': int(tracked.max())+1,
            'min_pelvis_up_z_while_tracking': round(float(up.min()), 3),
            'control_period_ms_mean_max': [round(1e3*steps.mean(), 2), round(1e3*steps.max(), 2)],
            'heading_alignment': next(l for l in text.splitlines() if 'heading rotated' in l).split('reference heading ')[1]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bridge', action='store_true', help='Also run the DDS loopback test (needs a built bridge)')
    args = parser.parse_args()
    config = json.loads((ROOT/'config/h2.json').read_text())
    deploy = json.loads((ROOT/'config/deploy.json').read_text())
    (ROOT/'reports').mkdir(exist_ok=True)
    report = {'static': static_checks(config, deploy)}
    print('static checks passed', flush=True)
    report['sim2sim'] = sim2sim(config)
    print('sim2sim passed', flush=True)
    report['closed_loop_standin'] = closed_loop(config)
    print('closed-loop stand-in passed', flush=True)
    if args.bridge:
        subprocess.run(PY+[str(ROOT/'scripts/test_bridge_loopback.py')], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
        report['bridge_loopback'] = json.loads((ROOT/'reports/bridge_loopback_test.json').read_text())['status']
    report['status'] = 'passed'
    (ROOT/'reports/check_package.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
