"""Record a sim2sim rollout to MP4 (white: robot, green: reference).

  ./scripts/python.sh scripts/record_video.py --motion motions/walk_stand_start_stop_12s.npz --output reports/walk.mp4
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from h2_deploy.sim2sim import Simulator  # noqa: E402


def ffmpeg():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        return 'ffmpeg'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--motion', type=Path)
    parser.add_argument('--seconds', type=float, help='Default: whole motion')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = json.loads((ROOT/'config/h2.json').read_text())
    sim = Simulator(cfg, ROOT/cfg['policy'], args.motion or ROOT/cfg['motion'])
    fps = round(1/cfg['control_dt'])
    frames = sim.motion.length-1 if args.seconds is None else min(sim.motion.length-1, round(args.seconds*fps))
    renderer = mj.Renderer(sim.model, height=720, width=1280)
    camera = mj.MjvCamera(); camera.distance, camera.azimuth, camera.elevation = 3.8, 135, -15
    option = mj.MjvOption(); option.geomgroup[2] = 1; option.geomgroup[3] = 0
    ghost = mj.MjModel.from_xml_path(str(sim.scene)); ghost_data = mj.MjData(ghost)
    ghost.geom_rgba[:] = [.25, .9, .35, .25]; ghost.geom_matid[:] = -1
    ghost_option = mj.MjvOption(); ghost_option.geomgroup[:] = 0; ghost_option.geomgroup[2] = 1
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
    except OSError:
        font = ImageFont.load_default()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen([ffmpeg(), '-y', '-v', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '1280x720',
                                '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
                                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(args.output)], stdin=subprocess.PIPE)
    m = sim.motion
    for index in range(frames):
        f = sim.frame
        camera.lookat[:] = (sim.data.xpos[sim.pelvis, 0], sim.data.xpos[sim.pelvis, 1], .95)
        ghost_data.qpos[:3] = m.body_pos_w[f, m.root]; ghost_data.qpos[3:7] = m.body_quat_w[f, m.root]
        ghost_data.qpos[sim.qpos_ids] = m.joint_pos[f]
        mj.mj_forward(ghost, ghost_data)
        renderer.update_scene(sim.data, camera=camera, scene_option=option)
        mj.mjv_addGeoms(ghost, ghost_data, ghost_option, mj.MjvPerturb(), mj.mjtCatBit.mjCAT_DYNAMIC, renderer.scene)
        picture = Image.fromarray(renderer.render()); draw = ImageDraw.Draw(picture)
        error = float(np.linalg.norm(sim.data.xpos[sim.torso]-m.body_pos_w[f, m.anchor]))
        speed = float(np.linalg.norm(m.body_lin_vel_w[f, m.root, :2]))
        draw.rectangle((10, 10, 1270, 70), fill=(20, 24, 32))
        draw.text((22, 16), f'H2 sim2sim | {m.path.name} | t={index/fps:5.2f}s', font=font, fill='white')
        draw.text((22, 42), f'white: robot   green: reference   reference speed {speed:.2f} m/s   root error {error:.2f} m',
                  font=font, fill=(195, 225, 195))
        encoder.stdin.write(np.asarray(picture).tobytes())
        sim.step()
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise RuntimeError('ffmpeg failed')
    renderer.close()
    print(args.output)


if __name__ == '__main__':
    main()
