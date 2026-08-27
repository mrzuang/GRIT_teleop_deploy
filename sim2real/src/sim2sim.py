import argparse
import json
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import yaml
from sshkeyboard import listen_keyboard, stop_listening

SRC_ROOT = Path(__file__).resolve().parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from common.joint_mapper import JointMapper
from common.udp_latest import LatestPacket
from common.udp_transport import UDPRobotLow
from common.utils import DictToClass, Timer
from paths import SUPPORTED_ROBOTS, bridge_config_path

np.set_printoptions(formatter={"float": lambda x: "{0:0.2f}".format(x)})

Keyboard2Button = {
    "a": "A",
    "s": "start",
    "x": "stop",
    "u": "up",
    "d": "down",
}
BUTTON_KEYS = ("start", "stop", "A", "up", "down")
STICK_KEYS = ("lx", "ly", "rx", "ry")
_MISSING = object()


def _cfg_value(data, name: str, path: str, default=_MISSING):
    if isinstance(data, dict):
        if name in data:
            return data[name]
    elif hasattr(data, name):
        return getattr(data, name)
    if default is not _MISSING:
        return default
    raise ValueError(f"{path} is required")


def _as_vector(data, *, name: str, size: int | None = None, dtype=np.float64) -> np.ndarray:
    arr = np.asarray(data, dtype=dtype)
    if arr.ndim != 1:
        raise ValueError(f"{name} must be a 1D vector")
    if size is not None and arr.size != size:
        raise ValueError(f"{name} has size {arr.size}, expected {size}")
    return arr


class Sim2Sim:
    def __init__(self, args, config):
        self.args = args
        self.config = config
        self.robot = str(args.robot).lower()
        self.log_prefix = f"[{self.robot.upper()}Sim2Sim]"

        freq_cfg = _cfg_value(config, "freq", "freq")
        self.low_level_freq = int(_cfg_value(freq_cfg, "physical_hz", "freq.physical_hz"))
        if self.low_level_freq <= 0:
            raise ValueError("freq.physical_hz must be positive")
        self.state_decimation = int(_cfg_value(freq_cfg, "state_decimation", "freq.state_decimation"))
        if self.state_decimation <= 0:
            raise ValueError("freq.state_decimation must be positive")
        self.state_freq = self.low_level_freq / self.state_decimation
        self.state_dt = 1.0 / self.state_freq
        self.low_level_dt = 1.0 / self.low_level_freq
        print(
            f"{self.log_prefix} freq: physical_hz={self.low_level_freq}, "
            f"state_decimation={self.state_decimation}, state_hz={self.state_freq:.3f}"
        )

        xml_candidate = Path(args.xml_path or _cfg_value(config, "xml_path", "xml_path"))
        if xml_candidate.is_absolute():
            model_path = str(xml_candidate)
        else:
            model_path = str((Path(config._config_dir) / xml_candidate).resolve())
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = self.low_level_dt
        mujoco_options = _cfg_value(config, "mujoco", "mujoco", {})
        enum_options = {
            "integrator": (mujoco.mjtIntegrator, "mjINT_"),
            "solver": (mujoco.mjtSolver, "mjSOL_"),
            "cone": (mujoco.mjtCone, "mjCONE_"),
            "jacobian": (mujoco.mjtJacobian, "mjJAC_"),
        }
        for option_name, option_value in mujoco_options.items():
            if option_name in enum_options:
                enum_type, prefix = enum_options[option_name]
                option_value = getattr(enum_type, prefix + str(option_value).upper())
            if not hasattr(self.model.opt, option_name):
                raise ValueError(f"Unknown MuJoCo option: {option_name}")
            setattr(self.model.opt, option_name, option_value)
        self.data = mujoco.MjData(self.model)

        self.policy_joint_names = list(_cfg_value(config, "policy_joint_names", "policy_joint_names"))
        self.mujoco_joint_names = list(
            _cfg_value(config, "mujoco_joint_names", "mujoco_joint_names", self.policy_joint_names)
        )
        self.n_policy_joints = len(self.policy_joint_names)
        self.n_mujoco_joints = len(self.mujoco_joint_names)
        if self.model.nu != self.n_mujoco_joints:
            raise ValueError(f"model.nu={self.model.nu} != configured mujoco joints={self.n_mujoco_joints}")
        if len(self.model.actuator_ctrlrange) != self.n_mujoco_joints:
            raise ValueError("actuator ctrl range size does not match configured mujoco joints")

        self.policy_to_mujoco = JointMapper(self.policy_joint_names, self.mujoco_joint_names)
        mapping_info = self.policy_to_mujoco.get_mapping_info()
        print(
            f"{self.log_prefix} policy->mujoco mapping: "
            f"{mapping_info['mapped_joints']}/{mapping_info['from_space_size']} joints mapped"
        )
        if mapping_info["unmapped_from_joints"]:
            raise ValueError(f"Unmapped policy joints: {mapping_info['unmapped_from_joints']}")
        if mapping_info["unmapped_to_joints"]:
            raise ValueError(f"Unmapped MuJoCo joints: {mapping_info['unmapped_to_joints']}")

        joint_armatures = _cfg_value(config, "joint_armatures", "joint_armatures", None)
        if joint_armatures is not None:
            armatures_policy = _as_vector(
                joint_armatures,
                name="joint_armatures",
                size=self.n_policy_joints,
            )
            armatures_mujoco = self._policy_to_mujoco(armatures_policy)
            if self.model.nv != 6 + self.n_mujoco_joints:
                raise ValueError(
                    f"model.nv={self.model.nv}; expected free base + "
                    f"{self.n_mujoco_joints} hinge joints"
                )
            self.model.dof_armature[6:] = armatures_mujoco
            print(
                f"{self.log_prefix} armature override: "
                f"min={armatures_mujoco.min():.6f}, max={armatures_mujoco.max():.6f}"
            )

        self.ctrl_lower = self.model.actuator_ctrlrange[:, 0]
        self.ctrl_upper = self.model.actuator_ctrlrange[:, 1]

        self.home_q_policy = _as_vector(
            _cfg_value(config, "home_q", "home_q"),
            name="home_q",
            size=self.n_policy_joints,
        )
        self.root_qpos_home = _as_vector(
            _cfg_value(config, "root_qpos_home", "root_qpos_home"),
            name="root_qpos_home",
            size=7,
        )
        self.root_qpos_control = _as_vector(
            _cfg_value(config, "root_qpos_control", "root_qpos_control"),
            name="root_qpos_control",
            size=7,
        )
        self.viewer_fps = int(_cfg_value(config, "viewer_fps", "viewer_fps", 10))
        self.max_external_force = float(_cfg_value(config, "max_external_force", "max_external_force", 30.0))

        self.data.qpos[:7] = self.root_qpos_home
        self.data.qpos[7:] = self._policy_to_mujoco(self.home_q_policy)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        self._ptargets_policy = self.home_q_policy.copy()
        self._kp_policy = np.zeros(self.n_policy_joints, dtype=np.float64)
        self._kd_policy = np.zeros(self.n_policy_joints, dtype=np.float64)
        self._have_command = False
        self._have_tracking_target = False
        self._reference_q_policy = None
        self._reference_root_pos = None
        self._reference_root_quat = None
        self._controller_stopped = False
        self._buttons = {k: False for k in BUTTON_KEYS}
        self.auto_start = bool(getattr(args, "auto_start", False))
        if self.auto_start:
            self._buttons["start"] = True
            # A rises after the deploy finishes its init-pose move so its
            # rising-edge detection triggers (asserting A from t=0 never rises).

        self._cmd_lock = threading.Lock()
        self._button_lock = threading.Lock()
        self._sim_lock = threading.Lock()
        self._policy_delay_lock = threading.Lock()
        self._policy_delay_count = 0
        self._policy_delay_sum_ms = 0.0
        self._policy_delay_min_ms = float("inf")
        self._policy_delay_max_ms = 0.0

        self.transport = UDPRobotLow(config.udp, on_command_packet=self.cmd_sub_handler)

        self.keyboard_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={"on_press": self.on_press, "on_release": self.on_release},
            daemon=False,
        )
        self.is_alive = True
        self.policy_queried = False

        self.render_gui = bool(config.render_gui) and not bool(getattr(args, "headless", False))
        self.viewer = None
        self._viewer_tick = 0
        self._physics_tick = 0
        self.viewer_decim = max(1, self.low_level_freq // max(1, self.viewer_fps))
        self.imu_lin_acc_adr, self.imu_lin_acc_dim = self._resolve_sensor_slice("imu_lin_acc")

        self.max_control_seconds = float(getattr(args, "max_control_seconds", 0.0))
        if self.max_control_seconds < 0.0:
            raise ValueError("--max-control-seconds must be >= 0")
        metrics_out = getattr(args, "metrics_out", None)
        self.metrics_out = None if metrics_out is None else Path(metrics_out)
        self._control_sample_count = 0
        self._tracking_sq_error_sum = 0.0
        self._tracking_error_max = 0.0
        self._root_z_min = float("inf")
        self._root_z_max = float("-inf")
        self._waist_names = (
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
        )
        self._waist_indices = np.asarray(
            [self.policy_joint_names.index(name) for name in self._waist_names],
            dtype=np.int64,
        )
        self._waist_mujoco_indices = np.asarray(
            [self.mujoco_joint_names.index(name) for name in self._waist_names],
            dtype=np.int64,
        )
        self._waist_sum = np.zeros(3, dtype=np.float64)
        self._waist_target_sum = np.zeros(3, dtype=np.float64)
        self._waist_min = np.full(3, np.inf, dtype=np.float64)
        self._waist_max = np.full(3, -np.inf, dtype=np.float64)
        self._waist_target_min = np.full(3, np.inf, dtype=np.float64)
        self._waist_target_max = np.full(3, -np.inf, dtype=np.float64)
        self._waist_target_sq_error_sum = np.zeros(3, dtype=np.float64)
        self._waist_target_error_sum = np.zeros(3, dtype=np.float64)
        self._waist_ctrl_sum = np.zeros(3, dtype=np.float64)
        self._waist_ctrl_abs_sum = np.zeros(3, dtype=np.float64)
        self._waist_ctrl_saturated_count = np.zeros(3, dtype=np.int64)
        self._reference_sample_count = 0
        self._reference_joint_sq_error_sum = 0.0
        self._reference_joint_sq_error_by_joint = np.zeros(
            self.n_mujoco_joints, dtype=np.float64
        )
        self._reference_joint_error_max = 0.0
        self._lower_joint_names = (
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
        )
        self._lower_mujoco_indices = np.asarray(
            [self.mujoco_joint_names.index(name) for name in self._lower_joint_names],
            dtype=np.int64,
        )
        self._reference_lower_sq_error_sum = 0.0
        self._reference_waist_sq_error_sum = np.zeros(3, dtype=np.float64)
        self._ghost_data = mujoco.MjData(self.model)
        self._ghost_vopt = mujoco.MjvOption()
        self._ghost_vopt.geomgroup[:] = 0
        self._ghost_vopt.geomgroup[2] = 1
        self._ghost_pert = mujoco.MjvPerturb()
        self._root_tilt_sum_deg = 0.0
        self._root_tilt_max_deg = 0.0
        self._summary_written = False

        # ---- keyboard perturbation (F key applies random impulse to pelvis) ----
        self.perturb_force = float(_cfg_value(config, "perturb_force", "perturb_force", 300.0))
        self.perturb_duration = float(_cfg_value(config, "perturb_duration", "perturb_duration", 0.2))
        self.perturb_body = str(_cfg_value(config, "perturb_body", "perturb_body", "pelvis"))
        self._perturb_dir = None
        self._perturb_until = 0.0

        signal.signal(signal.SIGINT, self.close)

    def _viewer_key_callback(self, keycode: int):
        """MuJoCo viewer key callback (GLFW key codes).
        F (70): apply a random-direction force impulse to the perturb body.
        R (82): apply a random torque impulse.
        """
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.perturb_body)
        if body_id == -1:
            print(f"{self.log_prefix} perturb body '{self.perturb_body}' not found")
            return
        with self._sim_lock:
            if keycode == 70:  # F
                direction = np.random.randn(3)
                direction[2] = abs(direction[2]) * 0.3  # mostly horizontal
                direction /= np.linalg.norm(direction) + 1e-8
                self.data.xfrc_applied[body_id, :3] = direction * self.perturb_force
                self._perturb_dir = direction
                self._perturb_until = time.time() + self.perturb_duration
                print(
                    f"{self.log_prefix} PERTURB: force={direction * self.perturb_force} "
                    f"on '{self.perturb_body}' for {self.perturb_duration:.2f}s"
                )
            elif keycode == 82:  # R
                torque = np.random.randn(3) * self.perturb_force * 0.5
                self.data.xfrc_applied[body_id, 3:] = torque
                self._perturb_until = time.time() + self.perturb_duration
                print(f"{self.log_prefix} PERTURB: torque={torque} on '{self.perturb_body}'")

    def _apply_perturbation(self):
        """Apply/clear keyboard-triggered perturbation during simulation."""
        if time.time() >= self._perturb_until:
            if self._perturb_dir is not None:
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.perturb_body)
                if body_id != -1:
                    self.data.xfrc_applied[body_id] = 0.0
                self._perturb_dir = None

    def _policy_to_mujoco(self, values: np.ndarray, default_values: np.ndarray | None = None) -> np.ndarray:
        return self.policy_to_mujoco.map_action_from_to(values, default_values=default_values)

    def _mujoco_to_policy(self, values: np.ndarray) -> np.ndarray:
        return self.policy_to_mujoco.map_state_to_from(values)

    def _set_button(self, name: str, value: bool) -> None:
        with self._button_lock:
            self._buttons[name] = value

    def _buttons_snapshot(self):
        with self._button_lock:
            return dict(self._buttons)

    def on_press(self, key):
        print(f"Key pressed: {key}")
        btn = Keyboard2Button.get(key, None)
        if btn is None:
            return
        self._set_button(btn, True)

    def on_release(self, key):
        btn = Keyboard2Button.get(key, None)
        if btn is None:
            return
        time.sleep(0.1)
        self._set_button(btn, False)

    def cmd_sub_handler(self, packet: LatestPacket):
        payload = packet.data
        q_des = np.asarray(
            payload.get("q_des", np.zeros(self.n_policy_joints, dtype=np.float32)),
            dtype=np.float64,
        )
        kp = np.asarray(payload.get("kp", np.zeros(self.n_policy_joints, dtype=np.float32)), dtype=np.float64)
        kd = np.asarray(payload.get("kd", np.zeros(self.n_policy_joints, dtype=np.float32)), dtype=np.float64)
        enable = int(payload.get("enable", 0))
        reference = payload.get("extra_command", {}).get("reference", None)
        if q_des.size != self.n_policy_joints or kp.size != self.n_policy_joints or kd.size != self.n_policy_joints:
            print(f"{self.log_prefix} Ignore UDP command with unexpected DOF size")
            return
        with self._cmd_lock:
            self._ptargets_policy[:] = q_des
            self._kp_policy[:] = kp
            self._kd_policy[:] = kd
            if reference is not None:
                ref_q = np.asarray(reference.get("joint_pos", []), dtype=np.float64)
                ref_pos = np.asarray(reference.get("root_pos", []), dtype=np.float64)
                ref_quat = np.asarray(
                    reference.get("root_quat_wxyz", []), dtype=np.float64
                )
                if ref_q.shape == (self.n_policy_joints,) and ref_pos.shape == (3,) and ref_quat.shape == (4,):
                    self._reference_q_policy = ref_q.copy()
                    self._reference_root_pos = ref_pos.copy()
                    self._reference_root_quat = ref_quat.copy()
        self._record_policy_delay(payload)
        was_policy_queried = self.policy_queried
        self.policy_queried |= bool(enable)
        if self.auto_start and was_policy_queried and not bool(enable):
            self._controller_stopped = True
        self._have_command = True

    def _record_policy_delay(self, payload):
        state_receive_time_ns = payload.get("state_receive_time_ns", None)
        if state_receive_time_ns is None:
            return
        try:
            delay_ms = (time.perf_counter_ns() - int(state_receive_time_ns)) * 1e-6
        except (TypeError, ValueError):
            return
        if delay_ms < 0.0:
            return
        with self._policy_delay_lock:
            self._policy_delay_count += 1
            self._policy_delay_sum_ms += delay_ms
            self._policy_delay_min_ms = min(self._policy_delay_min_ms, delay_ms)
            self._policy_delay_max_ms = max(self._policy_delay_max_ms, delay_ms)

    def _consume_policy_delay_stats(self):
        with self._policy_delay_lock:
            count = self._policy_delay_count
            if count == 0:
                return None
            mean_ms = self._policy_delay_sum_ms / count
            min_ms = self._policy_delay_min_ms
            max_ms = self._policy_delay_max_ms
            self._policy_delay_count = 0
            self._policy_delay_sum_ms = 0.0
            self._policy_delay_min_ms = float("inf")
            self._policy_delay_max_ms = 0.0
        return mean_ms, min_ms, max_ms, count

    def _resolve_sensor_slice(self, sensor_name: str):
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sid < 0:
            return None, 0
        return int(self.model.sensor_adr[sid]), int(self.model.sensor_dim[sid])

    def _linacc(self) -> np.ndarray:
        if self.imu_lin_acc_adr is None or self.imu_lin_acc_dim < 3:
            return np.zeros(3, dtype=np.float32)
        return self.data.sensordata[self.imu_lin_acc_adr : self.imu_lin_acc_adr + 3].copy().astype(np.float32)

    def _publish_state(self):
        with self._sim_lock:
            q = self._mujoco_to_policy(self.data.qpos[7:]).astype(np.float32)
            dq = self._mujoco_to_policy(self.data.qvel[6:]).astype(np.float32)
            quat = self.data.qpos[3:7].copy().astype(np.float32)
            # Publish BODY-frame (pelvis) angular velocity like
            # the real bridge IMU; data.qvel[3:6] is world frame.
            gyro = self.data.sensor("imu_ang_vel").data.copy().astype(np.float32)
            linacc = self._linacc()
        self.transport.send_state(
            q=q,
            dq=dq,
            quat_wxyz=quat,
            gyro=gyro,
            linacc=linacc,
            buttons=self._buttons_snapshot(),
            sticks={name: 0.0 for name in STICK_KEYS},
        )

    def _publish_state_if_due(self):
        self._physics_tick += 1
        if (self._physics_tick % self.state_decimation) == 0:
            self._publish_state()

    def wait_for_high_cmd(self):
        print("Waiting for high level controller...")
        state_timer = Timer(self.state_dt)
        while self.is_alive and not self._have_command:
            self._publish_state()
            state_timer.sleep()
        print("Connected to high level")
        if not self.auto_start:
            print('Press "s" to move to default pose')
        running_zero_cmd = True
        while self.is_alive and running_zero_cmd:
            buttons = self._buttons_snapshot()
            running_zero_cmd = not bool(buttons["start"])
            self._publish_state()
            state_timer.sleep()

    def simulate_gantry(self):
        if self.auto_start:
            print("Moving to default pose (auto-start enabled)...")
        else:
            print('Moving to default pose...\nPress "a" after the robot is in default pose to begin control loop')
        timer = Timer(self.low_level_dt)
        gantry_tick = 0
        while True:
            with self._cmd_lock:
                ptargets_mujoco = self._policy_to_mujoco(self._ptargets_policy)
            with self._sim_lock:
                self.data.qpos[:7] = self.root_qpos_home
                self.data.qvel[:6] = 0.0
                self.data.qpos[7:] = ptargets_mujoco
                self.data.qvel[6:] = 0.0
                self.data.ctrl[:] = 0.0
                mujoco.mj_forward(self.model, self.data)

            if not self._viewer_sync():
                break

            if self.auto_start:
                gantry_tick += 1
                if gantry_tick >= int(round(2.0 * self.low_level_freq)):
                    self._buttons["A"] = True

            self._publish_state_if_due()

            buttons = self._buttons_snapshot()
            running_default_pos = not (bool(buttons["A"]) or bool(buttons["stop"]))
            if not running_default_pos:
                break
            timer.sleep()

    def simulate_control(self):
        print("Running control loop...")
        with self._sim_lock:
            self.data.qpos[:7] = self.root_qpos_control
            mujoco.mj_forward(self.model, self.data)

        timer = Timer(self.low_level_dt)
        time_start = time.time()
        last_log_time = time_start
        loop_count = 0

        while self.is_alive:
            if self._controller_stopped:
                print(f"{self.log_prefix} high-level controller disabled; validation complete")
                self.is_alive = False
                break
            if not self.policy_queried:
                self._publish_state_if_due()
                timer.sleep()
                time_start = time.time()
                last_log_time = time_start
                loop_count = 0
                continue

            with self._cmd_lock:
                ptargets_mujoco = self._policy_to_mujoco(self._ptargets_policy)
                kp_mujoco = self._policy_to_mujoco(self._kp_policy)
                kd_mujoco = self._policy_to_mujoco(self._kd_policy)

            with self._sim_lock:
                qpos = self.data.qpos[7:]
                qvel = self.data.qvel[6:]
                if not self._have_tracking_target:
                    delta = ptargets_mujoco - qpos
                    if float(np.linalg.norm(delta)) > 1e-4:
                        self._have_tracking_target = True
                if not self._have_tracking_target:
                    self.data.qpos[:7] = self.root_qpos_control
                    self.data.qvel[:6] = 0.0
                    self.data.qpos[7:] = ptargets_mujoco
                    self.data.qvel[6:] = 0.0
                    self.data.ctrl[:] = 0.0
                    mujoco.mj_forward(self.model, self.data)
                else:
                    ctrl_raw = kp_mujoco * (ptargets_mujoco - qpos) + kd_mujoco * (0 - qvel)
                    ctrl = np.clip(ctrl_raw, self.ctrl_lower, self.ctrl_upper)
                    self.data.ctrl[:] = ctrl
                    self._limit_external_forces()
                    self._apply_perturbation()
                    mujoco.mj_step(self.model, self.data)

                tracking_error = float(np.sqrt(np.mean(np.square(ptargets_mujoco - self.data.qpos[7:]))))
                root_z = float(self.data.qpos[2])
                self._control_sample_count += 1
                self._tracking_sq_error_sum += tracking_error * tracking_error
                self._tracking_error_max = max(self._tracking_error_max, tracking_error)
                self._root_z_min = min(self._root_z_min, root_z)
                self._root_z_max = max(self._root_z_max, root_z)
                qpos_policy = self._mujoco_to_policy(self.data.qpos[7:])
                waist = qpos_policy[self._waist_indices]
                waist_target = self._mujoco_to_policy(ptargets_mujoco)[
                    self._waist_indices
                ]
                self._waist_sum += waist
                self._waist_target_sum += waist_target
                self._waist_min = np.minimum(self._waist_min, waist)
                self._waist_max = np.maximum(self._waist_max, waist)
                self._waist_target_min = np.minimum(
                    self._waist_target_min, waist_target
                )
                self._waist_target_max = np.maximum(
                    self._waist_target_max, waist_target
                )
                waist_target_error = waist_target - waist
                self._waist_target_sq_error_sum += np.square(waist_target_error)
                self._waist_target_error_sum += waist_target_error
                waist_ctrl = self.data.ctrl[self._waist_mujoco_indices]
                self._waist_ctrl_sum += waist_ctrl
                self._waist_ctrl_abs_sum += np.abs(waist_ctrl)
                if self._have_tracking_target:
                    waist_raw = ctrl_raw[self._waist_mujoco_indices]
                    self._waist_ctrl_saturated_count += (
                        np.abs(waist_raw - waist_ctrl) > 1e-9
                    )
                if self._reference_q_policy is not None:
                    reference_mujoco = self._policy_to_mujoco(
                        self._reference_q_policy
                    )
                    reference_error = self.data.qpos[7:] - reference_mujoco
                    self._reference_sample_count += 1
                    self._reference_joint_sq_error_sum += float(
                        np.sum(np.square(reference_error))
                    )
                    self._reference_joint_sq_error_by_joint += np.square(
                        reference_error
                    )
                    self._reference_joint_error_max = max(
                        self._reference_joint_error_max,
                        float(np.max(np.abs(reference_error))),
                    )
                    self._reference_lower_sq_error_sum += float(
                        np.sum(np.square(reference_error[self._lower_mujoco_indices]))
                    )
                    self._reference_waist_sq_error_sum += np.square(
                        reference_error[self._waist_mujoco_indices]
                    )
                # MuJoCo free-joint quaternion is wxyz. Tilt is the angle
                # between the pelvis local +Z axis and world +Z.
                qw, qx, qy, qz = self.data.qpos[3:7]
                del qw, qz
                root_up_z = float(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))
                root_tilt_deg = float(np.degrees(np.arccos(root_up_z)))
                self._root_tilt_sum_deg += root_tilt_deg
                self._root_tilt_max_deg = max(
                    self._root_tilt_max_deg, root_tilt_deg
                )

            if not self._viewer_sync():
                break

            self._publish_state_if_due()

            now = time.time()
            if now - last_log_time >= 1.0:
                seconds = loop_count * self.low_level_dt
                seconds_real = now - time_start
                with self._sim_lock:
                    root_z = float(self.data.qpos[2])
                delay_stats = self._consume_policy_delay_stats()
                if delay_stats is None:
                    delay_text = "policy_delay_ms=n/a"
                else:
                    mean_ms, min_ms, max_ms, delay_count = delay_stats
                    delay_text = (
                        f"policy_delay_ms mean={mean_ms:.3f} min={min_ms:.3f} "
                        f"max={max_ms:.3f} n={delay_count}"
                    )
                print(f"Time: {seconds:.2f}, Time real: {seconds_real:.2f}, Height: {root_z:.2f}, {delay_text}")
                last_log_time = now

            loop_count += 1
            if self.max_control_seconds > 0.0 and loop_count * self.low_level_dt >= self.max_control_seconds:
                print(f"{self.log_prefix} reached --max-control-seconds={self.max_control_seconds:.3f}")
                self.is_alive = False
                break
            timer.sleep()

        self.close()

    def _limit_external_forces(self):
        if self.max_external_force <= 0.0:
            return
        for i in range(self.model.nbody):
            force = self.data.xfrc_applied[i, :3]
            force_magnitude = np.linalg.norm(force)
            if force_magnitude > self.max_external_force:
                self.data.xfrc_applied[i, :3] = force * (self.max_external_force / force_magnitude)

    def _viewer_sync(self) -> bool:
        if self.viewer is None:
            return True
        if not self.viewer.is_running():
            self.is_alive = False
            return False
        self._viewer_tick += 1
        if (self._viewer_tick % self.viewer_decim) == 0:
            self._render_reference_ghost()
            self.viewer.sync()
        return True

    def _render_reference_ghost(self):
        if (
            self.viewer is None
            or self._reference_q_policy is None
            or self._reference_root_pos is None
            or self._reference_root_quat is None
        ):
            return
        reference_q = self._policy_to_mujoco(self._reference_q_policy)
        self._ghost_data.qpos[:3] = self._reference_root_pos
        self._ghost_data.qpos[3:7] = self._reference_root_quat
        self._ghost_data.qpos[7:] = reference_q
        self._ghost_data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self._ghost_data)

        scene = self.viewer.user_scn
        scene.ngeom = 0
        mujoco.mjv_addGeoms(
            self.model,
            self._ghost_data,
            self._ghost_vopt,
            self._ghost_pert,
            mujoco.mjtCatBit.mjCAT_DYNAMIC.value,
            scene,
        )
        for i in range(scene.ngeom):
            scene.geoms[i].rgba[:] = (0.1, 1.0, 0.2, 0.32)
            scene.geoms[i].category = mujoco.mjtCatBit.mjCAT_DECOR.value

    def run(self):
        if not self.auto_start:
            self.keyboard_thread.start()

        if self.render_gui:
            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=False,
                show_right_ui=False,
                key_callback=self._viewer_key_callback,
            ) as viewer:
                self.viewer = viewer
                try:
                    self.wait_for_high_cmd()
                    self.simulate_gantry()
                    self.simulate_control()
                finally:
                    self.viewer = None
        else:
            self.wait_for_high_cmd()
            self.simulate_gantry()
            self.simulate_control()

    def close(self, *args):
        if not self.is_alive and self._summary_written:
            return
        self.is_alive = False
        self._set_button("stop", True)
        if self.keyboard_thread.is_alive():
            stop_listening()
        self.transport.close()
        if self.keyboard_thread.is_alive() and threading.current_thread() is not self.keyboard_thread:
            self.keyboard_thread.join(timeout=1.0)
        self._write_summary()
        sys.exit(0)

    def _write_summary(self):
        if self._summary_written:
            return
        count = self._control_sample_count
        reference_count = self._reference_sample_count
        metrics = {
            "robot": self.robot,
            "control_samples": count,
            "simulated_control_seconds": count * self.low_level_dt,
            "root_z_min": None if count == 0 else self._root_z_min,
            "root_z_max": None if count == 0 else self._root_z_max,
            "joint_target_rmse": None if count == 0 else float(np.sqrt(self._tracking_sq_error_sum / count)),
            "joint_target_error_max": None if count == 0 else self._tracking_error_max,
            "root_tilt_mean_deg": None if count == 0 else self._root_tilt_sum_deg / count,
            "root_tilt_max_deg": None if count == 0 else self._root_tilt_max_deg,
            "waist_joint_order": list(self._waist_names),
            "waist_pos_mean_rad": None if count == 0 else (self._waist_sum / count).tolist(),
            "waist_target_mean_rad": None if count == 0 else (
                self._waist_target_sum / count
            ).tolist(),
            "waist_pos_min_rad": None if count == 0 else self._waist_min.tolist(),
            "waist_pos_max_rad": None if count == 0 else self._waist_max.tolist(),
            "waist_target_rmse_rad": None if count == 0 else np.sqrt(
                self._waist_target_sq_error_sum / count
            ).tolist(),
            "waist_target_error_mean_rad": None if count == 0 else (
                self._waist_target_error_sum / count
            ).tolist(),
            "waist_target_min_rad": None if count == 0 else self._waist_target_min.tolist(),
            "waist_target_max_rad": None if count == 0 else self._waist_target_max.tolist(),
            "waist_ctrl_mean_nm": None if count == 0 else (
                self._waist_ctrl_sum / count
            ).tolist(),
            "waist_ctrl_abs_mean_nm": None if count == 0 else (
                self._waist_ctrl_abs_sum / count
            ).tolist(),
            "waist_ctrl_saturation_fraction": None if count == 0 else (
                self._waist_ctrl_saturated_count / count
            ).tolist(),
            "reference_samples": reference_count,
            "reference_joint_rmse_rad": None if reference_count == 0 else float(
                np.sqrt(
                    self._reference_joint_sq_error_sum
                    / (reference_count * self.n_mujoco_joints)
                )
            ),
            "reference_joint_error_max_rad": None if reference_count == 0 else self._reference_joint_error_max,
            "reference_joint_rmse_by_name_rad": None if reference_count == 0 else {
                name: float(rmse)
                for name, rmse in zip(
                    self.mujoco_joint_names,
                    np.sqrt(
                        self._reference_joint_sq_error_by_joint / reference_count
                    ),
                )
            },
            "reference_lower_body_rmse_rad": None if reference_count == 0 else float(
                np.sqrt(
                    self._reference_lower_sq_error_sum
                    / (reference_count * len(self._lower_mujoco_indices))
                )
            ),
            "reference_waist_rmse_rad": None if reference_count == 0 else np.sqrt(
                self._reference_waist_sq_error_sum / reference_count
            ).tolist(),
        }
        print(f"{self.log_prefix} summary: {json.dumps(metrics, sort_keys=True)}")
        if self.metrics_out is not None:
            self.metrics_out.parent.mkdir(parents=True, exist_ok=True)
            self.metrics_out.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
            print(f"{self.log_prefix} metrics: {self.metrics_out}")
        self._summary_written = True


def load_config(path: str) -> DictToClass:
    with open(path, "r", encoding="utf-8") as f:
        cfg = DictToClass(yaml.load(f, Loader=yaml.FullLoader))
    setattr(cfg, "_config_dir", str(Path(path).resolve().parent))
    return cfg


def main(argv=None):
    try:
        import multiprocessing as mp

        if mp.get_start_method(allow_none=True) is None:
            mp.set_start_method("spawn", force=True)
    except Exception:
        pass

    parser = argparse.ArgumentParser(description="Unified UDP MuJoCo sim2sim runner")
    parser.add_argument("--robot", choices=list(SUPPORTED_ROBOTS), default="g1")
    parser.add_argument("--xml_path", type=str, default=None)
    parser.add_argument("--bridge-config", type=str, default=None)
    parser.add_argument("--headless", action="store_true", help="Disable the MuJoCo viewer.")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Assert the start/A buttons automatically for unattended validation.",
    )
    parser.add_argument(
        "--max-control-seconds",
        type=float,
        default=0.0,
        help="Stop after this many simulated control seconds (0 means unlimited).",
    )
    parser.add_argument(
        "--metrics-out",
        type=Path,
        default=None,
        help="Optional JSON path for root, waist, and joint-tracking metrics.",
    )
    args = parser.parse_args(argv)

    config_path = args.bridge_config or str(bridge_config_path(args.robot))
    config = load_config(config_path)
    Sim2Sim(args, config).run()


if __name__ == "__main__":
    main()
