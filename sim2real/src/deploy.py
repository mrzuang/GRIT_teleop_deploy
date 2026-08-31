import os
import queue
import sys
import threading
from pathlib import Path
from typing import Dict, Optional

import yaml

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening

from common.udp_transport import UDPRobotHigh
from common.utils import DictToClass

from runtime.grit_policy import GritPolicy
from runtime.policy import Policy
from paths import SIM2REAL_ROOT, SUPPORTED_ROBOTS, controller_config_path, tracking_config_path

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})


class DampingRequested(Exception):
    pass


def get_config(policy_cfg_path: str) -> DictToClass:
    policy_cfg_path = Path(policy_cfg_path)
    if not policy_cfg_path.is_absolute():
        policy_cfg_path = SIM2REAL_ROOT / policy_cfg_path
    with open(str(policy_cfg_path), 'r') as f:
        policy_cfg = DictToClass(yaml.load(f, Loader=yaml.FullLoader))
    policy_cfg._config_dir = str(policy_cfg_path.parent)
    return policy_cfg

class Controller:
    def __init__(self, args, ctrl_cfg):
        self.args = args
        self.config = ctrl_cfg
        self.control_dt = 1.0 / self.config.control_freq
        self.policy_joint_names = list(self.config.policy_joint_names)
        self.dof_size = len(self.policy_joint_names)

        self.qj = np.zeros(self.dof_size, dtype=np.float32)
        self.dqj = np.zeros(self.dof_size, dtype=np.float32)
        self.tau = np.zeros(self.dof_size, dtype=np.float32)
        self.quat = np.zeros(4, dtype=np.float32)
        self.gyro = np.zeros(3, dtype=np.float32)
        self.linacc = np.zeros(3, dtype=np.float32)

        self.default_qpos = np.array(self.config.default_qpos, dtype=np.float32)
        self.init_qpos = np.array(self.config.init_qpos, dtype=np.float32)
        self.kps = np.array(self.config.kps, dtype=np.float32)
        self.kds = np.array(self.config.kds, dtype=np.float32)

        self.counter = 0
        self.policy_step = 0
        self.is_alive = True

        self.transport = UDPRobotHigh(self.config.udp)
        self.cmd_q = self.init_qpos.copy()
        self.cmd_qd = np.zeros(self.dof_size, dtype=np.float32)
        self.cmd_kp = self.kps.copy()
        self.cmd_kd = self.kds.copy()
        self.cmd_enable = 0
        self.extra_command = dict(getattr(self.config, "extra_command", {}))
        self.hand_left = float(np.clip(float(self.extra_command.get("hand_left", 0.0)), 0.0, 1.0))
        self.hand_right = float(np.clip(float(self.extra_command.get("hand_right", 0.0)), 0.0, 1.0))
        if "hand_enable" in self.extra_command:
            self.set_hand_command(left=self.hand_left, right=self.hand_right, enable=int(self.extra_command.get("hand_enable", 1)))
        self.buttons = {
            "start": False,
            "stop": False,
            "A": False,
            "up": False,
            "down": False,
        }
        self._keyboard_start_event = threading.Event()
        self._keyboard_play_event = threading.Event()
        self._default_pose_ready_event = threading.Event()
        self._keyboard_npz_commands: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._keyboard_damping_event = threading.Event()
        self._keyboard_exit_event = threading.Event()
        self._keyboard_thread: Optional[threading.Thread] = None
        self.sticks = {
            "lx": 0.0,
            "ly": 0.0,
            "rx": 0.0,
            "ry": 0.0,
        }
        self.have_state = False
        self.last_state_seq: Optional[int] = None
        self.last_state_receive_time_ns: Optional[int] = None
        self.skipped_state_count = 0
        self._pending_state_arrival_ns: Optional[int] = None
        self._last_state_arrival_ns: Optional[int] = None
        self._state_interval_window_start_ns: Optional[int] = None
        self._state_interval_count = 0
        self._state_interval_sum_ms = 0.0
        self._state_interval_min_ms = float("inf")
        self._state_interval_max_ms = 0.0
        self._prev_buttons = None
        self.btn_rise = {
            "start": False,
            "stop": False,
            "A": False,
            "up": False,
            "down": False,
        }

        # --- Motor temperature monitoring (overheat warning) ---
        # Hardware-order motor names matching real_joint_names in bridge config.
        # The C++ bridge sends temperature in hardware order (29 motors × 2 values).
        self._motor_names_hw = [
            "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
            "left_ankle_pitch", "left_ankle_roll",
            "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
            "right_ankle_pitch", "right_ankle_roll",
            "waist_yaw", "waist_roll", "waist_pitch",
            "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
            "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
            "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
            "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
        ]
        self._num_motors = len(self._motor_names_hw)  # 29
        self._motor_high_temp = [False] * self._num_motors
        self.HIGH_TEMP_ENTER = 90   # °C — enter warning state
        self.HIGH_TEMP_EXIT = 85    # °C — exit warning state (hysteresis)
        self._high_temp_warning_active = False
        self._high_temp_print_counter = 0

        self.wait_for_low_state()

        tracking_cfg_path = tracking_config_path(self.args.robot, self.args.tracking_config)
        print(f"[Deploy] tracking config: {tracking_cfg_path}")
        tracking_cfg = get_config(tracking_cfg_path)
        if getattr(self.args, "policy_path", None):
            policy_path = Path(self.args.policy_path)
            if not policy_path.is_absolute():
                policy_path = (Path.cwd() / policy_path).resolve()
            tracking_cfg.policy_path = str(policy_path)
            print(f"[Deploy] policy override: {policy_path}")
        if getattr(self.args, "motion_file", None):
            motion_path = Path(self.args.motion_file)
            if not motion_path.is_absolute():
                motion_path = (Path.cwd() / motion_path).resolve()
            if not motion_path.is_file():
                raise FileNotFoundError(f"Motion file not found: {motion_path}")
            tracking_cfg.motion_source["type"] = "npz"
            tracking_cfg.motion_source["npz"]["primary"] = "cli_motion"
            tracking_cfg.motion_source["npz"]["loop"] = False
            tracking_cfg.motions = [
                {
                    "name": "cli_motion",
                    "path": str(motion_path),
                    "start": 0,
                    "end": -1,
                }
            ]
            print(f"[Deploy] motion override: {motion_path}")
        self.policies = {
            "tracking": GritPolicy("tracking", tracking_cfg, self),
        }
        self.current_policy: Optional[Policy] = None
        self.pending_policy: Optional[Policy] = None

    def _consume_low_state(self, msg) -> bool:
        if msg is None:
            return False

        self.low_state = msg
        self.qj[:] = np.asarray(msg["q"], dtype=np.float32)
        self.dqj[:] = np.asarray(msg["dq"], dtype=np.float32)
        self.tau[:] = 0.0
        self.quat[:] = np.asarray(msg["quat_wxyz"], dtype=np.float32)
        self.gyro[:] = np.asarray(msg["gyro"], dtype=np.float32)
        self.linacc[:] = np.asarray(msg.get("linacc", np.zeros(3, dtype=np.float32)), dtype=np.float32)

        # Parse motor temperature (2 values per motor: casing, winding) in hardware order
        motor_temp_raw = msg.get("motor_temperature")
        if motor_temp_raw is not None:
            motor_temp = np.asarray(motor_temp_raw, dtype=np.float32).reshape(-1)
            expected_len = self._num_motors * 2
            if motor_temp.shape[0] >= expected_len:
                self._check_motor_temperature(motor_temp[:expected_len])

        buttons = msg.get("buttons", {})
        self.buttons["start"] = bool(buttons.get("start", False))
        self.buttons["stop"] = bool(buttons.get("stop", False))
        self.buttons["A"] = bool(buttons.get("A", False))
        self.buttons["up"] = bool(buttons.get("up", False))
        self.buttons["down"] = bool(buttons.get("down", False))

        sticks = msg.get("sticks", {})
        self.sticks["lx"] = float(sticks.get("lx", 0.0))
        self.sticks["ly"] = float(sticks.get("ly", 0.0))
        self.sticks["ry"] = float(sticks.get("ry", 0.0))
        self.sticks["rx"] = float(sticks.get("rx", 0.0))

        state_receive_time_ns = msg.get("state_receive_time_ns", None)
        self.last_state_receive_time_ns = None if state_receive_time_ns is None else int(state_receive_time_ns)
        self.have_state = True
        return True

    def _check_motor_temperature(self, motor_temp: np.ndarray) -> None:
        """Hysteresis-based motor overheat detection.

        motor_temp: flat array of 58 floats (29 motors × 2 values per motor).
          Index layout per motor i: [i*2]=casing, [i*2+1]=winding.
        """
        any_high = False
        high_names = []
        for i in range(self._num_motors):
            max_temp = max(float(motor_temp[i * 2]), float(motor_temp[i * 2 + 1]))
            if self._motor_high_temp[i]:
                if max_temp < self.HIGH_TEMP_EXIT:
                    self._motor_high_temp[i] = False
                else:
                    any_high = True
                    high_names.append(f"{self._motor_names_hw[i]}({int(max_temp)})")
            else:
                if max_temp >= self.HIGH_TEMP_ENTER:
                    self._motor_high_temp[i] = True
                    any_high = True
                    high_names.append(f"{self._motor_names_hw[i]}({int(max_temp)})")

        prev_active = self._high_temp_warning_active
        self._high_temp_warning_active = any_high

        if any_high:
            # Print immediately on state change, otherwise every ~1s (50 control cycles)
            if not prev_active or self._high_temp_print_counter % 50 == 0:
                print(f"[HighTemp] Warning! High temperature at {', '.join(high_names)}")
            self._high_temp_print_counter += 1
        elif prev_active:
            # All motors cooled down below exit threshold
            print("[HighTemp] All motor temperatures returned to normal.")
            self._high_temp_print_counter = 0
        else:
            self._high_temp_print_counter = 0

    def send_cmd(self):
        self.transport.send_command(
            q_des=self.cmd_q,
            qd_des=self.cmd_qd,
            kp=self.cmd_kp,
            kd=self.cmd_kd,
            enable=self.cmd_enable,
            extra_command=self.extra_command,
            state_receive_time_ns=self.last_state_receive_time_ns,
        )
        self._record_pending_state_arrival_interval()

    def _publish_reference_debug_state(self):
        if not bool(getattr(self.args, "publish_reference", False)):
            self.extra_command.pop("reference", None)
            return
        policy = self.current_policy
        if policy is None:
            self.extra_command.pop("reference", None)
            return
        ref_joint_pos = getattr(policy, "ref_joint_pos", None)
        ref_root_pos = getattr(policy, "ref_root_pos", None)
        ref_root_quat = getattr(policy, "ref_root_quat", None)
        ref_len = int(getattr(policy, "ref_len", 0))
        if (
            ref_joint_pos is None
            or ref_root_pos is None
            or ref_root_quat is None
            or ref_len <= 0
        ):
            self.extra_command.pop("reference", None)
            return
        idx = int(np.clip(int(getattr(policy, "ref_idx", 0)), 0, ref_len - 1))
        self.extra_command["reference"] = {
            "joint_pos": np.asarray(ref_joint_pos[idx], dtype=np.float32),
            "root_pos": np.asarray(ref_root_pos[idx], dtype=np.float32),
            "root_quat_wxyz": np.asarray(ref_root_quat[idx], dtype=np.float32),
            "index": idx,
        }

    def set_hand_command(self, *, left: float, right: float, enable: int = 1):
        self.hand_left = float(np.clip(left, 0.0, 1.0))
        self.hand_right = float(np.clip(right, 0.0, 1.0))
        self.extra_command["hand_enable"] = int(enable)
        self.extra_command["hand_left"] = self.hand_left
        self.extra_command["hand_right"] = self.hand_right

    def set_zero_cmd(self):
        self.cmd_q[:] = 0.0
        self.cmd_qd[:] = 0.0
        self.cmd_kp[:] = 0.0
        self.cmd_kd[:] = 0.0
        self.cmd_enable = 0

    def set_damping_cmd(self):
        self.cmd_q[:] = 0.0
        self.cmd_qd[:] = 0.0
        self.cmd_kp[:] = 0.0
        self.cmd_kd[:] = 8.0
        self.cmd_enable = 0

    def wait_for_low_state(self):
        while not self.have_state:
            self.process_state(wait_next=True, timeout_s=1.0)
        print("Successfully connected to the robot.")

    def _apply_operator_button_overrides(self):
        policies = getattr(self, "policies", None)
        if not policies:
            return
        tracking = policies.get("tracking")
        source = getattr(tracking, "source", None)
        poll = getattr(source, "poll_operator_buttons", None)
        if not callable(poll):
            return
        operator_buttons = poll()
        self.buttons["A"] = bool(
            operator_buttons.get(
                "activate_policy",
                operator_buttons.get("A", False),
            )
        )
        self.buttons["stop"] = bool(operator_buttons.get("stop", False))

    def _uses_pico_operator_buttons(self) -> bool:
        policies = getattr(self, "policies", None)
        if not policies:
            return False
        source = getattr(policies.get("tracking"), "source", None)
        return callable(getattr(source, "poll_operator_buttons", None))

    def _on_keyboard_press(self, key: str) -> None:
        ch = str(key).lower()
        if ch == "q":
            if not self._keyboard_exit_event.is_set():
                print("[Deploy] keyboard 'q' received; emergency exit requested.")
            self._keyboard_exit_event.set()
            stop_listening()
            return
        if ch == "x":
            if not self._keyboard_damping_event.is_set():
                print("[Deploy] keyboard 'x' received; damping mode requested.")
            self._keyboard_damping_event.set()
            stop_listening()
            return
        if ch not in ("a", "s"):
            return

        current_policy = getattr(self, "current_policy", None)
        if (
            current_policy is not None
            and getattr(current_policy, "motion_source", None) == "npz"
        ):
            command = "default" if ch == "s" else "play"
            print(
                f"[Deploy] keyboard '{ch}' received; NPZ command='{command}'."
            )
            self._keyboard_npz_commands.put(command)
            return

        if ch == "a":
            tracking = getattr(self, "policies", {}).get("tracking")
            if (
                self._default_pose_ready_event.is_set()
                and getattr(tracking, "motion_source", None) == "npz"
            ):
                print("[Deploy] keyboard 'a' received; starting NPZ playback.")
                self._keyboard_play_event.set()
            else:
                print("[Deploy] keyboard 'a' ignored; press 's' first.")
            return

        if not self._keyboard_start_event.is_set():
            print("[Deploy] keyboard 's' received; leaving zero torque state.")
        self._keyboard_start_event.set()

    def _apply_keyboard_npz_request(self) -> None:
        command = None
        while True:
            try:
                command = self._keyboard_npz_commands.get_nowait()
            except queue.Empty:
                break
        if command is None:
            return

        current_policy = self.current_policy
        if current_policy is None or getattr(current_policy, "motion_source", None) != "npz":
            return
        source = getattr(current_policy, "source", None)
        handler_name = "return_to_default" if command == "default" else "play_from_start"
        handler = getattr(source, handler_name, None)
        if callable(handler):
            handler()

    def _raise_if_keyboard_exit_requested(self) -> None:
        if self._keyboard_exit_event.is_set():
            raise KeyboardInterrupt
        if self._keyboard_damping_event.is_set():
            raise DampingRequested

    def _keyboard_listener_loop(self) -> None:
        try:
            listen_keyboard(on_press=self._on_keyboard_press, until=None)
        except Exception as exc:
            print(f"[Deploy] keyboard listener unavailable: {exc}")

    def _start_keyboard_listener(self) -> bool:
        if not sys.stdin.isatty():
            print("[Deploy] stdin is not a terminal; use keyboard 's' in the G1 bridge terminal.")
            return False
        if self._keyboard_thread is not None and self._keyboard_thread.is_alive():
            return True
        self._keyboard_start_event.clear()
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener_loop,
            daemon=True,
            name="deploy-keyboard",
        )
        self._keyboard_thread.start()
        return True

    def _stop_keyboard_listener(self) -> None:
        thread = self._keyboard_thread
        if thread is None:
            return
        stop_listening()
        if thread.is_alive() and threading.current_thread() is not thread:
            thread.join(timeout=1.0)
        self._keyboard_thread = None

    def zero_torque_state(self):
        print("Enter zero torque state.")
        self._start_keyboard_listener()
        if bool(getattr(self.args, "auto_start", False)):
            print("Auto-start enabled; sending one zero-torque command.")
            self.set_zero_cmd()
            self.send_cmd()
            return
        print("Waiting for the start signal (press 's'; press 'q' for emergency exit)...")
        while not self.buttons["start"] and not self._keyboard_start_event.is_set():
            self.process_state(wait_next=True)
            self.set_zero_cmd()
            self.send_cmd()

    def move_to_default_qpos(self):
        print("Moving to init pos....")
        total_time = 2.0
        num_step = int(total_time / self.control_dt)

        init_dof_pos = self.qj.copy()

        for t in range(num_step):
            self.process_state(wait_next=True)
            alpha = t / num_step
            self.cmd_q[:] = init_dof_pos * (1 - alpha) + self.init_qpos * alpha
            self.cmd_qd[:] = 0.0
            self.cmd_kp[:] = self.kps
            self.cmd_kd[:] = self.kds
            self.send_cmd()
        self._default_pose_ready_event.set()

    def default_qpos_state(self):
        initial_policy: Optional[Policy] = None
        tracking_policy = self.policies["tracking"]
        npz_control = getattr(tracking_policy, "motion_source", None) == "npz"
        auto_start = bool(getattr(self.args, "auto_start", False))

        if npz_control:
            initial_policy = tracking_policy
            print("NPZ tracking policy active in default pose; press 'a' to play.")
        elif auto_start:
            initial_policy = tracking_policy
            print(f"Auto-start initial policy: {initial_policy.name}")
        else:
            if self._uses_pico_operator_buttons():
                print("Press PICO X to activate the tracking policy in default pose...")
            else:
                print("Press A to start the tracking policy...")

            while True:
                self.process_state(wait_next=True)

                self.cmd_q[:] = self.init_qpos
                self.cmd_qd[:] = 0.0
                self.cmd_kp[:] = self.kps
                self.cmd_kd[:] = self.kds
                self.send_cmd()

                if self.btn_rise["stop"]:
                    raise KeyboardInterrupt

                if self.btn_rise["A"] or self._keyboard_play_event.is_set():
                    self._keyboard_play_event.clear()
                    initial_policy = self.policies["tracking"]
                    print(f"Initial policy: {initial_policy.name}")
                    break

        self.current_policy = initial_policy
        self.current_policy.fade_in()
        if npz_control and (auto_start or self._keyboard_play_event.is_set()):
            self._keyboard_play_event.clear()
            play_from_start = getattr(self.current_policy.source, "play_from_start", None)
            if callable(play_from_start):
                play_from_start()
        self.cmd_enable = 1
        self.send_cmd()

    def process_state(self, *, wait_next: bool = False, timeout_s: float | None = None) -> bool:
        self._raise_if_keyboard_exit_requested()
        if wait_next:
            pkt = self.transport.read_next_state(after_seq=self.last_state_seq, timeout_s=timeout_s, with_meta=True)
        else:
            pkt = self.transport.read_latest_state(with_meta=True)
        self._raise_if_keyboard_exit_requested()
        if pkt is None:
            return False

        if self.last_state_seq is not None:
            seq_delta = int(pkt.seq) - int(self.last_state_seq)
            if seq_delta <= 0:
                return False
            if seq_delta > 1:
                skipped = seq_delta - 1
                self.skipped_state_count += skipped
                print(f"[Warning] skipped {skipped} bridge state packet(s): last={self.last_state_seq}, now={pkt.seq}")
        self.last_state_seq = int(pkt.seq)
        self._pending_state_arrival_ns = int(pkt.recv_time_ns)
        self._consume_low_state(pkt.data)
        self._apply_operator_button_overrides()

        now = self.buttons.copy()
        if self._prev_buttons is None:
            self._prev_buttons = now
            self.btn_rise = {k: False for k in now}
        else:
            self.btn_rise = {k: (not self._prev_buttons[k]) and now[k] for k in now}
            self._prev_buttons = now
        return True

    def _record_pending_state_arrival_interval(self) -> None:
        recv_time_ns = self._pending_state_arrival_ns
        self._pending_state_arrival_ns = None
        if recv_time_ns is None:
            return
        self._record_state_arrival_interval(recv_time_ns)

    def _record_state_arrival_interval(self, recv_time_ns: int) -> None:
        if recv_time_ns <= 0:
            return
        if self._state_interval_window_start_ns is None:
            self._state_interval_window_start_ns = recv_time_ns
        if self._last_state_arrival_ns is not None:
            interval_ms = (recv_time_ns - self._last_state_arrival_ns) * 1e-6
            if interval_ms >= 0.0:
                self._state_interval_count += 1
                self._state_interval_sum_ms += interval_ms
                self._state_interval_min_ms = min(self._state_interval_min_ms, interval_ms)
                self._state_interval_max_ms = max(self._state_interval_max_ms, interval_ms)
        self._last_state_arrival_ns = recv_time_ns

        if recv_time_ns - self._state_interval_window_start_ns < 1_000_000_000:
            return

        if self._state_interval_count > 0:
            mean_ms = self._state_interval_sum_ms / self._state_interval_count
            print(
                "[Deploy] state_interval_ms "
                f"mean={mean_ms:.3f} min={self._state_interval_min_ms:.3f} "
                f"max={self._state_interval_max_ms:.3f} n={self._state_interval_count}"
            )
        else:
            print("[Deploy] state_interval_ms n/a")

        self._state_interval_window_start_ns = recv_time_ns
        self._state_interval_count = 0
        self._state_interval_sum_ms = 0.0
        self._state_interval_min_ms = float("inf")
        self._state_interval_max_ms = 0.0

    def _apply_action(self, action_delta: np.ndarray):
        if action_delta is None or not np.all(np.isfinite(action_delta)):
            print("[Controller] action invalid; hold init PD")
            raise KeyboardInterrupt
        else:
            desired = self.default_qpos + action_delta
            target = desired

        self.cmd_q[:] = target
        self.cmd_qd[:] = 0.0
        self.cmd_kp[:] = self.kps
        self.cmd_kd[:] = self.kds
        self.cmd_enable = 1

    def run(self):
        print("Running high level...")

        while True:
            if not self.process_state(wait_next=True, timeout_s=1.0):
                print("[Warning] no bridge state packet for 1s")
                continue

            if self.btn_rise["stop"]:
                break

            self._apply_keyboard_npz_request()
            self.current_policy.update_obs()
            action = self.current_policy.compute_action()
            self._apply_action(action)
            self._publish_reference_debug_state()
            self.send_cmd()

            self.current_policy.post_step()
            self.policy_step += 1

            max_policy_steps = int(getattr(self.args, "max_policy_steps", 0))
            if max_policy_steps > 0 and self.policy_step >= max_policy_steps:
                print(f"[Deploy] reached --max-policy-steps={max_policy_steps}")
                break

    def close(self):
        print("Closing...")
        self.is_alive = False
        cleanup_steps = []
        current_policy = getattr(self, "current_policy", None)
        if current_policy is not None:
            cleanup_steps.append(("policy", current_policy.deactivate))
        cleanup_steps.extend(
            (
                ("keyboard listener", self._stop_keyboard_listener),
                ("UDP transport", self.transport.close),
            )
        )
        for label, cleanup in cleanup_steps:
            try:
                cleanup()
            except Exception as exc:
                print(f"[Deploy] Failed to close {label}: {exc}")

import traceback

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=list(SUPPORTED_ROBOTS), default="g1")
    parser.add_argument(
        "--controller-config",
        type=Path,
        default=None,
        help=(
            "Controller YAML. Relative paths resolve under the sim2real root; "
            "default: config/<robot>/controller.yaml."
        ),
    )
    parser.add_argument(
        "--tracking-config",
        default="tracking.yaml",
        help="Tracking YAML. Relative names resolve under config/<robot>/.",
    )
    parser.add_argument(
        "--policy-path",
        type=Path,
        default=None,
        help="Override policy_path from the tracking YAML.",
    )
    parser.add_argument(
        "--motion-file",
        type=Path,
        default=None,
        help="Override the tracking motion with one local NPZ file (one-shot).",
    )
    parser.add_argument(
        "--publish-reference",
        action="store_true",
        help="Publish the current reference pose for sim2sim metrics/ghost rendering.",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Skip start/A button waits. Intended for sim2sim validation only.",
    )
    parser.add_argument(
        "--max-policy-steps",
        type=int,
        default=0,
        help="Stop after this many policy steps (0 means unlimited).",
    )
    args = parser.parse_args()

    controller = None
    try:
        ctrl_path = args.controller_config or controller_config_path(args.robot)
        print(f"[Deploy] controller config: {ctrl_path}")
        controller = Controller(args, get_config(ctrl_path))
        controller.zero_torque_state()
        controller.move_to_default_qpos()
        controller.default_qpos_state()
        controller.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    except DampingRequested:
        print("Damping requested from keyboard. Entering damping mode...")
    except Exception as e:
        print(f"An exception occurred: {e}")
        traceback.print_exc()
    finally:
        if controller is not None:
            try:
                controller.set_damping_cmd()
                controller.send_cmd()
            except Exception as cleanup_exc:
                print(f"[Deploy] Failed to send shutdown damping command: {cleanup_exc}")
            finally:
                controller.close()
