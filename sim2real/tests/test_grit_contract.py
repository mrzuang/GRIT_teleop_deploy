import re
import sys
import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
import zmq


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from runtime.grit_observation import (
    GRIT_CONTEXT_FRAMES,
    GRIT_FEATURE_DIM,
    build_grit_reference_features,
    build_grit_streaming_reference_features,
    grit_context_offsets,
)
from runtime.grit_policy import (
    GRIT_ACTION_JOINT_NAMES,
    GRIT_ACTION_SCALE_BY_NAME,
    GritPolicy,
    GritONNXModel,
)
from runtime.math_utils import _slerp
from runtime.motion_sources import MotionSourceBase, VRMotionSource, _validate_default_motion
from deploy import Controller


class GritContractTest(unittest.TestCase):
    def test_reference_transition_slerp_uses_wxyz_quaternions(self):
        q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        q1 = np.array(
            [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)], dtype=np.float32
        )

        midpoint = _slerp(q0, q1, 1)[0]

        expected = np.array(
            [np.cos(np.pi / 8.0), 0.0, 0.0, np.sin(np.pi / 8.0)],
            dtype=np.float32,
        )
        np.testing.assert_allclose(midpoint, expected, atol=1e-6)

    def test_default_motion_rejects_multiple_frames(self):
        motions = {
            "default": {
                "joint_pos": np.zeros((2, 29), dtype=np.float32),
            }
        }
        with self.assertRaisesRegex(ValueError, "exactly one frame"):
            _validate_default_motion(motions, "test")

    def test_motion_loader_rejects_pickled_object_arrays(self):
        with TemporaryDirectory() as tmp_dir:
            motion_path = Path(tmp_dir) / "unsafe.npz"
            np.savez(
                motion_path,
                fps=np.array(50.0, dtype=np.float32),
                joint_pos=np.array([{"unsafe": True}], dtype=object),
                joint_vel=np.zeros((1, 1), dtype=np.float32),
                body_pos_w=np.zeros((1, 1, 3), dtype=np.float32),
                body_quat_w=np.array([[[1.0, 0.0, 0.0, 0.0]]], dtype=np.float32),
                body_lin_vel_w=np.zeros((1, 1, 3), dtype=np.float32),
                body_ang_vel_w=np.zeros((1, 1, 3), dtype=np.float32),
            )
            config = SimpleNamespace(
                _config_dir=tmp_dir,
                motions=[{"name": "unsafe", "path": motion_path, "start": 0, "end": -1}],
                motion_clips=[],
            )

            with self.assertRaisesRegex(ValueError, "Object arrays cannot be loaded"):
                MotionSourceBase(SimpleNamespace(), config)

    def test_vr_default_clip_matches_controller_default_pose(self):
        config_dir = PROJECT_ROOT / "config" / "g1"
        controller = yaml.safe_load(
            (config_dir / "controller.yaml").read_text(encoding="utf-8")
        )
        tracking = yaml.safe_load(
            (config_dir / "tracking_vr.yaml").read_text(encoding="utf-8")
        )

        self.assertFalse(
            any(motion["name"] == "default" for motion in tracking.get("motions", []))
        )
        default_clips = [
            clip for clip in tracking["motion_clips"] if clip["name"] == "default"
        ]
        self.assertEqual(len(default_clips), 1)
        default_clip = default_clips[0]

        clip_by_name = dict(
            zip(tracking["reference_joint_names"], default_clip["joint_pos"])
        )
        clip_in_controller_order = [
            clip_by_name[name] for name in controller["policy_joint_names"]
        ]
        np.testing.assert_allclose(
            clip_in_controller_order, controller["default_qpos"], atol=0.0, rtol=0.0
        )
        self.assertEqual(default_clip["root_quat"], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(default_clip["root_pos"], [0.0, 0.0, 0.78])

    def test_controller_close_continues_after_cleanup_failure(self):
        calls = []
        controller = Controller.__new__(Controller)
        controller.is_alive = True

        def fail_keyboard_cleanup():
            calls.append("keyboard")
            raise RuntimeError("listener failed")

        controller._stop_keyboard_listener = fail_keyboard_cleanup
        controller.transport = SimpleNamespace(close=lambda: calls.append("transport"))

        controller.close()

        self.assertFalse(controller.is_alive)
        self.assertEqual(calls, ["keyboard", "transport"])

    def test_deploy_keyboard_s_requests_zero_torque_exit(self):
        controller = Controller.__new__(Controller)
        controller._keyboard_start_event = threading.Event()

        controller._on_keyboard_press("a")
        self.assertFalse(controller._keyboard_start_event.is_set())

        controller._on_keyboard_press("s")
        self.assertTrue(controller._keyboard_start_event.is_set())

    def test_vr_fade_in_honors_controller_auto_start(self):
        for auto_start in (False, True):
            with self.subTest(auto_start=auto_start):
                source = VRMotionSource.__new__(VRMotionSource)
                source.policy = SimpleNamespace(
                    controller=SimpleNamespace(
                        args=SimpleNamespace(auto_start=auto_start)
                    )
                )
                appended = []
                source.append_motion_from_tail = appended.append

                source.on_fade_in()

                self.assertEqual(appended, ["default"])
                self.assertEqual(source._pending_start_request, auto_start)
                self.assertEqual(source._vr_user_enabled, auto_start)
                self.assertFalse(source._vr_active)

    def test_vr_fade_in_stands_until_pico_a_is_pressed_after_activation(self):
        source = VRMotionSource.__new__(VRMotionSource)
        source.policy = SimpleNamespace(
            controller=SimpleNamespace(args=SimpleNamespace(auto_start=False))
        )
        source.append_motion_from_tail = lambda _name: None
        source._vr_user_enabled = True
        source._latest_control_buttons = {"right_key_one": True}

        source.on_fade_in()

        self.assertFalse(source._pending_start_request)
        self.assertFalse(source._vr_user_enabled)
        self.assertTrue(source._prev_start_btn)

    def test_pico_x_y_override_policy_activation_and_stop_buttons(self):
        source = SimpleNamespace(
            poll_operator_buttons=lambda: {
                "activate_policy": True,
                "stop": True,
            }
        )
        controller = Controller.__new__(Controller)
        controller.policies = {
            "tracking": SimpleNamespace(source=source),
        }
        controller.buttons = {"A": False, "stop": False}

        controller._apply_operator_button_overrides()

        self.assertTrue(controller.buttons["A"])
        self.assertTrue(controller.buttons["stop"])

    def test_vr_operator_button_names_match_pico_a_x_y(self):
        self.assertEqual(VRMotionSource.POLICY_ACTIVATE_BUTTON, "left_key_one")
        self.assertEqual(VRMotionSource.POLICY_START_BUTTON, "right_key_one")
        self.assertEqual(VRMotionSource.RETURN_DEFAULT_BUTTON, "left_key_one")
        self.assertEqual(VRMotionSource.STOP_BUTTON, "left_key_two")

    def test_vr_control_messages_map_pico_a_to_start_x_to_default_and_y_to_stop(self):
        class FakeControlSocket:
            def __init__(self, payload):
                self.payload = payload

            def recv_string(self, *, flags):
                self.assert_nonblocking(flags)
                if self.payload is None:
                    raise zmq.Again()
                payload, self.payload = self.payload, None
                return payload

            @staticmethod
            def assert_nonblocking(flags):
                if flags != zmq.NOBLOCK:
                    raise AssertionError(flags)

        source = VRMotionSource.__new__(VRMotionSource)
        controller = SimpleNamespace(config={})
        events = []
        source.policy = SimpleNamespace(
            controller=controller,
            discard_future_ref_frames=lambda: events.append("discard") or 4,
        )
        source.append_motion_from_tail = (
            lambda name: events.append(f"append:{name}") or True
        )
        controller.current_policy = source.policy
        source._latest_control_buttons = {}
        source._latest_control_sticks = {}
        source._hand_control_cfg = {}
        source._prev_start_btn = False
        source._prev_default_btn = False
        source._prev_stop_btn = False
        source._vr_user_enabled = False
        source._pending_start_request = False
        source._req_inflight = False
        source._req_inflight_steps_left = 0
        source._vr_active = False
        source._vr_align_ready = False
        source._vr_in_transition = False
        source._vr_transition_count = 0

        source._ctrl_sock = FakeControlSocket(
            '{"controller_buttons":{"right_key_one":true,"left_key_one":false,"left_key_two":false}}'
        )
        source._drain_control()
        self.assertTrue(source._vr_user_enabled)
        self.assertTrue(source._pending_start_request)
        self.assertEqual(
            source.poll_operator_buttons(),
            {"activate_policy": False, "stop": False},
        )

        source._ctrl_sock = FakeControlSocket(
            '{"controller_buttons":{"right_key_one":false,"left_key_one":true,"left_key_two":false}}'
        )
        source._drain_control()
        self.assertFalse(source._vr_user_enabled)
        self.assertFalse(source._pending_start_request)
        self.assertEqual(events, ["discard", "append:default"])
        self.assertEqual(
            source.poll_operator_buttons(),
            {"activate_policy": True, "stop": False},
        )

        source._ctrl_sock = FakeControlSocket(
            '{"controller_buttons":{"right_key_one":true,"left_key_one":false,"left_key_two":false}}'
        )
        source._drain_control()
        self.assertTrue(source._vr_user_enabled)
        self.assertTrue(source._pending_start_request)

        source._ctrl_sock = FakeControlSocket(
            '{"controller_buttons":{"right_key_one":false,"left_key_one":false,"left_key_two":true}}'
        )
        source._drain_control()
        self.assertEqual(
            source.poll_operator_buttons(),
            {"activate_policy": False, "stop": True},
        )

    def test_pico_a_is_ignored_until_pico_x_activates_policy(self):
        class FakeControlSocket:
            def __init__(self):
                self.payload = (
                    '{"controller_buttons":{"right_key_one":true,'
                    '"left_key_one":false,"left_key_two":false}}'
                )

            def recv_string(self, *, flags):
                if flags != zmq.NOBLOCK:
                    raise AssertionError(flags)
                if self.payload is None:
                    raise zmq.Again()
                payload, self.payload = self.payload, None
                return payload

        controller = SimpleNamespace(config={}, current_policy=None)
        source = VRMotionSource.__new__(VRMotionSource)
        source.policy = SimpleNamespace(controller=controller)
        source._ctrl_sock = FakeControlSocket()
        source._latest_control_buttons = {}
        source._latest_control_sticks = {}
        source._hand_control_cfg = {}
        source._prev_start_btn = False
        source._prev_default_btn = False
        source._prev_stop_btn = False
        source._vr_user_enabled = False
        source._pending_start_request = False
        source._req_inflight = False
        source._req_inflight_steps_left = 0
        source._vr_active = False
        source._vr_align_ready = False
        source._vr_in_transition = False
        source._vr_transition_count = 0

        source._drain_control()

        self.assertFalse(source._vr_user_enabled)
        self.assertFalse(source._pending_start_request)

    def test_grit_discard_future_frames_keeps_feature_cache_aligned(self):
        policy = GritPolicy.__new__(GritPolicy)
        policy.ref_idx = 2
        policy.ref_len = 6
        policy.ref_joint_pos = np.arange(18, dtype=np.float32).reshape(6, 3)
        policy.ref_root_quat = np.arange(24, dtype=np.float32).reshape(6, 4)
        policy.ref_root_pos = np.arange(18, dtype=np.float32).reshape(6, 3)
        policy.grit_ref_features = np.arange(
            6 * GRIT_FEATURE_DIM, dtype=np.float32
        ).reshape(6, GRIT_FEATURE_DIM)
        policy.current_done = False

        discarded = policy.discard_future_ref_frames()

        self.assertEqual(discarded, 3)
        self.assertEqual(policy.ref_len, 3)
        self.assertTrue(policy.current_done)
        self.assertEqual(policy.ref_joint_pos.shape[0], 3)
        self.assertEqual(policy.ref_root_quat.shape[0], 3)
        self.assertEqual(policy.ref_root_pos.shape[0], 3)
        self.assertEqual(policy.grit_ref_features.shape[0], 3)

    def test_vr_alignment_preserves_relative_root_height(self):
        source = VRMotionSource.__new__(VRMotionSource)
        source._vr_align_ready = True
        source._vr_r_delta = R.identity()
        source._vr_source_root_pos0 = np.array(
            [1.0, 2.0, 0.70], dtype=np.float32
        )
        source._vr_target_anchor_pos = np.array(
            [0.0, 0.0, 0.78], dtype=np.float32
        )

        aligned = source._align_vr_frame(
            {
                "joint_pos": np.zeros(29, dtype=np.float32),
                "root_pos": np.array([1.1, 1.9, 0.60], dtype=np.float32),
                "root_quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            }
        )

        self.assertIsNotNone(aligned)
        np.testing.assert_allclose(
            aligned["root_pos"], [0.1, -0.1, 0.68], atol=1e-6
        )

    def test_streaming_reference_recovers_velocity_from_previous_frame(self):
        joint0 = np.zeros(29, dtype=np.float32)
        joint1 = np.full((1, 29), 0.02, dtype=np.float32)
        root0 = np.array([0.0, 0.0, 0.78], dtype=np.float32)
        root1 = np.array([[0.01, 0.0, 0.78]], dtype=np.float32)

        yaw0 = np.pi / 2.0
        yaw1 = yaw0 + 0.02
        quat0 = np.array(
            [np.cos(yaw0 / 2.0), 0.0, 0.0, np.sin(yaw0 / 2.0)],
            dtype=np.float32,
        )
        quat1 = np.array(
            [[np.cos(yaw1 / 2.0), 0.0, 0.0, np.sin(yaw1 / 2.0)]],
            dtype=np.float32,
        )

        features, previous_update = build_grit_streaming_reference_features(
            joint1,
            root1,
            quat1,
            fps=50.0,
            previous_joint_pos=joint0,
            previous_root_pos=root0,
            previous_root_quat_wxyz=quat0,
        )

        self.assertEqual(features.shape, (1, 70))
        self.assertIsNotNone(previous_update)
        np.testing.assert_allclose(previous_update[0, 29:58], 1.0, atol=1e-5)
        np.testing.assert_allclose(features[0, 29:58], 1.0, atol=1e-5)
        np.testing.assert_allclose(previous_update[0, 64:67], [0.0, -0.5, 0.0], atol=1e-5)
        np.testing.assert_allclose(previous_update[0, 67:70], [0.0, 0.0, 1.0], atol=1e-5)
        np.testing.assert_allclose(features[0, 67:70], [0.0, 0.0, 1.0], atol=1e-5)

        _, two_previous_updates = build_grit_streaming_reference_features(
            np.full((1, 29), 0.08, dtype=np.float32),
            np.array([[0.04, 0.0, 0.78]], dtype=np.float32),
            quat1,
            fps=50.0,
            previous_joint_pos=np.stack([joint0, joint1[0]]),
            previous_root_pos=np.stack([root0, root1[0]]),
            previous_root_quat_wxyz=np.stack([quat0, quat1[0]]),
        )
        # The middle frame uses (next - previous) / (2 * dt).
        np.testing.assert_allclose(
            two_previous_updates[-1, 29:58], 2.0, atol=1e-5
        )

    def test_runtime_profiles_match_grit_parameters(self):
        config_dir = PROJECT_ROOT / "config" / "g1"
        controller = yaml.safe_load(
            (config_dir / "controller.yaml").read_text(encoding="utf-8")
        )
        bridge = yaml.safe_load(
            (config_dir / "bridge.yaml").read_text(encoding="utf-8")
        )

        self.assertEqual(controller["control_freq"], 50)
        self.assertEqual(bridge["freq"], {"physical_hz": 200, "state_decimation": 4})
        self.assertEqual(controller["policy_joint_names"], bridge["policy_joint_names"])
        self.assertEqual(controller["default_qpos"], controller["init_qpos"])
        self.assertEqual(controller["default_qpos"], bridge["home_q"])

        names = controller["policy_joint_names"]
        kp = dict(zip(names, controller["kps"]))
        kd = dict(zip(names, controller["kds"]))
        armature = dict(zip(names, bridge["joint_armatures"]))
        self.assertAlmostEqual(kp["waist_roll_joint"], 28.50124619574858)
        self.assertAlmostEqual(kd["waist_roll_joint"], 1.814445686584846)
        self.assertAlmostEqual(armature["waist_roll_joint"], 0.00721945)
        self.assertAlmostEqual(kp["left_shoulder_pitch_joint"], 14.25062309787429)
        self.assertAlmostEqual(armature["left_hip_roll_joint"], 0.025101925)

    def test_tracking_configs_use_grit_action_order_and_scales(self):
        for name in ("tracking.yaml", "tracking_vr.yaml"):
            with self.subTest(config=name):
                path = PROJECT_ROOT / "config" / "g1" / name
                config = yaml.safe_load(path.read_text(encoding="utf-8"))
                self.assertEqual(config["transition_steps"], 100)
                policy_path = (path.parent / config["policy_path"]).resolve()
                self.assertEqual(policy_path, PROJECT_ROOT / "checkpoints" / "policy.onnx")
                action_names = config["action_joint_names"]
                self.assertEqual(action_names, config["reference_joint_names"])
                self.assertEqual(action_names, GRIT_ACTION_JOINT_NAMES)
                self.assertEqual(len(config["action_scale"]), len(action_names))

                expected_scales = []
                for joint_name in action_names:
                    matches = [
                        value
                        for pattern, value in GRIT_ACTION_SCALE_BY_NAME.items()
                        if re.fullmatch(pattern, joint_name)
                    ]
                    self.assertEqual(len(matches), 1, joint_name)
                    expected_scales.append(matches[0])
                np.testing.assert_allclose(config["action_scale"], expected_scales)

    def test_reference_contract_matches_grit_rate_conversion(self):
        self.assertEqual(GRIT_CONTEXT_FRAMES, 9)
        self.assertEqual(GRIT_FEATURE_DIM, 70)
        np.testing.assert_array_equal(
            grit_context_offsets(), [0, 2, 3, 5, 7, 8, 10, 12, 13]
        )

    def test_deployment_onnx_adapter_accepts_packed_runtime_observation(self):
        policy_path = PROJECT_ROOT / "checkpoints" / "policy.onnx"
        if not policy_path.is_file():
            self.skipTest("deployment checkpoint is not included in the source package")

        rng = np.random.default_rng(0)
        reference = rng.normal(0.0, 0.2, (2, 9, 70)).astype(np.float32)
        history = rng.normal(0.0, 0.2, (2, 990)).astype(np.float32)
        packed = np.concatenate((reference.reshape(2, -1), history), axis=-1)

        module = GritONNXModel(str(policy_path))
        self.assertEqual(module.mode, "deploy_multi_input")
        self.assertEqual(module.policy_input_dim, 1620)
        packed_actions = module({"policy": packed})["actions"]
        explicit_actions = module(
            {"reference_context": reference, "proprio_history": history}
        )["actions"]

        self.assertEqual(packed_actions.shape, (2, 29))
        self.assertTrue(np.isfinite(packed_actions).all())
        np.testing.assert_array_equal(packed_actions, explicit_actions)


if __name__ == "__main__":
    unittest.main()
