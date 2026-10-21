"""UDP endpoints shared by the policy runtime and the MuJoCo stand-in bridge.

Wire format is the native bridge's ULDP datagram (see udp_latest.py and
native_bridge/src/h2_udp_bridge.cpp). All joint arrays are in policy
order (config/h2.json joint_names); only the native bridge converts to H2 DDS motor order.
"""
import time

import numpy as np

from .udp_latest import UDPLatestReceiver, UDPLatestSender

BUTTONS = ('start', 'stop', 'A', 'up', 'down')


class RobotClient:
    """Policy side: receive bridge state, send PD commands."""
    def __init__(self, udp):
        self.state_receiver = UDPLatestReceiver(udp['state_bind_host'], int(udp['state_port']))
        self.command_sender = UDPLatestSender(udp['cmd_host'], int(udp['cmd_port']))
        self.state_receiver.start()

    def close(self):
        self.state_receiver.close()
        self.command_sender.close()

    def read_next_state(self, after_seq, timeout_s):
        return self.state_receiver.read_next_data(after_seq=after_seq, timeout_s=timeout_s, with_meta=True)

    def send_command(self, *, q_des, kp, kd, enable, state_receive_time_ns=None):
        payload = {'q_des': np.asarray(q_des, np.float32), 'qd_des': np.zeros(len(q_des), np.float32),
                   'kp': np.asarray(kp, np.float32), 'kd': np.asarray(kd, np.float32), 'enable': int(enable)}
        if state_receive_time_ns is not None:
            payload['state_receive_time_ns'] = int(state_receive_time_ns)
        return self.command_sender.send(payload)


class BridgeServer:
    """Bridge side for the MuJoCo stand-in: send state, receive PD commands."""
    def __init__(self, udp):
        self.state_sender = UDPLatestSender(udp['state_host'], int(udp['state_port']))
        self.command_receiver = UDPLatestReceiver(udp['cmd_bind_host'], int(udp['cmd_port']))
        self.command_receiver.start()

    def close(self):
        self.command_receiver.close()
        self.state_sender.close()

    def read_latest_command(self):
        return self.command_receiver.read_latest_data(with_meta=True)

    def send_state(self, *, q, dq, quat_wxyz, gyro, linacc, buttons):
        self.state_sender.send({
            'q': np.asarray(q, np.float32), 'dq': np.asarray(dq, np.float32),
            'quat_wxyz': np.asarray(quat_wxyz, np.float32), 'gyro': np.asarray(gyro, np.float32),
            'linacc': np.asarray(linacc, np.float32),
            'motor_temperature': np.zeros(2*len(q), np.float32),
            'buttons': {k: bool(buttons.get(k, False)) for k in BUTTONS},
            'sticks': {'lx': 0., 'ly': 0., 'rx': 0., 'ry': 0.},
            'state_receive_time_ns': time.perf_counter_ns()})
