"""ONNX Runtime inference for the H2 policy (CPU, no GPU required)."""
import hashlib
import json
from pathlib import Path
import numpy as np
import onnxruntime as ort
from .observations import ProprioHistory, proprio_frame


class H2Policy:
    def __init__(self, policy_path, config):
        self.path=Path(policy_path)
        self.config=config
        self.meta=json.loads(self.path.with_suffix('.json').read_text())
        if hashlib.sha256(self.path.read_bytes()).hexdigest()!=self.meta['onnx_sha256']:
            raise ValueError('ONNX checksum differs from its metadata')
        if self.meta['joint_names']!=config['joint_names']:
            raise ValueError('Policy/controller joint order mismatch')
        options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(self.path),sess_options=options,providers=['CPUExecutionProvider'])
        actual={v.name:tuple(v.shape[1:]) for v in self.session.get_inputs()}
        if actual!={'reference':(9,74),'state':(1050,)}:
            raise ValueError(f'Unexpected H2 policy inputs: {actual}')
        if [(v.name,tuple(v.shape[1:])) for v in self.session.get_outputs()]!=[('action',(31,))]:
            raise ValueError('Unexpected H2 policy output')
        self.default=np.array(config['default_joint_pos'],dtype=np.float32)
        self.scale=np.array(config['action_scale'],dtype=np.float32)
        self.encoder_offset=np.array(config['encoder_offset'],dtype=np.float32)
        self.history=ProprioHistory()
        self.last_action=np.zeros(31,np.float32)

    def reset(self):
        self.history.reset();self.last_action.fill(0)

    def infer(self, reference_context, history):
        context=np.asarray(reference_context,dtype=np.float32)
        history=np.asarray(history,dtype=np.float32)
        if context.shape!=(9,74) or history.shape!=(1050,) or not (np.isfinite(context).all() and np.isfinite(history).all()):
            raise ValueError('Expected finite [9,74] reference and [1050] history')
        actions=self.session.run(['action'],{'reference':context[None],'state':history[None]})[0][0]
        if actions.shape!=(31,) or not np.isfinite(actions).all():
            raise RuntimeError('Policy produced invalid actions')
        return actions

    def step(self, reference_context, reference_root_quat, *, gyro, root_quat, joint_pos, joint_vel):
        frame=proprio_frame(gyro,root_quat,reference_root_quat,joint_pos,joint_vel,self.last_action,self.default)
        actions=self.infer(reference_context,self.history.append(frame))
        self.last_action=actions.copy()
        return self.default+self.scale*actions-self.encoder_offset

    def to_dds_order(self, targets):
        return np.asarray(targets)[self.config['dds_from_policy_index']]
