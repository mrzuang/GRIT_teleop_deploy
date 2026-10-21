import * as ort from './vendor/ort.wasm.min.mjs';
import {qi,qm,rotate,rot6,referenceFeatures} from './math.js';
const dimensions=[3,3,6,29,29,29],offsets=[0,2,3,5,7,8,10,12,13];

export class GritState{
  constructor(config){this.config=config;this.histories=dimensions.map(d=>new Float32Array(d*10));this.last=new Float32Array(29);this.target=new Float64Array(config.default_joint_pos);}
  update(state,refquat){
    const terms=[state.gyro,rotate(qi(state.quaternion),[0,0,-1]),rot6(qm(qi(state.quaternion),refquat)),
      state.joints.map((x,i)=>Math.fround(x)-this.config.default_joint_pos[i]),state.joint_velocity,this.last];
    this.histories.forEach((h,i)=>{h.copyWithin(0,dimensions[i]);h.set(terms[i],h.length-dimensions[i]);});
  }
  reset(state,refquat){this.histories.forEach(h=>h.fill(0));this.last.fill(0);for(let i=0;i<10;i++)this.update(state,refquat);}
  history(){const output=new Float32Array(990);let index=0;this.histories.forEach(h=>{output.set(h,index);index+=h.length;});return output;}
  action(values){this.last.set(values);for(let i=0;i<29;i++)this.target[i]=this.config.default_joint_pos[i]+values[i]*this.config.action_scale[i]-this.config.encoder_offset[i];}
}

export async function loadPolicy(config){
  ort.env.wasm.numThreads=1;ort.env.wasm.proxy=false;ort.env.wasm.wasmPaths=new URL('./vendor/',import.meta.url).href;
  const modelURL=new URL(`./assets/${config.policy_file||'policy.onnx'}`,import.meta.url);
  modelURL.searchParams.set('sha256',config.policy_sha256);
  const response=await fetch(modelURL);if(!response.ok)throw new Error('GRIT策略加载失败');
  const bytes=new Uint8Array(await response.arrayBuffer());
  let verifiedHash=null;
  if(globalThis.crypto?.subtle){
    verifiedHash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');
    if(verifiedHash!==config.policy_sha256)throw new Error('GRIT策略哈希不匹配，请强制刷新页面');
  }
  const session=await ort.InferenceSession.create(bytes,{executionProviders:['wasm'],graphOptimizationLevel:'all'});
  if(!['reference_context','proprio_history'].every(n=>session.inputNames.includes(n)))throw new Error('GRIT模型接口不匹配');
  return {session,async infer(states,observations,references){
    const contexts=new Float32Array(states.length*630),histories=new Float32Array(states.length*990);
    states.forEach((state,i)=>{
      const features=referenceFeatures(references[i]);offsets.forEach((f,j)=>contexts.set(features[f],i*630+j*70));
      state.update(observations[i],references[i].r[0]);histories.set(state.history(),i*990);
    });
    const start=performance.now(),result=await session.run({reference_context:new ort.Tensor('float32',contexts,[states.length,9,70]),proprio_history:new ort.Tensor('float32',histories,[states.length,990])});
    const values=result.actions.data;if(values.length!==states.length*29||!values.every(Number.isFinite))throw new Error('GRIT输出无效');
    states.forEach((state,i)=>state.action(values.slice(i*29,(i+1)*29)));return performance.now()-start;
  },async validate(fixtures){
    const state=new GritState(config);let historyError=0,actionError=0;
    fixtures.forEach((f,index)=>{
      const obs={gyro:f.gyro,quaternion:f.quaternion,joints:f.joints,joint_velocity:f.joint_velocity};
      if(index===0)state.reset(obs,f.reference_quaternion);
      state.last.set(f.last_action);state.update(obs,f.reference_quaternion);
      const values=state.history();historyError=Math.max(historyError,...values.map((x,i)=>Math.abs(x-f.history[i])));
    });
    for(const fixture of fixtures){
      if(fixture.policy_sha256!==config.policy_sha256)throw new Error('GRIT校验样本版本不匹配，请强制刷新页面');
      const reference=new Float32Array(630);offsets.forEach((f,i)=>reference.set(fixture.reference_features[f],i*70));
      const result=await session.run({reference_context:new ort.Tensor('float32',reference,[1,9,70]),proprio_history:new ort.Tensor('float32',new Float32Array(fixture.history),[1,990])});
      actionError=Math.max(actionError,...result.actions.data.map((x,i)=>Math.abs(x-fixture.expected_actions[i])));
    }
    const batch=fixtures.length,reference=new Float32Array(batch*630),history=new Float32Array(batch*990);
    fixtures.forEach((f,i)=>{offsets.forEach((k,j)=>reference.set(f.reference_features[k],i*630+j*70));history.set(f.history,i*990);});
    const batched=await session.run({reference_context:new ort.Tensor('float32',reference,[batch,9,70]),proprio_history:new ort.Tensor('float32',history,[batch,990])});
    fixtures.forEach((f,i)=>{for(let j=0;j<29;j++)actionError=Math.max(actionError,Math.abs(batched.actions.data[i*29+j]-f.expected_actions[j]));});
    if(historyError>1e-5||actionError>5e-4)throw new Error(`GRIT接口校验未通过: ${historyError}, ${actionError}`);
    return {history_error:historyError,action_error:actionError,policy_sha256:config.policy_sha256,
      loaded_sha256:verifiedHash,validated_batches:[1,batch]};
  }};
}
