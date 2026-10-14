import * as ort from './vendor/ort.wasm.min.mjs';
import {loadPolicy,GritState} from './grit.js';
import {qi,qm,rotate,rot6,matrix,yaw,euler,clamp} from './math.js';

export const MODEL_CHOICES=[{id:'grit',label:'GRIT'},{id:'sonic',label:'SONIC'},{id:'heft',label:'HEFT'}];
const known=new Set(MODEL_CHOICES.map(m=>m.id));
const read=async name=>{const r=await fetch(new URL(name,import.meta.url),{cache:'no-store'});if(!r.ok)throw Error(`无法加载 ${name}`);return r.json();};
async function digest(bytes){if(!crypto?.subtle)throw Error('模型校验需要 HTTPS 或 localhost');return [...new Uint8Array(await crypto.subtle.digest('SHA-256',bytes))].map(b=>b.toString(16).padStart(2,'0')).join('');}
async function modelBytes(description,stage,label){
  const out=new Uint8Array(description.bytes);let offset=0;
  for(const part of description.parts){
    stage(`加载 ${label} · ${Math.round(offset/out.length*100)}%`);
    const url=new URL('./assets/controllers/'+part.file,import.meta.url);url.searchParams.set('sha256',part.sha256);
    const response=await fetch(url);if(!response.ok)throw Error(`${label} 下载失败 (${response.status})`);
    const bytes=new Uint8Array(await response.arrayBuffer());
    if(bytes.length!==part.bytes||await digest(bytes)!==part.sha256)throw Error(`${label} 文件校验失败，请刷新重试`);
    out.set(bytes,offset);offset+=bytes.length;
  }
  if(offset!==description.bytes||await digest(out)!==description.sha256)throw Error(`${label} 模型校验失败`);
  return out;
}
const assertFinite=(values,n,label)=>{if(values.length!==n||!values.every(Number.isFinite))throw Error(`${label} 输出无效`);};
const difference=(a,b)=>Math.max(...a.map((x,i)=>Math.abs(x-b[i])));
class History{
  constructor(width,length,newest=false){this.width=width;this.length=length;this.newest=newest;this.values=new Float32Array(width*length);}
  push(row){const h=this.values,w=this.width;if(this.newest){h.copyWithin(w,0,h.length-w);h.set(row,0);}else{h.copyWithin(0,w);h.set(row,h.length-w);}}
  fill(row){for(let i=0;i<this.length;i++)this.values.set(row,i*this.width);}
  take(steps){const out=[];for(const i of steps)out.push(...this.values.subarray(i*this.width,(i+1)*this.width));return out;}
}
class ControllerState{
  constructor(meta){this.meta=meta;this.policyId=meta.id;this.kp=meta.kp;this.kd=meta.kd;this.referenceFrames=meta.reference_frames;this.target=new Float64Array(meta.default_joint_pos);this.last=new Float32Array(29);this.inferenceMs=0;}
  action(values){this.last.set(values);for(let j=0;j<29;j++)this.target[j]=this.meta.default_joint_pos[j]+this.last[this.meta.from_policy[j]]*this.meta.action_scale[j];}
}
export class SonicState extends ControllerState{
  constructor(meta){super(meta);this.histories=[3,29,29,29,3].map(n=>new History(n,10));}
  terms(s){const m=this.meta;return [s.gyro,m.to_policy.map(i=>s.joints[i]-m.default_joint_pos[i]),m.to_policy.map(i=>s.joint_velocity[i]),this.last,rotate(qi(s.quaternion),[0,0,-1])];}
  reset(s){this.last.fill(0);this.terms(s).forEach((row,i)=>this.histories[i].fill(row));this.target.set(s.joints);}
  encoder(s,ref){
    this.terms(s).forEach((row,i)=>this.histories[i].push(row));
    const out=new Float32Array(1751),headingInv=qi(euler([0,0,yaw(s.quaternion)])),m=this.meta,n=ref.q.length;
    if(n<46)throw Error('SONIC 需要 46 帧连续参考');
    for(let k=0;k<10;k++){
      const i=k*5,lo=Math.max(0,i-1),hi=Math.min(n-1,i+1),scale=50/(hi-lo);
      out.set(m.to_policy.map(j=>ref.q[i][j]),4+k*29);
      out.set(m.to_policy.map(j=>(ref.q[hi][j]-ref.q[lo][j])*scale),294+k*29);
      out.set(rot6(qm(headingInv,ref.r[i])),584+k*6);
    }return out;
  }
  decoder(tokens){const out=new Float32Array(994);out.set(tokens);let offset=64;for(const h of this.histories){out.set(h.values,offset);offset+=h.values.length;}return out;}
}
export class HeftState extends ControllerState{
  constructor(meta){super(meta);this.steps=[meta.root_angvel_history_steps,meta.projected_gravity_history_steps,meta.joint_pos_history_steps,meta.joint_vel_history_steps];this.histories=[3,3,29,29].map((n,i)=>new History(n,Math.max(...this.steps[i])+1,true));this.previous=new History(29,meta.prev_action_steps,true);this.referenceHistory=[];this.boot=meta.boot_indicator_max;}
  terms(s){const g=rotate(qi(s.quaternion),[0,0,-1]),length=Math.hypot(...g)+1e-8;return [s.gyro,g.map(x=>x/length),this.meta.to_policy.map(i=>Math.fround(s.joints[i])),this.meta.to_policy.map(i=>Math.fround(s.joint_velocity[i]))];}
  reset(s){this.last.fill(0);this.terms(s).forEach((row,i)=>this.histories[i].fill(row));this.previous.fill(this.last);this.referenceHistory=[];this.boot=this.meta.boot_indicator_max;this.target.set(s.joints);}
  observation(s,ref){
    const m=this.meta,first={q:ref.q[0].slice(),p:ref.p[0].slice(),r:ref.r[0].slice()};
    if(!this.referenceHistory.length)this.referenceHistory=Array.from({length:16},()=>first);
    const frames=m.future_steps.map(i=>i<0?this.referenceHistory[Math.max(0,this.referenceHistory.length+i)]:{q:ref.q[i],p:ref.p[i],r:ref.r[i]});
    this.referenceHistory.push(first);if(this.referenceHistory.length>16)this.referenceHistory.shift();
    this.boot=Math.max(0,this.boot-1);const out=[this.boot/m.boot_indicator_max];
    if(m.include_root_pos)for(const f of frames.slice(1))out.push(...rotate(qi(first.r),f.p.map((v,j)=>v-first.p[j])));
    for(const f of frames){const r=matrix(qm(qi(s.quaternion),f.r));out.push(r[0],r[3],r[6],r[1],r[4],r[7]);}
    for(const f of frames)out.push(...m.to_policy.map(i=>f.q[i]));
    for(const f of frames)out.push(...m.to_policy.map(i=>f.q[i]-Math.fround(s.joints[i])));
    for(const f of frames)out.push(f.p[2]);
    for(const f of frames)out.push(...rotate(qi(f.r),[0,0,-1]));
    this.terms(s).forEach((row,i)=>{this.histories[i].push(row);out.push(...this.histories[i].take(this.steps[i]));});
    this.previous.push(this.last);out.push(...this.previous.values);
    if(out.length!==1693)throw Error(`HEFT 观测维度错误 ${out.length}`);return new Float32Array(out);
  }
  action(values){super.action(values.map(x=>clamp(x,-this.meta.action_clip,this.meta.action_clip)));}
}
export class ControllerBank{
  constructor(config,stage){this.config=config;this.stage=stage;this.models=new Map();this.fixtures=null;this.compatibility={};}
  async initialize(){
    this.compatibility=await read('./assets/controllers/compatibility.json');
    await this.ensure('grit');return this;
  }
  async ensure(id){
    if(!known.has(id))throw Error('未知控制器');if(this.models.has(id))return this.models.get(id);
    this.stage(`准备 ${id.toUpperCase()} 控制器`);
    let controller;
    if(id==='grit'){
      const policy=await loadPolicy(this.config),config=this.config;
      const contract=await policy.validate(await read('./assets/contract-fixtures.json'));
      controller={id,label:'GRIT',version:String(config.policy_iteration),referenceFrames:16,contract,
        createState(){const state=new GritState(config);Object.assign(state,{policyId:id,kp:config.kp,kd:config.kd,referenceFrames:16});return state;},infer:policy.infer};
    }else{
      const meta=await read(`./assets/controllers/${id}.json`);let sessions;
      if(id==='sonic'){
        const encoder=await modelBytes(meta.encoder,this.stage,'SONIC 编码器');
        const a=await ort.InferenceSession.create(encoder,{executionProviders:['wasm'],graphOptimizationLevel:'all'});
        const decoder=await modelBytes(meta.decoder,this.stage,'SONIC 控制器');
        const b=await ort.InferenceSession.create(decoder,{executionProviders:['wasm'],graphOptimizationLevel:'all'});sessions=[a,b];
      }else{
        const bytes=await modelBytes(meta.policy,this.stage,'HEFT 控制器');sessions=[await ort.InferenceSession.create(bytes,{executionProviders:['wasm'],graphOptimizationLevel:'all'})];
      }
      controller={id,label:meta.label,version:meta.version,referenceFrames:meta.reference_frames,meta,sessions,contract:null,
        createState:()=>id==='sonic'?new SonicState(meta):new HeftState(meta),
        async infer(states,observations,references){
          const start=performance.now();
          if(id==='sonic'){
            const batch=new Float32Array(states.length*994);
            for(let i=0;i<states.length;i++){
              const inputs=states[i].encoder(observations[i],references[i]);
              const encoded=await sessions[0].run({obs_dict:new ort.Tensor('float32',inputs,[1,1751])});
              const tokens=encoded.encoded_tokens.data;assertFinite(tokens,64,'SONIC token');batch.set(states[i].decoder(tokens),i*994);
            }
            const values=(await sessions[1].run({obs_dict:new ort.Tensor('float32',batch,[states.length,994])})).action.data;
            assertFinite(values,states.length*29,id);states.forEach((s,i)=>s.action(values.slice(i*29,(i+1)*29)));
          }else{
            for(let i=0;i<states.length;i++){
              const output=(await sessions[0].run({policy:new ort.Tensor('float32',states[i].observation(observations[i],references[i]),[1,1693])})).action_alias_6.data;
              assertFinite(output,29,id);states[i].action(output);
            }
          }
          const elapsed=performance.now()-start;states.forEach(s=>s.inferenceMs=elapsed/states.length);return elapsed;
        }
      };
      if(!this.fixtures)this.fixtures=await read('./assets/controllers/contracts.json');
      this.stage(`校验 ${meta.label} 原始接口`);const state=controller.createState();let inputError=0,actionError=0;
      for(let i=0;i<this.fixtures.length;i++){
        const f=this.fixtures[i];if(i===0)state.reset(f.state);
        if(id==='sonic'){
          const input=state.encoder(f.state,f.reference);inputError=Math.max(inputError,difference(input,f.sonic.encoder));
          const result=await sessions[0].run({obs_dict:new ort.Tensor('float32',input,[1,1751])});
          const decoder=state.decoder(result.encoded_tokens.data);inputError=Math.max(inputError,difference(decoder,f.sonic.decoder));
          const output=(await sessions[1].run({obs_dict:new ort.Tensor('float32',decoder,[1,994])})).action.data;
          actionError=Math.max(actionError,difference(output,f.sonic.actions));state.action(output);
        }else{
          const input=state.observation(f.state,f.reference);inputError=Math.max(inputError,difference(input,f.heft.observation));
          const output=(await sessions[0].run({policy:new ort.Tensor('float32',input,[1,1693])})).action_alias_6.data;
          actionError=Math.max(actionError,difference(output,f.heft.actions));state.action(output);
        }
      }
      if(inputError>5e-4||actionError>5e-4)throw Error(`${meta.label} 接口校验失败 ${inputError}, ${actionError}`);
      controller.contract={input_error:inputError,action_error:actionError,fixtures:this.fixtures.length,backend:'ONNX Runtime Web WASM',
        policy_sha256:(meta.decoder||meta.policy).sha256,encoder_sha256:meta.encoder?.sha256,version:meta.version};
    }
    controller.combat=this.compatibility[id]||{allowed:null,excluded:[]};this.models.set(id,controller);return controller;
  }
  get(id){const model=this.models.get(id);if(!model)throw Error(`${id} 尚未加载`);return model;}
  async infer(states,observations,references){
    const groups=new Map();states.forEach((s,i)=>{if(!groups.has(s.policyId))groups.set(s.policyId,[]);groups.get(s.policyId).push(i);});
    let elapsed=0;for(const [id,indices] of groups)elapsed+=await this.get(id).infer(indices.map(i=>states[i]),indices.map(i=>observations[i]),indices.map(i=>references[i]));return elapsed;
  }
}
