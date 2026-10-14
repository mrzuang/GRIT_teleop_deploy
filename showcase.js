import {GritState} from './grit.js';
import {sample} from './planner.js';
import {euler,mix,smooth,matrix,clamp} from './math.js';

const categories=['guard','movement','light','legacy_punch','heavy','combo','rapid','counter','defence','kick','taunt','warmup','victory'];
export class Showcase{
  static async create(mj,bank){
    const config=await (await fetch(new URL('./assets/showcase-config.json',import.meta.url),{cache:'no-store'})).json();
    const response=await fetch(new URL('./assets/showcase.mjb.gz',import.meta.url));
    const bytes=new Uint8Array(await new Response(response.body.pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
    const vfs=new mj.MjVFS();vfs.addBuffer('showcase.mjb',bytes);const model=mj.MjModel.from_binary_path('showcase.mjb',vfs);vfs.delete();
    return new Showcase(mj,model,config,bank);
  }
  constructor(mj,model,config,bank){
    this.mj=mj;this.model=model;this.config=config;this.bank=bank;this.data=new mj.MjData(model);
    this.control=new GritState(config);this.index=0;this.lap=1;this.completed=0;this.poseRevision=0;this.inferenceMs=0;
    this.names=Object.keys(bank.clips).filter(n=>!(bank.disabled_defences||[]).includes(n)&&!n.includes('flying_knee'));
    this.names.sort((a,b)=>categories.indexOf(bank.clips[a].family)-categories.indexOf(bank.clips[b].family));
    this.list=this.names.map(name=>({id:name,label:bank.clips[name].label||name,family:bank.clips[name].family}));
    this.select(0,true);
  }
  state(){
    const a=this.config.actor,d=this.data,qpos=d.qpos,qvel=d.qvel,sensor=d.sensordata;
    const root=Array.from(qpos.slice(0,7)),quaternion=root.slice(3);
    return {root,quaternion,joints:a.qadr.map(i=>qpos[i]),joint_velocity:a.vadr.map(i=>qvel[i]),
      gyro:Array.from(sensor.slice(a.gyro_adr,a.gyro_adr+3)),velocity:Array.from(qvel.slice(0,3)),up:matrix(quaternion)[8],torso_up:d.xmat[a.torso*9+8]};
  }
  select(index,restart=false){
    const time=restart?0:this.data.time;
    if(restart){this.completed=0;this.lap=1;}
    this.index=((index%this.names.length)+this.names.length)%this.names.length;
    this.clip=this.bank.clips[this.names[this.index]];this.startedAt=time+.6;
    this.mj.mj_resetData(this.model,this.data);this.data.qpos.set(this.config.initial_qpos);this.data.time=time;this.mj.mj_forward(this.model,this.data);
    const state=this.state();this.control=new GritState(this.config);this.control.reset(state,state.quaternion);this.poseRevision++;
  }
  reference(){
    const q=[],p=[],r=[],now=this.data.time,elapsed=now-this.startedAt,clip=this.clip,state=this.state();
    const first=sample(clip,Math.max(0,elapsed));
    for(let k=0;k<16;k++){
      const t=elapsed+k/50,source=sample(clip,Math.max(0,t)),intro=smooth(t/.18),outro=smooth((t-clip.duration+.18)/.18);
      const row=source.q.map((x,i)=>mix(mix(this.bank.guard[i],x,intro),this.bank.guard[i],outro));
      const dx=source.p[0]-first.p[0],dy=source.p[1]-first.p[1],heading=this.config.heading;
      const root=[state.root[0]+Math.cos(heading)*dx-Math.sin(heading)*dy,state.root[1]+Math.sin(heading)*dx+Math.cos(heading)*dy,
        mix(mix(this.bank.guard_root[2],source.p[2],intro),this.bank.guard_root[2],outro)];
      const angles=source.angles.map((x,i)=>mix(mix(this.bank.guard_angles[i],x,intro),this.bank.guard_angles[i],outro));
      // A turn ends facing its new direction. Reversing its accumulated yaw
      // inside the short exit blend would command an abrupt extra spin.
      angles[2]=mix(this.bank.guard_angles[2],source.angles[2],intro)+heading;
      q.push(row.map((v,i)=>clamp(v,this.bank.limits[i][0]+.02,this.bank.limits[i][1]-.02)));p.push(root);r.push(euler(angles));
    }
    return {q,p,r};
  }
  async step(policy){
    if(this.data.time>=this.startedAt+this.clip.duration+.8){
      this.completed++;if(this.index===this.names.length-1)this.lap++;
      this.select(this.index+1);
    }
    this.inferenceMs=await policy.infer([this.control],[this.state()],[this.reference()]);
    const a=this.config.actor;
    for(let step=0;step<40;step++){
      if(step%2===0){
        const q=this.data.qpos,v=this.data.qvel,ctrl=this.data.ctrl;
        for(let j=0;j<29;j++){const range=this.config.ctrlrange[a.aadr[j]];ctrl[a.aadr[j]]=clamp(this.config.kp[j]*(this.control.target[j]-q[a.qadr[j]])-this.config.kd[j]*v[a.vadr[j]],range[0],range[1]);}
      }
      this.mj.mj_step(this.model,this.data);
    }
    if(!Array.from(this.data.qpos).every(Number.isFinite))throw new Error('展示状态异常，请重开');
  }
  snapshot(){
    const d=this.data,xpos=d.xpos,xquat=d.xquat,poses=new Float32Array(this.model.nbody*7);
    for(let i=0;i<this.model.nbody;i++){for(let j=0;j<3;j++)poses[i*7+j]=xpos[i*3+j];for(let j=0;j<4;j++)poses[i*7+3+j]=xquat[i*4+j];}
    const elapsed=d.time-this.startedAt;
    return {poses,time:d.time,scene:'./assets/showcase-scene.json.gz',scene_lite:'./assets/showcase-scene-lite.json.gz',
      showcase:{index:this.index,total:this.names.length,id:this.clip.name,label:this.clip.label||this.clip.name,
        family:this.clip.family,progress:clamp(elapsed/this.clip.duration,0,1),duration:this.clip.duration,
        phase:elapsed<0?'准备':elapsed>this.clip.duration?'收式':'播放',lap:this.lap,completed:this.completed,list:this.list},
      inference_ms:this.inferenceMs,pose_revision:this.poseRevision,actions:[this.clip.label||this.clip.name,'']};
  }
}
