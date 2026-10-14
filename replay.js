import {clamp,slerp} from './math.js';

// Recorded physical poses only. Playback never calls or rewinds MuJoCo.
export class HighlightReplay{
  constructor(){this.round=null;this.speed=.25;this.reset();}
  reset(){this.frames=[];this.events=[];this.seen=new Set();this.clips=[];this.active=false;this.paused=false;this.time=0;this.clipIndex=0;this.lastWall=null;this.output=null;}
  get available(){return this.frames.length>1&&this.frames.at(-1).time-this.frames[0].time>.5;}
  record(packet){
    let reset=false;
    if(packet.round_index!==this.round||packet.control_mode==='showcase'){
      reset=this.frames.length>0||this.active;this.reset();this.round=packet.round_index;
    }
    if(packet.control_mode==='showcase'||!packet.poses||!packet.started||this.active)return reset;
    const last=this.frames.at(-1);
    if(!last||packet.time>last.time||packet.pose_revision!==last.revision){
      this.frames.push({time:packet.time,revision:packet.pose_revision,poses:packet.poses,score:packet.score.slice(),roundTime:packet.round_time||0,actions:packet.actions.slice()});
      if(this.frames.length>4500)this.frames.shift();
    }
    for(const event of packet.events||[]){
      if(event.kind!=='hit'||!['head','body','thigh'].includes(event.zone))continue;
      const key='hit:'+event.id;if(this.seen.has(key))continue;this.seen.add(key);
      this.events.push({time:event.time_s,revision:packet.pose_revision,caption:`${event.player===0?'红方':'蓝方'} · ${{head:'头部',body:'躯干',thigh:'大腿'}[event.zone]}命中`});
    }
    if(packet.result&&!this.seen.has('result')&&['knockdown','double_knockdown','balance_tko'].includes(packet.result.reason)){
      this.seen.add('result');this.events.push({time:packet.result.time_s,revision:packet.pose_revision,caption:packet.result.winner===null?'双方倒地':`击倒 · ${packet.result.winner===0?'红方':'蓝方'}获胜`});
    }
    return reset;
  }
  start(now){
    if(!this.available)return false;
    const first=this.frames[0],last=this.frames.at(-1);this.clips=[];
    const moments=this.events.filter(e=>e.time>=first.time&&e.time<=last.time).slice(-6);
    if(!moments.length)moments.push({time:last.time-1,revision:last.revision,caption:'最近动作 · 尚无有效命中'});
    for(const event of moments){
      const frames=this.frames.filter(f=>f.revision===event.revision);if(frames.length<2)continue;
      const start=Math.max(frames[0].time,event.time-1.5),end=Math.min(frames.at(-1).time,event.time+1.1);if(end-start<.3)continue;
      const previous=this.clips.at(-1);
      if(previous&&previous.revision===event.revision&&start<=previous.end+.15){previous.end=Math.max(previous.end,end);previous.caption=event.caption;}
      else this.clips.push({start,end,revision:event.revision,caption:event.caption});
    }
    if(!this.clips.length)return false;
    this.active=true;this.paused=false;this.clipIndex=0;this.time=this.clips[0].start;this.lastWall=now;this.output=new Float32Array(last.poses.length);return true;
  }
  stop(){this.active=false;this.paused=false;this.lastWall=null;}
  tick(now){
    if(!this.active)return null;
    const dt=Math.max(0,Math.min(.1,(now-this.lastWall)/1000));this.lastWall=now;
    if(!this.paused)this.time+=dt*this.speed;
    let clip=this.clips[this.clipIndex];
    if(this.time>clip.end){
      if(++this.clipIndex>=this.clips.length){this.active=false;return null;}
      clip=this.clips[this.clipIndex];this.time=clip.start;
    }
    const frames=this.frames;let lo=0,hi=frames.length-1;
    while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(frames[mid].time<=this.time)lo=mid;else hi=mid-1;}
    let a=frames[lo],b=frames[Math.min(frames.length-1,lo+1)];
    if(a.revision!==clip.revision)a=frames.find(f=>f.revision===clip.revision);
    if(b.revision!==clip.revision)b=a;
    const u=clamp((this.time-a.time)/(b.time-a.time||1),0,1);
    for(let i=0;i<a.poses.length;i+=7){
      for(let j=0;j<3;j++)this.output[i+j]=a.poses[i+j]+u*(b.poses[i+j]-a.poses[i+j]);
      this.output.set(slerp(Array.from(a.poses.subarray(i+3,i+7)),Array.from(b.poses.subarray(i+3,i+7)),u),i+3);
    }
    return {...a,poses:this.output,time:this.time,caption:clip.caption,progress:(this.clipIndex+(this.time-clip.start)/(clip.end-clip.start))/this.clips.length};
  }
}
