import {clamp,smooth,wrap,mix,norm,euler,yaw,matrix,slerp} from './math.js';

const slice=(array,start,n)=>Array.from(array.subarray(start,start+n));
export const AI_ACTION_PROBABILITIES=Object.freeze({punch:.40,leg:.40,guard:.20});
export function unpackMotions(bank){
  for(const [name,clip] of Object.entries(bank.clips)){
    clip.name=name;const bytes=Uint8Array.from(atob(clip.data),c=>c.charCodeAt(0));clip.values=new Float32Array(bytes.buffer);delete clip.data;
  }
  return bank;
}
export function sample(clip,time){
  const f=clamp(time/clip.duration*(clip.count-1),0,clip.count-1),a=Math.floor(f),b=Math.min(clip.count-1,a+1),u=f-a;
  const out=Array.from({length:35},(_,i)=>mix(clip.values[a*35+i],clip.values[b*35+i],u));
  return {q:out.slice(0,29),p:out.slice(29,32),angles:out.slice(32,35)};
}
function curve(values,phase){const f=phase*(values.length-1),a=Math.floor(f),b=Math.min(a+1,values.length-1),u=f-a;return Array.isArray(values[a])?values[a].map((x,i)=>mix(x,values[b][i],u)):mix(values[a],values[b],u);}

export class Planner{
  constructor(index,bank,seed=7,options={}){
    this.referenceFrames=options.referenceFrames||16;this.combatAllowed=options.combat?.allowed?new Set(options.combat.allowed):null;this.notice=null;
    this.index=index;this.manual=index===0;this.bank=bank;this.randomState=(seed+index*7919)>>>0;this.clip=null;this.clipStart=0;this.clipTempo=1;
    this.heading=0;this.freeHeading=null;this.velocity=[0,0];this.gaitPhase=0;this.gaitAmplitude=0;
    this.command=bank.guard.slice();this.entry=this.command.slice();this.sequence=[];this.pending=null;this.manualKeys=new Set();
    this.strikeBase=0;this.action='PICO READY';this.currentCommand=null;this.nextChoice=1.+index;
    this.lastAttack=null;this.scoreIdentity=0;this.guard=false;this.lastReference=null;this.handover=null;this.events=[];
    this.aiCategory=null;this.aiSelections=[];this.aiCategoryCounts={punch:0,leg:0,guard:0};
    this.warmupUntil=0;this.facingCorrection=0;
    this.kickSetup=null;this.kickAim=null;this.kickSetups=[];
  }
  random(){let x=this.randomState;x^=x<<13;x^=x>>>17;x^=x<<5;this.randomState=x>>>0;return this.randomState/4294967296;}
  select(values){return values[Math.min(values.length-1,Math.floor(this.random()*values.length))];}
  supports(name){
    const enabled=n=>!this.combatAllowed||this.combatAllowed.has(n);
    if(name==='guard')return Object.entries(this.bank.clips).some(([n,c])=>['defence','counter'].includes(c.family)&&!(this.bank.disabled_defences||[]).includes(n)&&enabled(n));
    return (this.bank.choices[name]||[]).some(c=>c.sequence.every(s=>enabled(s[0])));
  }
  request(name,now){
    if(!this.supports(name)){this.notice={until:now+1.5,text:'此模型未启用该类动作'};return;}
    this.notice=null;this.pending={name,time:now};
  }
  setManual(manual,now){
    if(this.manual===manual)return;
    this.manual=manual;this.pending=null;this.sequence=[];this.manualKeys.clear();this.freeHeading=null;this.guard=false;
    this.currentCommand=null;this.lastAttack=null;this.nextChoice=now;
    // Complete a kick's landing before the new driver takes another action.
    if(now>=this.warmupUntil&&this.clip?.family!=='kick')this.interrupt(now);
  }
  resetPose(heading,now){
    this.clip=null;this.sequence=[];this.pending=null;this.manualKeys.clear();this.guard=false;
    this.heading=heading;this.freeHeading=null;this.velocity=[0,0];this.gaitPhase=0;this.gaitAmplitude=0;
    this.command=this.bank.guard.slice();this.entry=this.command.slice();this.lastReference=null;this.handover=null;
    this.currentCommand=null;this.lastAttack=null;this.nextChoice=now+.3;this.warmupUntil=0;this.facingCorrection=0;
    this.kickSetup=null;this.kickAim=null;
    this.action=(this.manual?'YOU / ':'AI / ')+'PICO READY';
  }
  interrupt(now){
    if(this.lastReference)this.handover={time:now,q:this.lastReference.q[0].slice(),z:this.lastReference.p[0][2],r:this.lastReference.r[0].slice()};
    this.clip=null;this.sequence=[];this.action='READY';
    this.kickSetup=null;this.kickAim=null;
  }
  beginWarmup(now,until,state,opponent){
    this.currentCommand='warmup';this.aiCategory=null;this.sequence=[];this.warmupUntil=until;
    const names=['pico_warmup_forearms','pico_warmup_release'];
    const selected=[names[this.index],names[1-this.index]].find(n=>this.available(n,opponent));
    if(selected)this.start(selected,1,'body',now,state,opponent);
    this.nextChoice=until;
  }
  available(name,opponent){return (!this.combatAllowed||this.combatAllowed.has(name))&&opponent.clip?.name!==name;}
  selectAIAction(now,distance,opponent){
    // Draw the family first, independently of distance or incoming attacks.
    // Range may choose a variant inside that family, never a different family.
    const draw=this.random();
    const usable={punch:['light','heavy','combo'].some(c=>this.supports(c)),leg:['kick','flying_kick'].some(c=>this.supports(c)),guard:this.supports('guard')};
    const weights=Object.fromEntries(Object.entries(AI_ACTION_PROBABILITIES).map(([k,v])=>[k,usable[k]?v:0])),total=Object.values(weights).reduce((a,b)=>a+b,0);
    if(!total){this.nextChoice=now+.5;return;}
    const value=draw*total,category=value<weights.punch?'punch':value<weights.punch+weights.leg?'leg':'guard';
    let command='guard';
    if(category==='punch'){
      const variant=this.random();command=variant<.35?'light':variant<.65?'heavy':'combo';
    }else if(category==='leg')command=distance>.72&&this.random()<.30?'flying_kick':'kick';
    if(!this.supports(command))command=(category==='punch'?['light','heavy','combo']:category==='leg'?['kick','flying_kick']:['guard']).find(c=>this.supports(c));
    this.aiCategory=category;this.currentCommand=command;this.sequence=this.choose(command,opponent);this.nextChoice=now+.5;
    this.aiSelections.push({time_s:now,category,command,draw});
  }
  choose(command,opponent){
    if(command==='guard'){
      const names=Object.keys(this.bank.clips).filter(n=>['defence','counter'].includes(this.bank.clips[n].family)&&!(this.bank.disabled_defences||[]).includes(n)&&this.available(n,opponent));
      return names.length?[[this.select(names),1,'head']]:[];
    }
    const choices=(this.bank.choices[command]||[]).filter(c=>c.sequence.every(s=>!this.combatAllowed||this.combatAllowed.has(s[0]))&&this.available(c.sequence[0][0],opponent));
    let total=choices.reduce((v,c)=>v+c.weight,0),draw=this.random()*total;
    for(const c of choices){draw-=c.weight;if(draw<=0)return c.sequence.map(s=>s.slice());}
    return [];
  }
  start(name,tempo,zone,now,state,opponent){
    const clip=this.bank.clips[name];if(!clip||!this.available(name,opponent))return false;
    if(clip.family==='kick'&&(state.up<.86||state.root[2]<.55||state.support===0))return false;
    this.clip=clip;this.clipTempo=clip.family==='legacy_punch'?tempo:1;this.clipStart=now;this.clipHeading=this.heading;this.facingCorrection=0;
    if(clip.family==='kick'&&this.kickSetup?.name===name){
      this.clipHeading=this.kickSetup.heading;
      this.kickAim={...this.kickSetup};this.kickSetups.push({...this.kickSetup,started_at:now});this.kickSetup=null;
    }
    this.entry=this.command.slice();this.strikeBase+=100;this.zone=zone;this.scoreIdentity++;
    if(!this.manual&&this.aiCategory)this.aiCategoryCounts[this.aiCategory]++;
    this.events.push({time_s:now,method:name,command:this.currentCommand,tempo:clip.tempo||this.clipTempo,
      category:this.manual?null:this.aiCategory,kick_aim:clip.family==='kick'?this.kickAim:null});return true;
  }
  aimKick(clip,state,other,bearing,distance){
    const aim=clip.aim;
    if(!aim)return {heading:bearing,range:.76,heading_error:0,range_error:distance-.76};
    const target=(aim.height_m>1.22?(other.head||other.body):other.body)||other.root;
    const velocity=other.velocity||[0,0,0],lead=Math.min(.22,aim.impact_s*.3);
    const dx=target[0]-state.root[0]+clamp(velocity[0]*lead,-.10,.10),dy=target[1]-state.root[1]+clamp(velocity[1]*lead,-.10,.10);
    const direction=Math.atan2(dy,dx),normal=[Math.cos(direction),Math.sin(direction)];
    const targetOffset=(target[0]-other.root[0])*normal[0]+(target[1]-other.root[1])*normal[1];
    // Toe reach plus the target shell's near surface; reference tracking adds
    // uncertainty, so begin slightly inside the nominal contact radius.
    const range=clamp(aim.reach_m+.07-targetOffset,.52,.92);
    const heading=direction-aim.bearing_rad;
    return {heading,range,heading_error:wrap(heading-yaw(state.quaternion)),range_error:distance-range,target:target.slice(),bearing_offset:aim.bearing_rad};
  }
  strike(now,hand,limb){
    if(!this.clip)return null;
    const t=(now-this.clipStart)*this.clipTempo;
    for(let i=0;i<this.clip.strikes.length;i++){
      const s=this.clip.strikes[i];if(s.hand===hand&&(s.limb||'arm')===limb&&s.score_window_s[0]<=t&&t<s.score_window_s[1])
        return {...s,id:this.strikeBase+i,kind:this.clip.name};
    }
    return null;
  }
  gait(state,velocity,heading){
    const bank=this.bank,actual=yaw(state.quaternion),error=wrap(heading-actual),rate=clamp(error*4,-1.8,1.8);
    for(let i=0;i<2;i++)this.velocity[i]+=clamp(velocity[i]-this.velocity[i],-.084,.084);
    const c=Math.cos(actual),s=Math.sin(actual),local=[this.velocity[0]*c+this.velocity[1]*s,-this.velocity[0]*s+this.velocity[1]*c],speed=norm(local);
    let name=speed<.12&&Math.abs(error)>.12?(error>0?'pico_turn_left':'pico_turn_right'):Math.abs(local[1])>Math.abs(local[0])?(local[1]>0?'pico_sidestep_left':'pico_sidestep_right'):local[0]<-.02?'pico_walk_backward':speed>.8?'pico_run_forward':'pico_walk_forward';
    const gait=bank.gaits[name],ratio=clamp(speed/gait.nominal_speed,.6,1.45),cycle=gait.duration/ratio;
    this.gaitPhase=(this.gaitPhase+.02/cycle)%1;
    const requested=speed>.02?Math.min(1.12,speed/gait.nominal_speed):Math.min(1,Math.abs(error)*2);
    this.gaitAmplitude+=clamp(requested-this.gaitAmplitude,-.06,.06);
    const q=[],p=[],r=[],amp=this.gaitAmplitude;
    for(let k=0;k<this.referenceFrames;k++){
      const t=k/50,phase=(this.gaitPhase+t/cycle)%1,gq=curve(gait.q,phase),a=curve(gait.angles,phase),row=bank.guard.slice();
      for(let j=0;j<15;j++)row[j]+=amp*(gq[j]-bank.guard[j]);
      a[0]=bank.guard_angles[0]+amp*(a[0]-bank.guard_angles[0]);a[1]=bank.guard_angles[1]+amp*(a[1]-bank.guard_angles[1]);a[2]=actual+rate*(.06+t);
      q.push(row);p.push([state.root[0]+t*this.velocity[0],state.root[1]+t*this.velocity[1],bank.guard_root[2]+amp*(curve(gait.height,phase)-bank.guard_root[2])]);r.push(euler(a));
    }
    return {q,p,r};
  }
  update(now,state,other,opponent){
    const dx=other.root[0]-state.root[0],dy=other.root[1]-state.root[1],distance=Math.hypot(dx,dy),bearing=Math.atan2(dy,dx),actual=yaw(state.quaternion),keys=this.manualKeys;
    const warming=now<this.warmupUntil;
    let turn=warming?0:Number(keys.has('Q'))-Number(keys.has('E'));
    if(this.manual&&turn){if(this.freeHeading===null)this.freeHeading=actual;this.freeHeading+=turn*1.7*.02;}
    let desired=this.freeHeading===null?bearing:this.freeHeading;this.heading=actual+clamp(wrap(desired-actual),-.044,.044);
    const oldGuard=this.guard;this.guard=this.manual&&!warming&&keys.has('H');
    const kicking=this.clip?.family==='kick';
    if(this.clip&&now-this.clipStart>=this.clip.duration/this.clipTempo){this.clip=null;this.kickAim=null;this.nextChoice=now+.08;}
    if(this.manual&&!warming){
      if(this.guard&&!oldGuard&&!kicking){this.interrupt(now);this.pending=null;}
      if(!this.guard&&oldGuard&&this.currentCommand==='guard'&&!kicking)this.interrupt(now);
      if(this.pending&&!this.guard&&!kicking&&(!this.clip||now-this.clipStart>=.10)){
        const sequence=this.choose(this.pending.name,opponent);
        if(sequence.length){this.interrupt(now);this.currentCommand=this.pending.name;this.sequence=sequence;this.pending=null;}
      }
      if(this.guard&&!this.clip){this.currentCommand='guard';this.sequence=this.choose('guard',opponent);}
    }else if(!this.manual&&!warming&&!this.clip&&!this.sequence.length&&now>=this.nextChoice&&distance<1.12&&state.up>.86){
      this.selectAIAction(now,distance,opponent);
    }
    if(!this.manual&&!this.clip&&this.sequence.length&&!this.available(this.sequence[0][0],opponent)){
      // A queued kick can wait for its support foot without being redrawn as
      // a punch. If its source becomes busy, retry only the selected command.
      this.sequence=this.choose(this.currentCommand,opponent);
    }
    let preparing=false;
    if(!this.clip&&this.sequence.length){
      const [name,tempo,zone]=this.sequence[0],next=this.bank.clips[name];
      let canStart=true;
      if(next.family==='kick'&&next.aim){
        const aim=this.aimKick(next,state,other,bearing,distance);
        const since=this.kickSetup?.name===name?this.kickSetup.since:now;
        this.kickSetup={name,since,...aim};desired=aim.heading;
        this.heading=actual+clamp(wrap(desired-actual),-.044,.044);
        const aligned=Math.abs(aim.heading_error)<.18,spaced=Math.abs(aim.range_error)<.11;
        const supported=state.support>0&&state.up>.9&&norm((state.velocity||[0,0]).slice(0,2))<.45;
        // Manual players own their range and may intentionally kick into air.
        // Briefly aim, then execute without waiting indefinitely for AI spacing.
        canStart=this.manual?state.support>0&&state.up>.9&&now-since>=.16&&(aligned||now-since>=.45):aligned&&spaced&&supported&&now-since>=.16;
        preparing=!canStart;
      }
      if(canStart&&this.start(name,tempo,zone,now,state,opponent))this.sequence.shift();
    }
    let velocity;
    if(warming){velocity=[0,0];}
    else if(this.manual){
      let forward=Number(keys.has('W'))-Number(keys.has('S')),side=Number(keys.has('A'))-Number(keys.has('D'));const n=Math.max(1,Math.hypot(forward,side));forward/=n;side/=n;
      const speed=forward*(forward>=0?1.35:1.15),lateral=side*1.05;
      velocity=[Math.cos(desired)*speed-Math.sin(desired)*lateral,Math.sin(desired)*speed+Math.cos(desired)*lateral];
    }else{
      const preferred=preparing?this.kickSetup.range:opponent.clip?.family==='kick'?.78:.46;
      const radial=clamp((distance-preferred)*2.6,preparing?-.7:-.5,preparing?.7:1.2),lateral=preparing?0:distance<.72?.10*Math.sin(now*2):0;
      velocity=[Math.cos(bearing)*radial-Math.sin(bearing)*lateral,Math.sin(bearing)*radial+Math.cos(bearing)*lateral];
    }
    const inward=distance>1e-6?(velocity[0]*dx+velocity[1]*dy)/distance:0;
    if(distance<.34&&inward>0){velocity[0]-=dx/distance*inward;velocity[1]-=dy/distance*inward;}
    for(let i=0;i<2;i++)if(Math.abs(state.root[i])>2.7&&state.root[i]*velocity[i]>0)velocity[i]=0;
    let ref=this.gait(state,velocity,desired);
    if(this.clip){
      const clip=this.clip,elapsed=(now-this.clipStart)*this.clipTempo,first=sample(clip,elapsed),full=!['legacy_punch','rapid'].includes(clip.family);
      const spinning=clip.family==='kick'&&(clip.airborne||clip.name==='back_kick');
      const aimedKick=clip.family==='kick'&&this.kickAim;
      if(aimedKick&&elapsed<clip.aim.impact_s-.16){
        const aim=this.aimKick(clip,state,other,bearing,distance);
        this.clipHeading+=clamp(wrap(aim.heading-this.clipHeading),-.012,.012);
      }
      if(full&&!spinning&&!aimedKick){
        const desiredCorrection=wrap(bearing-(this.clipHeading+first.angles[2]));
        this.facingCorrection+=clamp(wrap(desiredCorrection-this.facingCorrection),-.044,.044);
      }
      for(let k=0;k<this.referenceFrames;k++){
        const t=k/50,source=sample(clip,elapsed+t*this.clipTempo),intro=smooth((elapsed+t*this.clipTempo)/(clip.transition||.14)),outro=smooth((elapsed+t*this.clipTempo-clip.duration+(clip.transition||.14))/(clip.transition||.14));
        let q=source.q.map((v,j)=>mix(this.entry[j],v,intro));q=q.map((v,j)=>mix(v,this.bank.guard[j],outro));
        if(full){
          // Re-aim grounded references continuously. A rotating kick retains
          // its source turn while airborne and reacquires the opponent on landing.
          const angle=this.clipHeading+(spinning||aimedKick?0:this.facingCorrection),offset=[source.p[0]-first.p[0],source.p[1]-first.p[1]];
          const p=[state.root[0]+Math.cos(angle)*offset[0]-Math.sin(angle)*offset[1],state.root[1]+Math.sin(angle)*offset[0]+Math.cos(angle)*offset[1],source.p[2]];
          source.angles[2]+=angle;let r=euler(source.angles);
          if(clip.family!=='kick'&&(norm(velocity)>.02||Math.abs(turn)>.01)){
            for(let j=0;j<12;j++)q[j]=ref.q[k][j]+.35*(q[j]-this.bank.guard[j]);p[0]=ref.p[k][0];p[1]=ref.p[k][1];p[2]=ref.p[k][2]+.35*(p[2]-this.bank.guard_root[2]);
            source.angles[2]=yaw(ref.r[k]);r=euler(source.angles);
          }
          ref.q[k]=q;ref.p[k]=p;ref.r[k]=r;
        }else{for(let j=15;j<29;j++)ref.q[k][j]=q[j];for(let j=12;j<15;j++)ref.q[k][j]=.65*q[j]+.35*ref.q[k][j];}
      }
      this.action=(this.manual?'YOU / ':'AI / ')+clip.name.toUpperCase();
    }else this.action=(this.manual?'YOU / ':'AI / ')+(warming?'WARMUP / READY':preparing?'KICK SETUP / '+this.kickSetup.name.toUpperCase():norm(velocity)>.03?'MOVE':'PICO READY');
    if(this.handover){
      for(let k=0;k<this.referenceFrames;k++){
        const blend=smooth((now-this.handover.time+k/50)/.12);ref.q[k]=ref.q[k].map((v,j)=>mix(this.handover.q[j],v,blend));
        ref.p[k][2]=mix(this.handover.z,ref.p[k][2],blend);ref.r[k]=slerp(this.handover.r,ref.r[k],blend);
      }
      if(now-this.handover.time>=.12)this.handover=null;
    }
    const rotating=preparing||this.clip?.family==='kick';
    if(!rotating){
      const chestError=wrap(bearing-(state.facing_yaw??actual));
      const correction=clamp(chestError*.4,-.22,.22);
      for(const row of ref.q)row[12]=clamp(row[12]+correction,-.65,.65);
    }
    ref.q=ref.q.map(row=>row.map((v,j)=>clamp(v,this.bank.limits[j][0]+.02,this.bank.limits[j][1]-.02)));
    if(this.notice&&now<this.notice.until)this.action+=' · '+this.notice.text;
    this.command=ref.q[0].slice();this.lastReference=ref;return ref;
  }
}
