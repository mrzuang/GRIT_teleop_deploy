import loadMujoco from './vendor/mujoco.js';
import {loadPolicy,GritState} from './grit.js';
import {Planner,unpackMotions,AI_ACTION_PROBABILITIES} from './planner.js';
import {knockdown} from './rules.js';
import {Showcase} from './showcase.js';
import {clamp,norm,matrix,yaw,euler,smooth,slerp,mix} from './math.js';

const KEY_ACTION={J:'light',K:'heavy',L:'combo',U:'kick',I:'flying_kick',O:'mixed_combo',Y:'taunt',B:'warmup'};
const send=(packet,transfer=[])=>self.postMessage(packet,transfer),stage=text=>send({type:'state',ready:false,stage:text,can_control:true});
let mj,model,data,config,bank,policy,actors,controls,forceBuffer,motionBuffer,force,motion;
let playing=false,started=false,ready=false,finished=false,round=0,lastInput=performance.now(),lastKeys=new Set(),pending=[],freeFace=false;
let controlMode='ai',modeRequest=null,repositionRequested=false,poseRevision=0,showcase=null,showcaseJump=null;
let roundStartedAt=null,warmupStartedAt=null,warmupUntil=0;
let result=null,score=[0,0],events=[],points=[],seen=new Set(),impulses=new Map(),fallSince=[{},{}],grounded=[false,false],support=[0,0];
let kickContacts=[null,null],kickUnstable=[null,null];
let inferenceMs=0,speed=1,lastPublish=0,settle=null,contract=null,loopBusy=false;
let nextTick=0,paceWall=0,paceSim=0,lastPoseTime=-1,profile={control_ms:0,physics_ms:0,inference_ms:0};
const velocities=new Float64Array(36);

async function read(url,gzip=false){const response=await fetch(new URL(url,import.meta.url),{cache:gzip?'default':'no-store'});if(!response.ok)throw new Error(`资源加载失败 ${url}`);return gzip?new Response(response.body.pipeThrough(new DecompressionStream('gzip'))).json():response.json();}
function observation(i){
  const a=config.actors[i],qpos=data.qpos,qvel=data.qvel,sensors=data.sensordata,xmat=data.xmat;
  const root=Array.from(qpos.slice(a.root_qadr,a.root_qadr+7)),quaternion=root.slice(3);
  const geom=data.geom_xpos;
  return {root,quaternion,joints:a.qadr.map(j=>qpos[j]),joint_velocity:a.vadr.map(j=>qvel[j]),
    gyro:Array.from(sensors.slice(a.gyro_adr,a.gyro_adr+3)),velocity:Array.from(qvel.slice(a.root_vadr,a.root_vadr+3)),
    up:matrix(quaternion)[8],torso_up:xmat[a.torso*9+8],facing_yaw:Math.atan2(xmat[a.torso*9+3],xmat[a.torso*9]),support:support[i],grounded:grounded[i],
    body:Array.from(geom.slice(a.body*3,a.body*3+3)),head:Array.from(geom.slice(a.head*3,a.head*3+3))};
}
function scanSupport(){
  grounded=[false,false];const feet=[new Set(),new Set()];
  const contactList=data.contact;
  for(let j=0;j<data.ncon;j++){
    const c=contactList.get(j);if(c.geom1!==config.floor&&c.geom2!==config.floor){c.delete();continue;}
    const other=c.geom1===config.floor?c.geom2:c.geom1,owner=config.owners[other];
    if(owner===undefined){c.delete();continue;}
    mj.mj_contactForce(model,data,j,forceBuffer.GetView());const foot=config.foot_geoms[other],normal=forceBuffer.GetView()[0];
    if(normal>1&&foot)feet[owner].add(foot[1]);else if(normal>15&&!foot)grounded[owner]=true;
    c.delete();
  }
  contactList.delete();
  support=feet.map(s=>s.size);
}
function finish(reason,winner){
  if(result)return;
  result={reason,winner,time_s:data.time,round_time_s:roundStartedAt===null?0:data.time-roundStartedAt,score:score.slice()};
  const states=[observation(0),observation(1)];
  settle=states.map(s=>({q:s.joints.slice(),r:s.quaternion.slice(),z:s.root[2],stableSince:null,raiseAt:null,cancelled:false}));
  actors.forEach(a=>{a.clip=null;a.pending=null;a.sequence=[];});pending=[];
  publish(true);
}
function judge(states){
  if(result)return;
  states.forEach((s,i)=>{
    const kick=kickContacts[i];
    if(kick&&data.time-kick.time<=1.25){
      const unstable=s.grounded||s.torso_up<=.3420201433||(s.up<.78&&(norm(s.velocity.slice(0,2))>.3||norm(s.gyro)>1.2));
      if(unstable){
        if(kickUnstable[i]===null)kickUnstable[i]=data.time;
        const hold=s.grounded?.05:s.torso_up<=.3420201433?.12:.20;
        if(data.time-kickUnstable[i]+1e-9>=hold){
          score[1-i]++;points.push({id:points.length,time_s:data.time,player:1-i,reason:'opponent_kick_self_unbalance',score:score.slice()});kickContacts[i]=null;kickUnstable[i]=null;
        }
      }else kickUnstable[i]=null;
    }else{kickContacts[i]=null;kickUnstable[i]=null;}
  });
  const down=knockdown(states,data.time,fallSince);
  if(down)finish(down.reason,down.winner);
  else if(roundStartedAt!==null&&data.time-roundStartedAt>=config.duration)finish(score[0]===score[1]?'points_draw':'points_decision',score[0]===score[1]?null:score[0]>score[1]?0:1);
}
function victory(i,state){
  const s=settle[i],elapsed=data.time-result.time_s,stable=state.up>.985&&state.torso_up>.985&&state.support===2&&!state.grounded&&norm(state.velocity.slice(0,2))<.22&&norm(state.gyro)<.65;
  if(stable){if(s.stableSince===null)s.stableSince=elapsed;}else s.stableSince=null;
  if(result.winner===i&&s.raiseAt===null&&!s.cancelled&&s.stableSince!==null&&elapsed-s.stableSince>=.5&&elapsed>=1.2)s.raiseAt=elapsed;
  if(s.raiseAt!==null&&(state.up<.93||state.torso_up<.92||state.grounded)){s.cancelled=true;s.raiseAt=null;}
  const ref={q:[],p:[],r:[]},stand=config.victory_stand||bank.guard,raised=config.victory_raised||stand;
  for(let k=0;k<16;k++){
    const t=elapsed+k/50,u=smooth(t),v=s.raiseAt===null?0:smooth((t-s.raiseAt)/1.6);
    ref.q.push(stand.map((x,j)=>mix(s.q[j],mix(x,raised[j],v),u)));
    ref.p.push([state.root[0],state.root[1],mix(s.z,config.stand_height||bank.guard_root[2],smooth(t/1.2))]);
    ref.r.push(slerp(s.r,euler([0,0,yaw(s.r)]),smooth(t/.6)));
  }
  actors[i].action=result.winner===i?(s.cancelled?'WIN / STAND':'WIN / RIGHT ARM'):'ROUND OVER / STAND';return ref;
}
function effectorSpeeds(){
  let active=false;
  for(let i=0;i<2;i++){
    if(!actors[i].clip)continue;
    const a=config.actors[i];
    for(let hand=0;hand<2;hand++)for(let limb=0;limb<3;limb++){
      const name=limb===0?'arm':limb===1?'leg':'knee';
      // Scoring below checks the state after this substep; sample its active
      // strike's velocity before impact, exactly as the unfiltered path did.
      if(!actors[i].strike(data.time+config.physics_dt,hand,name))continue;
      const id=limb===0?a.gloves[hand]:limb===1?a.feet[hand]:a.knees[hand];
      mj.mj_objectVelocity(model,data,limb===0?6:1,id,motionBuffer.GetView(),0);
      const view=motionBuffer.GetView(),offset=i*18+hand*9+limb*3;
      velocities[offset]=view[3];velocities[offset+1]=view[4];velocities[offset+2]=view[5];active=true;
    }
  }return active?velocities:null;
}
function contacts(velocities){
  if(result||roundStartedAt===null)return;
  const contactList=data.contact;
  for(let j=0;j<data.ncon;j++){
    const c=contactList.get(j),a=c.geom1,b=c.geom2;
    if(config.owners[a]===undefined||config.owners[b]===undefined||config.owners[a]===config.owners[b]){c.delete();continue;}
    for(const [source,target] of [[a,b],[b,a]]){
      const striker=config.strikers[source];if(!striker)continue;
      const [player,hand,limb]=striker,attack=actors[player].strike(data.time,hand,limb);if(!attack)continue;
      const t=config.targets[target],zone=t?t[1]:config.defences[target]!==undefined?'guard':null;if(!zone)continue;
      const kind=zone==='guard'?'blocked':'hit',key=`${player}:${attack.id}:${kind}`;
      mj.mj_contactForce(model,data,j,forceBuffer.GetView());const normal=forceBuffer.GetView()[0];if(normal<1)continue;
      const offset=player*18+hand*9+(limb==='arm'?0:limb==='leg'?3:6);
      const speed=Math.hypot(velocities[offset],velocities[offset+1],velocities[offset+2]);
      const ledger=impulses.get(key)||{last:data.time,impulse:0,speed:0};if(data.time-ledger.last>.05){ledger.impulse=0;ledger.speed=0;}
      ledger.impulse+=normal*.0005;ledger.speed=Math.max(ledger.speed,speed);ledger.last=data.time;impulses.set(key,ledger);
      if(limb!=='arm'&&ledger.impulse>=.08&&!seen.has(`kick:${player}:${attack.id}`)){
        const state=observation(player);
        if(state.up>=.85&&state.torso_up>=.75&&state.root[2]>=.53&&!state.grounded){
          seen.add(`kick:${player}:${attack.id}`);kickContacts[player]={time:data.time,id:attack.id};kickUnstable[player]=null;
        }
      }
      if(ledger.impulse>=.08&&speed>.35&&kickContacts[1-player]&&data.time-kickContacts[1-player].time>.02){kickContacts[1-player]=null;kickUnstable[1-player]=null;}
      if(seen.has(key))continue;
      if(ledger.impulse<.025||ledger.speed<.4)continue;
      seen.add(key);const position=Array.from(c.pos),event={id:events.length,time_s:data.time,player,hand,limb,kind,zone,position,method:attack.kind,strike_id:attack.id};events.push(event);
      const scoreKey=`score:${player}:${attack.id}`;
      if(kind==='hit'&&['head','body','thigh'].includes(zone)&&!seen.has(scoreKey)){
        seen.add(scoreKey);score[player]++;points.push({id:points.length,time_s:data.time,player,reason:zone+'_hit',score:score.slice()});
      }
    }
    c.delete();
  }
  contactList.delete();
}
function actuate(){
  const qpos=data.qpos,qvel=data.qvel,ctrl=data.ctrl;
  controls.forEach((control,i)=>{const a=config.actors[i];for(let j=0;j<29;j++){
    const torque=config.kp[j]*(control.target[j]-qpos[a.qadr[j]])-config.kd[j]*qvel[a.vadr[j]],range=config.ctrlrange[a.aadr[j]];
    ctrl[a.aadr[j]]=clamp(torque,range[0],range[1]);
  }});
}
function publish(force=false){
  if(!ready)return;
  if(controlMode==='showcase'&&showcase){
    const snapshot=showcase.snapshot();
    if(!force&&snapshot.time===lastPoseTime)return;lastPoseTime=snapshot.time;
    send({type:'state',ready:true,stage:'Ready',can_control:true,playing,started,finished:false,round_index:round,
      control_mode:'showcase',...snapshot,score:[0,0],result:null,limit:null,input_status:snapshot.showcase.phase,
      speed,performance:profile,blue_label:'GRIT',events:[],scoring_events:[],contract,engine:'MuJoCo 3.12 WASM + GRIT ONNX'},[snapshot.poses.buffer]);return;
  }
  const now=performance.now();if(!force&&data.time===lastPoseTime)return;lastPublish=now;lastPoseTime=data.time;
  const poses=new Float32Array(model.nbody*7),xpos=data.xpos,xquat=data.xquat,xmat=data.xmat;
  for(let i=0;i<model.nbody;i++){
    for(let j=0;j<3;j++)poses[i*7+j]=xpos[i*3+j];
    for(let j=0;j<4;j++)poses[i*7+3+j]=xquat[i*4+j];
  }
  send({type:'state',ready:true,stage:'Ready',can_control:true,playing,started,finished,round_index:round,time:data.time,limit:config.duration,
    round_time:roundStartedAt===null?0:Math.min(config.duration,data.time-roundStartedAt),round_started_at:roundStartedAt,
    match_phase:roundStartedAt===null?'warmup':'fight',warmup_remaining:Math.max(0,warmupUntil-data.time),
    control_mode:controlMode,pose_revision:poseRevision,
    scene:'./assets/scene.json.gz',poses,score:score.slice(),result,actions:actors.map(a=>a.action),input_status:actors[0].pending&&actors[0].clip?.family==='kick'?'AFTER LANDING':actors[0].action,
    scene_lite:'./assets/scene-lite.json.gz',
    speed,inference_ms:inferenceMs,blue_label:'GRIT',events:events.filter(e=>data.time-e.time_s<.8),scoring_events:points.filter(e=>data.time-e.time_s<.8),
    events_count:events.length,points_count:points.length,contract,engine:'MuJoCo 3.12 WASM + GRIT ONNX',performance:profile,
    diagnostics:{torso_up:config.actors.map(a=>xmat[a.torso*9+8]),commands:actors[0].events.slice(-12),
      actor_modes:actors.map(a=>a.manual?'manual':'ai'),action_counts:actors.map(a=>a.events.length),
      ai_action_probabilities:AI_ACTION_PROBABILITIES,category_counts:actors.map(a=>a.aiCategoryCounts),
      kick_setups:actors.map(a=>a.kickSetup),leg_contact_counts:actors.map((_,i)=>events.filter(e=>e.player===i&&e.limb==='leg').length),
      facing_error_deg:actors.map((a,i)=>{
        const own=config.actors[i],other=config.actors[1-i],bearing=Math.atan2(xpos[other.pelvis*3+1]-xpos[own.pelvis*3+1],xpos[other.pelvis*3]-xpos[own.pelvis*3]);
        const yaw=Math.atan2(xmat[own.torso*9+3],xmat[own.torso*9]);return Math.abs(Math.atan2(Math.sin(bearing-yaw),Math.cos(bearing-yaw)))*180/Math.PI;
      }),
      recent_methods:actors.map(a=>a.events.slice(-12).map(e=>e.method))}},[poses.buffer]);
}
function reset(){
  mj.mj_resetData(model,data);data.qpos.set(config.initial_qpos);mj.mj_forward(model,data);
  round++;score=[0,0];result=null;events=[];points=[];seen=new Set();impulses=new Map();fallSince=[{},{}];kickContacts=[null,null];kickUnstable=[null,null];playing=false;started=false;finished=false;pending=[];lastKeys=new Set();settle=null;
  roundStartedAt=null;warmupStartedAt=null;warmupUntil=0;
  actors=[0,1].map(i=>new Planner(i,bank,round+6));controls=[0,1].map(()=>new GritState(config));scanSupport();
  actors[0].setManual(false,0.);
  actors.forEach((a,i)=>{const s=observation(i);a.heading=yaw(s.quaternion);controls[i].reset(s,s.quaternion);});
  ready=true;repositionRequested=false;poseRevision++;lastPoseTime=-1;paceWall=performance.now();paceSim=0;speed=1;publish(true);
}
function reposition(){
  const now=data.time;
  // Explicit user reset only: preserve this round's clock, points and result.
  mj.mj_resetData(model,data);data.qpos.set(config.initial_qpos);data.time=now;mj.mj_forward(model,data);
  lastKeys.clear();pending=[];freeFace=false;impulses.clear();fallSince=[{},{}];kickContacts=[null,null];kickUnstable=[null,null];scanSupport();
  const states=[observation(0),observation(1)];
  actors.forEach((a,i)=>{a.resetPose(yaw(states[i].quaternion),now);controls[i].reset(states[i],states[i].quaternion);});
  if(roundStartedAt===null){warmupStartedAt=null;warmupUntil=0;}
  if(result)settle=states.map(s=>({q:s.joints.slice(),r:s.quaternion.slice(),z:s.root[2],stableSince:null,raiseAt:null,cancelled:false}));
  poseRevision++;lastPoseTime=-1;paceWall=performance.now();paceSim=data.time;publish(true);
}
async function tick(){
  if(loopBusy)return;loopBusy=true;const begin=performance.now();
  try{
    if(ready&&modeRequest!==null){
      const requested=modeRequest,previous=controlMode;modeRequest=null;
      lastKeys.clear();pending=[];freeFace=false;
      if(requested==='showcase'){
        if(!showcase){stage('加载单机器人展示舞台');showcase=await Showcase.create(mj,bank);}
        if(showcase.config.policy_sha256!==config.policy_sha256)throw new Error('展示策略配置版本不匹配，请强制刷新页面');
        if(previous!=='showcase'){showcase.select(0,true);round++;}
        controlMode='showcase';
      }else{
        controlMode='ai';
        if(previous==='showcase'||finished||result)reset();
        actors[0].setManual(false,data.time);
      }
      playing=true;started=true;lastInput=performance.now();paceWall=performance.now();paceSim=controlMode==='showcase'?showcase.data.time:data.time;
      lastPoseTime=-1;publish(true);
    }
    if(ready&&repositionRequested){
      repositionRequested=false;
      if(controlMode==='showcase'){showcase.select(showcase.index);lastPoseTime=-1;publish(true);}else reposition();
    }
    if(ready&&showcaseJump!==null&&controlMode==='showcase'){
      showcase.select(showcaseJump);showcaseJump=null;lastPoseTime=-1;publish(true);
    }
    if(ready&&controlMode==='showcase'){
      if(playing){
        if(begin-lastInput>3000){playing=false;publish(true);return;}
        await showcase.step(policy);const now=performance.now();
        profile={control_ms:now-begin,inference_ms:showcase.inferenceMs};
        if(now-paceWall>=500){speed=(showcase.data.time-paceSim)*1000/(now-paceWall);paceWall=now;paceSim=showcase.data.time;}
      }
      publish();return;
    }
    if(ready&&playing&&!finished){
      if(begin-lastInput>3000){playing=false;lastKeys.clear();pending=[];actors[0].pending=null;publish(true);return;}
      scanSupport();const states=[observation(0),observation(1)];
      if(roundStartedAt===null&&warmupStartedAt===null){
        warmupStartedAt=data.time;warmupUntil=data.time+Math.max(bank.clips.pico_warmup_forearms.duration,bank.clips.pico_warmup_release.duration)+.4;
        actors.forEach((a,i)=>a.beginWarmup(data.time,warmupUntil,states[i],actors[1-i]));
      }
      if(roundStartedAt===null&&data.time>=warmupUntil)roundStartedAt=data.time;
      judge(states);
      actors[0].manualKeys=controlMode==='manual'?new Set(lastKeys):new Set();if(freeFace){actors[0].freeHeading=null;freeFace=false;}
      for(const name of pending)actors[0].request(name,data.time);pending=[];
      const refs=result?states.map((s,i)=>victory(i,s)):states.map((s,i)=>actors[i].update(data.time,s,states[1-i],actors[1-i]));
      inferenceMs=await policy.infer(controls,states,refs);const physicsStart=performance.now();
      for(let step=0;step<40;step++){
        if(step%2===0)actuate();const velocities=result?null:effectorSpeeds();mj.mj_step(model,data);if(velocities)contacts(velocities);
      }
      if(!Array.from(data.qpos).every(Number.isFinite))throw new Error('仿真状态异常，请重开回合');
      if(!result&&roundStartedAt!==null&&data.time-roundStartedAt>=config.duration){scanSupport();judge([observation(0),observation(1)]);}
      if(result&&data.time-result.time_s>=9){finished=true;playing=false;}
      const now=performance.now();profile={control_ms:now-begin,physics_ms:now-physicsStart,inference_ms:inferenceMs};
      if(now-paceWall>=500){speed=(data.time-paceSim)*1000/(now-paceWall);paceWall=now;paceSim=data.time;}
    }
    publish();
  }catch(error){playing=false;ready=false;send({type:'state',ready:false,error:String(error.stack||error),stage:'ERROR',can_control:true});console.error(error);}
  finally{
    loopBusy=false;const now=performance.now();
    nextTick=Math.max(nextTick+20,now-20);
    setTimeout(tick,Math.max(0,nextTick-now));
  }
}
self.onmessage=event=>{
  const m=event.data;lastInput=performance.now();
  if(m.type==='heartbeat')return;
  if(!ready)return;
  if(m.type==='mode'){
    if(m.mode==='showcase'||m.mode==='ai')modeRequest=m.mode;
    return;
  }
  if(m.type==='showcase_jump'&&controlMode==='showcase'&&Number.isInteger(m.index)){
    showcaseJump=m.index;return;
  }
  if(m.type==='reposition'){repositionRequested=true;return;}
  if(m.type==='keys'){
    if(controlMode!=='manual')return;
    const next=new Set(m.keys);for(const key of next)if(!lastKeys.has(key)&&KEY_ACTION[key])pending.push(KEY_ACTION[key]);lastKeys=next;
  }else if(m.type==='toggle'){if(!finished||controlMode==='showcase'){playing=!playing;started=true;lastKeys.clear();pending=[];actors[0].pending=null;paceWall=performance.now();paceSim=controlMode==='showcase'?showcase.data.time:data.time;}}
  else if(m.type==='pause'){playing=false;lastKeys.clear();pending=[];actors[0].pending=null;}
  else if(m.type==='restart'){
    playing=false;modeRequest=null;repositionRequested=false;showcaseJump=null;
    const restart=()=>{if(loopBusy)setTimeout(restart,10);else if(controlMode==='showcase'){showcase.select(0,true);round++;started=false;lastPoseTime=-1;paceWall=performance.now();paceSim=0;publish(true);}else reset();};restart();
  }
  else if(m.type==='face'&&controlMode==='manual')freeFace=true;
  if(m.type!=='keys'&&m.type!=='face')publish(true);
};

async function init(){
  stage('加载 MuJoCo 与 GRIT 模型');
  config=await read('./assets/config.json');
  [mj,bank,policy]=await Promise.all([loadMujoco(),read('./assets/motions.json.gz',true).then(unpackMotions),loadPolicy(config)]);
  stage('校验 GRIT 控制接口');contract=await policy.validate(await read('./assets/contract-fixtures.json'));
  const response=await fetch(new URL('./assets/arena.mjb.gz',import.meta.url));
  const bytes=new Uint8Array(await new Response(response.body.pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
  const vfs=new mj.MjVFS();vfs.addBuffer('arena.mjb',bytes);model=mj.MjModel.from_binary_path('arena.mjb',vfs);vfs.delete();data=new mj.MjData(model);
  if(model.nq!==config.initial_qpos.length)throw new Error('机器人模型关节数不匹配');
  forceBuffer=new mj.DoubleBuffer(6);motionBuffer=new mj.DoubleBuffer(6);force=forceBuffer.GetView();motion=motionBuffer.GetView();
  stage('准备格斗动作');reset();tick();
}
init().catch(error=>{send({type:'state',ready:false,error:String(error.stack||error),stage:'ERROR',can_control:true});console.error(error);});
