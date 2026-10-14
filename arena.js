import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {HighlightReplay} from './replay.js';
import {SiteStats} from './stats.js';

const elements=new Map(),$=id=>{if(!elements.has(id))elements.set(id,document.getElementById(id));return elements.get(id);};
const held=new Set(),actionKeys=new Set('W S A D Q E J K L H U I O Y B'.split(' ')),uiValues=new Map();
function setText(id,value){const key=id+':text';if(uiValues.get(key)!==value){$(id).textContent=value;uiValues.set(key,value);uiValues.delete(id+':html');}}
function setHTML(id,value){const key=id+':html';if(uiValues.get(key)!==value){$(id).innerHTML=value;uiValues.set(key,value);uiValues.delete(id+':text');}}
let pendingControllerSelection=null;
let socket, frame=null, round=null, loadingRound=null, loadedRound=null, canControl=false, inputSeq=0;
let nodes=[], targets=[], starts=[], receivedAt=0, redRoot=0, blueRoot=0, following=true;
let pendingPacket=null,poseTime=-1,interpolationMs=20,sceneSource=null,sceneLoadToken=0,quality='auto',autoLevel=2,slowSeconds=0,fastSeconds=0;
let displayedPoseRevision=-1,displayedControlMode=null;
const sceneCache=new Map(),qualityNames={auto:'自动',smooth:'流畅',high:'高清'};
let contactsSeen=-1, pointsSeen=-1, resultSeen=false, wasStarted=false, lastTick=-1, lastPacketAt=0;
let fps=0, fpsCount=0, fpsSince=performance.now(), lastDraw=performance.now();
const effects=[], flashes=[0,0], renderer=new THREE.WebGLRenderer({canvas:$('canvas'),antialias:true});
let bodyMaterials=[[],[]],bodyFlashUntil=[0,0],hitBannerUntil=0;
renderer.setPixelRatio(Math.min(devicePixelRatio,1));renderer.shadowMap.enabled=true;
renderer.shadowMap.type=THREE.PCFSoftShadowMap;renderer.setClearColor(0x101a29);
const gl=renderer.getContext(),debugGraphics=gl.getExtension('WEBGL_debug_renderer_info');
const graphicsName=debugGraphics?gl.getParameter(debugGraphics.UNMASKED_RENDERER_WEBGL):'';
const softwareGraphics=/SwiftShader|llvmpipe|Software/i.test(graphicsName);
if(softwareGraphics){renderer.shadowMap.enabled=false;renderer.setPixelRatio(.65);autoLevel=0;}

renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.15;
THREE.Object3D.DEFAULT_UP.set(0,0,1);
const world=new THREE.Scene();world.fog=new THREE.Fog(0x101a29,17,44);
const camera=new THREE.PerspectiveCamera(42,1,.025,100);camera.up.set(0,0,1);camera.position.set(7,-9,6);
const orbit=new OrbitControls(camera,$('canvas'));orbit.target.set(0,0,.7);orbit.enableDamping=true;orbit.dampingFactor=.12;
orbit.minDistance=1.8;orbit.maxDistance=24;orbit.maxPolarAngle=Math.PI*.49;

world.add(new THREE.HemisphereLight(0xdaeaff,0x354050,2.6));
const light=new THREE.DirectionalLight(0xffe9cd,3.5);light.position.set(3,-4,9);light.castShadow=true;
light.shadow.mapSize.set(1024,1024);Object.assign(light.shadow.camera,{left:-5,right:5,top:5,bottom:-5,near:1,far:22});
light.shadow.bias=-.00015;light.shadow.normalBias=.013;world.add(light);
const fill=new THREE.DirectionalLight(0x8fc5ff,1.5);fill.position.set(-5,3,4);world.add(fill);
let arenaGroup=new THREE.Group();world.add(arenaGroup);
const tempP=new THREE.Vector3(),cameraDelta=new THREE.Vector3(),cameraOffset=new THREE.Vector3();

function decode(text,Type){const bytes=Uint8Array.from(atob(text),c=>c.charCodeAt(0));return new Type(bytes.buffer);}
function quat(q){return new THREE.Quaternion(q[1],q[2],q[3],q[0]);}
function disposeArena(){
  const geometries=new Set(),materials=new Set();
  arenaGroup.traverse(o=>{if(o.geometry)geometries.add(o.geometry);if(o.material)materials.add(o.material);});
  geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());world.remove(arenaGroup);
  arenaGroup=new THREE.Group();world.add(arenaGroup);
}
function buildScene(data){
  disposeArena();nodes=data.bodies.map(()=>{const n=new THREE.Group();arenaGroup.add(n);return n;});
  bodyMaterials=[[],[]];bodyFlashUntil=[0,0];
  redRoot=data.bodies.indexOf('a_pelvis');blueRoot=data.bodies.indexOf('b_pelvis');
  const meshes=new Map();
  for(const [id,mesh] of Object.entries(data.meshes)){
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(decode(mesh.vertices,Float32Array),3));
    geometry.setIndex(new THREE.BufferAttribute(decode(mesh.faces,Uint32Array),1));geometry.computeVertexNormals();meshes.set(Number(id),geometry);
  }
  for(const geom of data.geoms){
    let geometry,scale=new THREE.Vector3(1,1,1);const s=geom.size;
    switch(geom.kind){
      case 0:geometry=new THREE.PlaneGeometry(s[0]?2*s[0]:100,s[1]?2*s[1]:100);break;
      case 2:geometry=new THREE.SphereGeometry(s[0],20,12);break;
      case 3:geometry=new THREE.CapsuleGeometry(s[0],2*s[1],6,14);geometry.rotateX(Math.PI/2);break;
      case 4:geometry=new THREE.SphereGeometry(1,24,16);scale.set(...s);break;
      case 5:geometry=new THREE.CylinderGeometry(s[0],s[0],2*s[1],24);geometry.rotateX(Math.PI/2);break;
      case 6:geometry=new THREE.BoxGeometry(2*s[0],2*s[1],2*s[2]);break;
      case 7:geometry=meshes.get(geom.mesh);break;
      default:continue;
    }
    if(!geometry)continue;
    const [r,g,b,a]=geom.rgba, material=new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(r,g,b,THREE.SRGBColorSpace),
      metalness:geom.kind===7?.2:.03,roughness:geom.kind===7?.53:.76,transparent:a<1,opacity:a});
    const mesh=new THREE.Mesh(geometry,material);mesh.position.set(...geom.position);mesh.quaternion.copy(quat(geom.quaternion));
    const bodyName=data.bodies[geom.body],owner=bodyName.startsWith('a_')?0:bodyName.startsWith('b_')?1:-1;
    if(owner>=0&&(bodyName.endsWith('torso_link')||bodyName.endsWith('pelvis')))bodyMaterials[owner].push(material);
    mesh.scale.copy(scale);mesh.castShadow=geom.body>0;mesh.receiveShadow=true;mesh.name=geom.name;nodes[geom.body].add(mesh);
  }
  starts=nodes.map(()=>({p:new THREE.Vector3(),q:new THREE.Quaternion()}));
  targets=nodes.map(()=>({p:new THREE.Vector3(),q:new THREE.Quaternion()}));poseTime=-1;
}
async function loadScene(packet){
  const token=++sceneLoadToken;
  loadingRound=packet.round_index;$('loading').hidden=false;$('loading-title').textContent='正在载入三维场景';
  const source=quality==='high'?packet.scene:(packet.scene_lite||'./assets/scene-lite.json.gz');
  try{
    if(!sceneCache.has(source)){
      const response=await fetch(source);if(!response.ok)throw new Error('场景仍在准备，请稍候');
      sceneCache.set(source,await new Response(response.body.pipeThrough(new DecompressionStream('gzip'))).json());
    }
    if(token!==sceneLoadToken||frame?.round_index!==packet.round_index)return;
    if(source!==sceneSource){buildScene(sceneCache.get(source));sceneSource=source;}
    loadedRound=packet.round_index;poseTime=-1;acceptPoses(frame.poses,true,frame.time);$('loading').hidden=true;
    $('start').disabled=!canControl||!!frame.result;
    $('mode').disabled=!canControl;
    $('manual').disabled=!canControl;
    $('showcase').disabled=!canControl;
    $('reposition').disabled=!canControl;
    updateControllerSetup(frame);
    if(!softwareGraphics&&renderer.compileAsync)renderer.compileAsync(world,camera).catch(()=>{});
  }catch(error){$('loading-detail').textContent=error.message;loadingRound=null;}
}
function acceptPoses(poses,instant=false,time=0,arrival=performance.now()){
  if(!poses||poses.length!==nodes.length*7)return;
  if(!instant&&time===poseTime)return;
  const now=arrival,interval=now-receivedAt;
  if(!instant&&interval>8&&interval<100)interpolationMs=Math.min(50,.75*interpolationMs+.25*interval);
  nodes.forEach((n,i)=>{
    const offset=i*7;starts[i].p.copy(targets[i].p);starts[i].q.copy(targets[i].q);
    targets[i].p.fromArray(poses,offset);targets[i].q.set(poses[offset+4],poses[offset+5],poses[offset+6],poses[offset+3]);
    if(instant){starts[i].p.copy(targets[i].p);starts[i].q.copy(targets[i].q);}
  });
  receivedAt=now;poseTime=time;
  if(instant)nodes.forEach((n,i)=>{n.position.copy(targets[i].p);n.quaternion.copy(targets[i].q);});
}

function applyQuality(){
  const level=quality==='high'?3:quality==='smooth'?0:autoLevel;
  const scale=level===3?Math.min(devicePixelRatio,1.5):level===2?Math.min(devicePixelRatio,1):level===1?.85:.65;
  renderer.setPixelRatio(scale);renderer.shadowMap.enabled=level>=2;
  const shadowSize=level===3?2048:1024;
  if(light.shadow.mapSize.x!==shadowSize){light.shadow.mapSize.set(shadowSize,shadowSize);light.shadow.map?.dispose();light.shadow.map=null;}
  renderer.shadowMap.needsUpdate=true;
  setText('quality','画质 '+qualityNames[quality]);
}
function cycleQuality(){
  quality={auto:'smooth',smooth:'high',high:'auto'}[quality];slowSeconds=0;fastSeconds=0;applyQuality();
  try{localStorage.setItem('grit-arena-quality',quality);}catch{}
  if(frame?.ready){const requested=quality==='high'?frame.scene:(frame.scene_lite||'./assets/scene-lite.json.gz');if(requested!==sceneSource)void loadScene(frame);}
}
try{const requested=new URLSearchParams(location.search).get('quality')||localStorage.getItem('grit-arena-quality');if(requested in qualityNames)quality=requested;}catch{}
applyQuality();$('quality').onclick=cycleQuality;

class Sounds{
  constructor(){this.context=null;this.buffers={};this.muted=false;this.initialized=false;}
  async unlock(){
    if(!this.context){
      this.context=new AudioContext();
      for(const name of ['hit','block','point','start','win','lose','draw','tick','down'])
        fetch(`./sound/${name}.wav`).then(r=>r.arrayBuffer()).then(b=>this.context.decodeAudioData(b)).then(b=>{this.buffers[name]=b;}).catch(()=>{});
    }
    if(this.context.state==='suspended')await this.context.resume();
  }
  play(name){
    if(this.muted||!this.context||this.context.state!=='running'||!this.buffers[name])return;
    const source=this.context.createBufferSource(),gain=this.context.createGain();source.buffer=this.buffers[name];gain.gain.value=.65;
    source.connect(gain).connect(this.context.destination);source.start();source.onended=()=>{source.disconnect();gain.disconnect();};
  }
  toggle(){this.muted=!this.muted;$('sound').innerHTML=`音效 ${this.muted?'关':'开'} <kbd>M</kbd>`;}
}
const sounds=new Sounds();
const statistics=new SiteStats({onChange:(values,errors)=>{
  for(const [kind,id] of [['views','site-views'],['matches','site-matches']]){
    setText(id,values[kind]===null?'—':values[kind].toLocaleString('zh-CN'));
    $(id).title=errors[kind]?'统计服务暂不可用，稍后刷新':kind==='views'?'全站页面浏览量':'全站已结束的比赛数量';
    $(id).classList.toggle('stats-unavailable',!!errors[kind]);
  }
}});
void statistics.start();
setInterval(()=>{if(!document.hidden)void statistics.refresh();},60000);
window.addEventListener('online',()=>void statistics.online());
function send(message){socket?.postMessage(message);}
function sendKeys(){if(canControl)send({type:'keys',seq:++inputSeq,keys:[...held]});paintKeys();}
function clearKeys(){held.clear();sendKeys();}
function paintKeys(){document.querySelectorAll('[data-key]').forEach(b=>b.classList.toggle('held',held.has(b.dataset.key)));}
function command(type){if(!canControl||pendingControllerSelection)return;finishReplay(false);void sounds.unlock();clearKeys();send({type});}
function pause(){if(replay.active){replay.paused=true;setText('replay-pause','继续回放');}if(canControl){clearKeys();send({type:'pause'});}}
function chooseMode(mode){
  if(!canControl||pendingControllerSelection||!frame?.ready||loadedRound!==frame.round_index)return;
  finishReplay(false);void sounds.unlock();clearKeys();send({type:'mode',mode});
}
function showHelp(show){$('controls-panel').hidden=!show;$('help').setAttribute('aria-expanded',String(show));}
function jumpAction(delta){if(frame?.control_mode==='showcase')send({type:'showcase_jump',index:frame.showcase.index+delta});}
window.addEventListener('keydown',event=>{
  if(event.ctrlKey||event.altKey||event.metaKey)return;
  const key=event.code.startsWith('Key')?event.code.slice(3):event.code;
  if(event.target.closest?.('input,textarea,select,[contenteditable="true"]'))return;
  if(!actionKeys.has(key)&&!['Enter','NumpadEnter','Space','Escape','Backspace','R','M','C','P','V','F','T','Slash','ArrowLeft','ArrowRight'].includes(key))return;
  event.preventDefault();if(event.repeat)return;void sounds.unlock();
  if(key==='Enter'||key==='NumpadEnter'){if(replay.active){toggleReplayPause();return;}command('toggle');return;}
  if(key==='Space'){command('reposition');return;}
  if(key==='Escape'){finishReplay(false);pause();return;}
  if(key==='R'||key==='Backspace'){command('restart');return;}
  if(key==='T'){startReplay();return;}
  if(key==='C'){chooseMode('manual');return;}
  if(key==='P'){chooseMode('ai');return;}
  if(key==='V'){chooseMode('showcase');return;}
  if(key==='ArrowLeft'||key==='ArrowRight'){jumpAction(key==='ArrowLeft'?-1:1);return;}
  if(key==='Slash'){showHelp($('controls-panel').hidden);return;}
  if(key==='M'){sounds.toggle();return;}
  if(replay.active||frame?.control_mode!=='manual'||!frame.playing||frame.result||frame.match_phase!=='fight'||loadedRound!==frame.round_index)return;
  if(key==='F'){send({type:'face'});return;}
  if(actionKeys.has(key)&&!held.has(key)){held.add(key);sendKeys();}
});
window.addEventListener('keyup',event=>{const key=event.code.startsWith('Key')?event.code.slice(3):event.code;if(actionKeys.has(key)){event.preventDefault();held.delete(key);sendKeys();}});
window.addEventListener('blur',pause);
document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
window.addEventListener('pagehide',pause);
$('start').onclick=()=>command('toggle');$('restart').onclick=$('again').onclick=()=>command('restart');
$('reposition').onclick=()=>command('reposition');
$('manual').onclick=()=>chooseMode('manual');$('mode').onclick=()=>chooseMode('ai');$('showcase').onclick=()=>chooseMode('showcase');$('help').onclick=()=>showHelp($('controls-panel').hidden);$('close-help').onclick=()=>showHelp(false);
$('previous-action').onclick=()=>jumpAction(-1);$('next-action').onclick=()=>jumpAction(1);
$('action-select').onchange=()=>{send({type:'showcase_jump',index:Number($('action-select').value)});$('canvas').focus({preventScroll:true});};
$('sound').onclick=()=>{void sounds.unlock();sounds.toggle();};
function setFollowing(){following=true;$('follow').classList.add('selected');}
$('follow').onclick=()=>{setFollowing();const center=orbit.target;camera.position.set(center.x+3,center.y-4,center.z+2.6);};
const replay=new HighlightReplay();let replayResumePlaying=false,replayHelpOpen=false;
function startReplay(){
  if(replay.active){finishReplay(true);return;}
  if(!frame?.ready||loadedRound!==frame.round_index||frame.control_mode==='showcase'||!replay.start(performance.now()))return;
  replayResumePlaying=!!frame?.playing;replayHelpOpen=!$('controls-panel').hidden;showHelp(false);clearKeys();send({type:'pause'});
  replay.speed=Number($('replay-speed').value);$('replay-panel').hidden=false;$('result').hidden=true;
  setText('replay-pause','暂停回放');setText('highlights','返回比赛');
}
function finishReplay(resume=true){
  if(!replay.active&&$('replay-panel').hidden)return;
  const shouldResume=resume&&replayResumePlaying;replayResumePlaying=false;replay.stop();$('replay-panel').hidden=true;showHelp(replayHelpOpen);
  setHTML('highlights','Highlights <kbd>T</kbd>');
  if(frame?.poses){acceptPoses(frame.poses,true,frame.time);receive(frame);}
  if(shouldResume&&frame&&!frame.playing&&!frame.finished)send({type:'toggle'});
}
function toggleReplayPause(){replay.paused=!replay.paused;setText('replay-pause',replay.paused?'继续回放':'暂停回放');}
function drawReplay(now){
  if(!replay.active)return;
  const sample=replay.tick(now);if(!sample){finishReplay(true);return;}
  acceptPoses(sample.poses,true,sample.time,now);$('result').hidden=true;
  setText('red-score',sample.score[0]);setText('blue-score',sample.score[1]);setText('red-action',sample.actions[0]);setText('blue-action',sample.actions[1]);
  const remaining=Math.max(0,Math.ceil(60-sample.roundTime));setText('timer',`${String(Math.floor(remaining/60)).padStart(2,'0')}:${String(remaining%60).padStart(2,'0')}`);
  setText('status',`回放 ${replay.speed}×`);setText('replay-caption',`${replay.clipIndex+1}/${replay.clips.length} · ${sample.caption}`);
  $('replay-progress').value=sample.progress;setText('hint','真实比赛慢放 · Enter 暂停回放 · T 返回比赛');
}
$('highlights').onclick=startReplay;$('replay-close').onclick=()=>finishReplay(true);$('replay-pause').onclick=toggleReplayPause;
$('replay-speed').onchange=()=>{replay.speed=Number($('replay-speed').value);$('canvas').focus({preventScroll:true});};
function updateControllerSetup(packet){
  const show=packet?.control_mode==='showcase',beforeMatch=!show&&!packet?.started;
  const blocked=!canControl||!packet?.ready||loadedRound!==packet.round_index||!!pendingControllerSelection;
  $('red-policy').disabled=blocked||!beforeMatch;$('blue-policy').disabled=blocked||!beforeMatch;
  $('showcase-policy').disabled=blocked||!show;
  $('match-setup').hidden=!beforeMatch;document.body.classList.toggle('match-preparation',beforeMatch);
  setText('setup-status',pendingControllerSelection?'正在加载所选模型…':blocked?'正在准备模型…':'选好后按 Enter 开始对战');
  if(pendingControllerSelection)for(const id of ['start','manual','mode','showcase'])$(id).disabled=true;
}
function selectController(index,id){
  if(!frame?.ready||pendingControllerSelection||loadedRound!==frame.round_index||(frame.control_mode!=='showcase'&&frame.started))return;
  finishReplay(false);clearKeys();const ids=(frame.controllers||[{id:'grit'},{id:'grit'}]).map(c=>c.id);ids[index]=id;
  pendingControllerSelection={ids,round:frame.round_index};updateControllerSetup(frame);
  send({type:'controllers',ids});$('canvas').focus({preventScroll:true});
}
$('red-policy').onchange=()=>selectController(0,$('red-policy').value);
$('blue-policy').onchange=()=>selectController(1,$('blue-policy').value);
$('showcase-policy').onchange=()=>selectController(0,$('showcase-policy').value);


function feedback(packet){
  if(packet.round_index!==round){round=packet.round_index;contactsSeen=-1;pointsSeen=-1;resultSeen=false;wasStarted=false;lastTick=-1;clearKeys();}
  const roundActive=packet.started&&packet.match_phase!=='warmup';
  if(roundActive&&!wasStarted)sounds.play('start');wasStarted=roundActive;
  for(const event of packet.events||[]){
    if(event.id<=contactsSeen)continue;
    contactsSeen=event.id;if(!packet.playing||packet.time-event.time_s>.5)continue;
    sounds.play(event.kind==='blocked'?'block':'hit');
    const color=event.kind==='blocked'?0x86d8ff:event.zone==='head'?0xffdc7b:0xff925a;
    const bodyHit=event.kind==='hit'&&event.zone==='body';
    const orb=new THREE.Mesh(new THREE.SphereGeometry(bodyHit?.065:.022,12,8),new THREE.MeshBasicMaterial({color,transparent:true,opacity:.85,depthWrite:false}));
    orb.position.set(...event.position);world.add(orb);effects.push({orb,start:performance.now()});
    if(bodyHit){
      bodyFlashUntil[1-event.player]=performance.now()+750;
      const ring=new THREE.Mesh(new THREE.RingGeometry(.13,.15,32),new THREE.MeshBasicMaterial({color:0xffb257,transparent:true,opacity:.9,depthWrite:false,side:THREE.DoubleSide}));
      ring.position.set(...event.position);ring.quaternion.copy(camera.quaternion);world.add(ring);effects.push({orb:ring,start:performance.now(),duration:750,billboard:true});
      const scored=(packet.scoring_events||[]).some(p=>p.player===event.player&&p.reason==='body_hit'&&Math.abs(p.time_s-event.time_s)<.04);
      setText('hit-banner',`${event.player===0?'红方':'蓝方'} · 躯干命中${scored?' +1':''}`);$('hit-banner').hidden=false;hitBannerUntil=performance.now()+850;
    }
  }
  for(const event of packet.scoring_events||[]){
    if(event.id<=pointsSeen)continue;
    pointsSeen=event.id;if(packet.playing)sounds.play('point');flashes[event.player]=performance.now()+650;
    const label={head_hit:'头部 +1',body_hit:'躯干 +1',thigh_hit:'大腿 +1',opponent_kick_self_unbalance:'对方踢击失衡 +1'}[event.reason]||'+1';
    setText(event.player===0?'red-hit':'blue-hit',label);
  }
  if(packet.result&&!resultSeen){
    resultSeen=true;if(['knockdown','double_knockdown','balance_tko'].includes(packet.result.reason))sounds.play('down');
    sounds.play(packet.result.winner===null?'draw':packet.result.winner===0?'win':'lose');
  }
  if(packet.playing&&!packet.result&&packet.limit!==null){const remaining=Math.ceil(packet.limit-(packet.round_time??packet.time));if(remaining>0&&remaining<=10&&remaining!==lastTick)sounds.play('tick');lastTick=remaining;}
}
function receive(packet,arrival=performance.now()){
  statistics.record(packet);frame=packet;lastPacketAt=performance.now();canControl=packet.can_control;
  if(packet.controllers){
    if(pendingControllerSelection&&(packet.controller_notice||(packet.round_index>pendingControllerSelection.round&&packet.controllers.every((c,i)=>c.id===pendingControllerSelection.ids[i]))))pendingControllerSelection=null;
    const [red,blue]=packet.controllers;
    for(const [id,c] of [['red-controller',red],['blue-label',blue]])setText(id,`${c.label} ${c.version.split(' · ')[0]}`);
    $('red-policy').value=red.id;$('blue-policy').value=blue.id;$('showcase-policy').value=red.id;
    for(const [id,c] of [['red-policy',red],['blue-policy',blue],['showcase-policy',red]])$(id).title=`${c.label} ${c.version} · 比赛可用 ${c.combat_actions??37}/37 段 · ${id==='showcase-policy'?'选择展示小脑':'仅赛前可更换，本局开始后锁定'}`;
    setHTML('title',`${red.label} <span>vs</span> ${blue.label}`);
    $('title').title=`${red.label} ${red.version} vs ${blue.label} ${blue.version}`;
  }
  if(!sounds.initialized){sounds.initialized=true;if(packet.muted_default)sounds.toggle();}
    setText('connection',canControl?'浏览器仿真':'正在准备');$('connection').className='connection online';
  const ready=packet.ready&&loadedRound===packet.round_index;
  $('start').disabled=!ready||!canControl||!!packet.result;$('restart').disabled=!packet.ready||!canControl;
  $('reposition').disabled=!ready||!canControl;
  $('manual').disabled=!ready||!canControl;
  $('mode').disabled=!ready||!canControl;
  $('showcase').disabled=!ready||!canControl;
  updateControllerSetup(packet);
  const show=packet.control_mode==='showcase',manual=packet.control_mode==='manual',ai=packet.control_mode==='ai';
  if(packet.control_mode!==displayedControlMode){displayedControlMode=packet.control_mode;clearKeys();}
  $('manual').setAttribute('aria-pressed',String(manual));
  $('manual-keys').hidden=show;
  $('highlights').disabled=!ready||!canControl||show||!replay.available;
  $('mode').setAttribute('aria-pressed',String(ai));$('showcase').setAttribute('aria-pressed',String(show));
  setText('red-label',manual?'红方 · 你':'红方 · AI');setText('red-action-label',show?'展示动作':manual?'你的动作':'红方动作');
  setText('mode-note',show?'单机器人依次播放全部可用动作；每段结束后回到舞台中央。可用左右方向键或下拉菜单选择动作。':manual?'你控制红方，蓝方自动应战。开场热身结束后可操作；自行控制距离，踢腿落地后衔接下一招。蓝方选招：格挡20%、踢腿40%、出拳40%。':'双方先热身再靠近，按格挡20%、踢腿40%、出拳40%选招。按 C 可随时接管红方。');
  $('scoreboard').hidden=show;$('showcase-panel').hidden=!show;$('blue-action-row').hidden=show;
  if(show&&packet.showcase){
    const s=packet.showcase;setText('showcase-name',s.label);setText('showcase-counter',`${s.index+1} / ${s.total} · 第 ${s.lap} 轮`);
    $('showcase-progress').value=s.progress;setText('showcase-note',`${s.phase} · ${s.combat_eligible?'展示完整动作库':'该动作不用于此模型比赛，仍可展示'} · 每段结束回到中心。`);
    const select=$('action-select');
    if(select.options.length!==s.list.length){select.replaceChildren(...s.list.map((item,i)=>new Option(`${i+1}. ${item.label}`,String(i))));}
    select.value=String(s.index);
  }
  $('again').disabled=!canControl;setHTML('start',packet.result?'回合结束':packet.playing?'暂停 <kbd>Enter</kbd>':packet.started?'继续对战 <kbd>Enter</kbd>':'开始对战 <kbd>Enter</kbd>');
  if(packet.error){$('loading').hidden=false;$('loading-title').textContent='仿真启动失败';$('loading-detail').textContent=packet.error.trim().split('\n').at(-1);return;}
  if(!packet.ready){
    $('loading').hidden=false;$('loading-title').textContent=packet.stage.startsWith('Saving')?'正在保存并准备新回合':'正在准备拳台';$('loading-detail').textContent=packet.stage;
    $('result').hidden=true;return;
  }
  if(loadedRound!==packet.round_index&&loadingRound!==packet.round_index)void loadScene(packet);
  if(loadedRound===packet.round_index){
    const resetPose=displayedPoseRevision!==packet.pose_revision;displayedPoseRevision=packet.pose_revision;
    $('loading').hidden=true;if(!replay.active)acceptPoses(packet.poses,resetPose,packet.time,arrival);
    if(resetPose){
      for(const effect of effects){world.remove(effect.orb);effect.orb.geometry.dispose();effect.orb.material.dispose();}effects.length=0;
      bodyFlashUntil=[0,0];hitBannerUntil=0;
      clearKeys();
    }
  }
  if(replay.record(packet))finishReplay(false);$('highlights').disabled=show||!replay.available;
  feedback(packet);setText('red-score',packet.score[0]);setText('blue-score',packet.score[1]);
  setText('round',`ROUND ${String(packet.round_index).padStart(2,'0')}`);
  const time=packet.result?(packet.result.round_time_s??packet.result.time_s):(packet.round_time??packet.time),remaining=packet.limit===null?null:Math.max(0,Math.ceil(packet.limit-time));
  setText('timer',remaining===null?'∞':`${String(Math.floor(remaining/60)).padStart(2,'0')}:${String(remaining%60).padStart(2,'0')}`);
  $('timer').style.color=remaining!==null&&remaining<=10?'#f2cd83':'';
  setText('status',packet.result?'回合结束':!packet.started?'等待开始':packet.playing?(packet.match_phase==='warmup'?'双方热身中':manual?'你 vs AI':'AI 对打中'):'已暂停');
  setText('red-action',packet.actions[0]);setText('blue-action',packet.actions[1]);
  setText('hint',!packet.started?'在左侧选好双方模型 · Enter 开始对战':!packet.playing?'已暂停 · 按 Enter 继续':show?'← / → 切换动作 · Space 重播当前动作 · Enter 暂停':packet.match_phase==='warmup'?(manual?'热身结束后可控制红方 · 按键说明在右下角':'双方热身后开始靠近 · 热身不计入回合时长'):manual?'你控制红方 · WASD 移动 · J/K/L 出拳 · H 格挡 · U/I/O 踢击':'AI 自动对打 · C 接管红方 · Space 重置站位');
  if(packet.controller_notice)setText('hint',packet.controller_notice);
  $('result').hidden=!packet.result||replay.active;
  if(packet.result){
    setHTML('start','回合结束');setText('hint','按 R 开始新回合');
    $('winner').textContent=packet.result.winner===null?'平局':packet.result.winner===0?'红方获胜':'蓝方获胜';
    $('winner').style.color=packet.result.winner===null?'#f2cd83':packet.result.winner===0?'#ff9ba6':'#98d4ff';
    $('result-reason').textContent={knockdown:'倒地判胜',double_knockdown:'双方同时倒地',points_decision:'满时比分决胜',points_draw:'满时同分',balance_tko:'失衡判胜'}[packet.result.reason]||'回合结束';
    $('final-score').textContent=`最终比分  ${packet.result.score[0]} : ${packet.result.score[1]}`;
  }
}
function connect(){
  socket=new Worker(new URL('./physics.js',import.meta.url),{type:'module'});
  socket.onmessage=event=>{if(event.data.ready)pendingPacket={packet:event.data,arrival:performance.now()};else receive(event.data);};
  socket.onerror=event=>{$('loading').hidden=false;$('loading-title').textContent='仿真加载失败';$('loading-detail').textContent=event.message;};
}

setInterval(()=>{if(canControl&&!document.hidden)send({type:'heartbeat'});},200);

const resize=new ResizeObserver(()=>{const rect=$('arena').getBoundingClientRect();renderer.setSize(rect.width,rect.height,false);camera.aspect=rect.width/rect.height;camera.updateProjectionMatrix();});resize.observe($('arena'));
function draw(now){
  requestAnimationFrame(draw);const dt=Math.min(.1,(now-lastDraw)/1000);lastDraw=now;
  if(pendingPacket){const {packet,arrival}=pendingPacket;pendingPacket=null;receive(packet,arrival);}
  drawReplay(now);
  const alpha=Math.max(0,Math.min(1,(performance.now()-receivedAt)/interpolationMs));
  if(targets.length===nodes.length)nodes.forEach((n,i)=>{n.position.lerpVectors(starts[i].p,targets[i].p,alpha);n.quaternion.slerpQuaternions(starts[i].q,targets[i].q,alpha);});
  if(following&&targets[redRoot]){
    const a=targets[redRoot].p,b=targets[blueRoot]?.p||a,center=tempP.copy(a).add(b).multiplyScalar(.5);center.z=.72;
    const distance=targets[blueRoot]?Math.max(4.1,2.8+a.distanceTo(b)*1.12):3.4,delta=cameraDelta.copy(center).sub(orbit.target);
    orbit.target.addScaledVector(delta,Math.min(1,dt*6));camera.position.addScaledVector(delta,Math.min(1,dt*6));
    const offset=cameraOffset.copy(camera.position).sub(orbit.target);offset.setLength(THREE.MathUtils.lerp(offset.length(),distance,Math.min(1,dt*2)));
    camera.position.copy(orbit.target).add(offset);
  }
  for(let i=effects.length-1;i>=0;i--){const effect=effects[i],age=(now-effect.start)/(effect.duration||500);if(age>=1){world.remove(effect.orb);effect.orb.geometry.dispose();effect.orb.material.dispose();effects.splice(i,1);}else{effect.orb.scale.setScalar(1+age*2);effect.orb.material.opacity=.8*(1-age);if(effect.billboard)effect.orb.quaternion.copy(camera.quaternion);}}
  bodyMaterials.forEach((materials,i)=>{const intensity=Math.max(0,(bodyFlashUntil[i]-now)/750);for(const material of materials){material.emissive.setHex(0xff631b);material.emissiveIntensity=intensity*1.7;}});
  $('hit-banner').hidden=now>=hitBannerUntil;
  for(let p=0;p<2;p++){const id=p===0?'red':'blue';$(id+'-panel').classList.toggle('flash',now<flashes[p]);if(now>=flashes[p])setText(id+'-hit','');}
  orbit.update();renderer.render(world,camera);fpsCount++;
  if(now-fpsSince>=1000){
    fps=fpsCount*1000/(now-fpsSince);fpsCount=0;fpsSince=now;
    setText('performance',`${fps.toFixed(0)} FPS · 仿真 ${(frame?.speed??1).toFixed(2)}× · ${softwareGraphics?'软件渲染':'显卡渲染'} · 鼠标拖动 / 滚轮缩放`);
    if(quality==='auto'&&loadedRound!==null){
      slowSeconds=fps<50?slowSeconds+1:0;fastSeconds=fps>58?fastSeconds+1:0;
      if(slowSeconds>=2&&autoLevel>0){autoLevel--;slowSeconds=0;fastSeconds=0;applyQuality();}
      else if(fastSeconds>=15&&autoLevel<2&&!softwareGraphics){autoLevel++;slowSeconds=0;fastSeconds=0;applyQuality();}
    }
  }
}
window.__arena={get state(){return frame;},get loadedRound(){return loadedRound;},get fps(){return fps;},get bodyCount(){return nodes.length;},get renderedRoots(){return [nodes[redRoot]?.position.toArray(),nodes[blueRoot]?.position.toArray()];},get held(){return [...held];},get replay(){return {active:replay.active,available:replay.available,clips:replay.clips.length,time:replay.time,paused:replay.paused,frames:replay.frames.length};},
  get performance(){return {quality,autoLevel,pixel_ratio:renderer.getPixelRatio(),shadows:renderer.shadowMap.enabled,triangles:renderer.info.render.triangles,draw_calls:renderer.info.render.calls,interpolation_ms:interpolationMs};},
  get graphics(){const gl=renderer.getContext(),ext=gl.getExtension('WEBGL_debug_renderer_info');return ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER);}};
window.addEventListener('error',event=>{$('loading-detail').textContent=event.message;});
connect();requestAnimationFrame(draw);
