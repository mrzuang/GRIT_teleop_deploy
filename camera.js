import * as THREE from 'three';

export class ArenaCamera{
  constructor(camera,canvas){
    this.camera=camera;this.mode='third';this.zoom=1;this.distance=null;
    this.target=new THREE.Vector3();this.forward=new THREE.Vector3(1,0,0);this.line=new THREE.Vector3(1,0,0);this.offset=new THREE.Vector3();this.probe=new THREE.Vector3();this.redPosition=new THREE.Vector3();
    this.pointers=new Map();this.pinchDistance=null;canvas.style.touchAction='none';
    canvas.addEventListener('wheel',event=>{event.preventDefault();this.zoom=THREE.MathUtils.clamp(this.zoom*Math.exp(event.deltaY*.001),.65,2.2);},{passive:false});
    canvas.addEventListener('pointerdown',event=>{this.pointers.set(event.pointerId,[event.clientX,event.clientY]);canvas.setPointerCapture(event.pointerId);this.pinchDistance=this.pinch();});
    canvas.addEventListener('pointermove',event=>{
      if(!this.pointers.has(event.pointerId))return;this.pointers.set(event.pointerId,[event.clientX,event.clientY]);const next=this.pinch();
      if(next&&this.pinchDistance)this.zoom=THREE.MathUtils.clamp(this.zoom*this.pinchDistance/next,.65,2.2);this.pinchDistance=next;
    });
    const release=event=>{this.pointers.delete(event.pointerId);this.pinchDistance=this.pinch();};
    canvas.addEventListener('pointerup',release);canvas.addEventListener('pointercancel',release);
  }
  pinch(){if(this.pointers.size!==2)return null;const [a,b]=[...this.pointers.values()];return Math.hypot(a[0]-b[0],a[1]-b[1])||null;}
  reset(){this.distance=null;}
  setMode(mode){if(!['third','side'].includes(mode))return;this.mode=mode;this.reset();}
  update(red,blue,torso,dt){
    if(!red)return;const anchor=torso||red;this.redPosition.copy(red.position);
    this.probe.set(1,0,0).applyQuaternion(anchor.quaternion);this.probe.z=0;
    if(this.probe.lengthSq()>1e-5)this.forward.copy(this.probe).normalize();
    const horizontalTan=Math.tan(THREE.MathUtils.degToRad(this.camera.fov/2))*Math.max(.25,this.camera.aspect);
    let desired;
    if(this.mode==='third'){
      this.target.copy(anchor.position);this.target.z=Math.max(.45,this.target.z+.12);
      desired=Math.max(3.4,.65/(horizontalTan*.9))*this.zoom;
      this.offset.copy(this.forward).multiplyScalar(-1);
    }else{
      let separation=0;
      if(blue){
        this.probe.subVectors(blue.position,red.position);this.probe.z=0;separation=this.probe.length();
        if(separation>.05)this.line.copy(this.probe).normalize();
        this.target.copy(red.position).add(blue.position).multiplyScalar(.5);this.target.z=Math.max(.65,this.target.z+.27);
      }else{this.line.copy(this.forward);this.target.copy(anchor.position);this.target.z=Math.max(.5,this.target.z+.1);}
      desired=Math.max(3.1,(separation*.5+.65)/(horizontalTan*.88))*this.zoom;
      // A fixed side of the red-to-blue line keeps both fighters in profile.
      this.offset.set(this.line.y,-this.line.x,0);
    }
    this.distance=this.distance===null?desired:THREE.MathUtils.lerp(this.distance,desired,1-Math.exp(-5*dt));
    this.camera.position.copy(this.target).addScaledVector(this.offset,this.distance);
    this.camera.position.z+=this.distance*(this.mode==='third'?.30:.40);
    this.camera.lookAt(this.target);
  }
  snapshot(){return {mode:this.mode,position:this.camera.position.toArray(),target:this.target.toArray(),red_forward:this.forward.toArray(),fighter_line:this.line.toArray(),red_position:this.redPosition.toArray(),zoom:this.zoom};}
}
