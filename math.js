export const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export const smooth=x=>{x=clamp(x,0,1);return x*x*(3-2*x);};
export const wrap=x=>Math.atan2(Math.sin(x),Math.cos(x));
export const mix=(a,b,t)=>a+(b-a)*t;
export const norm=v=>Math.hypot(...v);
export const add=(a,b)=>a.map((x,i)=>x+b[i]);
export const sub=(a,b)=>a.map((x,i)=>x-b[i]);
export const mul=(a,k)=>a.map(x=>x*k);
export function qm(a,b){const[w,x,y,z]=a,[v,i,j,k]=b;return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v];}
export const qi=q=>[q[0],-q[1],-q[2],-q[3]];
export function euler(a){const [x,y,z]=a.map(v=>v/2),cx=Math.cos(x),sx=Math.sin(x),cy=Math.cos(y),sy=Math.sin(y),cz=Math.cos(z),sz=Math.sin(z);return [cx*cy*cz+sx*sy*sz,sx*cy*cz-cx*sy*sz,cx*sy*cz+sx*cy*sz,cx*cy*sz-sx*sy*cz];}
export function matrix(q){let[w,x,y,z]=q;const s=1/Math.hypot(w,x,y,z);w*=s;x*=s;y*=s;z*=s;return [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];}
export function rotate(q,v){const m=matrix(q);return [m[0]*v[0]+m[1]*v[1]+m[2]*v[2],m[3]*v[0]+m[4]*v[1]+m[5]*v[2],m[6]*v[0]+m[7]*v[1]+m[8]*v[2]];}
export const rot6=q=>{const m=matrix(q);return [m[0],m[1],m[3],m[4],m[6],m[7]];};
export function rotvec(q){if(q[0]<0)q=q.map(x=>-x);const s=Math.hypot(q[1],q[2],q[3]),angle=2*Math.atan2(s,q[0]);return q.slice(1).map(x=>x*(s<1e-10?2:angle/s));}
export function slerp(a,b,t){let d=a.reduce((s,x,i)=>s+x*b[i],0);if(d<0){b=b.map(x=>-x);d=-d;}if(d>.9995){const q=a.map((x,i)=>mix(x,b[i],t));return mul(q,1/norm(q));}const angle=Math.acos(clamp(d,-1,1)),s=Math.sin(angle);return a.map((x,i)=>(x*Math.sin((1-t)*angle)+b[i]*Math.sin(t*angle))/s);}
export const yaw=q=>{const m=matrix(q);return Math.atan2(m[3],m[0]);};

export function referenceFeatures(reference){
  const {q,p,r}=reference,n=q.length,features=[];
  for(let i=0;i<n;i++){
    const lo=Math.max(0,i-1),hi=Math.min(n-1,i+1),scale=50/(hi-lo||1);
    const jv=q[hi].map((x,k)=>(x-q[lo][k])*scale),v=rotate(qi(r[i]),mul(sub(p[hi],p[lo]),scale));
    let w;
    if(i===0)w=mul(rotvec(qm(qi(r[0]),r[1])),50);
    else if(i===n-1)w=mul(rotvec(qm(qi(r[n-2]),r[n-1])),50);
    else w=mul(sub(rotvec(qm(qi(r[i]),r[i+1])),rotvec(qm(qi(r[i]),r[i-1]))),25);
    features.push([...q[i],...jv,...rot6(r[i]),...v,...w]);
  }
  return features;
}
