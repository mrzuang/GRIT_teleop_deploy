export function knockdown(states,now,since){
  const losers=[];
  states.forEach((state,i)=>{
    for(const [kind,condition,hold] of [['fallen',state.torso_up<=.3420201433,.12],['ground',state.grounded,.05]]){
      if(!condition)delete since[i][kind];else if(since[i][kind]===undefined)since[i][kind]=now;
      if(condition&&now-since[i][kind]+1e-9>=hold){losers.push(i);break;}
    }
  });
  return losers.length?{reason:losers.length===2?'double_knockdown':'knockdown',winner:losers.length===2?null:1-losers[0]}:null;
}
