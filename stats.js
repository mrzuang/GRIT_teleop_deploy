// Shared anonymous totals. Never upload scores, model choices, or user details.
const API='https://abacus.jasoncameron.dev';
const NAMESPACE='mrzuang-grit-teleop-deploy-arena-20261014';
const KEYS={views:'page-views',matches:'completed-matches'};
export class SiteStats{
  constructor({request=globalThis.fetch?.bind(globalThis),writable=location.hostname==='mrzuang.github.io'&&!new URLSearchParams(location.search).has('stats-readonly'),onChange=()=>{}}={}){
    this.request=request;this.writable=writable;this.onChange=onChange;this.values={views:null,matches:null};this.errors={};this.completed=new Set();this.started=false;this.offline=[];
  }
  async query(kind,increment=false){
    const action=increment?'hit':'get',url=new URL(`${API}/${action}/${NAMESPACE}/${KEYS[kind]}`);
    url.searchParams.set('_',String(Date.now()));
    try{
      const response=await this.request(url.href,{cache:'no-store',credentials:'omit',referrerPolicy:'no-referrer',keepalive:increment,signal:AbortSignal.timeout(12000)});
      if(response.status===404&&!increment){this.values[kind]=this.values[kind]??0;delete this.errors[kind];}
      else{
        if(!response.ok)throw Error('统计服务暂不可用');const result=await response.json();
        if(!Number.isSafeInteger(result.value)||result.value<0)throw Error('统计返回无效');
        // A short server cache may lag the increment acknowledgement.
        this.values[kind]=Math.max(this.values[kind]??0,result.value);delete this.errors[kind];
      }
    }catch(error){this.errors[kind]=true;}
    this.onChange({...this.values},{...this.errors});
  }
  increment(kind){
    if(!this.writable)return this.query(kind);
    if(globalThis.navigator?.onLine===false){this.offline.push(kind);this.errors[kind]=true;this.onChange({...this.values},{...this.errors});return Promise.resolve();}
    // An ambiguous timeout must not automatically send another increment:
    // the server may already have counted it. Read-only refresh can recover
    // the displayed total without double-counting the same match.
    return this.query(kind,true);
  }
  start(){if(this.started)return;this.started=true;return Promise.allSettled([this.increment('views'),this.query('matches')]);}
  refresh(){return Promise.allSettled([this.query('views'),this.query('matches')]);}
  async online(){const pending=this.offline.splice(0);for(const kind of pending)await this.increment(kind);await this.refresh();}
  record(packet){
    if(!packet?.ready||!packet.started||packet.control_mode==='showcase'||!packet.result)return;
    const key=packet.round_index;if(this.completed.has(key))return;
    if(!['knockdown','double_knockdown','balance_tko','points_decision','points_draw'].includes(packet.result.reason))return;
    this.completed.add(key);void this.increment('matches');
  }
}
