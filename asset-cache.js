// Content-addressed browser storage: large immutable assets survive page reloads.
const networkFetch=globalThis.fetch.bind(globalThis),cacheName='grit-arena-assets-v1';
let manifestPromise,cachePromise,installed=false;const pending=new Map();
export const assetCacheMetrics={hits:0,misses:0,cache_bytes:0,download_bytes:0};
export function prepareAssetCache(){
  if(!manifestPromise)manifestPromise=networkFetch(new URL('./loading-assets.json',import.meta.url),{cache:'no-store'}).then(r=>r.ok?r.json():{}).catch(()=>({}));
  if(!cachePromise)cachePromise=(globalThis.caches?caches.open(cacheName):Promise.resolve(null)).catch(()=>null);
  return Promise.all([manifestPromise,cachePromise]);
}
async function hash(bytes){if(!globalThis.crypto?.subtle)return null;return [...new Uint8Array(await crypto.subtle.digest('SHA-256',bytes))].map(v=>v.toString(16).padStart(2,'0')).join('');}
export async function cachedAssetFetch(input,options={}){
  const url=new URL(input instanceof Request?input.url:input,import.meta.url),base=new URL('.',import.meta.url);
  if(url.origin!==base.origin||!url.pathname.startsWith(base.pathname)||(options.method&&options.method!=='GET'))return networkFetch(input,options);
  const [manifest,cache]=await prepareAssetCache(),name=url.pathname.slice(base.pathname.length),entry=manifest.files?.[name];
  if(!entry||(url.searchParams.has('sha256')&&url.searchParams.get('sha256')!==entry.sha256))return networkFetch(input,options);
  const key=new URL(name,base);key.searchParams.set('asset_sha256',entry.sha256);
  if(!pending.has(key.href)){
    const task=(async()=>{
      if(cache){
        try{
          const stored=await cache.match(key.href);
          if(stored){
            const bytes=new Uint8Array(await stored.arrayBuffer());
            if(bytes.length===entry.bytes&&await hash(bytes)===entry.sha256){assetCacheMetrics.hits++;assetCacheMetrics.cache_bytes+=bytes.length;return {bytes,type:entry.type};}
            await cache.delete(key.href);
          }
        }catch{}
      }
      const response=await networkFetch(input,options);if(!response.ok)throw Error(`资源加载失败 ${name} (${response.status})`);
      const bytes=new Uint8Array(await response.arrayBuffer()),actualHash=await hash(bytes);
      if(bytes.length!==entry.bytes||(actualHash&&actualHash!==entry.sha256))throw Error(`资源版本不匹配 ${name}，请刷新重试`);
      assetCacheMetrics.misses++;assetCacheMetrics.download_bytes+=bytes.length;
      const type=entry.type||response.headers.get('Content-Type')||'application/octet-stream';
      if(cache&&actualHash)try{await cache.put(key.href,new Response(bytes,{headers:{'Content-Type':type}}));}catch{}
      return {bytes,type};
    })();
    pending.set(key.href,task);task.finally(()=>pending.delete(key.href)).catch(()=>{});
  }
  const {bytes,type}=await pending.get(key.href);
  return new Response(bytes,{status:200,headers:{'Content-Type':type||'application/octet-stream','Content-Length':String(bytes.length)}});
}
export async function installAssetCache(){await prepareAssetCache();if(!installed){globalThis.fetch=cachedAssetFetch;installed=true;}}
