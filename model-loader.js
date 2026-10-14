// Avoid addBuffer's per-byte JS/C++ conversion for the large compiled scene.
// MEMFS owns the same binary bytes; MuJoCo parses the unchanged MJB file.
export function loadBinaryModel(mj,name,bytes){
  const path=`/grit-${name}.mjb`,vfs=new mj.MjVFS();let written=false;
  try{
    mj.FS.writeFile(path,bytes,{canOwn:true});written=true;
    return mj.MjModel.from_binary_path(path,vfs);
  }finally{
    vfs.delete();if(written)mj.FS.unlink(path);
  }
}
