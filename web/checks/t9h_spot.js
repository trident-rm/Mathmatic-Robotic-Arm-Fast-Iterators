const { loadEnv, makePoses, makeSeeds } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const LIM = A.qLimits();
const poses = makePoses(A, 320);
const seeds = makeSeeds(A, 320, 64);
const T = poses.map(P => ({ Eb: A.taskToBase([P.x,P.y,P.z]), dir: A.dirToBase(A.coreFrame(P).e).map(v=>-v) }));
const FIX = { wOgate:false, keepBest:true, iters:400, posIters:200 };
for(const i of [124,170,274,286,298]){
  let firstOk=-1, bestE=1e9, bestA=1e9;
  for(let s=0;s<64;s++){
    const r = V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, FIX);
    if(r.e<bestE){bestE=r.e;}
    if(r.ang<bestA){bestA=r.ang;}
    if(r.ok && firstOk<0) firstOk=s;
  }
  // 页面原样跑 64 起点
  let firstOkPage=-1;
  for(let s=0;s<64;s++){ if(A.ikSolve6(T[i].Eb, T[i].dir, seeds[i][s], 200, 1).ok){ firstOkPage=s; break; } }
  console.log(`#${i}  ph=${poses[i].ph.toFixed(0)}° z=${poses[i].z.toFixed(0)}  修好版: 第${firstOk+1}个起点命中(共64) 最优位置误差${bestE.toFixed(2)}mm 最小姿态偏差${(bestA*180/Math.PI).toFixed(2)}°  |  页面原样: ${firstOkPage<0?'64起点仍失败':'第'+(firstOkPage+1)+'个起点命中'}`);
}
