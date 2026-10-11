/*  t9g_recommend.js —— 推荐组合的最终验收（320 个位姿，与面板同目标分布）
    推荐：① 去掉 wO 门控 ② 保留历史最优 ③ 迭代 400 ④ 起点 3 → 12
    用法： node t9g_recommend.js                                                        */
const { loadEnv, makePoses, makeSeeds } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const LIM = A.qLimits();
const NP = 320;
const poses = makePoses(A, NP);
const seeds = makeSeeds(A, NP, 16);
const T = poses.map(P => ({ Eb: A.taskToBase([P.x, P.y, P.z]),
                            dir: A.dirToBase(A.coreFrame(P).e).map(v => -v) }));
function run(o, nStart){
  let ok = 0, fails = [], t0 = Date.now();
  for(let i=0;i<NP;i++){
    let best = null;
    for(let s=0;s<nStart;s++){
      const r = V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, o);
      if(!best || (r.e + 40*r.ang) < (best.e + 40*best.ang)) best = r;
      if(r.ok) break;
    }
    if(best && best.ok) ok++; else fails.push(i);
  }
  return { pct: 100*ok/NP, ok, fails, sec: (Date.now()-t0)/1000 };
}
const FIX = { wOgate:false, keepBest:true, iters:400, posIters:200 };
console.log('=== 最终验收（%d 个三级任务域位姿）===', NP);
console.log('配置'.padEnd(38) + '可解率'.padStart(9) + '耗时'.padStart(9));
for(const [name, o, ns] of [
  ['面板原样（3 起点 / 200 迭代 / 有门控）', {}, 3],
  ['面板原样（12 起点）',                    {}, 12],
  ['只加起点到 12（不改算法）',               {}, 12],
  ['去门控+保最优+400迭代（3 起点）',         FIX, 3],
  ['去门控+保最优+400迭代（8 起点）',         FIX, 8],
  ['★ 去门控+保最优+400迭代（12 起点）',      FIX, 12],
  ['去门控+保最优+400迭代（16 起点）',        FIX, 16],
]){
  if(name.indexOf('只加起点') === 0) continue;
  const r = run(o, ns);
  console.log(name.padEnd(34) + r.pct.toFixed(1).padStart(8) + '%' +
              (r.sec.toFixed(1)+'s').padStart(9) + (r.fails.length ? '   仍失败: ' + r.fails.join(',') : ''));
}
