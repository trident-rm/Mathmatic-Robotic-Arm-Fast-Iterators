/*  t9c_lm_probe.js —— 修复方案对照：页面原样 vs 最小补丁 vs 真 LM
    关键：objective 统一成真残差范数 ||[位置误差 ; 40·姿态轴角误差]||（= DLS 实际最小化的量），
    这样"保留最优/接受判据"才有意义。
    用法： node t9c_lm_probe.js [位姿数=320]                                        */
const { loadEnv, makePoses, makeSeeds } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const LIM = A.qLimits(), L = A.RULES.levels[3];
const NP = parseInt(process.argv[2] || '320', 10);
const poses = makePoses(A, NP);
const seeds = makeSeeds(A, NP, 12);
const T = poses.map(P => ({ Eb: A.taskToBase([P.x, P.y, P.z]),
                            dir: A.dirToBase(A.coreFrame(P).e).map(v => -v) }));

/* 每种配置：nStart 个起点里命中即停 */
function rate(o, nStart){
  let ok = 0, used = 0, ms = 0;
  const t0 = Date.now();
  for(let i=0;i<NP;i++){
    for(let s=0;s<nStart;s++){
      const r = V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, o);
      if(r.ok){ ok++; used += s+1; break; }
    }
  }
  ms = Date.now() - t0;
  return { pct: 100*ok/NP, used: ok ? used/ok : NaN, ms: ms/NP };
}

const CFG = [
  ['P0 页面原样',                          {}],
  ['P1 保留最优解',                        {keepBest:true}],
  ['P2 P1 + λ 下限 5',                     {keepBest:true, lamMin:5}],
  ['P3 P2 + 去掉 wO 门控',                 {keepBest:true, lamMin:5, wOgate:false}],
  ['P4 P3 + 迭代 600',                     {keepBest:true, lamMin:5, wOgate:false, iters:600}],
  ['L1 真LM(接受判据+λ自适应)',            {keepBest:true, wOgate:false, lamAdapt:true, lam0:10, iters:200}],
  ['L2 L1 + 位置种子 + 无门控',            {keepBest:true, wOgate:false, lamAdapt:true, lam0:10, posIters:200, iters:200}],
  ['L3 L2 + 迭代 400',                     {keepBest:true, wOgate:false, lamAdapt:true, lam0:10, posIters:200, iters:400}],
  ['L4 L3 + 线搜索',                       {keepBest:true, wOgate:false, lamAdapt:true, lineSearch:true, lam0:10, posIters:200, iters:400}],
];
console.log('=== 修复方案对照（%d 个三级任务域位姿，起点序列完全相同）===', NP);
console.log('配置'.padEnd(34) + '1起点'.padStart(8) + '3起点'.padStart(8) +
            '8起点'.padStart(8) + '平均用起点'.padStart(11) + 'ms/次'.padStart(8));
const rows = [];
for(const [name, o] of CFG){
  const r1 = rate(o,1), r3 = rate(o,3), r8 = rate(o,8);
  rows.push({name, r1, r3, r8});
  console.log(name.padEnd(30) + r1.pct.toFixed(1).padStart(9) + '%' +
              r3.pct.toFixed(1).padStart(7) + '%' + r8.pct.toFixed(1).padStart(7) + '%' +
              r3.used.toFixed(2).padStart(11) + r1.ms.toFixed(2).padStart(8));
}
const base = rows[0], best = rows[rows.length-1];
console.log('\n对比：3 起点可解率 ' + base.r3.pct.toFixed(1) + '% → ' + best.r3.pct.toFixed(1) + '%' +
            ' ；1 起点 ' + base.r1.pct.toFixed(1) + '% → ' + best.r1.pct.toFixed(1) + '%');
