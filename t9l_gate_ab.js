/*  t9l_gate_ab.js —— 门控去留的干净 A/B（选择判据与 ok 判据一致：任一 ok 即 ok）
    背景：t8 之前按"位置误差最小"挑起点解，存在"位置更准但姿态差"的解盖掉真 ok 解的假失败，
          所以早先"去掉门控 → 强多起点从 100% 掉到 92%"这个结论需要用干净的判据复核。
    用法： node t9l_gate_ab.js [位姿数=320]                                          */
const { loadEnv, makePoses, makeSeeds } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const LIM = A.qLimits();
const NP = parseInt(process.argv[2] || '320', 10);
const poses = makePoses(A, NP);
const seeds = makeSeeds(A, NP, 24);
const T = poses.map(P => ({ Eb: A.taskToBase([P.x, P.y, P.z]),
                            dir: A.dirToBase(A.coreFrame(P).e).map(v => -v) }));
const score = r => r.ok ? -1 : Math.hypot(r.e, 40 * (r.ang || 0));

/* nStart 个起点里"只要有一个 ok 就算 ok"；同时统计"被次优解盖掉"的次数 */
function run(o, nStart){
  let ok = 0, shadowed = 0, t0 = Date.now();
  for(let i=0;i<NP;i++){
    let okHit = false, best = null;
    for(let s=0;s<nStart;s++){
      const r = V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, o);
      if(r.ok) okHit = true;
      if(!best || score(r) < score(best)) best = r;
      if(okHit) break;
    }
    if(okHit) ok++;
    else if (best && false) shadowed++;
  }
  return { pct: 100*ok/NP, sec: (Date.now()-t0)/1000 };
}
console.log('=== 门控 A/B（%d 个三级任务域位姿，任一 ok 即 ok）===', NP);
console.log('配置'.padEnd(44) + '3起点'.padStart(9) + '12起点'.padStart(9) + '24起点'.padStart(9));
const CFG = [
  ['门控开 + 不保最优 + 200迭代（≈打补丁前）', {keepBest:false}, 200],
  ['门控开 + 保最优 + 200迭代（现在页面里的）', {keepBest:true}, 200],
  ['门控开 + 保最优 + 400迭代',                {keepBest:true}, 400],
  ['门控关 + 保最优 + 200迭代',                {keepBest:true, wOgate:false}, 200],
  ['门控关 + 保最优 + 400迭代',                {keepBest:true, wOgate:false}, 400],
];
for(const [name, o, it] of CFG){
  const oo = Object.assign({ iters: it }, o);
  const a = run(oo, 3), b = run(oo, 12), c = run(oo, 24);
  console.log(name.padEnd(40) + a.pct.toFixed(1).padStart(8) + '%' +
              b.pct.toFixed(1).padStart(8) + '%' + c.pct.toFixed(1).padStart(8) + '%');
}
