/*  t9d_step_probe.js —— 步长处理方式的影响：每个关节各自裁 / 整条等比缩小 / 不裁
    假说：页面把 Δq 按【每个关节各自】裁到 ±0.2rad，会破坏"J·Δq = r"这个线性约束，
          位置本来已经解准了，一裁就把它推走，之后再也回不来。
    用法： node t9d_step_probe.js [位姿数=320]                                       */
const { loadEnv, makePoses, makeSeeds } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const LIM = A.qLimits();
const NP = parseInt(process.argv[2] || '320', 10);
const poses = makePoses(A, NP);
const seeds = makeSeeds(A, NP, 12);
const T = poses.map(P => ({ Eb: A.taskToBase([P.x, P.y, P.z]),
                            dir: A.dirToBase(A.coreFrame(P).e).map(v => -v) }));

function rate(o, nStart){
  let ok = 0;
  for(let i=0;i<NP;i++) for(let s=0;s<nStart;s++){
    if(V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, o).ok){ ok++; break; }
  }
  return 100*ok/NP;
}
/* 也给"零位起点"单独看：面板第 1 个起点就是它 */
function rateSeed0(o){
  let ok = 0;
  for(let i=0;i<NP;i++) if(V.solve6(A, T[i].Eb, T[i].dir, seeds[i][0], LIM, o).ok) ok++;
  return 100*ok/NP;
}

const CFG = [
  ['A 页面原样（逐关节裁 0.2rad）',   {}],
  ['B 不裁（λ 控制步长）',            {clampMode:'none'}],
  ['C 整条等比缩到 0.2rad',           {clampMode:'scale'}],
  ['D 整条等比缩到 0.6rad',           {clampMode:'scale', maxStep:0.6}],
  ['E 逐关节裁 0.6rad',               {maxStep:0.6}],
  ['F 整条等比 0.2 + 无 wO 门控',     {clampMode:'scale', wOgate:false}],
  ['G 整条等比 0.2 + 无门控 + 保最优', {clampMode:'scale', wOgate:false, keepBest:true}],
  ['H G + λ下限 5',                   {clampMode:'scale', wOgate:false, keepBest:true, lamMin:5}],
  ['I G + 迭代 600',                  {clampMode:'scale', wOgate:false, keepBest:true, iters:600}],
];
console.log('=== 步长处理对照（%d 个位姿）===', NP);
console.log('配置'.padEnd(34) + '零位起点'.padStart(10) + '1起点'.padStart(9) +
            '3起点'.padStart(9) + '8起点'.padStart(9));
for(const [name, o] of CFG){
  console.log(name.padEnd(30) + rateSeed0(o).toFixed(1).padStart(10) + '%' +
              rate(o,1).toFixed(1).padStart(8) + '%' + rate(o,3).toFixed(1).padStart(8) + '%' +
              rate(o,8).toFixed(1).padStart(8) + '%');
}
