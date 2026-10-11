/*  t9f_activeset_probe.js —— 活动集（冻结顶限位关节 + 重新分配步长）值不值？
    用法： node t9f_activeset_probe.js [位姿数=320]                                   */
const { loadEnv, makePoses, makeSeeds } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const LIM = A.qLimits();
const NP = parseInt(process.argv[2] || '320', 10);
const poses = makePoses(A, NP);
const seeds = makeSeeds(A, NP, 8);
const T = poses.map(P => ({ Eb: A.taskToBase([P.x, P.y, P.z]),
                            dir: A.dirToBase(A.coreFrame(P).e).map(v => -v) }));
function rate(o, nStart){
  let ok = 0;
  for(let i=0;i<NP;i++) for(let s=0;s<nStart;s++){
    if(V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, o).ok){ ok++; break; }
  }
  return 100*ok/NP;
}
const CFG = [
  ['A 页面原样',                            {}],
  ['B +活动集',                             {activeSet:true}],
  ['C +活动集 +无 wO 门控',                 {activeSet:true, wOgate:false}],
  ['D +活动集 +无门控 +保最优',              {activeSet:true, wOgate:false, keepBest:true}],
  ['E D +迭代 500',                         {activeSet:true, wOgate:false, keepBest:true, iters:500}],
  ['F E +λ下限5 +λ上限200',                 {activeSet:true, wOgate:false, keepBest:true, iters:500, lamMin:5, lamMax:200}],
  ['G F +整条等比缩步',                      {activeSet:true, wOgate:false, keepBest:true, iters:500, lamMin:5, lamMax:200, clampMode:'scale'}],
];
console.log('=== 活动集对照（%d 个位姿，起点序列相同）===', NP);
console.log('配置'.padEnd(30) + '1起点'.padStart(9) + '2起点'.padStart(9) +
            '3起点'.padStart(9) + '5起点'.padStart(9));
for(const [name, o] of CFG){
  console.log(name.padEnd(26) + rate(o,1).toFixed(1).padStart(9) + '%' +
              rate(o,2).toFixed(1).padStart(8) + '%' + rate(o,3).toFixed(1).padStart(8) + '%' +
              rate(o,5).toFixed(1).padStart(8) + '%');
}
