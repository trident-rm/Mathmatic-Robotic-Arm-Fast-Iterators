/*  t11_search.js —— 离线跑连杆搜索（直接用 web/src/modules/t11_link_mod.js 这个引擎）
    跑法：
      node t11_search.js                # 默认：搜索 3 个随机种子各一轮，再复核
      node t11_search.js --quick        # 只跑一轮小预算（约 30 s）
      node t11_search.js --big          # 大预算（几分钟）
    输出：控制台表格 + t11_best.json（推荐连杆）
*/
const fs = require('fs'), path = require('path');
const { loadEnv } = require('./t9_load.js');
const P = require('../lib/paths');
const { A, LINK } = loadEnv({ variants:false, link:true });
const L = LINK;
const ARGV = process.argv.slice(2);
const QUICK = ARGV.includes('--quick'), BIG = ARGV.includes('--big');
const dir = __dirname;

A.buildChain();
L.init(true);
const K = L.knobs();
const sc = k => k.on ? '' : '·';
console.log('=== 基准臂 ===');
console.log('  杆长 = ' + L.segLens().map(v=>v.toFixed(1)).join(' / ') + '   和 = ' + L.armLen().toFixed(1) + ' mm');
console.log('  限位 = ' + L.limSig());
console.log('  旋钮 = ' + K.map(k=>k.name+(k.on?'*':'')).join(' '));

const BASE = { score:null };
const NAME = k => k.name;

/* ---- 预算档位 ---- */
const PROBE = QUICK ? {nPose:10, budget:3, iters:180} : {nPose:12, budget:4, iters:220};
const FINAL = QUICK ? {nPose:24, budget:6, iters:280}
                    : BIG  ? {nPose:64, budget:12, iters:420}
                           : {nPose:48, budget:8, iters:320};
const SHOW = r => `分数 ${r.score.toFixed(1).padStart(5)}  合格 ${(100*r.pass).toFixed(0).padStart(3)}%  ` +
                  `裕度中位 ${r.margMed.toFixed(1).padStart(5)}°  姿态中位 ${r.angMed.toFixed(2).padStart(5)}°  ` +
                  `未解出 ${(100*(1-r.found)).toFixed(0).padStart(3)}%  杆长和 ${r.armLen.toFixed(0)}`;

function evaluateAt(vals, tag, cfg, ws){
  const keep = L.S.modelVals;
  const r = L.eval(vals, tag, Object.assign({}, L.cfg, cfg), {ws:ws});
  return r;
}
function baseVals(){ return L.readModel(); }

/* ---- ① 基准（探测预算） ---- */
Object.assign(L.cfg, PROBE);
let t0 = Date.now();
const rBase = L.eval(baseVals(), '基准', L.cfg);
BASE.score = rBase.score;
console.log('\n① 基准（' + PROBE.nPose + '位姿 × ' + PROBE.budget + '起点 × ' + PROBE.iters + '迭代）：');
console.log('  ' + SHOW(rBase) + '   (' + ((Date.now()-t0)/1000).toFixed(1) + 's)');
console.log('  逐位姿最紧关节裕度 = ' + rBase.per.map(p=>p.found?p.marg.toFixed(0):'×').join(' '));

/* ---- ② 灵敏度扫描 ---- */
console.log('\n② 灵敏度扫描（每根杆 7 档）');
t0 = Date.now();
const sw = L.sweepSync(PROBE);
console.log('  ' + sw.n + ' 个候选，' + (sw.ms/1000).toFixed(1) + ' s，限位未改动 = ' + sw.limitOK);
const byKnob = new Map();
sw.points.forEach(p=>{ if(!byKnob.has(p.key)) byKnob.set(p.key, {name:p.name, a:[]}); byKnob.get(p.key).a.push(p); });
const rank = [];
byKnob.forEach((g,k)=>{
  const mx = Math.max(...g.a.map(p=>p.score)), mn = Math.min(...g.a.map(p=>p.score));
  let bi = 0; g.a.forEach((p,i)=>{ if(p.score > g.a[bi].score) bi = i; });
  rank.push({k, name:g.name, span:mx-mn, mx, mn, best:g.a[bi], a:g.a});
});
rank.sort((x,y)=>y.span-x.span);
console.log('  按"这根杆能撬动多少分"排序：');
rank.forEach(r=>console.log(`    ${r.name.padEnd(18)} 分数区间 ${r.mn.toFixed(1)}~${r.mx.toFixed(1)}（动${r.span.toFixed(1)}）` +
  `  最好 ${r.best.val.toFixed(0)} → ${r.best.score.toFixed(1)}   全部: ` +
  r.a.map(p=>p.val.toFixed(0)+':'+p.score.toFixed(0)).join(' ')));

/* ---- ③ 搜索 ---- */
const SEEDS = QUICK ? [20260925] : (BIG ? [20260925, 777, 31415] : [20260925, 777]);
let best = null;
for(const s of SEEDS){
  console.log('\n③ 搜索  seed=' + s + '  起点数 ' + (QUICK?6:12) + '  预算 ' + JSON.stringify(PROBE));
  t0 = Date.now();
  const r = L.searchSync(Object.assign({}, PROBE, {seed:s, restarts:QUICK?6:12}));
  console.log('  评估 ' + r.n + ' 个候选，' + (r.ms/1000).toFixed(1) + 's，限位未改动 = ' + r.limitOK);
  if(r.best) console.log('  最好：' + SHOW(r.best));
  if(r.best && (!best || r.best.score > best.score)) best = r.best;
}

/* ---- ④ 复核：换一套位姿（另一个种子）+ 大预算，看是不是过拟合 ---- */
function verify(vals, tag){
  const cfg = Object.assign({}, L.cfg, FINAL);
  const r = L.eval(vals, tag, cfg, {ws:true});
  return r;
}
console.log('\n④ 复核（' + FINAL.nPose + '位姿 × ' + FINAL.budget + '起点 × ' + FINAL.iters + '迭代）');
const vBase = verify(baseVals(), '基准');
console.log('  基准  ' + SHOW(vBase));
if(best){
  const vBest = verify(best.vals, '搜索最优');
  console.log('  最优  ' + SHOW(vBest));
  console.log('\n  改动（基准 → 最优）：');
  L.S.knobs.forEach((k,i)=>{
    const a = k.v0, b = best.vals[i];
    if(Math.abs(a-b) > 0.5) console.log(`    ${k.name.padEnd(18)} ${a.toFixed(1).padStart(8)} → ${b.toFixed(1).padStart(8)}  (${b>a?'+':''}${(b-a).toFixed(1)})`);
  });
  const out = { when:new Date().toISOString(),
    budget:{probe:PROBE, final:FINAL},
    limits:L.limSig(),
    baseline:{score:+vBase.score.toFixed(2), pass:+(100*vBase.pass).toFixed(1), margMed:+vBase.margMed.toFixed(2),
              angMed:+vBase.angMed.toFixed(2), found:+(100*vBase.found).toFixed(1), armLen:+vBase.armLen.toFixed(2),
              reach:+(vBase.reach||0).toFixed(0), posRate:+(100*(vBase.posRate||0)).toFixed(1),
              segs:vBase.segs.map(v=>+v.toFixed(2))},
    best:{ score:+vBest.score.toFixed(2), pass:+(100*vBest.pass).toFixed(1), margMed:+vBest.margMed.toFixed(2),
           angMed:+vBest.angMed.toFixed(2), found:+(100*vBest.found).toFixed(1), armLen:+vBest.armLen.toFixed(2),
           reach:+(vBest.reach||0).toFixed(0), posRate:+(100*(vBest.posRate||0)).toFixed(1),
           segs:vBest.segs.map(v=>+v.toFixed(2)),
           links:L.S.knobs.map((k,i)=>({name:k.name, li:k.li, c:k.c, v0:+k.v0.toFixed(2), v:+best.vals[i].toFixed(2)})) },
    final:{score:+vBest.score.toFixed(2)},
    sensitivity: rank.map(r=>({name:r.name, span:+r.span.toFixed(2), best:r.best.val,
                               pts:r.a.map(p=>[+p.val.toFixed(1), +p.score.toFixed(2)])}))
  };
  fs.writeFileSync(path.join(P.DATA,'t11_best.json'), JSON.stringify(out,null,1), 'utf8');
  console.log('\n  已写 t11_best.json');
}else console.log('  搜索没给出候选（旋钮一个都没勾？）');
