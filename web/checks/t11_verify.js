/*  t11_verify.js —— 把候选连杆放到【多套独立位姿集】上复核（防止只在搜索用的那套位姿上好看）
    跑法： node t11_verify.js            （约 1~2 min）
    读：   t11_best.json（t11_search.js 的产物）；没有就只用内置的候选
    打印：每个候选 × 3 套位姿（不同随机种子）的 分数 / 合格率 / 裕度 / 姿态 / 未解出 / 杆长和 /
          最大伸展 / 位置可达率
*/
const fs = require('fs'), path = require('path');
const { loadEnv } = require('./t9_load.js');
const P = require('../lib/paths');
const { A, sandbox } = loadEnv({ variants:false, file:'armLink.html' });
const L = sandbox.LINK;
const dir = __dirname;
A.buildChain(); L.init(true);

const K = L.knobs();
const NAMES = K.map(k=>k.name);
const BASE = L.readModel();
const at = obj => BASE.map((v,i)=> (obj[NAMES[i]] !== undefined ? obj[NAMES[i]] : v));
const best = fs.existsSync(path.join(dir,'t11_best.json')) ?
  JSON.parse(fs.readFileSync(path.join(dir,'t11_best.json'),'utf8')) : null;
const bestVals = best && best.best ? best.best.links.map(o=>o.v) : null;
const bestObj = bestVals ? Object.fromEntries(NAMES.map((n,i)=>[n, bestVals[i]])) : null;

const CANDS = [];
CANDS.push({tag:'基准（原臂）', vals:BASE});
CANDS.push({tag:'A 工具150 + 横偏45',            vals:at({'gripper.z':150,'link3.x':45})});
CANDS.push({tag:'B A + 侧偏−85',                 vals:at({'gripper.z':150,'link3.x':45,'link4.y':-85})});
CANDS.push({tag:'C A + 肩高125',                 vals:at({'gripper.z':150,'link3.x':45,'link1.z':125})});
CANDS.push({tag:'D A + 大臂235',                 vals:at({'gripper.z':150,'link3.x':45,'link2.y':235})});
CANDS.push({tag:'E A + 小臂275',                 vals:at({'gripper.z':150,'link3.x':45,'link4.z':275})});
CANDS.push({tag:'F 工具120 + 横偏45',            vals:at({'gripper.z':120,'link3.x':45})});
CANDS.push({tag:'G 工具150 + 横偏30',            vals:at({'gripper.z':150,'link3.x':30})});
CANDS.push({tag:'H 六根全改（圆整）',            vals:at({'gripper.z':140,'link3.x':110,'link4.y':-85,
                                                         'link1.z':125,'link2.y':235,'link4.z':275})});
if(bestObj) CANDS.push({tag:'搜索最优', vals:at(bestObj)});

/* ---- 可选：用外部 json 覆盖候选/位姿集/预算 ----
   格式 { "sets":[20260925,111], "cfg":{"nPose":96,"budget":12,"iters":400},
          "cands":[{"tag":"D","vals":{"gripper.z":150,"link3.x":45,"link2.y":235}}] }
   vals 里只写要改的旋钮名（其余按基准） */
let SETS_ARG = null, CFG_ARG = null, CANDS_ARG = null;
const argFile = process.argv.slice(2).find(a=>/\.json$/i.test(a));
if(argFile){
  const j = JSON.parse(fs.readFileSync(path.resolve(argFile),'utf8'));
  if(j.sets) SETS_ARG = j.sets;
  if(j.cfg)  CFG_ARG  = j.cfg;
  if(j.cands) CANDS_ARG = j.cands.map(c=>({tag:c.tag, vals:at(c.vals)}));
  console.log('（用 ' + argFile + ' 覆盖：' + (j.cands?j.cands.length+' 个候选':'候选不变') + '）');
}

let SETS = SETS_ARG || [20260925, 111, 222];
let CFG  = Object.assign({nPose:48, budget:10, iters:350, eTol:3, aTol:3, mGate:5, mCap:15}, CFG_ARG||{});
if(CANDS_ARG) CANDS.length = 0;
if(CANDS_ARG) CANDS.push(...CANDS_ARG);
console.log('复核条件：每位姿集 ' + CFG.nPose + ' 位姿 × ' + CFG.budget + ' 起点 × ' + CFG.iters +
            ' 迭代，判据 位置<' + CFG.eTol + 'mm 姿态<' + CFG.aTol + '° 裕度>' + CFG.mGate + '°');
console.log('位姿集种子：' + SETS.join(' / ') + '（每个种子 = 一套独立的分层采样）\n');

const hdr = ['方案'.padEnd(34), '位姿集', '分数', '合格率', '裕度中位', '姿态中位', '未解出', '杆长和', '最大伸展', '位置可达'];
console.log(hdr.join('  '));
const rows = [];
for(const c of CANDS){
  const per = [];
  for(const s of SETS){
    const cfg = Object.assign({}, L.cfg, CFG, {seed:s});
    const r = L.eval(c.vals, c.tag, cfg, {ws:true, wsN:2500});
    per.push(r);
    console.log([c.tag.padEnd(30), String(s).padEnd(8),
      r.score.toFixed(1).padStart(5), (100*r.pass).toFixed(0).padStart(5)+'%',
      (r.margMed.toFixed(1)+'°').padStart(7), (r.angMed.toFixed(2)+'°').padStart(7),
      ((100*(1-r.found)).toFixed(0)+'%').padStart(5),
      r.armLen.toFixed(0).padStart(6), (r.reach||0).toFixed(0).padStart(7),
      ((100*(r.posRate||0)).toFixed(1)+'%').padStart(7)].join('  '));
  }
  const mean = k => per.reduce((a,r)=>a+r[k],0)/per.length;
  rows.push({tag:c.tag, vals:c.vals, score:mean('score'), pass:mean('pass'), marg:mean('margMed'),
             ang:mean('angMed'), found:mean('found'), armLen:per[0].armLen, reach:per[0].reach,
             posRate:per[0].posRate, segs:per[0].segs, limitOK:per.every(r=>r.limitOK)});
  console.log(''.padEnd(30) + '  平均    ' + rows[rows.length-1].score.toFixed(1).padStart(4) + '\n');
}
const b = rows[0];
console.log('=== 汇总（3 套位姿的平均）===');
rows.sort((x,y)=>y.score-x.score);
rows.forEach(r=>{
  const d = [];
  NAMES.forEach((n,i)=>{ if(Math.abs(r.vals[i]-BASE[i])>0.5) d.push(n+' '+BASE[i].toFixed(0)+'→'+r.vals[i].toFixed(0)); });
  console.log(`  ${r.tag.padEnd(32)} 分数 ${r.score.toFixed(1).padStart(5)}  ` +
    `合格 ${(100*r.pass).toFixed(0).padStart(3)}%  相对基准 ${(r.score-b.score>=0?'+':'')}${(r.score-b.score).toFixed(1)}  ` +
    `杆长和 ${r.armLen.toFixed(0)}  限位未改=${r.limitOK}\n      ${d.join(' · ')||'（没改）'}`);
});
console.log('\n各段杆长：');
rows.forEach(r=>console.log('  ' + r.tag.padEnd(32) + r.segs.map(v=>v.toFixed(0).padStart(5)).join('')));
console.log('  ' + '基准'.padEnd(32) + (function(){ const keep=L.readModel();
  L.writeVals(BASE); const g=L.segLens(); L.writeVals(keep); return g.map(v=>v.toFixed(0).padStart(5)).join(''); })());
fs.writeFileSync(path.join(P.DATA,'t11_verify.json'), JSON.stringify(
  rows.map(r=>({tag:r.tag, score:+r.score.toFixed(2), pass:+(100*r.pass).toFixed(1),
                margMed:+r.marg.toFixed(2), angMed:+r.ang.toFixed(2), found:+(100*r.found).toFixed(1),
                armLen:+r.armLen.toFixed(1), reach:+(r.reach||0).toFixed(0),
                posRate:+(100*(r.posRate||0)).toFixed(2), segs:r.segs.map(v=>+v.toFixed(2)),
                links:NAMES.map((n,i)=>({name:n, v0:+BASE[i].toFixed(2), v:+r.vals[i].toFixed(2)}))})), null, 1), 'utf8');
console.log('\n已写 t11_verify.json');
