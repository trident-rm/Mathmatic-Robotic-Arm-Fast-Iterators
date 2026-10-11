/*  t11_summary.js —— 出报告要用的几何数字（30k 采样的可达域对照） */
const { loadEnv } = require('./t9_load.js');
const { A, LINK } = loadEnv({ variants:false, link:true });
const L = LINK;
A.buildChain(); L.init(true);
const NAMES = L.knobs().map(k=>k.name);
const BASE = L.readModel();
const D = Object.assign({}, Object.fromEntries(NAMES.map((n,i)=>[n,BASE[i]])),
  {'link2.y':235, 'link3.x':45, 'gripper.z':150});
const Dv = NAMES.map(n=>D[n]);

function look(vals){
  const keep = L.readModel();
  L.writeVals(vals);
  const ws = L.workspace(30000);
  const segs = L.segLens();
  const out = { segs, armLen: segs.reduce((a,b)=>a+b,0), ...ws };
  L.writeVals(keep);
  return out;
}
const b = look(BASE), d = look(Dv);
const f = v => v.toFixed(1);
console.log('项                        基准            推荐D');
console.log('各段杆长 mm               ' + b.segs.map(v=>v.toFixed(0)).join('/') + '   ' + d.segs.map(v=>v.toFixed(0)).join('/'));
console.log('杆长之和 mm               ' + f(b.armLen).padStart(8) + '        ' + f(d.armLen));
console.log('最大伸展 mm（受限于限位） ' + f(b.reach).padStart(8) + '        ' + f(d.reach));
console.log('末端高度范围 z            ' + (b.zmin.toFixed(0)+'~'+b.zmax.toFixed(0)).padStart(8) + '        ' + (d.zmin.toFixed(0)+'~'+d.zmax.toFixed(0)));
console.log('距 Z 轴最大水平距离       ' + f(b.rhomax).padStart(8) + '        ' + f(d.rhomax));
console.log('三级任务域位置可达率      ' + (100*b.posRate).toFixed(1).padStart(7) + '%       ' + (100*d.posRate).toFixed(1) + '%');
console.log('可达体素（50mm，30k 采样）' + String(b.vox).padStart(8) + '        ' + String(d.vox));
/* ---- --diag：报告 §3 那张"逐位姿最紧关节裕度"表（基准臂，给足预算） ---- */
if(process.argv.includes('--diag')){
  const NP = 24, NS = 24, IT = 600;
  console.log('\n=== 基准臂逐位姿诊断（' + NP + ' 位姿 × ' + NS + ' 起点 × ' + IT + ' 迭代）===');
  const LV = A.RULES.levels[3], rng = require('./t9_load.js').mulberry32(1234567);
  const rr = ([lo,hi]) => lo + rng()*(hi-lo);
  const LIM = A.qLimits(), NAM = A.CHAIN.filter(c=>c.movable).map(c=>c.joint);
  const hits = NAM.map(()=>0);
  console.log('位姿  最佳位置误差  最佳姿态误差  该解的关节裕度(°)                    顶到<3°的关节');
  const margs = [], angs = [];
  for(let i=0;i<NP;i++){
    const P = {x:rr(LV.x), y:rr(LV.y), z:rr(LV.z), th:rr(LV.th), ph:rr(LV.ph), al:rr(LV.al)};
    const Eb = A.taskToBase([P.x,P.y,P.z]);
    const eB = A.dirToBase(A.coreFrame(P).e);
    const ey = [-eB[0],-eB[1],-eB[2]];
    let best = null;
    for(let s=0;s<NS;s++){
      const seed = s===0 ? new Array(A.NJ).fill(0) : LIM.map(([lo,hi])=>lo+Math.random()*(hi-lo));
      const r = A.ikSolve6(Eb, ey, seed, IT, 1);
      const sc = Math.hypot(r.e, 40*(r.ang||0));
      if(!best || sc < best.sc) best = {sc, r};
    }
    const r = best.r, m = r.q.map((q,k)=>Math.min(q-LIM[k][0], LIM[k][1]-q));
    margs.push(Math.min(...m)); angs.push((r.ang||0)*180/Math.PI);
    m.forEach((v,k)=>{ if(v<3) hits[k]++; });
    console.log(`  ${String(P.x.toFixed(0)).padStart(4)},${String(P.y.toFixed(0)).padStart(4)},${String(P.z.toFixed(0)).padStart(4)}` +
      `  ${r.e.toFixed(2).padStart(7)}mm  ${((r.ang||0)*180/Math.PI).toFixed(2).padStart(6)}°   ` +
      `[${m.map(v=>v.toFixed(0).padStart(4)).join(' ')}]  ` +
      m.map((v,k)=> v<3 ? NAM[k] : null).filter(Boolean).join(','));
  }
  const med = a=>{ const s=a.slice().sort((x,y)=>x-y); return s[Math.floor(s.length/2)]; };
  console.log('中位最佳姿态误差 = ' + med(angs).toFixed(2) + '°　中位最紧关节裕度 = ' + med(margs).toFixed(1) + '°');
  console.log('顶到限位(<3°)次数：' + NAM.map((n,k)=>n+' '+hits[k]).join(' · '));
}

