/*  t9_ik_probe.js —— 定位 IK 求解器本身的缺陷：单起点为什么失败、需要几个起点、哪个改动最有效
    用法： node t9_ik_probe.js [位姿数=160] [最大起点数=32]
    依赖： armBuilder.html（被抽出原样执行） + t9_variants.js（变体求解器）          */
const fs = require('fs'), path = require('path'), vm = require('vm');

const html = fs.readFileSync(path.join(__dirname, 'armBuilder.html'), 'utf8');
const blocks = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
const variants = fs.readFileSync(path.join(__dirname, 't9_variants.js'), 'utf8');

const mkEl = () => new Proxy({ style:{}, classList:{ add(){}, remove(){} },
  dataset:{}, value:'30000', textContent:'', innerHTML:'', checked:true,
  addEventListener(){}, removeEventListener(){}, appendChild(){}, insertBefore(){},
  querySelectorAll(){ return []; }, querySelector(){ return mkEl(); },
  getBoundingClientRect(){ return {width:800,height:600}; }, click(){},
  getContext(){ return { setTransform(){}, clearRect(){}, beginPath(){}, moveTo(){},
    lineTo(){}, stroke(){}, fill(){}, arc(){}, closePath(){}, fillText(){}, strokeText(){},
    setLineDash(){}, save(){}, restore(){}, drawImage(){} }; } },
  { get(t,k){ if(k in t) return t[k]; if(k==='firstChild') return null; return undefined; },
    set(t,k,v){ t[k]=v; return true; } });
const sandbox = { console, Math, JSON, Date, performance:{ now:()=>Date.now() },
  requestAnimationFrame(){ return 0; }, cancelAnimationFrame(){}, setTimeout, clearTimeout,
  localStorage:{ getItem(){ throw new Error('stub'); }, setItem(){ throw new Error('stub'); } },
  location:{ hash:'' },
  document:{ getElementById(){ return mkEl(); }, createElement(){ return mkEl(); },
             querySelector(){ return mkEl(); }, querySelectorAll(){ return []; },
             addEventListener(){}, body:mkEl() },
  Blob:function(){}, URL:{ createObjectURL(){ return ''; } }, FileReader:function(){},
  alert(){}, confirm(){ return false; } };
sandbox.window = sandbox; sandbox.globalThis = sandbox; sandbox.window.addEventListener = () => {};
vm.createContext(sandbox);
const epilogue = `
globalThis.__api = {
  get MODEL(){return MODEL}, set MODEL(v){MODEL=v},
  get CHAIN(){return CHAIN}, get NJ(){return NJ},
  demoModel, buildChain, qLimits, fkAll, ikPos, ikSolve6,
  taskToBase, dirToBase, coreFrame, RULES, MAP, D2R };`;
try { vm.runInContext(blocks.join('\n') + '\n' + epilogue + '\n' + variants, sandbox, { filename:'armBuilder+variants.js' }); }
catch (e) { console.error('脚本执行失败：', e && e.stack || e); process.exit(1); }
const A = sandbox.__api, V = sandbox.__V;

A.MODEL = A.demoModel(); A.buildChain();
const LIM = A.qLimits(), L = A.RULES.levels[3];

function mulberry32(a){ return function(){ a|=0; a=a+0x6D2B79F5|0;
  let t=Math.imul(a^a>>>15,1|a); t=t+Math.imul(t^t>>>7,61|t)^t;
  return ((t^t>>>14)>>>0)/4294967296; }; }

/* ---- 与 t8 完全相同的位姿序列（同种子同调用顺序） ---- */
const NP = parseInt(process.argv[2] || '160', 10);
const KMAX = parseInt(process.argv[3] || '32', 10);
const poseRng = mulberry32(1234567);
const rrP = ([lo,hi]) => lo + poseRng()*(hi-lo);
const poses = [];
for(let i=0;i<NP;i++) poses.push({x:rrP(L.x),y:rrP(L.y),z:rrP(L.z),th:rrP(L.th),ph:rrP(L.ph),al:rrP(L.al)});
const seedRng = mulberry32(999);
const seeds = poses.map(() => { const s=[new Array(A.NJ).fill(0)];
  for(let k=0;k<KMAX;k++) s.push(LIM.map(([lo,hi]) => lo + seedRng()*(hi-lo)));
  return s; });

const task = P => ({ Eb: A.taskToBase([P.x,P.y,P.z]),
                      dir: A.dirToBase(A.coreFrame(P).e).map(v=>-v) });
const T = poses.map(task);

/* ============ EXP1：变体求解器与页面 ikSolve6 的保真度 ============ */
console.log('=== EXP1 保真度：__V.solve6({}) 应等价于页面 ikSolve6 ===');
let maxd = 0, agree = 0, cnt = 0;
for(let i=0;i<Math.min(40,NP);i++){
  for(let s=0;s<3;s++){
    const a = A.ikSolve6(T[i].Eb, T[i].dir, seeds[i][s], 200, 1);
    const b = V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, {});
    maxd = Math.max(maxd, Math.abs(a.e - b.e));
    if(!!a.ok === !!b.ok) agree++;
    cnt++;
  }
}
console.log('  位置误差最大差 ' + maxd.toExponential(2) + ' mm   ok 判定一致 ' +
            agree + '/' + cnt + '\n');

/* ============ EXP2：页面求解器的"起点数-成功率"曲线 ============ */
console.log('=== EXP2 页面 ikSolve6：几个起点才够（%d 个位姿）===', NP);
const firstHit = new Array(NP).fill(-1);
for(let i=0;i<NP;i++){
  for(let s=0;s<seeds[i].length;s++){
    if(A.ikSolve6(T[i].Eb, T[i].dir, seeds[i][s], 200, 1).ok){ firstHit[i]=s; break; }
  }
}
console.log('  起点数   累计可解率      （单起点成功率 = 用 1 个随机起点）');
for(const k of [1,2,3,4,6,8,12,16,24,KMAX]){
  const ok = firstHit.filter(h => h>=0 && h<=k-1).length;
  console.log('   ' + String(k).padEnd(6) + (100*ok/NP).toFixed(1).padStart(7) + '%');
}
const never = firstHit.filter(h=>h<0).length;
console.log('  用满 ' + KMAX + ' 个起点仍失败的: ' + never + ' 个');
const need = firstHit.filter(h=>h>=0).map(h=>h+1).sort((a,b)=>a-b);
if(need.length) console.log('  成功所需起点数: 中位 ' + need[Math.floor(need.length/2)] +
  '  平均 ' + (need.reduce((a,b)=>a+b,0)/need.length).toFixed(2) +
  '  最大 ' + need[need.length-1]);

/* ============ EXP3：改动对照（都是"3 起点"，与面板同条件） ============ */
console.log('\n=== EXP3 单点改动对照（统一 3 起点，%d 个位姿）===', NP);
const VARIANTS = [
  ['V0 页面原样',                 {}],
  ['V1 +保留最优解 keepBest',      {keepBest:true}],
  ['V2 去掉 wO 门控',              {wOgate:false}],
  ['V3 真 LM（步长自适应）',        {lamAdapt:true}],
  ['V4 线搜索',                    {lineSearch:true}],
  ['V5 迭代 200→600',              {iters:600}],
  ['V6 位置种子 90→400 次',        {posIters:400}],
  ['V7 keepBest+线搜索+LM',        {keepBest:true, lineSearch:true, lamAdapt:true, iters:400}],
  ['V8 V7 + 位置种子 400',         {keepBest:true, lineSearch:true, lamAdapt:true, iters:400, posIters:400}],
];
for(const [name, o] of VARIANTS){
  let ok = 0, sumStart = 0;
  for(let i=0;i<NP;i++){
    let got = false;
    for(let s=0;s<3;s++){ const r = V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, o);
      if(r.ok){ got = true; sumStart += s+1; break; } }
    if(got) ok++;
  }
  console.log('  ' + name.padEnd(26) + (100*ok/NP).toFixed(1).padStart(6) + '%' +
              (ok ? '   平均用起点 ' + (sumStart/ok).toFixed(2) : ''));
}

/* ============ EXP4：修好之后单起点成功率 ============ */
console.log('\n=== EXP4 修好之后：只用 1 个随机起点的成功率 ===');
const FIX = {keepBest:true, lineSearch:true, lamAdapt:true, iters:400, posIters:400};
let ok1 = 0, ok0 = 0;
for(let i=0;i<NP;i++){
  if(V.solve6(A, T[i].Eb, T[i].dir, seeds[i][1], LIM, FIX).ok) ok1++;
  if(V.solve6(A, T[i].Eb, T[i].dir, seeds[i][0], LIM, FIX).ok) ok0++;
}
console.log('  页面原样 · 零位起点 1 个        ' + (100*ok0/NP).toFixed(1) + '%');
console.log('  修好版   · 随机起点 1 个        ' + (100*ok1/NP).toFixed(1) + '%');

/* ============ EXP5：失败样本的收敛轨迹（页面原样） ============ */
console.log('\n=== EXP5 典型失败样本轨迹（页面 ikSolve6，零位起点）===');
const fails = [];
for(let i=0;i<NP && fails.length<3;i++){
  const r = A.ikSolve6(T[i].Eb, T[i].dir, seeds[i][0], 200, 1);
  if(!r.ok) fails.push({i, e:r.e, ang:r.ang});
}
/* 逐迭代打点：抄一份页面算法，每 25 步记录一次 (|e|, 姿态角) */
function tracePage(i, seed, axIdx=1){
  const tz = V.unit(T[i].dir), Eb = T[i].Eb;
  let q = A.ikPos(Eb, seed, 90).q, lam = 40;          // 与页面一致：先纯位置
  const out = [];
  for(let it=0; it<200; it++){
    const F = A.fkAll(q), ee = F[F.length-1];
    const ep = V.subv(Eb, ee.t);
    const zc = [ee.R[0][axIdx], ee.R[1][axIdx], ee.R[2][axIdx]];
    const ax = V.cross(zc, tz), s = Math.hypot(...ax), c = V.dot3(zc, tz);
    const ang = (s<1e-8 && c>0) ? 0 : Math.atan2(s,c);
    const dp = Math.hypot(...ep);
    if(it % 25 === 0 || it === 199) out.push([it, dp, ang*180/Math.PI, lam]);
    if(dp<2 && ang<0.15) break;
    const wO = dp > 25 ? 0.02 : (dp > 8 ? 0.25 : 1);
    const axv = s<1e-8 ? (c>0?[0,0,0]:V.unit(Math.abs(zc[0])<0.9?V.cross(zc,[1,0,0]):V.cross(zc,[0,1,0]))) : V.scl(ax,1/s);
    const eo = (s<1e-8 && c>0) ? [0,0,0] : V.scl(axv, ang*40);
    const eoW = V.scl(eo, wO);
    const J=[];
    for(let k=1;k<F.length;k++){ const f=F[k]; if(!f.movable) continue;
      const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
      const a=V.mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
      J.push(V.cross(a, V.subv(ee.t,f.t)).concat(V.scl(a, 40*wO))); }
    const M=[]; for(let a2=0;a2<6;a2++){ const row=new Array(6).fill(0); M.push(row); }
    for(let a2=0;a2<6;a2++)for(let b2=0;b2<6;b2++){ let s2=0; for(const jc of J) s2+=jc[a2]*jc[b2];
      M[a2][b2]=s2+(a2===b2?lam*lam:0); }
    const y = V.solveLin(M, ep.concat(eoW));
    for(let k=0;k<J.length;k++){ let d=0; for(let a2=0;a2<6;a2++) d+=J[k][a2]*y[a2];
      d = Math.max(-0.2, Math.min(0.2, d))/V.D2R;
      q[k]=Math.min(Math.max(q[k]+d, LIM[k][0]), LIM[k][1]); }
    lam *= 0.985;
  }
  return out;
}
for(const f of fails){
  const tr = tracePage(f.i, seeds[f.i][0]);
  console.log('  位姿 #' + f.i + ' 最终 |e|=' + f.e.toFixed(1) + 'mm 姿态角=' +
              (f.ang*180/Math.PI).toFixed(1) + '°   ph=' + poses[f.i].ph.toFixed(0) + '°');
  console.log('     it: |e|mm / 姿态角° / λ  →  ' +
    tr.map(r => r[0]+':'+r[1].toFixed(1)+'/'+r[2].toFixed(0)+'/'+r[3].toFixed(1)).join('  '));
  const r2 = V.solve6(A, T[f.i].Eb, T[f.i].dir, seeds[f.i][1], LIM, FIX);
  console.log('     同一目标换"修好版"+另一个随机起点： ' +
              (r2.ok ? '成功' : '仍失败') + '  |e|=' + r2.e.toFixed(2) + 'mm 姿态=' +
              (r2.ang*180/Math.PI).toFixed(1) + '°');
}
