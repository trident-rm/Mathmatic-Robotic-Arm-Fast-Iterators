/*  t8_ik_diag.js —— 为什么 IK 率不是 100%？
    把 armBuilder.html 里的 IK / 采样 / 体素判定【原样】在 Node 里跑起来，分层量化：
      A. 体素可达率 vs 采样数（30k / 100k / 300k）—— 看 79/85 是不是"采样密度假象"
      B. 面板同款 IK 率（320 位姿 · 3 起点）
      C. 强多起点 IK 率（K 起点）—— 这是这份求解器的上限；B<C 说明是求解器问题
      D. 只解位置（强多起点）—— 位置本身能不能到
      E. 失败样本归因：位置不可达 / 姿态不可达 / 求解器局部极小
    用法： node t8_ik_diag.js [位姿数=200] [强起点数=24]
    输出： t8_ik_diag.csv（逐个位姿明细，供 Python 独立复核）
*/
const fs = require('fs'), path = require('path'), vm = require('vm');

const html = fs.readFileSync(path.join(__dirname, 'armBuilder.html'), 'utf8');
const blocks = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
if (blocks.length < 2) { console.error('没抽到两个 script 块，实际 ' + blocks.length); process.exit(1); }

/* ---- 最小 DOM 桩（同 t7_ik_node.js） ---- */
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

const sandbox = {
  console, Math, JSON, Date, performance:{ now:()=>Date.now() },
  requestAnimationFrame(){ return 0; }, cancelAnimationFrame(){},
  setTimeout, clearTimeout,
  localStorage:{ getItem(){ throw new Error('stub'); }, setItem(){ throw new Error('stub'); } },
  location:{ hash:'' },
  document:{ getElementById(){ return mkEl(); }, createElement(){ return mkEl(); },
             querySelector(){ return mkEl(); }, querySelectorAll(){ return []; },
             addEventListener(){}, body:mkEl() },
  Blob:function(){}, URL:{ createObjectURL(){ return ''; } }, FileReader:function(){},
  alert(){}, confirm(){ return false; },
};
sandbox.window = sandbox; sandbox.globalThis = sandbox; sandbox.window.addEventListener = () => {};

const epilogue = `
globalThis.__api = {
  get MODEL(){return MODEL}, set MODEL(v){MODEL=v},
  get CHAIN(){return CHAIN}, get Q(){return Q}, set Q(v){Q=v},
  get NJ(){return NJ},
  demoModel, buildChain, syncQ, qLimits, fkAll, ikPos, ikSolve6,
  taskToBase, dirToBase, coreFrame, RULES, MAP, D2R
};`;
vm.createContext(sandbox);
try { vm.runInContext(blocks.join('\n') + epilogue, sandbox, { filename:'armBuilder.js' }); }
catch (e) { console.error('脚本执行失败：', e.message); process.exit(1); }
const A = sandbox.__api;

/* ---- 确定性 PRNG ---- */
function mulberry32(a){ return function(){ a|=0; a=a+0x6D2B79F5|0;
  let t=Math.imul(a^a>>>15,1|a); t=t+Math.imul(t^t>>>7,61|t)^t;
  return ((t^t>>>14)>>>0)/4294967296; }; }
const seedRng = mulberry32(20260920);
const rr0 = ([lo,hi]) => lo + seedRng()*(hi-lo);

/* ---- 建模型（与界面上一致） ---- */
A.MODEL = A.demoModel();
A.buildChain();
A.Q = new Array(A.NJ).fill(0);
const LIM = A.qLimits();
const L = A.RULES.levels[3];

console.log('=== 模型 ===');
console.log('  关节数 ' + A.NJ + '  base.yaw = ' + A.MODEL.base.yaw + '  MAP.swap = ' + A.MAP.swap);
console.log('  限位   ' + A.CHAIN.filter(c=>c.movable).map((c,i)=>
    c.joint + '[' + LIM[i][0] + ',' + LIM[i][1] + ']').join(' '));
console.log('  任务域 x[' + L.x + '] y[' + L.y + '] z[' + L.z + ']');
console.log('  ph[' + L.ph + '] th[' + L.th + '] al[' + L.al + ']');

const posSamples = n => { const a=[]; for(let i=0;i<n;i++) a.push({x:rr0(L.x),y:rr0(L.y),z:rr0(L.z)}); return a; };

/* ================= A. 体素可达率 vs 采样数 ================= */
/* 完全复刻 computeFeasible（vs=50）+ analyzeTask 的命中判定（±1 邻域 / 严格） */
const vs = 50, NEVAL = 4000;
const evalPts = posSamples(NEVAL).map(p => A.taskToBase([p.x,p.y,p.z]));
function voxSet(N){
  const keys = new Set();
  for(let s=0;s<N;s++){
    const q = LIM.map(([lo,hi]) => lo + Math.random()*(hi-lo));
    const p = A.fkAll(q).slice(-1)[0].t;
    keys.add(Math.floor(p[0]/vs)+','+Math.floor(p[1]/vs)+','+Math.floor(p[2]/vs));
  }
  return keys;
}
function voxHit(keys, p, tol){
  const i=Math.floor(p[0]/vs), j=Math.floor(p[1]/vs), k=Math.floor(p[2]/vs);
  if(!tol) return keys.has(i+','+j+','+k);
  for(let a=-1;a<=1;a++) for(let b=-1;b<=1;b++) for(let c=-1;c<=1;c++)
    if(keys.has((i+a)+','+(j+b)+','+(k+c))) return true;
  return false;
}
console.log('\n=== A. 体素可达率 vs 采样数（%d 个固定任务域位置，与面板同款判定）===' , NEVAL);
console.log('  采样数      体素数      可达率(±1体素)   严格同体素');
for(const N of [30000, 100000, 300000]){
  const keys = voxSet(N);
  let h=0, h0=0;
  for(const p of evalPts){ if(voxHit(keys,p,true)) h++; if(voxHit(keys,p,false)) h0++; }
  console.log('  ' + String(N).padEnd(11) + String(keys.size).padEnd(12) +
              (100*h/NEVAL).toFixed(1).padStart(8) + '%' + (100*h0/NEVAL).toFixed(1).padStart(14) + '%');
}

/* ================= B~E. IK ================= */
const NP = parseInt(process.argv[2] || '200', 10);
const NS = parseInt(process.argv[3] || '24', 10);
const rng = mulberry32(1234567);
const rr = ([lo,hi]) => lo + rng()*(hi-lo);
const poses = [];
for(let i=0;i<NP;i++) poses.push({x:rr(L.x),y:rr(L.y),z:rr(L.z),th:rr(L.th),ph:rr(L.ph),al:rr(L.al)});

const q0 = new Array(A.NJ).fill(0);
function rndSeed(){ return LIM.map(([lo,hi]) => lo + rng()*(hi-lo)); }

/* 强多起点：NS 个随机起点，取误差最小
   ★ 选择判据必须和 ok 判据一致：只要有一个起点 ok 就算 ok。
     曾经按"位置误差最小"选，结果 ikSolve6 一旦返回"位置更准但姿态差"的解，
     就会把另一个真正 ok 的解盖掉，凭空造出 5~8% 的假失败。 */
const scOf = r => r.ok ? -1 : Math.hypot(r.e, 40 * (r.ang || 0));
function strongSolve(P, nSeed){
  const Eb = A.taskToBase([P.x,P.y,P.z]), eB = A.dirToBase(A.coreFrame(P).e);
  const dir = eB.map(v=>-v);
  let best = null;
  for(let s=0;s<nSeed;s++){
    const r = A.ikSolve6(Eb, dir, s===0 ? q0 : rndSeed(), 200, 1);
    if(!best || scOf(r) < scOf(best)) best = r;
    if(best.ok) break;
  }
  return best;
}
/* 与面板同款：3 起点，同样"任一 ok 即 ok" */
function panelSolve(P){
  const Eb = A.taskToBase([P.x,P.y,P.z]), eB = A.dirToBase(A.coreFrame(P).e);
  const dir = eB.map(v=>-v);
  let best = null;
  for(let s=0;s<3;s++){
    const r = A.ikSolve6(Eb, dir, s===0 ? q0 : rndSeed(), 200, 1);
    if(!best || scOf(r) < scOf(best)) best = r;
    if(best.ok) break;
  }
  return best;
}
/* 只解位置：强多起点 */
function strongPos(P, nSeed){
  const Eb = A.taskToBase([P.x,P.y,P.z]);
  let best = null;
  for(let s=0;s<nSeed;s++){
    const r = A.ikPos(Eb, s===0 ? q0 : rndSeed(), 200);
    if(!best || r.e < best.e) best = r;
    if(best.ok) break;
  }
  return best;
}

console.log('\n=== B~D. IK（%d 个三级任务域位姿）===', NP);
const rows = [];
let pOK=0, sOK=0, spOK=0, pAngOK=0, sAngOK=0;
const t0 = Date.now();
for(let i=0;i<NP;i++){
  const P = poses[i];
  const Eb = A.taskToBase([P.x,P.y,P.z]), eB = A.dirToBase(A.coreFrame(P).e);
  const dir = eB.map(v=>-v);
  const p = panelSolve(P);
  const s = strongSolve(P, NS);
  const sp = strongPos(P, NS);
  // 强解里"位置到了但姿态没对上"的度量：位置解的姿态误差
  const angOf = q => { const F=A.fkAll(q), R=F[F.length-1].R;
    const y=[R[0][1],R[1][1],R[2][1]];
    return Math.acos(Math.max(-1,Math.min(1, y[0]*dir[0]+y[1]*dir[1]+y[2]*dir[2]))); };
  const sAng = angOf(s.q);
  const spAng = angOf(sp.q);
  if(p.ok) pOK++;
  if(s.ok) sOK++;
  if(sp.ok) spOK++;
  if(s.ok) sAngOK++;
  if(spAng < 0.2) pAngOK++;              // 位置解的姿态恰好也对上
  rows.push({i, ...P, panel_ok:p.ok?1:0, panel_e:p.e, panel_ang:(p.ang??NaN),
             strong_ok:s.ok?1:0, strong_e:s.e, strong_ang:sAng,
             pos_ok:sp.ok?1:0, pos_e:sp.e, pos_ang:spAng});
  if((i+1)%25===0) process.stderr.write('  ...'+ (i+1) +'/'+NP+'  '+((Date.now()-t0)/1000).toFixed(0)+'s\n');
}
const pct = (a,b) => (100*a/b).toFixed(1)+'%';
console.log('  B. 面板同款（3 起点）可解率      ' + pct(pOK,NP) + '   (' + pOK + '/' + NP + ')');
console.log('  C. 强多起点（' + NS + ' 起点）可解率    ' + pct(sOK,NP) + '   (' + sOK + '/' + NP + ')');
console.log('  D. 只解位置（强多起点）可达率    ' + pct(spOK,NP) + '   (' + spOK + '/' + NP + ')');
console.log('     位置可解的那些位姿里，姿态也天然对上的：' + pct(pAngOK, spOK));
console.log('  B/C 差 = 求解器多起点不足造成的漏判：' + (sOK-pOK) + ' 个');

/* ---- E. 失败归因 ---- */
console.log('\n=== E. 失败归因（强多起点仍失败者）===');
const fail = rows.filter(r => !r.strong_ok);
const failNoPos = fail.filter(r => !r.pos_ok);
const failPos = fail.filter(r => r.pos_ok);
console.log('  强解失败总数            ' + fail.length);
console.log('  其中【位置也到不了】    ' + failNoPos.length + '  → 真·位置不可达');
console.log('  其中【位置能到、姿态对不上】' + failPos.length + '  → 姿态约束损失');
if(failPos.length){
  const angs = failPos.map(r=>r.pos_ang*180/Math.PI).sort((a,b)=>a-b);
  console.log('     这些位姿在"位置最优解"处的姿态偏差(°)：中位 ' +
              angs[Math.floor(angs.length/2)].toFixed(1) + '  最小 ' + angs[0].toFixed(1) +
              '  最大 ' + angs[angs.length-1].toFixed(1));
}
if(failNoPos.length){
  const es = failNoPos.map(r=>r.pos_e).sort((a,b)=>a-b);
  console.log('     位置也到不了的位姿：位置误差中位 ' + es[Math.floor(es.length/2)].toFixed(0) + ' mm');
}
/* 位置可达但姿态对不上：按 e 的极角 ph 分层看 */
if(failPos.length){
  const by = {};
  for(const r of failPos){ const k = Math.floor(r.ph/15)*15; (by[k]=by[k]||[]).push(r); }
  console.log('     按 ph 分层（ph=0 竖直 / ph=90 水平）：');
  for(const k of Object.keys(by).sort((a,b)=>a-b))
    console.log('        ph ' + k + '~' + (+k+15) + '° : ' + by[k].length + ' 个失败');
}

const hdr = 'idx,x,y,z,th,ph,al,panel_ok,panel_e,panel_ang,strong_ok,strong_e,strong_ang,pos_ok,pos_e,pos_ang';
fs.writeFileSync(path.join(__dirname,'t8_ik_diag.csv'),
  [hdr].concat(rows.map(r=>[r.i,r.x.toFixed(4),r.y.toFixed(4),r.z.toFixed(4),r.th.toFixed(4),
    r.ph.toFixed(4),r.al.toFixed(4),r.panel_ok,r.panel_e.toFixed(4),isNaN(r.panel_ang)?'':r.panel_ang.toFixed(4),
    r.strong_ok,r.strong_e.toFixed(4),r.strong_ang.toFixed(4),
    r.pos_ok,r.pos_e.toFixed(4),r.pos_ang.toFixed(4)].join(','))).join('\n'),'utf8');
console.log('\n已导出 t8_ik_diag.csv（逐位姿明细，供 Python 独立复核）');
