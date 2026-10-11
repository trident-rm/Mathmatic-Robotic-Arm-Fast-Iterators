/*  t7_ik_node.js —— 把 armBuilder.html 里的 IK 代码在 Node 里原样跑起来
    目的：① 直接测这份 IK（不是抄一份）② 导出测试位姿给 MATLAB RST 做交叉验证
    用法：node t7_ik_node.js [目标数N]                                        */
const fs = require('fs'), path = require('path'), vm = require('vm');

const html = fs.readFileSync(path.join(__dirname, '..', '..', 'build', 'armBuilder.html'), 'utf8');
// 抽出两个 <script> 块的内容
const blocks = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
if (blocks.length < 2) { console.error('没抽到两个 script 块，实际 ' + blocks.length); process.exit(1); }

/* ---- 最小 DOM 桩：只为让两个脚本块的顶层代码能跑完（不触发任何渲染） ---- */
const mkEl = () => new Proxy({ style:{}, classList:{ add(){}, remove(){} },
  dataset:{}, value:'30000', textContent:'', innerHTML:'', checked:true,
  addEventListener(){}, removeEventListener(){}, appendChild(){}, insertBefore(){},
  querySelectorAll(){ return []; }, querySelector(){ return mkEl(); },
  getBoundingClientRect(){ return {width:800,height:600}; }, click(){},
  getContext(){ return { setTransform(){}, clearRect(){}, beginPath(){}, moveTo(){},
    lineTo(){}, stroke(){}, fill(){}, arc(){}, closePath(){}, fillText(){}, strokeText(){},
    setLineDash(){}, save(){}, restore(){}, drawImage(){} }; } },
  { get(t,k){ if(k in t) return t[k];
      if(k==='firstChild') return null;
      return undefined; },
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
sandbox.window = sandbox; sandbox.globalThis = sandbox;
sandbox.window.addEventListener = () => {};

/* ---- 把两个脚本块 + 导出尾巴一起跑 ---- */
const epilogue = `
globalThis.__api = {
  get MODEL(){return MODEL}, set MODEL(v){MODEL=v},
  get CHAIN(){return CHAIN}, get Q(){return Q}, set Q(v){Q=v},
  get NJ(){return NJ}, get LIM_WIDEN(){return LIM_WIDEN}, set LIM_WIDEN(v){LIM_WIDEN=v},
  demoModel, buildChain, syncQ, qLimits, fkAll, ikPos, ikSolve6,
  taskToBase, dirToBase, coreFrame, RULES, MAP
};`;
vm.createContext(sandbox);
try { vm.runInContext(blocks.join('\n') + epilogue, sandbox, { filename:'armBuilder.js' }); }
catch (e) { console.error('脚本执行失败：', e.message); process.exit(1); }
const A = sandbox.__api;

/* ---- 确定性 PRNG（结果可复现） ---- */
function mulberry32(a){ return function(){ a|=0; a=a+0x6D2B79F5|0;
  let t=Math.imul(a^a>>>15,1|a); t=t+Math.imul(t^t>>>7,61|t)^t;
  return ((t^t>>>14)>>>0)/4294967296; }; }
const rnd = mulberry32(20260919);
const rr = ([lo,hi]) => lo + rnd()*(hi-lo);

/* ---- 建模型（示例臂，与界面上的一致） ---- */
A.MODEL = A.demoModel();
A.buildChain();
A.Q = new Array(A.NJ).fill(0);
const L = A.RULES.levels[3];

/* ---- 生成三级任务域的确定性位姿，跑这份 IK ---- */
const N = parseInt(process.argv[2] || '200', 10);
const q0 = A.Q.slice();
function solveOne(P){
  const Eb = A.taskToBase([P.x,P.y,P.z]);
  const eB = A.dirToBase(A.coreFrame(P).e);
  let best = null;
  for(let s=0;s<3;s++){
    const lim = A.qLimits();
    const seed = s===0 ? q0 : lim.map(([lo,hi]) => lo + Math.random()*(hi-lo));
    const r = A.ikSolve6(Eb, eB.map(v=>-v), seed, 200, 1);
    if(!best || r.e < best.e) best = r;
    if(best.ok) break;
  }
  return best;
}
const rows = [];
for(let i=0;i<N;i++){
  const P = { x:rr(L.x), y:rr(L.y), z:rr(L.z), th:rr(L.th), ph:rr(L.ph), al:rr(L.al) };
  const t0 = Date.now();
  const r = solveOne(P);
  rows.push({ i, ...P, ok:r.ok?1:0, err:r.e, ang:(r.ang===undefined?NaN:r.ang),
              ms:Date.now()-t0, q:r.q });
}
/* ---- 导出 CSV 给 MATLAB ---- */
const hdr = 'idx,x,y,z,th,ph,al,ok,err,ang,ms,q1,q2,q3,q4,q5,q6';
const csv = [hdr].concat(rows.map(r =>
  [r.i, r.x.toFixed(4), r.y.toFixed(4), r.z.toFixed(4), r.th.toFixed(4),
   r.ph.toFixed(4), r.al.toFixed(4), r.ok, r.err.toFixed(4),
   (isNaN(r.ang)?'':r.ang.toFixed(4)), r.ms].concat(r.q.map(v=>v.toFixed(4))).join(',')));
fs.writeFileSync(path.join(__dirname, '..', '..', 'docs', 'data', 't7_ik_js.csv'), csv.join('\n'), 'utf8');

const nOK = rows.filter(r=>r.ok).length;
const errs = rows.map(r=>r.err).sort((a,b)=>a-b);
console.log('=== armBuilder.html 里的 IK（Node 原样运行）===');
console.log('  目标数        ' + N);
console.log('  可解          ' + nOK + ' / ' + N + '  = ' + (100*nOK/N).toFixed(1) + '%');
console.log('  位置误差 中位 ' + errs[Math.floor(N/2)].toFixed(2) + ' mm');
console.log('  位置误差 <5mm ' + (100*rows.filter(r=>r.err<5).length/N).toFixed(1) + '%');
console.log('  单次耗时 中位 ' + rows.map(r=>r.ms).sort((a,b)=>a-b)[Math.floor(N/2)] + ' ms');
console.log('  已导出 t7_ik_js.csv（含位姿与结果，供 MATLAB 用同一批目标复核）');
