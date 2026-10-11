/*  t9_load.js —— 把构建产物里的 <script>（+ 可选变体文件）在 Node 里装进一个沙箱
    返回 { A, V, sandbox }：A = 页面导出的 API，V = t9_variants.js 里的变体求解器
    用法： const { loadEnv } = require('./t9_load.js'); const {A,V} = loadEnv();
            带变体： loadEnv({variants:true})
            t11：   loadEnv({file:'armLink.html'})

    ★ HTML 一律从 build/ 读（构建产物），源码在 web/src/。
      改完源码先跑 node web/build.js。                                              */
const fs = require('fs'), path = require('path'), vm = require('vm');
const P = require('../lib/paths');

function mkEl(){ return new Proxy({ style:{}, classList:{ add(){}, remove(){}, toggle(){ return true; }, contains(){ return false; } },
  dataset:{}, value:'30000', textContent:'', innerHTML:'', checked:true,
  addEventListener(){}, removeEventListener(){}, appendChild(){}, insertBefore(){},
  querySelectorAll(){ return []; }, querySelector(){ return mkEl(); },
  getBoundingClientRect(){ return {width:800,height:600}; }, click(){},
  getContext(){ return { setTransform(){}, clearRect(){}, beginPath(){}, moveTo(){},
    lineTo(){}, stroke(){}, fill(){}, arc(){}, closePath(){}, fillText(){}, strokeText(){},
    setLineDash(){}, save(){}, restore(){}, drawImage(){},
    /* t11：画布桩补全（连杆模块的对比小图要 fillRect/strokeRect/measureText） */
    fillRect(){}, strokeRect(){}, rect(){}, translate(){}, rotate(){}, scale(){},
    ellipse(){}, bezierCurveTo(){}, quadraticCurveTo(){}, clip(){},
    measureText(){ return {width:10}; }, createLinearGradient(){ return {addColorStop(){}}; } }; } },
  { get(t,k){ if(k in t) return t[k]; if(k==='firstChild') return null; return undefined; },
    set(t,k,v){ t[k]=v; return true; } }); }

function loadEnv(opts){
  opts = opts || {};
  const file = opts.file || 'armBuilder.html';
  const htmlPath = path.join(P.BUILD, file);          // 构建产物目录
  if (!fs.existsSync(htmlPath)) {
    throw new Error('找不到 ' + htmlPath + '\n先跑： node web/build.js');
  }
  const html = fs.readFileSync(htmlPath, 'utf8');
  const blocks = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
  if (blocks.length < 2) throw new Error('没抽到两个 script 块，实际 ' + blocks.length);
  const extra = opts.variants === false ? '' :
    fs.readFileSync(path.join(__dirname, 't9_variants.js'), 'utf8');
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
  get CHAIN(){return CHAIN}, get NJ(){return NJ}, get Q(){return Q}, set Q(v){Q=v},
  demoModel, buildChain, qLimits, fkAll, ikPos, ikSolve6, syncQ,
  taskToBase, dirToBase, coreFrame, D2R,
  get RULES(){return RULES}, get MAP(){return MAP},   // ★ 必须 getter：applyConfig 会重新赋值
  /* ★ 配置层（t12 起）：模型/规则/底盘都从配置来，引擎不再写死同济的值 */
  applyConfig, listConfigs, activeConfigId, configModel, initDefaultConfig,
  /* ★ 多臂（t13 起）*/
  armCount, armOfJointQI, armModel, armEnd, sampleReach,
  get ARMS(){return ARMS}, get ARMSTATS(){return ARMSTATS},
  fkAllArm, fkAllArms,
  get CONFIG(){return CONFIG},
  /* ★ t10 用：底盘碰撞箱 / 伸展尺寸（纯追加，不影响原有脚本） */
  chassisGeom, armStats, sizePanelHTML, drawSizeOverlay, draw3D, boxRange,
  get CHASSIS_V(){return CHASSIS_V},
  get CHASSIS(){return CHASSIS}, set CHASSIS(v){CHASSIS=v},
  get SIZER(){return SIZER},     set SIZER(v){SIZER=v},
  get SHOWBOX(){return SHOWBOX}, set SHOWBOX(v){SHOWBOX=v},
  get CLOUD(){return CLOUD},     set CLOUD(v){CLOUD=v} };`;
  vm.runInContext(blocks.join('\n') + '\n' + epilogue + '\n' + extra, sandbox,
                  { filename:'armBuilder+variants.js' });
  const A = sandbox.__api;
  A.MODEL = A.demoModel(); A.buildChain(); A.Q = new Array(A.NJ).fill(0);
  return { A, V: sandbox.__V, sandbox };
}

function mulberry32(a){ return function(){ a|=0; a=a+0x6D2B79F5|0;
  let t=Math.imul(a^a>>>15,1|a); t=t+Math.imul(t^t>>>7,61|t)^t;
  return ((t^t>>>14)>>>0)/4294967296; }; }

/* 与 t8 完全同序的三级任务域位姿序列（同种子，同调用顺序） */
function makePoses(A, NP){
  const L = A.RULES.levels[3], rng = mulberry32(1234567);
  const rr = ([lo,hi]) => lo + rng()*(hi-lo);
  const out = [];
  for(let i=0;i<NP;i++) out.push({x:rr(L.x),y:rr(L.y),z:rr(L.z),th:rr(L.th),ph:rr(L.ph),al:rr(L.al)});
  return out;
}
function makeSeeds(A, NP, K){
  const LIM = A.qLimits(), rng = mulberry32(999), out = [];
  for(let i=0;i<NP;i++){
    const s = [new Array(A.NJ).fill(0)];
    for(let k=0;k<K;k++) s.push(LIM.map(([lo,hi]) => lo + rng()*(hi-lo)));
    out.push(s);
  }
  return out;
}
module.exports = { loadEnv, mulberry32, makePoses, makeSeeds };
