/*  t9m_panel_e2e.js —— 端到端跑一次真正的 analyzeTask()（不是复刻），把面板渲染出来的数字抓下来
    用一个"按 id 缓存元素"的 DOM 桩，这样才能读回 dataBody.innerHTML。
    这条路径覆盖：computeFeasible → 体素/位置可达率 → IK 复核 → 失败诊断 bestOfSeeds → 安装朝向扫描。
    用法： node t9m_panel_e2e.js                                                      */
const fs = require('fs'), path = require('path'), vm = require('vm');
const html = fs.readFileSync(path.join(__dirname, '..', '..', 'build', 'armBuilder.html'), 'utf8');
const blocks = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);

const ctx2d = new Proxy({}, { get: () => () => ({}) });
const cache = new Map();
function el(id){
  if(cache.has(id)) return cache.get(id);
  const e = { id, style:{}, classList:{ add(){}, remove(){}, contains(){ return false; } },
    dataset:{}, value:'30000', checked:true, textContent:'', innerHTML:'',
    addEventListener(){}, removeEventListener(){}, appendChild(){}, insertBefore(){},
    querySelectorAll(){ return []; }, querySelector(){ return el(id+'_q'); },
    getBoundingClientRect(){ return { width:900, height:600 }; }, click(){},
    getContext(){ return ctx2d; }, firstChild:null };
  cache.set(id, e); return e;
}
const sandbox = { console, Math, JSON, Date, performance:{ now:()=>Date.now() },
  requestAnimationFrame(){ return 0; }, cancelAnimationFrame(){}, setTimeout, clearTimeout,
  localStorage:{ getItem(){ throw new Error('stub'); }, setItem(){ throw new Error('stub'); } },
  location:{ hash:'' },
  document:{ getElementById:el, createElement(){ return el('tmp'+Math.random()); },
             querySelector(){ return el('q'); }, querySelectorAll(){ return []; },
             addEventListener(){}, body: el('body') },
  Blob:function(){}, URL:{ createObjectURL(){ return ''; } }, FileReader:function(){},
  alert(){}, confirm(){ return false; } };
sandbox.window = sandbox; sandbox.globalThis = sandbox; sandbox.window.addEventListener = () => {};
vm.createContext(sandbox);
vm.runInContext(blocks.join('\n') + `
globalThis.__api = { get MODEL(){return MODEL}, set MODEL(v){MODEL=v}, get CHAIN(){return CHAIN},
  get NJ(){return NJ}, get Q(){return Q}, set Q(v){Q=v}, get VOX(){return VOX}, set VOX(v){VOX=v},
  demoModel, buildChain, computeFeasible, analyzeTask, draw3D, drawList, fkAll, qLimits,
  RULES, MAP, ikSolve6, bestOfSeeds, taskToBase, dirToBase, coreFrame, W_ORI, D2R };`,
  sandbox, { filename:'armBuilder.js' });
const A = sandbox.__api;
A.MODEL = A.demoModel(); A.buildChain(); A.Q = new Array(A.NJ).fill(0);

const t0 = Date.now();
A.computeFeasible();
const tVox = Date.now() - t0;
const t1 = Date.now();
A.analyzeTask();
const tPanel = Date.now() - t1;
const txt = (cache.get('dataBody').innerHTML || '')
  .replace(/<[^>]*>/g, '\n').replace(/&lt;/g,'<').replace(/&gt;/g,'>').replace(/&amp;/g,'&')
  .split('\n').map(s => s.trim()).filter(Boolean);

console.log('=== analyzeTask() 端到端（真代码路径）===');
console.log('  computeFeasible %d ms   体素 %d 个   面板整段 %d ms', tVox, A.VOX.size, tPanel);
console.log('  ── dataBody 渲染出来的内容 ──');
console.log(txt.map(s => '    ' + s).join('\n'));
