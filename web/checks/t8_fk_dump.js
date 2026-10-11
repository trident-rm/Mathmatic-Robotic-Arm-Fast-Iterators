/*  t8_fk_dump.js —— 导出 armBuilder.html 的 FK 参考位姿，供 Python 独立实现比对
    用法： node t8_fk_dump.js   → 写 t8_fk_ref.json */
const fs = require('fs'), path = require('path'), vm = require('vm');
const html = fs.readFileSync(path.join(__dirname, '..', '..', 'build', 'armBuilder.html'), 'utf8');
const blocks = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
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
vm.runInContext(blocks.join('\n') + `
globalThis.__api = { get MODEL(){return MODEL}, set MODEL(v){MODEL=v},
  get CHAIN(){return CHAIN}, get NJ(){return NJ},
  demoModel, buildChain, fkAll, taskToBase, dirToBase, coreFrame, RULES, MAP };`, sandbox,
  { filename:'armBuilder.js' });
const A = sandbox.__api;
A.MODEL = A.demoModel(); A.buildChain();

/* 5 组确定性的关节角（覆盖各限位）+ 3 个任务域位姿的映射 */
const qs = [
  [0,0,0,0,0,0], [30,60,-40,120,-10,45], [-120,15,150,-60,200,-90],
  [90,122,200,180,220,180], [45,-30,90,-135,33,-77]
];
const out = { chain: A.CHAIN.map(c => ({name:c.name, joint:c.joint, movable:c.movable,
  offset:c.offset, Rf:c.Rf, axis:c.axis, limits:c.limits})),
  base: A.MODEL.base, swap: A.MAP.swap, baseH: A.RULES.baseH,
  cases: [] };
for(const q of qs){
  const F = A.fkAll(q);
  out.cases.push({ q, frames: F.map(f => ({ name:f.name, R:f.R, t:f.t })) });
}
/* 任务域 -> base 映射抽样（含 e 方向） */
out.taskmap = [];
for(const P of [{x:-100,y:100,z:500,th:-90,ph:0,al:-45},{x:0,y:300,z:700,th:90,ph:90,al:45},
                {x:-50,y:200,z:600,th:0,ph:45,al:0}]){
  const F0 = A.coreFrame(P);
  out.taskmap.push({ P, Eb: A.taskToBase([P.x,P.y,P.z]),
                     e: A.dirToBase(F0.e), u: A.dirToBase(F0.u), v: A.dirToBase(F0.v) });
}
fs.writeFileSync(path.join(__dirname, '..', '..', 'docs', 'data', 't8_fk_ref.json'), JSON.stringify(out), 'utf8');
console.log('已写 t8_fk_ref.json ： ' + out.cases.length + ' 组关节角 / ' + out.taskmap.length + ' 个映射样例');
