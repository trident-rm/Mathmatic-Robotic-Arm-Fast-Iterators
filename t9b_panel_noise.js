/*  t9b_panel_noise.js —— 面板上那个"IK 复核 · 可解（320 位姿 · 3 起点）"到底有多不稳？
    完全复刻 analyzeTask 里的 IK 复核写法，重复 R 次（每次重新抽 320 个位姿 + 重新抽起点），
    看这个数字的分布。用它回答："30k 采样时 79%、100k 采样时 85%" 是不是同一个量的抖动。
    用法： node t9b_panel_noise.js [重复次数R=40] [每次位姿数=320]                   */
const fs = require('fs'), path = require('path'), vm = require('vm');
const html = fs.readFileSync(path.join(__dirname, 'armBuilder.html'), 'utf8');
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
  get CHAIN(){return CHAIN}, get NJ(){return NJ}, get Q(){return Q}, set Q(v){Q=v},
  demoModel, buildChain, qLimits, fkAll, ikPos, ikSolve6,
  taskToBase, dirToBase, coreFrame, RULES, MAP };`, sandbox, { filename:'armBuilder.js' });
const A = sandbox.__api;
A.MODEL = A.demoModel(); A.buildChain(); A.Q = new Array(A.NJ).fill(0);
const LIM = A.qLimits(), L = A.RULES.levels[3];
const q0 = A.Q.slice();
const rnd = ([a,b]) => a + Math.random()*(b-a);

/* ---- 与面板 analyzeTask 的 IK 复核逐句同构 ---- */
function panelIK(N){
  let ikOK = 0, ikPosOK = 0;
  for(let i=0;i<N;i++){
    const P = {x:rnd(L.x), y:rnd(L.y), z:rnd(L.z), th:rnd(L.th), ph:rnd(L.ph), al:rnd(L.al)};
    const Eb = A.taskToBase([P.x,P.y,P.z]);
    const eB = A.dirToBase(A.coreFrame(P).e);
    let best = null, posOnly = 0;
    for(let s=0;s<3;s++){
      const seed = s===0 ? q0 : q0.map((v,i)=>{ const lim=A.qLimits()[i];
        return lim[0]+Math.random()*(lim[1]-lim[0]); });
      const r0 = A.ikPos(Eb, seed, 140);
      if(r0.e < 5) posOnly++;
      const r = A.ikSolve6(Eb, eB.map(v=>-v), seed, 200, 1);
      if(!best || r.e < best.e) best = r;
      if(best.ok) break;
    }
    if(posOnly) ikPosOK++;
    if(best.ok) ikOK++;
  }
  return { ikOK, ikPosOK };
}
const R = parseInt(process.argv[2] || '40', 10);
const N = parseInt(process.argv[3] || '320', 10);
console.log('=== 面板 IK 复核线的重复性（' + R + ' 次独立重复，每次 ' + N + ' 位姿 × 3 起点）===');
const rates = [], posRates = [];
for(let r=0;r<R;r++){
  const o = panelIK(N);
  rates.push(100*o.ikOK/N); posRates.push(100*o.ikPosOK/N);
}
const stat = a => { const s=a.slice().sort((x,y)=>x-y);
  return { min:s[0], max:s[s.length-1], p50:s[Math.floor(s.length/2)],
           mean:a.reduce((x,y)=>x+y,0)/a.length,
           sd:Math.sqrt(a.reduce((x,y)=>x+(y-a.reduce((p,q)=>p+q,0)/a.length)**2,0)/a.length) }; };
const s1 = stat(rates), s2 = stat(posRates);
console.log('  IK 复核 · 可解     : 最小 ' + s1.min.toFixed(1) + '%  最大 ' + s1.max.toFixed(1) +
            '%  中位 ' + s1.p50.toFixed(1) + '%  均值 ' + s1.mean.toFixed(1) +
            '%  标准差 ' + s1.sd.toFixed(2) + ' 个百分点');
console.log('  IK 复核 · 只解位置 : 最小 ' + s2.min.toFixed(1) + '%  最大 ' + s2.max.toFixed(1) +
            '%  中位 ' + s2.p50.toFixed(1) + '%  标准差 ' + s2.sd.toFixed(2) + ' 个百分点');
console.log('  40 次原始值: ' + rates.map(v=>v.toFixed(0)).join(' '));
const inRange = rates.filter(v => v>=79 && v<=85).length;
console.log('  落在 79%~85% 区间内的次数: ' + inRange + '/' + R + '（这就是"30k 时 79、100k 时 85"的来源）');
