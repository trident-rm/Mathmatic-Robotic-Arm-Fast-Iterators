/*  t9j_panel_after.js —— 打完补丁之后：
      ① 面板那行数字现在是多少、抖多大（重复 R 次，完全复刻打补丁后的 analyzeTask 块）
      ② 插入动画那条路径的回归（同 seed 连续推进，看解是否连续、是否 ok）
    用法： node t9j_panel_after.js [重复次数R=10] [每次位姿数=320]                 */
const { loadEnv, mulberry32 } = require('./t9_load.js');
const { A } = loadEnv({ variants: false });          // 只要页面本身的函数
const LIM = A.qLimits(), L = A.RULES.levels[3];
const q0 = new Array(A.NJ).fill(0);
const rnd = ([a,b]) => a + Math.random()*(b-a);
const W = 40;                                        // = 页面里的 W_ORI

/* ---- ① 打补丁后的面板块（逐句同构） ---- */
const IK_START = 12, IK_ITER = 400;
function panelIK(N){
  let ikOK = 0, ikPosOK = 0, ikPlus = 0, sumStart = 0, t0 = Date.now();
  for(let i=0;i<N;i++){
    const P = {x:rnd(L.x), y:rnd(L.y), z:rnd(L.z), th:rnd(L.th), ph:rnd(L.ph), al:rnd(L.al)};
    const Eb = A.taskToBase([P.x,P.y,P.z]);
    const eB = A.dirToBase(A.coreFrame(P).e);
    let best = null, okBest = null, bestSc = Infinity, posOnly = 0;
    // 对照：只解位置，固定 3 起点、独立跑（与页面一致）
    for(let s=0;s<3;s++){
      const seed = s===0 ? q0 : q0.map((v,ii)=>{ const lim=A.qLimits()[ii];
        return lim[0]+Math.random()*(lim[1]-lim[0]); });
      if(A.ikPos(Eb, seed, 140).e < 5) posOnly++;
    }
    if(posOnly) ikPosOK++;
    for(let s=0;s<IK_START;s++){
      const seed = s===0 ? q0 : q0.map((v,ii)=>{ const lim=A.qLimits()[ii];
        return lim[0]+Math.random()*(lim[1]-lim[0]); });
      const r = A.ikSolve6(Eb, eB.map(v=>-v), seed, IK_ITER, 1);
      if(r.ok && !okBest) okBest = r;
      const sc = Math.hypot(r.e, W*(r.ang||0));
      if(sc < bestSc){ bestSc = sc; best = r; }
      if(okBest) break;
    }
    best = okBest || best;
    if(A.ikSolve6(Eb, eB, q0, IK_ITER, 1).ok) ikPlus++;
    if(best.ok) ikOK++;
  }
  return { ikOK, ikPosOK, ikPlus, avgStart: sumStart/N, sec: (Date.now()-t0)/1000 };
}
const R = parseInt(process.argv[2] || '10', 10), N = parseInt(process.argv[3] || '320', 10);
console.log('=== ① 打补丁后的面板数字（%d 次独立重复 × %d 位姿 × 最多 %d 起点）===', R, N, IK_START);
const rates = [], posR = [], plusR = [];
let sec = 0, ans = 0;
for(let r=0;r<R;r++){
  const o = panelIK(N);
  rates.push(100*o.ikOK/N); posR.push(100*o.ikPosOK/N); plusR.push(100*o.ikPlus/N);
  sec += o.sec; ans += o.avgStart;
}
const st = a => { const s=a.slice().sort((x,y)=>x-y);
  const m=a.reduce((x,y)=>x+y,0)/a.length;
  return {min:s[0], max:s[s.length-1], mean:m,
          sd:Math.sqrt(a.reduce((x,y)=>x+(y-m)**2,0)/a.length)}; };
const s1=st(rates), s2=st(posR), s3=st(plusR);
console.log('  IK 复核 · 可解     : 最小 %.1f%%  最大 %.1f%%  均值 %.1f%%  标准差 %.2f 个百分点',
            s1.min, s1.max, s1.mean, s1.sd);
console.log('  IK 复核 · 只解位置 : 最小 %.1f%%  最大 %.1f%%  均值 %.1f%%', s2.min, s2.max, s2.mean);
console.log('  对齐取 +e⃗ 时       : 均值 %.1f%%  （单起点对照，故意只给 1 个起点）', s3.mean);
console.log('  每轮耗时 %.1fs（320 位姿整段），平均用起点 %.2f 个', sec/R, ans/R);
console.log('  原始值: ' + rates.map(v=>v.toFixed(1)).join(' '));

/* ---- ② 插入动画路径回归：目标从外侧沿 Q 轴推进，seed 取上一帧的解 ---- */
console.log('\n=== ② 插入动画路径回归（同 seed 连续推进 120 帧，夹爪 y ∥ Q 轴）===');
const I = A.RULES.init;
const CORE = {x:I.x, y:I.y, z:I.z, th:I.th, ph:I.ph, al:I.al};
function qasmGeomJS(P, A2){                       // 与页面 qasmGeom 同构（只用到 O/e/e0b）
  const F0 = A.coreFrame(P);
  let e = A.dirToBase(F0.e);
  const e0b = e.slice();
  const rotY = th => [[Math.cos(th),0,Math.sin(th)],[0,1,0],[-Math.sin(th),0,Math.cos(th)]];
  const rotX = th => [[1,0,0],[0,Math.cos(th),-Math.sin(th)],[0,Math.sin(th),Math.cos(th)]];
  const mv = (M,v) => [M[0][0]*v[0]+M[0][1]*v[1]+M[0][2]*v[2],
                       M[1][0]*v[0]+M[1][1]*v[1]+M[1][2]*v[2],
                       M[2][0]*v[0]+M[2][1]*v[1]+M[2][2]*v[2]];
  e = mv(rotY((A2.lift||0)*A.D2R), e); e = mv(rotX((A2.qfix||0)*A.D2R), e);
  return { O: A.taskToBase([P.x,P.y,P.z]), e, e0b };
}
let seed = new Array(A.NJ).fill(0), okN = 0, maxJump = 0, prevQ = seed.slice();
const insLen = 220, standoff = 40;
for(let f=0; f<=120; f++){
  const ins = f/120;
  const G = qasmGeomJS(CORE, {ins});
  const seat = A.taskToBase([CORE.x, CORE.y, CORE.z]);
  const tgt = [0,1,2].map(k => seat[k] + G.e0b[k]*(standoff + (1-ins)*insLen));
  const dir = G.e0b.map(v => -v);
  let r = A.ikSolve6(tgt, dir, seed, 320, 1);
  for(let t=0; t<2 && !r.ok && r.e > 8; t++){        // 与页面动画一致的补种子逻辑
    const lim = A.qLimits();
    const seed2 = lim.map(([lo,hi]) => lo + Math.random()*(hi-lo));
    const r2 = A.ikSolve6(tgt, dir, seed2, 320, 1);
    if(r2.e < r.e) r = r2;
  }
  if(r.q && r.q.length === A.NJ){
    let jump = 0;
    for(let k=0;k<A.NJ;k++) jump = Math.max(jump, Math.abs(r.q[k]-prevQ[k]));
    if(f > 0) maxJump = Math.max(maxJump, jump);
    prevQ = r.q.slice(); seed = r.q.slice();
  }
  if(r.ok) okN++;
  const fin = r.q.every(v => Number.isFinite(v));
  if(!fin) console.log('  ✗ 第 ' + f + ' 帧出现非有限关节角');
}
console.log('  121 帧里 ok 的帧数: %d (%.1f%%)   单帧最大关节跳变: %.1f°（连续运动不应出现大跳）',
            okN, 100*okN/121, maxJump);
