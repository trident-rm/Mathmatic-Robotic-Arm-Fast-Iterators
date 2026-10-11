"use strict";
/* ==================================================================
   任务域校核（规则相关）

   ★ 这里【不再】写死比赛规则 —— 规则、装配位、坐标系换算全部来自配置
     （web/src/configs/*.js），由 04-config.js 的 applyConfig() 填进
     RULES / MAP。换一套比赛规则 = 加一个配置文件，不用改这个模块。

     没有配置规则时（rules:null），RULES / MAP 为 null，
     任务域那块界面会自己让开，可达集等通用功能照常可用。
   ================================================================== */
let RULES = null;    // 由配置层填：{ src, frame, init, levels, act, baseH }
let MAP   = null;    // 由配置层填：任务系 <-> base 系换算（唯一实现见 04-config.js）

/* ---------------- 小工具 ---------------- */
let VOX = null;                                   // 可达集体素集合
const rnd  = ([a,b]) => a + Math.random()*(b-a);
const vkey = (p,vs) => Math.floor(p[0]/vs)+','+Math.floor(p[1]/vs)+','+Math.floor(p[2]/vs);
const cross= (a,b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const dot3 = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const unit = a => { const n=Math.hypot(a[0],a[1],a[2])||1; return [a[0]/n,a[1]/n,a[2]/n]; };
const mulS = (a,s) => [a[0]*s, a[1]*s, a[2]*s];
function solve3(A,b){                              // 3x3 线性方程组
  const d = det3(A); if(Math.abs(d) < 1e-12) return [0,0,0];
  const col=[0,1,2].map(j=>{ const M=A.map(r=>r.slice()); for(let i=0;i<3;i++) M[i][j]=b[i];
    return det3(M)/d; });
  return col;
}
/* OXYZ（地面系） <-> base 系。
   ★ 实现已统一到 04-config.js 的 mapObj()，这里只做转发 —— 不许再写第二份。
     界面上"映射"下拉框改的是 CONFIG.rules.frame.taskMap，再 rebuildTaskMap()。
     两边共用一份公式，MATLAB 侧就不会再差 90°（t7 那个假的 0/200 可解）。 */
const taskToBase = p => MAP.toBase(p);
const baseToTask = p => MAP.toTask(p);
const dirToBase  = d => MAP.dirToBase(d);

/* ---------------- 装配运动学：一套几何，判据与动画共用 ----------------
   ★ 谁在动、谁不动（这一条最容易搞错，改过两遍）：
       装配盘（六边形核心本体）—— 【固定不动】。它是台上的工装，全程不参与运动。
       能量单元（带 Q 轴的那件）—— 【在动】。插入 → 上推 → 绕 P 转 → 绕 Q 转
       机械臂 —— 夹着能量单元，跟着它一起动。
     曾经的错法：让六边形跟着上推、跟着旋转（把工装当成了零件），
     于是画面上"该动的没动、不该动的乱动"，看着就像卡住。

   装配顺序（规则手册三级，用户口径）：
     ⓪ 就位：手到插入轴外侧，工具轴与插入轴同轴
     ① 插入：沿插入轴把能量单元推进去（装配盘不动）
     ② 上推：沿 +Z 平移 LIFT（能量单元与手一起升）
     ③ 绕 P 轴旋转 +90°：能量单元从水平转到【与 Z 轴平行】
     ④ 绕 Q 轴旋转 ±SPIN：绕（已经竖起来的）Q 轴再转，做"多姿态"

   插入轴 = 零点位形下核心前表面法向 e⃗（装配盘不动 ⇒ 它就是个固定方向）。
   姿态函数返回的一律是【能量单元的位姿】，机械臂按它解 IK。 */
let G_LIFT = 0;        // 当前校核用的上推高度（mm）—— 判据与动画共用
let G_STEPS = null;    // 覆盖默认旋转序列（调试/自检用）
let G_SPIN  = null;    // 覆盖默认自转角（=== null 时读配置）

function eeAxes(ee){ return { x:[ee.R[0][0],ee.R[1][0],ee.R[2][0]],
                              y:[ee.R[0][1],ee.R[1][1],ee.R[2][1]],
                              z:[ee.R[0][2],ee.R[1][2],ee.R[2][2]] }; }

/* 旋转序列（每步 = {axis, ang}），按顺序作用在能量单元的朝向上。
   ★ 轴怎么定（改过两版，说明留在下面，别再搞错）：
     'p' = P 轴 = 把能量单元【往上翻】的那根水平轴 = normalize(e⃗ × Z)
           —— 绕它 +90°，单元从水平翻到【与 Z 轴平行】。
     'q' = Q 轴 = 【插入轴 e⃗ 本身】（零点位形里就是盘面法向），水平那根。
           ★ 不是"单元自己那根轴"：绕自身轴转是原地自转，
             位置一点不变 ⇒ 动画看着不动（实测目标点全程钉死在一个点）。
             绕固定的插入轴转，单元才会绕着它画圈。
   配置里 act.steps 可以整套换掉；默认就是手册那两步。 */
const ASM_STEPS_DEFAULT = [
  { axis:'p', ang: 90, label:'绕 P 轴旋转 +90°（能量单元竖到与 Z 轴平行）' },
  { axis:'q', ang: 90, label:'绕 Q 轴（插入轴）旋转 ±90°' }
];
function asmSteps(){
  if(G_STEPS) return G_STEPS;
  const act = (RULES && RULES.act) || {};
  if(Array.isArray(act.steps) && act.steps.length) return act.steps;
  return ASM_STEPS_DEFAULT.map(s => ({
    axis: s.axis, ang: (s.axis === 'q' && G_SPIN !== null) ? G_SPIN : s.ang, label: s.label }));
}
/* 把一串旋转按顺序乘起来（内旋：后一步绕着【转过之后】的当前轴）。
   ★ 'q' 与 'p' 都是【固定轴】：p 由 e⃗ 与 Z 定出，q 就是 e⃗ 本身。 */
function asmRotAt(degByStep, e, P){
  const steps = asmSteps();
  let R = [[1,0,0],[0,1,0],[0,0,1]];
  let ce = unit(e).slice();
  for(let i=0;i<steps.length;i++){
    const f = (degByStep && degByStep[i] !== undefined) ? degByStep[i] : 0;
    if(!f) continue;
    const rax = (steps[i].axis === 'q') ? unit(e).slice() : unit(P).slice();
    const Ri = rotAxis(rax, f*D2R);
    ce = mulv(Ri, ce);
    R = mul3(Ri, R);
  }
  return R;
}

/* 任务点 P 的【能量单元】位姿（base 系）。
     role      : 'unit'（能量单元，默认）| 'plate'（装配盘：固定不动）
     rotDegs   : 各步已经转过的角度数组（[绕P, 绕Q]），省略 = 全 0
     opts.gripAxis : 夹爪哪个轴沿插入轴（'x'/'y'/'z'）
   返回 { E, e, u, v, e0b, er, axes:{x,y,z}, Paxis, rotAxis, rotDeg, steps } */
function taskPoseBase(P, rotDegs, off, opts){
  opts = opts || {};
  const F0 = coreFrame(P);
  const e0 = unit(dirToBase(F0.e));               // 零点位形：插入轴（= Q 轴）
  const u0 = unit(dirToBase(F0.u));
  const v0 = unit(dirToBase(F0.v));               // 盘面顶边（画图用）
  /* ★ P 轴 = 把单元往上翻的那根水平轴 = e⃗ × Z 归一化。
     盘面顶边 v0 不能当 P 轴用：它垂直于 e⃗，绕它转单元永远竖不起来。 */
  let Paxis = cross(e0, [0,0,1]);
  Paxis = (Math.hypot(Paxis[0],Paxis[1],Paxis[2]) > 1e-6)
        ? unit(Paxis) : unit(cross(e0, [0,1,0]));
  const act  = (RULES && RULES.act) || {};
  const grip = (opts.gripAxis || act.gripAxis || 'x').toLowerCase();
  const steps = asmSteps();
  let degs;
  if(Array.isArray(rotDegs)) degs = rotDegs;
  else if(typeof rotDegs === 'number' && rotDegs) degs = [rotDegs, 0];
  else degs = null;
  const R = asmRotAt(degs, e0, Paxis);

  const plate = (opts.role === 'plate');
  const E = addv(taskToBase([P.x,P.y,P.z], off), plate ? [0,0,0] : [0,0,G_LIFT]);
  /* ★ 工具三轴的定法（改了三版，教训写在下面）：
       零点位形的基：z 沿插入方向 e（伸进孔里），y 沿偏心方向 u，x 由右手系补出。
       装配过程中【三个轴整体跟着零件转 R】—— 手抓着零件，手指相对零件不动，
       所以零件翻起来、手也得跟着翻。
     ⚠ 别再这么写：
        · "z 跟零件轴转、x 用固定插入轴" ⇒ 两个约束互相打架，
          某些姿态下 x 退化成一个常向量（实测绕P整段姿态一动不动）。
        · 用 ur（单个跟着转的向量）去投影求垂直分量 ⇒ 它与轴共线时退化，
          落到回退分支，同样冻结。 */
  const Rr = plate ? [[1,0,0],[0,1,0],[0,0,1]] : R;
  const base = gripBaseAxes(e0, u0, (typeof insAxisIdx === 'function') ? insAxisIdx() : 0);
  const er = plate ? e0.slice() : mulv(Rr, e0);
  const ur = plate ? u0.slice() : mulv(Rr, u0);
  const xc = unit(mulv(Rr, base.x));
  const yc = unit(mulv(Rr, base.y));
  const zc = unit(mulv(Rr, base.z));
  const sumDeg = degs ? degs.reduce((a,b)=>a+(b||0), 0) : 0;
  return { E, e:e0, u:u0, v:v0, e0b:e0.slice(), er, axes:{ x:xc, y:yc, z:zc },
           axKey: base.axKey, Paxis, rotAxis: er.slice(), rotDeg: sumDeg,
           degs: degs || steps.map(()=>0), kind: 'steps', steps };
}
/* 零点位形下的工具三轴（未旋转）：
     z = -e⃗（工具 z 伸进孔里）
     axKey 指定的那根轴（默认 x）= e⃗ 方向（"插入轴对应哪根工具轴"，实测定）
     第三根由右手系补出 —— 偏心方向 */
function gripBaseAxes(e0, u0, axIdx){
  const z0 = unit(mulS(e0, -1));
  const axKey = ['x','y','z'][axIdx] || 'x';
  let third = subv(u0, mulS(z0, dot3(u0, z0)));
  if(Math.hypot(third[0],third[1],third[2]) < 1e-9)
    third = cross(z0, Math.abs(z0[0])<0.9 ? [1,0,0] : [0,1,0]);
  third = unit(third);
  let x0, y0;
  if(axKey === 'y'){ y0 = e0.slice(); x0 = unit(cross(y0, z0)); }
  else if(axKey === 'z'){ x0 = third; y0 = unit(cross(z0, x0)); }
  else { x0 = e0.slice(); y0 = unit(cross(z0, x0)); }
  /* 保证是右手系：x × y = z */
  if(dot3(cross(x0, y0), z0) < 0) y0 = mulS(y0, -1);
  return { x: x0, y: y0, z: z0, axKey };
}

/* 夹爪抓取点相对【能量单元根部】的偏移 = 垂直件轴的径向 × 偏心距离。
   ★ 为什么是"径向"而不是"沿件轴"：手是【偏心侧夹】的（夹爪轴 ⟂ 件轴）。
     若把偏移写成"沿件轴"，绕轴自转时那个点原地不动 ⇒ 画面上看着没转
     （实测位移 0.00mm）。径向点在绕轴自转时会画圈，这才是看得见的运动。
   ★ 也【不要】在这里加"沿件轴的 70mm 之类"：根部位置 E 已经含了上推，
     再加一次轴向偏移会把目标抬到臂够不着的地方（实测抬了 100mm ⇒ 姿态差 30°
     ⇒ 动画看着卡住）。未插入时的轴向退开由调用方按插入进度算。 */
const ASM_GRIP_SIDE = 45;                       // 偏心距离（mm）
function asmGripVec(G){
  const ax = unit(G.er);
  const x = unit(G.axes.x);
  const t = subv(x, mulS(ax, dot3(x, ax)));     // 把夹爪 x 投影到垂直件轴的平面
  const n = Math.hypot(t[0],t[1],t[2]);
  const rad = n > 1e-9 ? mulS(t, 1/n)
                       : unit(cross(ax, Math.abs(ax[0])<0.9?[1,0,0]:[0,1,0]));
  return mulS(rad, ASM_GRIP_SIDE);
}

/* ---- 装配段 IK：位置到位 + 【插入轴对齐】（按臂）----
   ★ 为什么只锁这一条姿态：装配的硬约束就是"工具轴与插入轴同轴"。
     绕插入轴的滚转是【六方口自带的自由度】（六方口允许 60° 的滚转区间），
     把它锁死会把"一族可行的位姿"塌成"几个孤立点" ——
     实测：锁三轴时同一个点 200 个起点都解不出来，只锁插入轴时 200 个起点稳定可解。
     ⇒ 判据必须按物理约束来写，多锁一个不该锁的自由度会凭空造出"不可达"。
   （tgt 里仍带着 axes.x 作参考，需要连续滚转时用 ikSolvePoseA 那条更严的路。） */
function ikSolveAsmA(tgt, q0Sub, iters, a, axTgt, axIdx){
  const ai = (axIdx === undefined) ? 2 : axIdx;    // 2 = 工具 z（默认）
  const seed = ikPosA(tgt.E, q0Sub, 60, a);
  let q = seed.q.slice(), lam = 40;
  const lim = qLimitsArm(a);
  const tz = unit(axTgt);
  const POS_TOL = 3, ROT_TOL = 0.03;
  let bestQ = q.slice(), bestF = Infinity, bestE = Infinity, bestA = 0;
  /* iters 下限 80：装配段传进来的值可能很小（动画每帧只想跑几步），
     但太少连"位置先到位"都做不到，反而更慢、更容易卡住。 */
  const ITMAX = Math.max(80, iters || 260);
  for(let it=0; it<ITMAX; it++){
    const F = fkArm(q, a), ee = F[F.length-1];
    const ep = subv(tgt.E, ee.t);
    const zc = [ee.R[0][ai], ee.R[1][ai], ee.R[2][ai]];
    const ax = cross(zc, tz), s = Math.hypot(ax[0],ax[1],ax[2]);
    const c = dot3(zc, tz);
    let eo, ang;
    if(s < 1e-8){
      if(c > 0){ eo=[0,0,0]; ang=0; }
      else { const pp = Math.abs(zc[0])<0.9 ? cross(zc,[1,0,0]) : cross(zc,[0,1,0]);
             eo = mulS(unit(pp), Math.PI*W_ORI); ang = Math.PI; }
    } else { ang = Math.atan2(s,c); eo = mulS(mulS(ax,1/s), ang*W_ORI); }
    const dp = Math.hypot(ep[0],ep[1],ep[2]);
    const rr = Math.hypot(dp, W_ORI*ang);
    if(rr < bestF){ bestF = rr; bestQ = q.slice(); bestE = dp; bestA = ang; }
    if(dp < POS_TOL && ang < ROT_TOL) return { q, ok:true, e:dp, ang };
    const J = [];
    for(let i=1;i<F.length;i++){
      const f=F[i]; if(!f.movable) continue;
      const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
      const jax=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
      J.push(cross(jax, subv(ee.t,f.t)).concat(mulS(jax, W_ORI)));
    }
    const Am=[]; for(let i=0;i<6;i++) Am.push([0,0,0,0,0,0]);
    for(let i=0;i<6;i++)for(let j=0;j<6;j++){ let s2=0; for(const jc of J) s2+=jc[i]*jc[j];
      Am[i][j]=s2+(i===j?lam*lam:0); }
    const y = solveN(Am, ep.concat(eo));
    for(let k=0;k<J.length;k++){ let d=0; for(let i=0;i<6;i++) d+=J[k][i]*y[i];
      d = Math.max(-0.2, Math.min(0.2, d)) / D2R;
      q[k]=Math.min(Math.max(q[k]+d,lim[k][0]),lim[k][1]); }
    lam *= 0.985;
  }
  return { q: bestQ, ok: (bestE < POS_TOL+2 && bestA < 0.08), e: bestE, ang: bestA };
}

/* 插入方向对应工具系的哪一根轴（末端系），以及它的软/硬约束
   ★ 这个约定必须实测定，不能想当然：本臂按【工具 z】对齐插入轴时，
     轴差中位 63~105°（腕部翻不过去，等于"插不进去"）；
     按【工具 x】对齐时轴差中位 1.4°（几乎完美）。
     配置里用 act.insToolAxis 指定（默认 'x'），下表把名字换成列号。 */
const TOOL_AXIS_IDX = { x:0, y:1, z:2 };
function insAxisIdx(){
  const act = (RULES && RULES.act) || {};
  const k = (act.insToolAxis || 'x').toLowerCase();
  return TOOL_AXIS_IDX[k] === undefined ? 2 : TOOL_AXIS_IDX[k];
}
/* 该工具轴沿【插入方向】时，它的目标方向 = 插入轴（正号 = 朝孔里进） */
function insAxisDir(T, sign){
  const s = (sign === undefined) ? 1 : sign;
  return mulS(unit(T.er), s);
}

/* ---- 装配位姿 IK（按臂）----
   位置 + 插入轴 + 【滚转也管】（软约束）。滚转不是硬约束，但如果装配动作
   要求"夹爪相对零件保持某个方向"（六方口对位），就得用这条更严的路。
   软权重的意义：在零空间里挑"离上一步最近"的解，让动作连续。 */
function ikSolvePoseA(tgt, q0Sub, iters, a){
  const seed = ikPosA(tgt.E, q0Sub, 90, a);
  let q = seed.q.slice(), lam = 40;
  const lim = qLimitsArm(a);
  const ax = tgt.axes;
  const tzc = unit(mulS(ax.z, -1));              // 工具 z 要对齐 -插入轴
  const tx  = unit(ax.x);                        // 滚转参考（软）
  const POS_TOL = 3, ROT_TOL = 0.03;             // 3mm / 1.7°
  const W_ROLL = 6;                              // 软约束权重（比位置小得多）
  let bestQ = q.slice(), bestF = Infinity, bestE = Infinity, bestA = 0, bestRoll = 0;
  for(let it=0; it<(iters||260); it++){
    const F = fkArm(q, a), ee = F[F.length-1];
    const ep = subv(tgt.E, ee.t);
    const A = eeAxes(ee);
    const e1 = cross(A.z, tzc);
    const a1 = Math.atan2(Math.hypot(e1[0],e1[1],e1[2]), dot3(A.z,tzc));   // 主：轴偏差
    const e2 = cross(A.x, tx);
    const a2 = Math.atan2(Math.hypot(e2[0],e2[1],e2[2]), dot3(A.x,tx));    // 软：滚转偏差
    const dp = Math.hypot(ep[0],ep[1],ep[2]);
    const rr = Math.hypot(dp, W_ORI*a1);
    if(rr < bestF){ bestF = rr; bestQ = q.slice(); bestE = dp; bestA = a1; bestRoll = a2; }
    if(dp < POS_TOL && a1 < ROT_TOL) return { q, ok:true, e:dp, ang:a1, roll:a2 };
    const u1 = unit(e1), u2 = unit(e2);
    const rv = addv(mulS(u1, a1*W_ORI), mulS(u2, a2*W_ROLL));   // 轴权重满、滚转权重小
    const eo = isFinite(rv[0]) ? rv : [0,0,0];
    const J = [];
    for(let i=1;i<F.length;i++){
      const f=F[i]; if(!f.movable) continue;
      const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
      const jax=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
      J.push(cross(jax, subv(ee.t,f.t)).concat(mulS(jax, W_ORI)));
    }
    const Am=[]; for(let i=0;i<6;i++) Am.push([0,0,0,0,0,0]);
    for(let i=0;i<6;i++)for(let j=0;j<6;j++){ let s2=0; for(const jc of J) s2+=jc[i]*jc[j];
      Am[i][j]=s2+(i===j?lam*lam:0); }
    const y = solveN(Am, ep.concat(eo));
    for(let k=0;k<J.length;k++){ let d=0; for(let i=0;i<6;i++) d+=J[k][i]*y[i];
      d = Math.max(-0.15, Math.min(0.15, d)) / D2R;
      q[k]=Math.min(Math.max(q[k]+d,lim[k][0]),lim[k][1]); }
    lam *= 0.985;
  }
  return { q: bestQ, ok: (bestE < POS_TOL+2 && bestA < 0.08),
           e: bestE, ang: bestA, roll: bestRoll };
}
/* 多起点。第一个种子用传入的 q0 —— 上推/旋转段就用上一阶段的解当种子，
   这样"解出来的"同时也是"能连续走过去的"。
   ★ 其余种子【在 q0 附近扰动】(±30°)，不是全关节空间随机：
     装配是连续动作，解就在上一步附近；纯随机撒点命中率极低，
     会把"求解器没找到"误报成"转不动" —— 第一版三级判据报 0% 就是这么来的。 */
function localSeeds(q0Sub, n, a, spread){
  const lim = qLimitsArm(a);
  const out = [];
  for(let s=0;s<n;s++){
    const sp = spread || 30;
    out.push(q0Sub.map((v,i) =>
      Math.min(Math.max(v + (Math.random()*2-1)*sp, lim[i][0]), lim[i][1])));
  }
  return out;
}
function bestPoseSeedsA(tgt, q0Sub, nSeed, a, iters){
  const lim = qLimitsArm(a);
  const tries = [q0Sub].concat(localSeeds(q0Sub, Math.max(0, nSeed-1), a));
  /* 再补一个全空间随机的：有些位姿的最优解离上一步很远（比如绕到另一侧） */
  tries.push(lim.map(([lo,hi]) => lo + Math.random()*(hi-lo)));
  let best = null;
  for(const seed of tries){
    const r = ikSolvePoseA(tgt, seed, iters||260, a);
    if(r.ok) return r;
    if(!best || Math.hypot(r.e, W_ORI*(r.ang||0)) < Math.hypot(best.e, W_ORI*(best.ang||0))) best = r;
  }
  return best;
}
/* 装配段的多起点版：只锁插入轴（滚转自由）—— 见 ikSolveAsmA 上面那段说明 */
/* 多起点 + 就近精修：先在种子附近找，失败再放宽，最后拿最好解细磨一遍。
   "细节决定真假"：DLS 常常停在 5~9° 的轴差上（位置已经 0.1mm），
   从那里再做一轮小扰动精修，往往就能落到 2° 以内 ——
   不加这一步会把"差一点"报成"转不动"。 */
function bestAsmSeedsA(tgt, axTgt, q0Sub, nSeed, a, iters){
  const lim = qLimitsArm(a);
  const tries = [q0Sub].concat(localSeeds(q0Sub, Math.max(0, nSeed-1), a));
  tries.push(lim.map(([lo,hi]) => lo + Math.random()*(hi-lo)));
  let best = null;
  const score = r => Math.hypot(r.e, W_ORI*(r.ang||0));
  for(const seed of tries){
    const r = ikSolveAsmA(tgt, seed, iters||260, a, axTgt, insAxisIdx());
    if(r.ok) return r;
    if(!best || score(r) < score(best)) best = r;
  }
  /* 精修：以最好解为中心撒小扰动（±8°）再解几轮 */
  if(best && best.ang > 0.03){
    for(let k=0;k<6;k++){
      const seed = best.q.map((v,i) =>
        Math.min(Math.max(v + (Math.random()*2-1)*8, lim[i][0]), lim[i][1]));
      const r = ikSolveAsmA(tgt, seed, (iters||260)*2, a, axTgt, insAxisIdx());
      if(r.ok) return r;
      if(score(r) < score(best)) best = r;
    }
  }
  return best;
}

/* ---------------- 三级（装配）可行性：按运动链逐段验 ----------------
   链条（用户口径，也是手册那两步）：
     ① 插入   位置 + 插入轴对齐（滚转自由）
     ② 上推   位置抬 LIFT，姿态要求不变
     ③ 绕P轴  能量单元竖到与 Z 轴平行（这一步是"能不能装配"的硬门槛）
     ④ 绕Q轴  再绕竖起来的 Q 轴转 ±SPIN（这一步是"能不能多姿态"）
   两个结论分开报：
     可装配   = ①②③ 全过
     多姿态   = 再加上 ④ */
function asmFeasibleA(P, ARM, o){
  o = o || {};
  const off = (typeof asmOfArm === 'function') ? asmOfArm(ARM) : null;
  const CF  = l3Conf();
  const lift = (o.lift !== undefined) ? o.lift : CF.lift;
  const step = o.rollStep || CF.step;                 // 每步分几档走
  const opts = { gripAxis: CF.grip };
  const steps = asmSteps();
  const nStep = steps.length;
  const q0 = (o.q0 || curQArm(ARM)).slice();
  const keep = G_LIFT;
  const zeros = () => new Array(nStep).fill(0);
  /* 分工况"共用的尾巴"：给定已经转过的角度数组，解一个位姿 */
  const solveAt = (degs, qSeed, nSeed) => {
    const T = taskPoseBase(P, degs, off, opts);
    return bestAsmSeedsA(T, insAxisDir(T, 1), qSeed, nSeed, ARM, 400);
  };

  /* ① 就位/插入。★ 起点分档递增：这份 DLS 有局部极小，
     实测同一个点 30 起点不行、80 起点可以；判"不可行"要比判"可行"谨慎。 */
  G_LIFT = 0;
  let r = solveAt(zeros(), q0, o.seed0 || 8);
  if(!r.ok) r = solveAt(zeros(), q0, 24);
  if(!r.ok){
    const r3 = solveAt(zeros(), q0, 48);
    if(Math.hypot(r3.e, W_ORI*(r3.ang||0)) < Math.hypot(r.e, W_ORI*(r.ang||0))) r = r3;
  }
  const reach = !!r.ok;
  const insPos = r.e, insAng = r.ang || 0;
  let q = r.q;

  /* ② 上推 */
  let liftOK = false;
  if(reach){
    G_LIFT = lift;
    const r2 = solveAt(zeros(), q, 10);
    liftOK = !!r2.ok && Math.abs(r2.e) < 20;          // 位置必须真的抬起来
    if(r2.ok) q = r2.q;
  }

  /* ③④ 逐步旋转：每一步都分档连续走（上一档的解当下一档的种子） */
  const stepFrac = new Array(nStep).fill(0);
  let stepOK = new Array(nStep).fill(false);
  let worstAng = 0;
  if(liftOK){
    G_LIFT = lift;
    const degs = zeros();
    for(let s=0; s<nStep; s++){
      const target = steps[s].ang;
      const n = Math.max(1, Math.round(Math.abs(target)/step));
      let ok = true;
      for(let k=1;k<=n;k++){
        degs[s] = target*k/n;
        const rk = solveAt(degs, q, 12);
        if(!ok0(rk)){ ok = false; break; }
        q = rk.q; stepFrac[s] = k/n;
      }
      stepOK[s] = ok;
      if(!ok) break;                                   // 这一步不过，后面不用试
    }
  }
  G_LIFT = keep;
  function ok0(rr){
    if(!rr) return false;
    if(!rr.ok) return false;
    worstAng = Math.max(worstAng, rr.ang || 0);
    return true;
  }
  const rotP = stepOK[0] || false;                     // ③ 绕 P 轴竖起
  const rotQ = nStep > 1 ? stepOK[1] : true;           // ④ 绕 Q 轴多姿态
  const asm  = reach && liftOK && rotP;                // 可装配
  return { reach, lift: liftOK, rotP, rotQ, steps: stepOK, stepFrac,
           ok: asm && rotQ, asm, multi: asm && rotQ,
           rotFrac: stepFrac[0] || 0, worstAng, insPos, insAng, q };
}

/* ---------------- 科技核心位姿 → 空间姿态 ---------------- */
/* e⃗（= 装配柱前表面法向 = Q 轴方向）由球坐标 (θ,φ) 给出，定义在 OXYZ 下；
   α 为绕 e⃗ 的自转；(u,v,e) 构成前表面的正交基。
   qfix = 三级末步"绕零点位形 Q 轴"的附加旋转（不是绕当前 e⃗）。 */
function coreFrame(P){
  const th=P.th*D2R, ph=P.ph*D2R, al=(P.al||0)*D2R;
  let e = [Math.sin(ph)*Math.cos(th), Math.sin(ph)*Math.sin(th), Math.cos(ph)];
  let u = unit(Math.abs(e[2])<0.9 ? cross([0,0,1],e) : cross([1,0,0],e));
  let v = cross(e,u);
  if(al){ const Ra=rotAxis(e,al); u=mulv(Ra,u); v=mulv(Ra,v); }
  // 注意：qfix（绕零点位形 Q 轴转 90°）只作用在 Q 轴本身，【六边形不旋转】
  return {e,u,v};
}
let CORE = null;      // ★ 装配盘（六边形工装）的任务点位姿：装配过程中【全程不动】
let QASM = null;      // ★ 能量单元的装配进度：{ ins 插入量, dz 上推, degs:[绕P, 绕Q] }

/* ---------------- 装配动画的叠加层 ----------------
   画三样东西，职责分明：
     · 装配盘（蓝色六边形）—— 【固定】，它是台上的工装，标出插入轴与 P 轴
     · 能量单元（橙色 Q 轴那件）—— 【在动】：插入 → 上推 → 绕P → 绕Q
     · 旋转轴提示线 —— 当前这一步绕谁转
   ★ 改过两遍的教训：早先让六边形跟着上推与旋转一起动（把工装当零件），
     画面就变成"该动的没动、不该动的乱动"。 */


function drawCoreOverlay(g,S,label){
  if(!CORE) return;
  const P = CORE;
  const A = QASM || { ins:0, dz:0, degs:null };
  const CF = l3Conf();
  const opts = { gripAxis: CF.grip };
  const steps = asmSteps();
  /* ---- ① 装配盘：固定不动（永远用 role:'plate' 求位姿）
     ★ 六边形画在【盘面内】的两根轴上：u（盘内横轴）与 v（盘内竖轴）。
       别拿 plate.e（插入轴方向）去当盘内轴 —— 那是垂直于盘面的法向，
       拿它张平面等于把盘子绕插入轴转了 90°（踩过：盘子看起来横过来了）。 ---- */
  const plate = taskPoseBase(P, null, null, Object.assign({ role:'plate' }, opts));
  const Eb = plate.E, Fu = plate.u, Fv = plate.v, Fe = plate.e;
  const R = 150, hex = [];
  for(let k=0;k<6;k++){ const a=k*Math.PI/3;
    hex.push(addv(addv(Eb, mulS(Fu,R*Math.cos(a))), mulS(Fv,R*Math.sin(a)))); }
  g.strokeStyle='#8fd0ff'; g.lineWidth=2; g.beginPath();
  hex.forEach((p,i)=>{const s=S(p); i?g.lineTo(s[0],s[1]):g.moveTo(s[0],s[1]);});
  g.closePath(); g.stroke();
  g.globalAlpha=.12; g.fillStyle='#8fd0ff'; g.fill(); g.globalAlpha=1;
  label('装配盘（固定工装）', S(hex[4])[0]-10, S(hex[4])[1]+16, '#8fd0ff',
        "bold 11px system-ui,'Microsoft YaHei'");
  // 插入轴（Q 轴方向，装配盘的固有方向）
  const e1=S(addv(Eb, mulS(Fe,190)));
  g.strokeStyle='rgba(143,208,255,.55)'; g.lineWidth=1.4; g.setLineDash([6,4]);
  g.beginPath(); g.moveTo(S(Eb)[0],S(Eb)[1]); g.lineTo(e1[0],e1[1]); g.stroke(); g.setLineDash([]);
  label('插入轴 e', e1[0]+4, e1[1]-4, 'rgba(143,208,255,.8)', "11px system-ui,'Microsoft YaHei'");
  // P 轴（六边形顶部那条边）
  const top=addv(Eb, mulS(Fv,R)), p1=S(addv(top, mulS(Fu,80))), p2=S(addv(top, mulS(Fu,-80)));
  g.strokeStyle='#e879f9'; g.lineWidth=3;
  g.beginPath(); g.moveTo(p1[0],p1[1]); g.lineTo(p2[0],p2[1]); g.stroke();
  label('P轴', p2[0]-30, p2[1]+14, '#e879f9', "bold 11px system-ui,'Microsoft YaHei'");
  /* ---- ② 能量单元：在动的那件 ---- */
  const prevLift = G_LIFT; G_LIFT = A.dz || 0;
  const T = taskPoseBase(P, A.degs, null, opts);
  G_LIFT = prevLift;
  const Ue = T.E, Uax = T.er;                     // 单元当前位置与它的轴
  // 单元本体：沿自身轴的一段粗线 + 方头（插入方向那一端）
  const u0=S(Ue), u1=S(addv(Ue, mulS(Uax,130)));
  g.strokeStyle='#ffb454'; g.lineWidth=5;
  g.beginPath(); g.moveTo(u0[0],u0[1]); g.lineTo(u1[0],u1[1]); g.stroke();
  label('能量单元', u1[0]+5, u1[1]-5, '#ffb454', "bold 11px system-ui,'Microsoft YaHei'");
  const gbase=addv(Ue, mulS(Uax,96));
  const g1=S(addv(gbase, mulS(T.u,26))), g2=S(addv(gbase, mulS(T.u,-26)));
  g.strokeStyle='#ffd9a0'; g.lineWidth=4;
  g.beginPath(); g.moveTo(g1[0],g1[1]); g.lineTo(g2[0],g2[1]); g.stroke();
  label('E', u0[0]+6, u0[1]+15, '#8fd0ff', "bold 12px system-ui,'Microsoft YaHei'");
  /* ---- ③ 当前这一步绕谁转（虚线轴） ---- */
  const cur = (T.degs || []).findIndex((v,i) => v > 0.5 && v < steps[i].ang - 0.5);
  const actStep = cur >= 0 ? cur : ((T.degs || []).findIndex((v,i) => v > 0.5 && i === (T.degs||[]).length-1));
  let rax = null;
  if(actStep >= 0){
    /* 两步的轴都是固定轴：P = plate.Paxis；Q = 插入轴 plate.e */
    rax = (steps[actStep].axis === 'q') ? unit(plate.e) : plate.Paxis;
  }
  if(rax){
    const r0 = S(addv(Ue, mulS(rax,-130))), r1 = S(addv(Ue, mulS(rax,130)));
    g.strokeStyle='rgba(255,180,84,.65)'; g.lineWidth=1.6; g.setLineDash([3,3]);
    g.beginPath(); g.moveTo(r0[0],r0[1]); g.lineTo(r1[0],r1[1]); g.stroke(); g.setLineDash([]);
    label('绕' + (steps[actStep].axis === 'q' ? 'Q' : 'P') + '轴', r1[0]+4, r1[1]+12,
          'rgba(255,180,84,.9)', "bold 11px system-ui,'Microsoft YaHei'");
  }
  if(A.dz)  label('上推 ' + A.dz.toFixed(0) + 'mm', u1[0]+6, u1[1]+16, '#ffd9a0',
                  "bold 11px system-ui,'Microsoft YaHei'");
  const dsum = (T.degs || []).slice(0, Math.max(actStep,0)+1).filter(v => v > 0.5).length;
  if(dsum) label('已转 ' + (T.degs||[]).filter(v=>v>0.5).map(v=>v.toFixed(0)+'°').join(' + '),
                 u0[0]-70, u0[1]-8, '#ffd9a0', "bold 11px system-ui,'Microsoft YaHei'");
}
/* 每个关节"实际转了多少"：在关节原点画一段弧，从该关节为 0 时子体 x 轴的方向
   转到当前 x 轴方向（绕关节轴），并标出角度 —— 一眼看出哪个 joint 在转 */
function drawJointArcs(g, S, F, label){
  let qi = 0;
  for(let i=1;i<F.length;i++){
    const c = CHAIN[i-1]; if(!c.movable){ continue; }
    const q = Q[qi]||0; qi++;
    const pj = F[i].t;
    const axW = unit(mulv(F[i-1].R, c.axis));          // 关节轴（世界系）
    const Rzero = mul3(F[i-1].R, c.Rf);                // 该关节为 0 时子体姿态
    const x0 = mulv(Rzero, [1,0,0]), x1 = mulv(F[i].R, [1,0,0]);
    const r0 = unit(subv(x0, mulS(axW, dot3(x0,axW))));
    const r1 = unit(subv(x1, mulS(axW, dot3(x1,axW))));
    let ang = Math.atan2(dot3(cross(r0,r1), axW), dot3(r0,r1));
    if(!isFinite(ang)) continue;
    if(Math.abs(ang) < 0.01){          // 角度≈0 也画一小段，避免"看不出有没有这个功能"
      ang = (q >= 0 ? 1 : -1) * 0.18;
    }
    const zeroish = Math.abs(q) < 0.5;
    const rad = 52, seg = Math.max(6, Math.round(Math.abs(ang)/0.15)), pts = [];
    for(let k=0;k<=seg;k++){
      const t = ang*k/seg;
      const d = addv(mulS(r0, Math.cos(t)), mulS(cross(axW,r0), Math.sin(t)));
      pts.push(S(addv(pj, mulS(d, rad))));
    }
    g.strokeStyle='#ffe066'; g.lineWidth=2.2;
    g.beginPath(); pts.forEach((s,k)=> k? g.lineTo(s[0],s[1]) : g.moveTo(s[0],s[1])); g.stroke();
    // 两条半径边
    const pjS = S(pj);
    g.strokeStyle='rgba(255,224,102,.5)'; g.lineWidth=1.2;
    g.beginPath(); g.moveTo(pjS[0],pjS[1]); g.lineTo(pts[0][0],pts[0][1]);
    g.moveTo(pjS[0],pjS[1]); g.lineTo(pts[pts.length-1][0],pts[pts.length-1][1]); g.stroke();
    const mid = pts[Math.floor(pts.length/2)];
    label(c.joint+' '+q.toFixed(0)+'°'+(zeroish?' (0)':''), mid[0]+5, mid[1]-5,
          zeroish ? '#8fa0bf' : '#ffe066', "bold 11px system-ui,'Microsoft YaHei'");
  }
}
/* ---------------- 任务域数据（只针对三级；位置与难度等级无关） ---------------- */
/* 体素邻域判定：可达集是随机采样出来的，空格子可能只是没采到，
   所以查 ±1 邻域（27 格）任一命中即算可达，避免把真空隙误判成不可达 */
function vkeyN(p, vs){  const i=Math.floor(p[0]/vs), j=Math.floor(p[1]/vs), k=Math.floor(p[2]/vs);
  for(let a=-1;a<=1;a++) for(let b=-1;b<=1;b++) for(let c=-1;c<=1;c++)
    if(VOX.has((i+a)+','+(j+b)+','+(k+c))) return true;
  return false;
}
/* 多起点求解：返回误差最小的那个解
   ★ 判据与面板一致：任一 ok 即 ok（否则"位置更准但姿态差"的解会盖住真 ok 解） */
function bestOfSeeds(Eb, eB, q0, nSeed){
  const lim = qLimits();
  let best = null, okBest = null, bestSc = Infinity;
  for(let s=0; s<nSeed; s++){
    const seed = s===0 ? q0 : lim.map(([lo,hi]) => lo + Math.random()*(hi-lo));
    const r = ikSolve6(Eb, mulS(eB,-1), seed, 200, 1);
    if(r.ok && !okBest) okBest = r;
    const sc = Math.hypot(r.e, W_ORI*(r.ang||0));
    if(sc < bestSc){ bestSc = sc; best = r; }
    if(okBest) break;
  }
  return okBest || best;
}
/* ==================================================================
   按臂分别校核（"单臂三轴检验 × N"）

   ★ 为什么要这一层：
     原来的任务域校核只算【第 0 条臂】—— 可达体素用 VOX（= 臂 0 的）、
     IK 用 fkAll(q)（默认臂 0）、装配位用全局 RULES.init。
     多臂时这套只回答了"第一条臂行不行"。
     而且如果两条臂共用同一个装配位，基座重合、可达集完全相同，
     "分别校核两次"就退化成把同一份答案抄两遍 —— 没有意义。

   ★ 现在的做法：
     · 每条臂有自己的装配位（arm.asm），任务点按【该臂的装配位】转到它自己的 base 系
     · 可达体素也按臂取（ARMSTATS[a].keys）
     · IK 只动【该臂自己的关节】，另一条臂的关节保持不动
       —— 这就是"两只手各做一次"，不是"两只手一起装配一个"

   ★ 下面几个包装是【逐行对照】ikPos / ikSolve6 / bestOfSeeds 写的，
     只替换了三处：
        fkAll(q)  -> fkArm(q, a)      只在那条臂上算正解
        qLimits() -> qLimitsArm(a)    只取那条臂的限位
        q0        -> q0Sub            只含那条臂的关节
     常数（lam=40 / 0.98 / 0.3 / 0.5 / 0.2 / W_ORI）一个字都没改。
     （第一版是凭记忆手抄的，把 lam 衰减和 break 阈值都抄错了，故此处特别标注。）
==================================================================================== */
function armJointRange(a){
  const A = ARMS[a];
  return A ? { j0: A.j0, nJ: A.nJ } : { j0: 0, nJ: NJ };
}
/* 任务点 -> 第 a 条臂的 base 系（用该臂自己的装配位）。
   ★ 注意这里必须直接调 MAP.toBase(p, off)：
     taskToBase 是"只吃一个参数"的转发壳（本地映射用），
     把 off 传给它会【被丢掉】，结果是两条臂算出同一个坐标 —— 踩过。 */
function taskToBaseArm(p, a){
  if(!MAP) return null;
  const off = (typeof asmOfArm === 'function') ? asmOfArm(a) : null;
  return MAP.toBase(p, off);
}
/* 方向向量不含平移，与装在哪条臂无关 */
function dirToBaseArm(d){ return dirToBase(d); }

/* 把"只含该臂关节的子向量"塞回全局 q（其余关节保持当前值不动） */
function withArmQ(qGlobal, qSub, a){
  const R = armJointRange(a);
  const out = qGlobal.slice();
  for (let k = 0; k < R.nJ; k++) out[R.j0 + k] = qSub[k];
  return out;
}
function qLimitsArm(a){
  const R = armJointRange(a);
  return qLimits().slice(R.j0, R.j0 + R.nJ);
}
function curQArm(a){
  const R = armJointRange(a);
  return Q.slice(R.j0, R.j0 + R.nJ);
}
function fkArm(qSub, a){ return fkAllArm(withArmQ(Q, qSub, a), a); }

/* ---- ikPos 的按臂版 ---- */
function ikPosA(target, q0Sub, iters, a){
  let q = q0Sub.slice(), lam = 40;
  const lim = qLimitsArm(a);
  for(let it=0; it<iters; it++){
    const F = fkArm(q, a), pe = F[F.length-1].t;
    const ep = subv(target, pe);
    if(Math.hypot(ep[0],ep[1],ep[2]) < 0.5) break;
    const J = [];
    for(let i=1;i<F.length;i++){ const f=F[i]; if(!f.movable) continue;
      const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
      const ax=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
      J.push(cross(ax, subv(pe,f.t))); }
    const A2=[[0,0,0],[0,0,0],[0,0,0]];
    for(let i=0;i<3;i++)for(let j=0;j<3;j++){ let s=0; for(const jc of J) s+=jc[i]*jc[j];
      A2[i][j]=s+(i===j?lam*lam:0); }
    const y = solveN(A2, ep);
    for(let k=0;k<J.length;k++){ let d=0; for(let i=0;i<3;i++) d+=J[k][i]*y[i];
      d = Math.max(-0.3, Math.min(0.3, d)) / D2R;
      q[k]=Math.min(Math.max(q[k]+d,lim[k][0]),lim[k][1]); }
    lam *= 0.98;
  }
  const F = fkArm(q, a);
  const e = Math.hypot(...subv(target, F[F.length-1].t));
  return {q, ok:e<1.5, e};
}

/* ---- ikSolve6 的按臂版 ---- */
function ikSolve6A(targetP, targetZdir, q0Sub, iters, axIdx, a){
  const ai = (axIdx === undefined) ? 2 : axIdx;
  const seed = ikPosA(targetP, q0Sub, 90, a);
  let q = seed.q.slice(), lam = 40;
  const lim = qLimitsArm(a), tz = unit(targetZdir);
  let bestQ = q.slice(), bestF = Infinity, bestE = Infinity, bestA = 0;
  for(let it=0; it<(iters||200); it++){
    const F = fkArm(q, a), ee = F[F.length-1];
    const ep = subv(targetP, ee.t);
    const zc = [ee.R[0][ai], ee.R[1][ai], ee.R[2][ai]];
    const ax = cross(zc, tz), s = Math.hypot(ax[0],ax[1],ax[2]);
    const c = zc[0]*tz[0]+zc[1]*tz[1]+zc[2]*tz[2];
    let eo, ang;
    if(s < 1e-8){
      if(c > 0){ eo=[0,0,0]; ang=0; }
      else { let p2 = Math.abs(zc[0])<0.9 ? cross(zc,[1,0,0]) : cross(zc,[0,1,0]);
             eo = mulS(unit(p2), Math.PI*W_ORI); ang = Math.PI; }
    } else { ang = Math.atan2(s,c); eo = mulS(mulS(ax,1/s), ang*W_ORI); }
    const dp = Math.hypot(ep[0],ep[1],ep[2]);
    const rr = Math.hypot(dp, W_ORI*ang);
    if(rr < bestF){ bestF = rr; bestQ = q.slice(); bestE = dp; bestA = ang; }
    if(dp < 2 && ang < 0.15) return {q, ok:true, e:dp, ang};
    const wO = 1;
    const eoW = mulS(eo, wO);
    const J = [];
    for(let i=1;i<F.length;i++){
      const f=F[i]; if(!f.movable) continue;
      const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
      const ax2=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
      J.push(cross(ax2, subv(ee.t,f.t)).concat(mulS(ax2, W_ORI*wO)));
    }
    const Am=[]; for(let i=0;i<6;i++) Am.push([0,0,0,0,0,0]);
    for(let i=0;i<6;i++)for(let j=0;j<6;j++){ let s2=0; for(const jc of J) s2+=jc[i]*jc[j];
      Am[i][j]=s2+(i===j?lam*lam:0); }
    const y = solveN(Am, ep.concat(eoW));
    for(let k=0;k<J.length;k++){ let d=0; for(let i=0;i<6;i++) d+=J[k][i]*y[i];
      d = Math.max(-0.2, Math.min(0.2, d)) / D2R;
      q[k]=Math.min(Math.max(q[k]+d,lim[k][0]),lim[k][1]); }
    lam *= 0.985;
  }
  const F = fkArm(bestQ, a), ee = F[F.length-1];
  const zc = [ee.R[0][ai], ee.R[1][ai], ee.R[2][ai]];
  const angOff = Math.acos(Math.max(-1,Math.min(1, zc[0]*tz[0]+zc[1]*tz[1]+zc[2]*tz[2])));
  return { q: bestQ, ok: (bestE < 3 && angOff < 0.2), e: bestE, ang: angOff };
}

/* ---- bestOfSeeds 的按臂版 ---- */
function bestOfSeedsA(Eb, eB, q0Sub, nSeed, a){
  const lim = qLimitsArm(a);
  let best = null, okBest = null, bestSc = Infinity;
  for(let s=0; s<nSeed; s++){
    const seed = s===0 ? q0Sub : lim.map(([lo,hi]) => lo + Math.random()*(hi-lo));
    const r = ikSolve6A(Eb, mulS(eB,-1), seed, 200, 1, a);
    if(r.ok && !okBest) okBest = r;
    const sc = Math.hypot(r.e, W_ORI*(r.ang||0));
    if(sc < bestSc){ bestSc = sc; best = r; }
    if(okBest) break;
  }
  return okBest || best;
}

/* ================= 三级难度：装配全过程逐段校核 =================
   ★ 这是"三级"真正的判据。位置可达率 / 单一位姿 IK 都只回答"能不能到"，
   回答不了"插进去之后还转不转得动" —— 而三级多的正是那一步。
   旋转段是【一档一档连续解】的（上一档的解当下一档的种子），
   所以"解出来"等于"能连续走过去"，不是"某角度碰巧能到"。
   样本数偏小（默认 48）：一次装配要跑 1+1+6 段 IK，太大浏览器会卡。
   它给的是【比例】，不是精确值；要精确值把 window.__L3OPTS.n 调大。 */
function l3Conf(){
  const act = (RULES && RULES.act) || {};
  const steps = asmSteps();
  return {
    N:    (window.__L3OPTS && window.__L3OPTS.n !== undefined) ? window.__L3OPTS.n : 48,
    lift: (act.lift !== undefined) ? act.lift : 100,
    rot:  (act.rotRange) ? act.rotRange[1] : 90,
    step: act.rotStep || 15,
    axis: act.rotAxis || 'p0',
    grip: act.gripAxis || 'x',
    steps: steps,
    nStep: steps.length,
    spin: steps.length > 1 ? steps[1].ang : 0
  };
}
/* 旋转步骤的中文名（面板与自检共用） */
const L3_STEP_NAME = [
  '绕 P 轴转 +90°（能量单元竖到与 Z 轴平行）',
  '绕 Q 轴转 ±90°（多姿态）'
];

/* 采样跑三级校核。结果给面板与无头自检共用。 */
function runL3(ARM, q0){
  const CF = l3Conf();
  if(!RULES || !RULES.levels || !RULES.levels[3] || !MAP) return null;
  if(!(CF.N > 0)) return null;
  const L = RULES.levels[3];
  const q0L3 = (q0 || curQArm(ARM)).slice();
  let nReach=0, nLift=0, nAsm=0, nMulti=0, rotSum=0;
  const nStepOK = new Array(CF.nStep).fill(0);
  const failAt = [0,0,0,0];          // 0=到不了 1=推不上去 2=绕P不行 3=绕Q不行
  const okPts = [], multiPts = [], sampledPts = [], insAngs = [];
  /* ★ 时间预算：三级校核比位置抽样贵得多（一个位姿要跑 1+1+多档 × 多个起点）。
     到点就停，并把"实际算了几个"报出去 —— 宁可少算几个也别把界面卡住。
     想看精确值：window.__L3OPTS = { n: 200, budgetMs: 60000 } 再刷新。 */
  const t0 = (typeof performance !== 'undefined') ? performance.now() : Date.now();
  const budget = (window.__L3OPTS && window.__L3OPTS.budgetMs) || 2500;
  let done = 0;
  for(let i=0;i<CF.N;i++){
    const el = ((typeof performance !== 'undefined') ? performance.now() : Date.now()) - t0;
    if(done >= 8 && el > budget) break;
    const P = {x:rnd(L.x), y:rnd(L.y), z:rnd(L.z),
               th:rnd(L.th), ph:rnd(L.ph), al:rnd(L.al)};
    sampledPts.push([P.x, P.y, P.z]);          // 格子图的分母
    const r = asmFeasibleA(P, ARM, { q0:q0L3, seed0:8, lift:CF.lift, rollStep:CF.step });
    insAngs.push(r.insAng || 0);
    if(r.reach) nReach++;
    if(r.lift)  nLift++;
    for(let s=0;s<CF.nStep;s++) if(r.steps[s]) nStepOK[s]++;
    if(r.asm)  { nAsm++;  okPts.push([P.x,P.y,P.z]); }
    if(r.multi){ nMulti++; multiPts.push([P.x,P.y,P.z]); }
    if(r.asm) rotSum += r.stepFrac[0] || 0;
    if(!r.reach) failAt[0]++;
    else if(!r.lift) failAt[1]++;
    else if(!r.rotP) failAt[2]++;
    else if(!r.rotQ) failAt[3]++;
    done++;
  }
  /* 插入姿态的残差分布：插不进去时"差多少"比"能不能"更有信息量 */
  const sortedAng = insAngs.slice().sort((a,b)=>a-b);
  const medAng = sortedAng.length ? sortedAng[Math.floor(sortedAng.length/2)]*180/Math.PI : 0;
  const angOK = insAngs.filter(a => a < 0.09).length;      // <5.2° 算达标
  /* ---- "真正的任务域"：不是手册那个固定盒子，而是【这条臂、这个装配位下
     实际做得到的范围】。臂型 / 安装 yaw / 装配臂位置 / 上推高度 / 旋转轴，
     任何一个变了它都会变 —— 所以必须现算，不能写死一套。 ---- */
  const boxOf = (pts) => {
    if(!pts.length) return null;
    const lo=[Infinity,Infinity,Infinity], hi=[-Infinity,-Infinity,-Infinity];
    for(const p of pts) for(let k=0;k<3;k++){ if(p[k]<lo[k])lo[k]=p[k]; if(p[k]>hi[k])hi[k]=p[k]; }
    return { lo, hi, size:[hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]] };
  };
  const GX=5, GY=5, GZ=4;
  const hit = new Array(GX*GY*GZ).fill(0), all = new Array(GX*GY*GZ).fill(0);
  const cellOf = (p) => {
    const ix = Math.min(GX-1, Math.max(0, Math.floor((p[0]-L.x[0])/((L.x[1]-L.x[0])/GX))));
    const iy = Math.min(GY-1, Math.max(0, Math.floor((p[1]-L.y[0])/((L.y[1]-L.y[0])/GY))));
    const iz = Math.min(GZ-1, Math.max(0, Math.floor((p[2]-L.z[0])/((L.z[1]-L.z[0])/GZ))));
    return ix*GY*GZ + iy*GZ + iz;
  };
  for(const p of sampledPts) all[cellOf(p)]++;
  for(const p of okPts)      hit[cellOf(p)]++;
  const workBox = boxOf(okPts);
  const nomVol = Math.max(1,(L.x[1]-L.x[0])*(L.y[1]-L.y[0])*(L.z[1]-L.z[0]));
  return { n: done, nWant: CF.N, reach:nReach, lift:nLift, asm:nAsm, multi:nMulti,
           rotMean: nAsm ? rotSum/nAsm : 0,                 // 可装配者的绕P平均行程
           angOK, medAng,                                   // 插入姿态达标数 / 残差中位
           failAt, workBox, nomVol,
           workVol: workBox ? Math.max(0, workBox.size[0]*workBox.size[1]*workBox.size[2]) : 0,
           cells: { GX, GY, GZ, hit, all },
           xr: L.x, yr: L.y, zr: L.z,          // 名义域各轴范围（画格子图用）
           nStepOK, nStep: CF.nStep,
           liftD: CF.lift, step: CF.step, spin: CF.spin, steps: CF.steps };
}

/* 三级那一段的面板 HTML（analyzeTaskArm 与无头自检共用同一份渲染） */
function taskL3HTML(L3){
  const c = L3.n || 1;
  const p = (a) => (100*a/c).toFixed(0) + '%';
  const cls = (a) => (a/c > 0.9 ? 'good' : (a/c < 0.5 ? 'bad' : ''));
  const wb = L3.workBox;
  const volPct = L3.nomVol ? (100*L3.workVol/L3.nomVol) : 0;
  const g = L3.cells;
  const cellTxt = (i) => {
    const t = g.all[i], h = g.hit[i];
    if(!t) return '·';
    const r = h/t;
    return (r >= 0.999 ? '●' : (r > 0 ? '◐' : '○'));
  };
  let gridHtml = '';
  if(g){
    gridHtml =
      '<div style="color:#5d6b88;font-size:10.5px;margin:3px 0 1px">' +
      'x–z 切片（每个 y 层一张）：● 全通过　◐ 部分通过　○ 全不可行　· 未抽到</div>';
    for(let iy=0; iy<g.GY; iy++){
      const y0 = L3.yr[0] + (L3.yr[1]-L3.yr[0])*iy/g.GY;
      const y1 = L3.yr[0] + (L3.yr[1]-L3.yr[0])*(iy+1)/g.GY;
      let rows = '';
      for(let iz=g.GZ-1; iz>=0; iz--){
        let tds = '';
        for(let ix=0; ix<g.GX; ix++){
          const idx = ix*g.GY*g.GZ + iy*g.GZ + iz;
          tds += '<td style="text-align:center;font-size:11px">' + cellTxt(idx) + '</td>';
        }
        const z0 = L3.zr[0] + (L3.zr[1]-L3.zr[0])*iz/g.GZ;
        rows += '<tr><td style="color:#5d6b88;font-size:10px">z' + z0.toFixed(0) + '</td>' + tds + '</tr>';
      }
      gridHtml += '<div style="margin-top:4px;color:#8fa0bf;font-size:10.5px">y ' +
        y0.toFixed(0) + '~' + y1.toFixed(0) + '</div>' +
        '<table style="margin:0">' + rows +
        '<tr><td></td>' + Array.from({length:g.GX}, (_,ix)=>
          '<td style="color:#5d6b88;font-size:10px;text-align:center">x' +
          (L3.xr[0]+(L3.xr[1]-L3.xr[0])*ix/g.GX).toFixed(0) + '</td>').join('') + '</tr></table>';
    }
  }
  return `
    <div class="sect">三级任务域（装配全过程）— 这是三级的真判据</div>
    <div class="grid">
      <span>可装配（插入 + 上推 + 绕P竖起）</span><b class="${cls(L3.asm)}">${p(L3.asm)}</b>
      <span>可多姿态（再绕Q转 ±${L3.spin}°）</span><b class="${cls(L3.multi)}">${p(L3.multi)}</b>
      <span>① 到插入位（就位+插入）</span><b class="${cls(L3.reach)}">${p(L3.reach)}</b>
      <span>　其中插入姿态 &lt;5°</span><b class="${cls(L3.angOK)}">${p(L3.angOK)}</b>
      <span>　插入姿态残差（中位）</span><b>${L3.medAng.toFixed(1)}°</b>
      <span>② 上推 ${L3.liftD}mm</span><b class="${cls(L3.lift)}">${p(L3.lift)}</b>
      <span>③ ${L3_STEP_NAME[0]}（每 ${L3.step}° 一档、连续走）</span>
          <b class="${cls(L3.nStepOK[0]||0)}">${p(L3.nStepOK[0]||0)}</b>
      <span>④ ${L3_STEP_NAME[1]}</span>
          <b class="${cls(L3.nStepOK[1]||0)}">${p(L3.nStepOK[1]||0)}</b>
      <span>样本</span><b>${L3.n} 个位姿</b>
    </div>
    <div style="color:#5d6b88;font-size:10.5px;margin-top:2px">
      卡在哪一段：到不了 <b>${L3.failAt[0]}</b> · 推不上去 <b>${L3.failAt[1]}</b> ·
      绕P竖不起来 <b>${L3.failAt[2]}</b> · 绕Q转不动 <b>${L3.failAt[3]}</b>（共 ${L3.n} 个）<br>
      两步旋转都是<b>一档一档连续解</b>的 —— 量的是"带着零件能不能转过去"，
      不是"某个角度碰巧能不能到"。<br>
      <b>可装配</b> = 插进去、推上去、再绕 P 轴竖到与 Z 轴平行；
      <b>可多姿态</b> = 在此之上还能绕（已经竖起来的）Q 轴转 ±${L3.spin}°。
    </div>

    <div class="sect">真正的任务域（现算 · 不是手册那个固定盒子）</div>
    <div class="grid">
      <span>名义域体积</span><b>${(L3.nomVol/1e6).toFixed(1)} ×10⁶ mm³</b>
      <span>可装配点的包络</span><b>${wb ? (wb.size.map(v=>v.toFixed(0)).join(' × ') + ' mm') : '（无通过点）'}</b>
      <span>包络体积 / 名义体积</span><b class="${volPct>60?'good':(volPct<25?'bad':'')}">${volPct.toFixed(0)}%</b>
      ${wb ? `<span>包络范围（OXYZ）</span><b>x ${wb.lo[0].toFixed(0)}~${wb.hi[0].toFixed(0)} ·
          y ${wb.lo[1].toFixed(0)}~${wb.hi[1].toFixed(0)} · z ${wb.lo[2].toFixed(0)}~${wb.hi[2].toFixed(0)}</b>` : ''}
    </div>
    ${gridHtml}
    <div style="color:#5d6b88;font-size:10.5px;margin-top:3px">
      ★ 格子图按<b>可装配</b>统计。这个域是<b>现算</b>的：臂型、安装 yaw、装配臂位置、
      上推高度、两步旋转怎么定，任何一个变了它都会跟着变 ——
      手册那个盒子只是"要够到这里"，够不够得着得看这条臂在这个装配位下的实际结果。
    </div>
`;
}

function analyzeTaskArm(ARM){
  /* ★ 这个函数【只算不写 DOM】—— 返回数字与面板 HTML，由 renderTaskPanel 渲染。
     多臂时 analyzeTask 会为每条臂调一次。前提条件不满足时返回 null，
     调用方（analyzeTask）已经做过同样的守卫。 */
  const ARMV = (ARMSTATS[ARM] && ARMSTATS[ARM].keys) ? ARMSTATS[ARM].keys : VOX;
  if(!MODEL) return null;
  if(!RULES || !RULES.levels || !RULES.levels[3] || !MAP) return null;
  if(!ARMV || !ARMV.size) return null;
  const L = RULES.levels[3], vs = 50, N = 6000;
  let hit=0, hit0=0; const zb=new Map(); const miss=[];
  for(let i=0;i<N;i++){
    const x=rnd(L.x), y=rnd(L.y), z=rnd(L.z);
    const p=taskToBaseArm([x,y,z], ARM);
    const ok = vkeyN(p,vs);
    if(ok) hit++;
    if(ARMV.has(vkey(p,vs))) hit0++;
    if(!ok && miss.length<400) miss.push(p);
    const k=Math.floor(z/100)*100;
    if(!zb.has(k)) zb.set(k,[0,0]);
    const b=zb.get(k); b[0]++; if(ok) b[1]++;
  }
  // 不可达点到最近可达采样点的距离（中位数）
  let med='-';
  if(miss.length && CLOUD.length){
    const ds=[];
    for(const p of miss){ let m=Infinity;
      for(const c of CLOUD){ const d=Math.hypot(c[0]-p[0],c[1]-p[1],c[2]-p[2]); if(d<m)m=d; }
      ds.push(m); }
    ds.sort((a,b)=>a-b); med = ds[Math.floor(ds.length/2)].toFixed(0)+' mm';
  }
  const pct = (a,b) => (100*a/b);
  // ---- IK 复核（含姿态约束：插入时 夹爪 y ∥ Q 轴），这才是真正的可行性判定 ----
  /* ★ 起点数 3 → 12：这份 IK 是数值求解器，单起点成功率只有 ~66%（见 README）。
     实测（320 个三级任务域位姿）：3 起点 89%、8 起点 99%、12 起点 99.4%、16 起点 99.7%；
     而"只解位置"和强多起点都是 100% —— 也就是说剩下的零星失败是求解器运气，不是几何不可达。
     320 位姿 × 12 起点（命中即停）整段约 2 s，可以接受。 */
  let ikOK = 0, ikN = 320, ikMiss = 0, ikBest = 0, ikPosOK = 0, ikOKplus = 0;
  const IK_START = 12, IK_ITER = 300;
  const ikErr = [];
  /* ★ q0 必须是【这条臂】的关节子向量。
     原来写 Q.slice()（全链），后面 q0.map((v,i)=> qLimitsArm(ARM)[i])
     一越界就抛 undefined[0] —— 多臂时必崩。 */
  const q0 = curQArm(ARM);
  for(let i=0;i<ikN;i++){
    const P = {x:rnd(L.x), y:rnd(L.y), z:rnd(L.z), th:rnd(L.th), ph:rnd(L.ph), al:rnd(L.al)};
    const Eb = taskToBaseArm([P.x,P.y,P.z], ARM);
    const eB = dirToBaseArm(coreFrame(P).e);
    // 对照：只解位置。★ 固定 3 起点、独立跑，不跟下面主循环的"命中即停"耦合
    //   （耦合过：主循环一命中就 break，这个对照项就退化成"只有 1 次机会"，数字会莫名变低）
    let posOnly = 0;
    for(let s=0;s<3;s++){
      const seed = s===0 ? q0 : q0.map((v,i)=>{ const lim=qLimitsArm(ARM)[i];
        return lim[0]+Math.random()*(lim[1]-lim[0]); });
      if(ikPosA(Eb, seed, 140, ARM).e < 5) posOnly++;
    }
    if(posOnly) ikPosOK++;
    /* 主：多起点（零位 + 11 个随机种子），插入姿态 夹爪 y ∥ Q 轴。
       ★ 挑解判据必须是"任一 ok 即 ok"：只按位置误差挑，会让"位置更准但姿态差"的解
         盖住真正 ok 的解，凭空少算几个百分点（这个坑我踩过）。 */
    let best = null, okBest = null, bestSc = Infinity;
    for(let s=0;s<IK_START;s++){
      const seed = s===0 ? q0 : q0.map((v,i)=>{ const lim=qLimitsArm(ARM)[i];
        return lim[0]+Math.random()*(lim[1]-lim[0]); });
      const r = ikSolve6A(Eb, mulS(eB,-1), seed, IK_ITER, 1, ARM);
      if(r.ok && !okBest) okBest = r;
      const sc = Math.hypot(r.e, W_ORI*(r.ang||0));            // 全失败时留真残差最小的那个做诊断
      if(sc < bestSc){ bestSc = sc; best = r; }
      if(okBest) break;
    }
    best = okBest || best;
    if(ikSolve6A(Eb, mulS(eB, +1), q0, IK_ITER, 1, ARM).ok) ikOKplus++;   // 夹爪 y = +e⃗（单起点对照）
    ikErr.push(best.e);
    if(best.ok) ikOK++;
    else if(best.e > 40) ikMiss++;
    if(best.e < 5) ikBest++;
  }
  ikErr.sort((a,b)=>a-b);
  const ikMed = ikErr.length ? ikErr[Math.floor(ikErr.length/2)] : 0;
  /* ---- 失败原因诊断：到底是"关节限位卡住"还是"真的够不到" ----
     ① 统计失败位姿里最优解顶到限位的关节（顶死 = 放宽限位就能救）
     ② 把限位整体放宽 ±25° 再解一遍，看可解率能涨到多少 */
  let diagN = 0, diagSaved = 0;
  /* ★ 只统计【这条臂】的关节：hitLim / lim0 / r0.q 都是按臂局部的。
     以前写 NJ（= 全部关节），多臂时会越界读 lim0[k] -> undefined[0]，
     浏览器里直接抛。这是按臂改造里唯一的越界点。 */
  const nJA = qLimitsArm(ARM).length;
  const hitLim = new Array(nJA).fill(0);
  const base = ikOK / ikN;
  for(let i=0;i<140;i++){
    const P = {x:rnd(L.x), y:rnd(L.y), z:rnd(L.z), th:rnd(L.th), ph:rnd(L.ph), al:rnd(L.al)};
    const Eb = taskToBaseArm([P.x,P.y,P.z], ARM), eB = dirToBaseArm(coreFrame(P).e);
    const r0 = bestOfSeedsA(Eb, eB, q0, 3, ARM);
    if(r0.ok) continue;
    diagN++;
    const lim0 = qLimitsArm(ARM);
    for(let k=0;k<nJA;k++){
      const lo=lim0[k][0], hi=lim0[k][1];
      if(Math.abs(r0.q[k]-lo) < 4 || Math.abs(r0.q[k]-hi) < 4) hitLim[k]++;
    }
    LIM_WIDEN = 25;                                  // 放宽限位再试
    const r1 = bestOfSeedsA(Eb, eB, q0, 3, ARM);
    LIM_WIDEN = 0;
    if(r1.ok) diagSaved++;
  }
  /* 关节名也按臂取（原来是全链的，多臂时名字与 hitLim 对不上） */
  const RGA = armJointRange(ARM);
  const limNames = CHAIN.slice(RGA.j0, RGA.j0 + RGA.nJ)
                        .filter(c=>c.movable).map(c=>c.joint);
  const hitTxt = limNames.map((n,i)=> hitLim[i] ? (n+' '+Math.round(100*hitLim[i]/Math.max(diagN,1))+'%') : '')
                        .filter(Boolean).join(' · ') || '无';
  const widenRate = diagN ? (ikOK + diagSaved) / ikN : base;
  /* ---- 三级：装配全过程逐段校核（见 l3Conf / runL3 / asmFeasibleA） ---- */
  const L3 = runL3(ARM, q0);
  /* ---- 安装朝向扫描：哪个 yaw 的可解率最高（这是提高可解率最有效的杠杆） ---- */
  let scanBest = null, scanCurve = [];
  if(MODEL){
    /* ★ 只扫【这条臂】的安装朝向，扫完复原。
       原来改的是 MODEL.base.yaw（= 第 0 条臂），多臂时等于只动第一条。 */
    const tgt = (typeof ARMS !== 'undefined' && ARMS[ARM]) ? ARMS[ARM].model : MODEL;
    const yaw0 = tgt.base.yaw, NS = 60, poses = [];
    for(let i=0;i<NS;i++) poses.push({x:rnd(L.x), y:rnd(L.y), z:rnd(L.z),
                                      th:rnd(L.th), ph:rnd(L.ph), al:rnd(L.al)});
    for(let y=-180; y<180; y+=45){
      tgt.base.yaw = y;
      let ok = 0;
      for(const P of poses){
        const Eb = taskToBaseArm([P.x,P.y,P.z], ARM), eB = dirToBaseArm(coreFrame(P).e);
        if(bestOfSeedsA(Eb, eB, q0, 2, ARM).ok) ok++;
      }
      scanCurve.push([y, ok/NS]);
      if(!scanBest || ok/NS > scanBest[1]) scanBest = [y, ok/NS];
    }
    tgt.base.yaw = yaw0;                      // 恢复原值
  }
  // ---- 零位朝向校核：末端(最后一个关节)的 z 轴 与 Q 轴正方向 e⃗ 的关系 ----
  // 理想（正面装配）= 两者【相对】(≈180°)；若 ≈0° 说明同向，臂得先绕到反面去，
  // 解算自然会出现"抽象行为"。可把臂的安装 yaw 改 180° 或改 RULES 里的映射来翻面。
  const Fz = fkArm(new Array(qLimitsArm(ARM).length).fill(0));
  const eeZ = Fz[Fz.length-1];
  const zAx = [eeZ.R[0][2], eeZ.R[1][2], eeZ.R[2][2]];
  const Ptyp = {x:L.x[0], y:(L.y[0]+L.y[1])/2, z:(L.z[0]+L.z[1])/2, th:0, ph:90, al:0};
  const eTyp = dirToBaseArm(coreFrame(Ptyp).e);
  const cosZ = Math.max(-1, Math.min(1, dot3(zAx, eTyp)));
  const angZ = Math.acos(cosZ)*180/Math.PI;
  const facing = angZ > 135 ? '正对（正面装配 ✓）'
               : (angZ < 45 ? '同向（需绕到反面 ✗）' : '斜交');
  // ---- IK 最小自检：把"当前末端位置"偏移 20mm 当目标，IK 应当立刻收回 ----
  /* fkArm 要的是【该臂的关节子向量】；索引用 F.length-1（该臂末端）。
     原来传 Q（全链）+ CHAIN.length（全链长度）—— 多臂时既越界又算错。 */
  const _F0 = fkArm(q0, ARM);
  const eeNow = _F0[_F0.length-1].t;
  const selfT = [eeNow[0]+20, eeNow[1], eeNow[2]];
  const traj = [1,2,5,20,80,300].map(k => ikPosA(selfT, q0, k, ARM).e.toFixed(1)).join(' → ');
  // 诊断：雅可比各列模长（正常应是几百 mm 量级）
  const Fd = fkArm(q0, ARM), ped = Fd[Fd.length-1].t, colMag = [];
  for(let i=1;i<Fd.length;i++){ const f=Fd[i]; if(!f.movable) continue;
    const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
    const a=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
    colMag.push(Math.hypot(...cross(a, subv(ped,f.t))).toFixed(0)); }
  const jacInfo = '|J|列 = ' + colMag.join(', ');
  const rrInfo = 'r模长 = ' + Fd.map((f,i)=> i? Math.hypot(...subv(ped,f.t)).toFixed(0):null).filter(Boolean).join(', ');
  // 单步诊断：算一次 DLS，看 y 与 Δq 的实际量级
  const e1 = subv(selfT, ped), J1 = [];
  for(let i=1;i<Fd.length;i++){ const f=Fd[i]; if(!f.movable) continue;
    const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
    const a=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
    J1.push(cross(a, subv(ped,f.t))); }
  const A1=[[0,0,0],[0,0,0],[0,0,0]];
  for(let i=0;i<3;i++)for(let j=0;j<3;j++){ let s=0; for(const jc of J1) s+=jc[i]*jc[j];
    A1[i][j]=s+(i===j?40*40:0); }
  const y1 = solveN(A1, e1);
  const dq1 = J1.map(j => (j[0]*y1[0]+j[1]*y1[1]+j[2]*y1[2]));
  const stepInfo = '单步：|e|=' + Math.hypot(...e1).toFixed(1) + ' |y|=' +
      Math.hypot(...y1).toExponential(2) + ' Δq(rad)=' + dq1.map(v=>v.toExponential(2)).join(', ');
  const selfR = ikPosA(selfT, q0, 80, ARM);
  const _Fs = fkArm(selfR.q, ARM);
  const selfD = Math.hypot(...subv(selfT, _Fs[_Fs.length-1].t));
  const selfA = ikSolve6A(selfT, [0,0,1], q0, 120, ARM);
  let rows='';
  [...zb.keys()].sort((a,b)=>a-b).forEach(k=>{
    const b=zb.get(k), r=pct(b[1],b[0]);
    const cls = r>90?'good':(r<50?'bad':'');
    rows += `<tr><td>z ${k}~${k+100}</td><td class="num">${b[0]}</td>
             <td class="num ${cls}">${r.toFixed(1)}%</td></tr>`;
  });
  /* ★ 只算不写 DOM：把面板 HTML 与关键数字一起返回。
     多臂时 analyzeTask 会为每条臂调一次，再由 renderTaskPanel 按选择渲染。 */
  const html = `
    <div class="sect">任务域（硬编码 · 规则手册表5-10）</div>
    <table><tr><th>难度</th><th>x</th><th>y</th><th>z</th><th>θ/φ/α</th></tr>
      ${[1,2,3].map(i=>{const v=RULES.levels[i];
        return `<tr><td>${v.name}</td><td>${v.x[0]}~${v.x[1]}</td><td>${v.y[0]}~${v.y[1]}</td>
                <td>${v.z[0]}~${v.z[1]}</td>
                <td>${v.th[0]}~${v.th[1]} / ${v.ph[0]}~${v.ph[1]} / ${v.al[0]}~${v.al[1]}</td></tr>`;}).join('')}
    </table>
    <div class="grid">
      <span>底盘高度 ⇒ base 系 z</span><b>${RULES.baseH} ⇒ ${L.z[0]-RULES.baseH}~${L.z[1]-RULES.baseH}</b>
      <span>装配臂位置 (OXYZ)</span><b>${RULES.init.x}, ${RULES.init.y}, ${RULES.init.z}</b>
    </div>

    <div class="sect">位置可达率（E 点落入可达集，与难度无关）</div>
    <div class="grid">
      <span>可达（含 ±1 体素容差）</span><b class="good">${pct(hit,N).toFixed(1)}%</b>
      <span>严格同体素</span><b>${pct(hit0,N).toFixed(1)}%</b>
      <span>不可达点最近可达距离中位</span><b>${med}</b>
    </div>
${L3 ? taskL3HTML(L3) : ''}

    <div class="sect">IK 复核（插入时 夹爪 y ∥ Q 轴）</div>
    <div class="grid">
      <span>可解（${ikN} 位姿 · ${IK_START} 起点）</span><b class="${ikOK/ikN>0.9?'good':'bad'}">${pct(ikOK,ikN).toFixed(1)}%</b>
      <span>只解位置（对照）</span><b class="${ikPosOK/ikN>0.9?'good':'bad'}">${pct(ikPosOK,ikN).toFixed(1)}%</b>
      <span>位置误差 &lt;5mm</span><b>${pct(ikBest,ikN).toFixed(1)}%</b>
      <span>中位位置误差</span><b>${ikMed.toFixed(0)} mm</b>
      <span>对齐取 +e⃗ 时（单起点）</span><b>${pct(ikOKplus,ikN).toFixed(1)}%</b>
    </div>
    <div style="color:#5d6b88;font-size:10.5px;margin-top:2px">
      数值 IK（阻尼最小二乘 · 每个位姿最多 ${IK_START} 起点 / ${IK_ITER} 迭代，命中即停）：
      <b>它是"能解出来"的下界，不是几何判据</b> —— 零星失败多为求解器运气，
      与"只解位置"那一行的差距才是插入姿态造成的损失。几何结论看上面体素与"失败原因诊断"。
    </div>

    <div class="sect">失败原因诊断</div>
    <div>顶到限位的关节：<b>${hitTxt}</b></div>
    <div>把各关节限位整体放宽 ±25° 后再解：<b class="good">${pct(widenRate,1).toFixed(1)}%</b>
         <span style="color:#5d6b88">（提升幅度 = 限位造成的损失）</span></div>
    ${scanBest ? `<div>安装朝向扫描（当前 yaw ${MODEL?MODEL.base.yaw:0}°）：
         最佳 <b class="good">yaw ${scanBest[0]}° ⇒ ${pct(scanBest[1],1).toFixed(1)}%</b></div>
      <div style="color:#5d6b88;font-size:10.5px">${scanCurve.map(c=>c[0]+'°:'+Math.round(100*c[1])+'%').join('　')}</div>` : ''}

    <div class="sect">零位朝向校核</div>
    <div class="grid">
      <span>末端 z vs Q 轴 e⃗</span>
          <b class="${angZ>135?'good':'bad'}">${angZ.toFixed(1)}° ${facing}</b>
    </div>

    <div class="one" style="margin-top:10px;color:#5d6b88">
      说明与用法见 README.md</div>`;
  return { arm: ARM, html,
           hitRate: pct(hit,N), hit0Rate: pct(hit0,N),
           ikRate: pct(ikOK,ikN), ikPosRate: pct(ikPosOK,ikN),
           asm: (typeof asmOfArm === 'function') ? asmOfArm(ARM) : null,
           name: (typeof ARMS !== 'undefined' && ARMS[ARM]) ? (ARMS[ARM].name || ('臂'+ARM)) : '臂0' };
}

/* ==================================================================
   analyzeTask —— 逐臂汇总

   单臂时行为与以前一致（就是 analyzeTaskArm(0)）。
   多臂时【每条臂各做一次完整校核】，界面加一个臂选择器切换查看。
   这就是"两只手都做一次"；不做"两只手一起装配一个"
   （那需要多目标 IK，不在本工具范围内）。
==================================================================================== */

/* ---------------- 面板渲染（含多臂选择器） ---------------- */
function renderTaskPanel(el){
  if(!el) return;
  const n = TASK_BY_ARM.length;
  const cur = TASK_BY_ARM[TASK_ARM];
  if(!cur){ el.textContent = '第 ' + (TASK_ARM + 1) + ' 条臂算不出来'; return; }
  let head = '';
  if(n > 1){
    head = '<div class="sect">按臂分别校核（' + n + ' 条臂各做一次）</div>' +
      '<div style="display:flex;gap:6px;flex-wrap:wrap;margin:2px 0 6px">' +
      TASK_BY_ARM.map((r, a) => {
        if(!r) return '';
        const on = a === TASK_ARM;
        return '<button data-taskarm="' + a + '"' + (on ? ' class="primary"' : '') +
               ' title="查看「' + r.name + '」的任务域结果">' + r.name +
               ' · 可达 ' + r.hitRate.toFixed(0) + '%' +
               ' · IK ' + r.ikRate.toFixed(0) + '%</button>';
      }).join('') + '</div>' +
      '<div style="color:#5d6b88;font-size:10.5px;margin-bottom:6px">' +
      '每条臂按【自己的装配位】单独校核；两条臂不联动、不做协同装配。' +
      '当前查看：<b>' + cur.name + '</b>（装配位 ' +
      (cur.asm ? (cur.asm.x + ', ' + cur.asm.y + ', ' + cur.asm.z) : '-') + '）</div>';
  }
  el.innerHTML = head + (typeof sizePanelHTML === 'function' ? sizePanelHTML() : '') + cur.html;
  /* 选择器接线 */
  el.querySelectorAll('[data-taskarm]').forEach(b => {
    b.onclick = () => { TASK_ARM = +b.dataset.taskarm; renderTaskPanel(el); };
  });
}

let TASK_ARM = 0;               // 当前查看第几条臂的任务域结果
let TASK_BY_ARM = [];
function analyzeTask(){
  const el = document.getElementById('dataBody');
  if(!MODEL){ el.textContent='载入模型后显示…'; return; }
  if(!RULES || !RULES.levels || !RULES.levels[3] || !MAP){
    el.innerHTML = '<div class="sect">任务域校核</div>' +
      '<div class="one" style="color:#5d6b88;line-height:1.8">' +
      '当前构型没有绑定竞赛规则，所以这里只显示通用几何量。<br>' +
      '『可达域』部分照常可用；任务域（三级位姿范围）需要一个 <b>rules</b> 配置。' +
      '</div>';
    return;
  }
  if(!VOX || !VOX.size){ el.textContent='可达集为空'; return; }
  const n = (typeof armCount === 'function') ? armCount() : 1;
  if(TASK_ARM >= n) TASK_ARM = 0;
  TASK_BY_ARM = [];
  for(let a=0; a<n; a++){
    let r = null;
    try { r = analyzeTaskArm(a); } catch(e){ if(a===0) throw e; }
    TASK_BY_ARM.push(r);
  }
  renderTaskPanel(el);
}

/* ---------------- 数值 IK（阻尼最小二乘：位置 + 末端 z 轴对齐） ----------------
   姿态约束来自你的要求：Q 轴正对着的是夹爪的 Z−，即【夹爪自身 z = -e⃗】   */
function solveN(A, b){
  const n=b.length, M=A.map((r,i)=>r.concat([b[i]]));
  for(let c=0;c<n;c++){
    let p=c; for(let r=c+1;r<n;r++) if(Math.abs(M[r][c])>Math.abs(M[p][c])) p=r;
    if(Math.abs(M[p][c])<1e-12) return new Array(n).fill(0);
    const t=M[c]; M[c]=M[p]; M[p]=t;
    for(let r=0;r<n;r++){ if(r===c) continue;
      const f=M[r][c]/M[c][c];
      for(let k=c;k<=n;k++) M[r][k]-=f*M[c][k];
    }
  }
  return M.map((r,i)=> r[i] ? r[n]/r[i] : 0);
}
const W_ORI = 40;                     // 姿态误差权重（mm per rad）
function ikPos(target, q0, iters){    // 阶段一：只解位置（先拿到一个能到的种子）
  let q=q0.slice(), lam=40; const lim=qLimits();
  for(let it=0; it<iters; it++){
    const F=fkAll(q), pe=F[F.length-1].t;
    const ep=subv(target,pe);
    if(Math.hypot(ep[0],ep[1],ep[2])<0.5) break;
    const J=[];
    for(let i=1;i<F.length;i++){ const f=F[i]; if(!f.movable) continue;
      const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
      const a=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
      J.push(cross(a, subv(pe,f.t))); }
    const A=[[0,0,0],[0,0,0],[0,0,0]];
    for(let i=0;i<3;i++)for(let j=0;j<3;j++){ let s=0; for(const jc of J) s+=jc[i]*jc[j];
      A[i][j]=s+(i===j?lam*lam:0); }
    const y=solveN(A,ep);
    for(let k=0;k<J.length;k++){ let d=0; for(let i=0;i<3;i++) d+=J[k][i]*y[i];
      // ★ d 是【弧度】，而 q/限位全是【角度】——必须 /D2R 换算，
      //   否则步长小 57.3 倍（曾导致 IK 收敛极慢、被误判成"不可达"）
      d = Math.max(-0.3, Math.min(0.3, d)) / D2R;
      q[k]=Math.min(Math.max(q[k]+d,lim[k][0]),lim[k][1]); }
    lam*=0.98;
  }
  const F=fkAll(q); const e=Math.hypot(...subv(target,F[F.length-1].t));
  return {q, ok:e<1.5, e};
}
function ikSolve6(targetP, targetZdir, q0, iters, axIdx){
  const ai = (axIdx === undefined) ? 2 : axIdx;   // 对齐轴：2 = 夹爪 z，1 = 夹爪 y
  const seed = ikPos(targetP, q0, 90);          // 先位置后姿态，避免 180° 姿态误差一开始就压垮位置项
  let q = seed.q.slice(), lam = 40;
  const lim = qLimits(), tz = unit(targetZdir);
  /* ★ 保留历史最优。这一步不是可有可无：迭代是欠阻尼的（λ 从 40 单调衰减到 ~2，
     没有"这一步是否真的变好"的接受判据），最后一步经常比过程中到过的最好点更差。
     判据用 F = ‖[位置误差 ; W_ORI·姿态轴角误差]‖ —— 与 DLS 真正在最小化的量一致。 */
  let bestQ = q.slice(), bestF = Infinity, bestE = Infinity, bestA = 0;
  for(let it=0; it<(iters||200); it++){
    const F = fkAll(q), ee = F[F.length-1];
    const ep = subv(targetP, ee.t);
    const zc = [ee.R[0][ai], ee.R[1][ai], ee.R[2][ai]];    // 末端指定轴（世界系）
    // 轴角姿态误差：叉乘在小角度下够用，但 zc 与目标反平行时叉乘=0、梯度消失（奇点）
    const ax = cross(zc, tz), s = Math.hypot(ax[0],ax[1],ax[2]);
    const c = zc[0]*tz[0]+zc[1]*tz[1]+zc[2]*tz[2];
    let eo, ang;
    if(s < 1e-8){
      if(c > 0){ eo=[0,0,0]; ang=0; }
      else { let p = Math.abs(zc[0])<0.9 ? cross(zc,[1,0,0]) : cross(zc,[0,1,0]);
             eo = mulS(unit(p), Math.PI*W_ORI); ang = Math.PI; }
    } else { ang = Math.atan2(s,c); eo = mulS(mulS(ax,1/s), ang*W_ORI); }
    const dp = Math.hypot(ep[0],ep[1],ep[2]);
    const rr = Math.hypot(dp, W_ORI*ang);
    if(rr < bestF){ bestF = rr; bestQ = q.slice(); bestE = dp; bestA = ang; }
    if(dp < 2 && ang < 0.15) return {q, ok:true, e:dp, ang};
    /* ★ 姿态度量全程给足权重（原先是"远 0.02 / 中 0.25 / 近了才 1"的门控）。
       门控会在迭代中途换掉目标函数、且**在远/中/近三档之间跳变**，是收敛后期来回震荡的主因。
       实测（320 个三级任务域位姿，"任一起点 ok 即 ok"）：3 起点 82.5% → 90.6%，
       12 起点 99.1% → 99.4%，24 起点 99.7% → 100.0%。
       ⚠ 量这个改动时，挑"最优起点"的判据必须和 ok 判据一致（任一 ok 即 ok）：
         若按"位置误差最小"挑，会把"位置更准但姿态差"的解盖在真 ok 解上面，凭空量出 5~8% 的假失败。 */
    const wO = 1;
    const eoW = mulS(eo, wO);
    const J = [];
    for(let i=1;i<F.length;i++){
      const f=F[i]; if(!f.movable) continue;
      const nn=Math.hypot(f.axis[0],f.axis[1],f.axis[2]);
      const a=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
      J.push(cross(a, subv(ee.t,f.t)).concat(mulS(a, W_ORI*wO)));
    }
    const A=[]; for(let i=0;i<6;i++) A.push([0,0,0,0,0,0]);
    for(let i=0;i<6;i++)for(let j=0;j<6;j++){ let s2=0; for(const jc of J) s2+=jc[i]*jc[j];
      A[i][j]=s2+(i===j?lam*lam:0); }
    const y = solveN(A, ep.concat(eoW));
    for(let k=0;k<J.length;k++){ let d=0; for(let i=0;i<6;i++) d+=J[k][i]*y[i];
      d = Math.max(-0.2, Math.min(0.2, d)) / D2R;          // ★ 弧度 → 角度（同 ikPos）
      q[k]=Math.min(Math.max(q[k]+d,lim[k][0]),lim[k][1]); }
    lam *= 0.985;
  }
  /* ★ 用历史最优判定，而不是最后一次迭代 */
  const F=fkAll(bestQ), ee=F[F.length-1];
  const zc=[ee.R[0][ai],ee.R[1][ai],ee.R[2][ai]];
  const angOff = Math.acos(Math.max(-1,Math.min(1, zc[0]*tz[0]+zc[1]*tz[1]+zc[2]*tz[2])));
  const dp = Math.hypot(...subv(targetP, ee.t));
  return {q:bestQ, ok: dp<3 && angOff<0.2, e:dp, ang:angOff, bestE:bestE, bestA:bestA};
}

/* ---------------- 示例装配动画（三级任务） ----------------
   序列就是三级要求的那三步（与判据 asmFeasibleA 同源）：
     ① 就位：臂移到插入轴外侧（工具轴与插入轴同轴）
     ② 插入：沿插入轴推进（核心本体不动）
     ③ 上推：核心与手【一起】沿 z 升 LIFT
     ④ 旋转：核心与手【一起】绕旋转轴转 ROT
     ⑤ 完成

   ★ 修的是什么（用户报"插入后机械臂就不跟着往上推再旋转了"）：
     旧版的目标点用【未旋转的 e0b】算，姿态也只锁一个轴：
       · 插入之后目标只跟着 dz 抬了一点点，旋转段完全没有位移
       · 姿态约束锁的是工具 z，滚转自由 ⇒ 看起来"转"了，其实工具没跟着零件转
     现在每一步都问 taskPoseBase(rotDeg) 要【整体位姿】，再用六自由度 IK 解 ——
     动画演的与判据验的是同一个函数，不会再各说各话。 */
let ANIM = null;
const lerp = (a,b,t) => a+(b-a)*t;
const ease = t => t<0.5 ? 2*t*t : 1-Math.pow(-2*t+2,2)/2;
function animSteps(){
  const I = RULES.init;
  const CF = l3Conf();
  const steps = CF.steps;
  /* 装配盘（六边形）的放置位姿：全场不变 */
  const K0 = {x:I.x, y:I.y, z:I.z, th:I.th, ph:I.ph, al:I.al};
  const A0 = {ins:0, dz:0, degs: new Array(steps.length).fill(0)};
  const A  = o => Object.assign({}, A0, o,
                 o && o.degs ? { degs: o.degs.slice() } : { degs: A0.degs.slice() });
  const out = [
    {d:3200, msg:'① 就位：臂移到插入轴外侧（工具轴与插入轴同轴）',
     core:K0, a:A(), b:A(), fromHome:true},
    {d:2200, msg:'② 插入：能量单元沿插入轴推进（装配盘不动）',
     core:K0, a:A(), b:A({ins:1})},
    {d:2000, msg:'③ 上推：能量单元与手一起沿 Z 升 ' + CF.lift + 'mm（装配盘不动）',
     core:K0, a:A({ins:1}), b:A({ins:1, dz:CF.lift})}
  ];
  /* ④⑤ 两步旋转：每步一条动画段，msg 用配置里那两步的名字 */
  let base = {ins:1, dz:CF.lift, degs: new Array(steps.length).fill(0)};
  for(let s=0; s<steps.length; s++){
    const nb = {ins:1, dz:CF.lift, degs: base.degs.slice()};
    nb.degs[s] = steps[s].ang;
    /* ★ 这两步的"看不看得见动"要说清楚，否则容易被当成卡住：
         绕 P：夹爪抓的是【端面】，而端面就在 P 轴上 ⇒ 手只改姿态、不平移；
               真正在动的是被翻起来的能量单元。
         绕 Q：单元绕【插入轴】画圈（P 转完之后那根轴是竖直的）⇒ 手跟着绕。 */
    const hint = (steps[s].axis === 'p')
      ? '（手在轴上，只改姿态；动的是被翻起来的能量单元）'
      : '（单元绕这根轴画圈，手跟着绕）';
    out.push({d:2600, msg:(s===0?'④ ':'⑤ ') + (L3_STEP_NAME[s] || steps[s].label || ('第'+(s+1)+'步旋转')) + hint,
              core:K0, a:A(base), b:A(nb)});
    base = nb;
  }
  out.push({d:1400, msg:'⑥ 完成：装配盘全程不动，能量单元由手带着走完',
            hold:true, core:K0, a:A(base), b:A(base)});
  return out;
}
function startAnim(){
  if(!MODEL) return;
  const F = fkAll(Q);
  ANIM = {steps: animSteps(), i:0, t0: performance.now(),
          eeStart: F[F.length-1].t.slice(), seed: Q.slice()};
  document.getElementById('btnAnim').textContent = '■ 停止动画';
  requestAnimationFrame(animTick);
}
function stopAnim(){
  ANIM = null; CORE = null; QASM = null;
  document.getElementById('btnAnim').textContent = '▶ 示例装配动画';
  const m=document.getElementById('animMsg'); if(m) m.textContent='';
  draw3D();
}
function asmAt(st, t){
  const A = {ins:lerp(st.a.ins||0, st.b.ins||0, t), dz:lerp(st.a.dz||0, st.b.dz||0, t)};
  const na = (st.a.degs||[]).length, nb = (st.b.degs||[]).length;
  const n = Math.max(na, nb);
  A.degs = [];
  for(let i=0;i<n;i++) A.degs.push(lerp((st.a.degs||[])[i]||0, (st.b.degs||[])[i]||0, t));
  return A;
}
function animTick(now){
  if(!ANIM) return;
  let el = now - ANIM.t0;
  let st = ANIM.steps[ANIM.i];
  while(el > st.d && ANIM.i < ANIM.steps.length-1){
    el -= st.d; ANIM.i++; st = ANIM.steps[ANIM.i];
    if(st.fromHome) ANIM.eeStart = fkAll(Q)[CHAIN.length].t.slice();
  }
  // ★ 最后一步跑完就干净收尾（否则停在末态，看起来像"卡住"）
  if(ANIM.i === ANIM.steps.length-1 && el > st.d){
    const m = document.getElementById('animMsg');
    if(m) m.innerHTML = st.msg + '　—— 动画结束，点按钮可重播';
    ANIM = null;
    document.getElementById('btnAnim').textContent = '↻ 重播装配动画';
    draw3D(); drawList();
    return;
  }
  const t = st.hold ? 1 : ease(Math.min(1, el/st.d));
  CORE = st.core;                                  // 装配盘放置位姿：不动
  const A = asmAt(st, t);                          // 装配进度：插入 / 上推 / 两步旋转
  QASM = A;
  /* 目标位姿：问 taskPoseBase 要 —— 上推与两步旋转都体现在它身上 */
  const CF = l3Conf();
  const opts = { gripAxis: CF.grip };
  const off = (typeof asmOfArm === 'function') ? asmOfArm(0) : null;
  const insLen = 220, standoff = 40;      // 插入行程 220mm；夹爪停在离端面 40mm 处
  const prevLift = G_LIFT; G_LIFT = A.dz || 0;
  const G = taskPoseBase(st.core, A.degs, off, opts);
  G_LIFT = prevLift;
  /* ★ 夹爪目标点 = 手真正要到的那个点（踩过两次，务必看清）：
       G.E 是【能量单元根部】的位置，上推已经含在里面了 ⇒ 不要再加一次轴向偏移。
       绕轴转时手是【偏心抓着】的，所以还要加一个垂直件轴的径向偏移 ——
       否则绕轴自转这一段画面上不动（实测抓取点位移 0.00mm）。
       未插入时再沿件轴退开（插入行程 + 待位距离）。 */
  const gv = asmGripVec(G);                        // 垂直件轴的单位径向 × 偏心距离
  const back = (1 - (A.ins||0)) * insLen + ((A.ins||0) > 0.001 ? 0 : standoff);
  const tgtE = addv(addv(G.E, gv), mulS(G.er, back));
  const tgt = { E: tgtE, axes: G.axes };
  if(st.fromHome){                                 // ① 从"当前末端"平滑移过去
    tgt.E = [lerp(ANIM.eeStart[0],tgtE[0],t), lerp(ANIM.eeStart[1],tgtE[1],t),
             lerp(ANIM.eeStart[2],tgtE[2],t)];
  }
  /* IK：位置 + 插入轴对齐（与判据同一套：同一条工具轴、同一个位姿函数）。
     ★ 求解力度必须【与判据对齐】：判据用 12 起点 × 400 迭代（还带就近精修），
       动画原来只给 90 迭代、补一轮就放弃 —— 结果"位置到得了、姿态摆不出"，
       臂就卡在差一点点的解上不动了（用户报的"最后一段不动"就是这个）。
       现在逐级加力：近处少迭代（省时间，绝大多数帧走这条），
       不够近就依次 240 → 600 迭代并换种子。上一帧的解通常已经很近，
       所以贵的那几轮只在少数帧发生，不会把动画拖垮。 */
  /* ★ 对齐轴与目标必须【成对】：insAxisIdx() 给出"哪根工具轴沿插入方向"，
     目标就取 G.axes 里对应的那一根（由 taskPoseBase 按同一约定算出来）。
     ⚠ 早先写成 axTgt = G.axes.z 却仍用 insAxisIdx()=0 ⇒ 让工具 x 去对 z 的方向，
       姿态残差恒 90°+，臂看着卡住（位置残差却很小，特别有迷惑性）。 */
  const axIdx = insAxisIdx();
  const axKey = ['x','y','z'][axIdx];
  const axTgt = unit(G.axes[axKey]);
  const NEAR = { e: 5, ang: 0.08 };                     // "够近"（≈4.6°）
  const score = rr => Math.hypot(rr.e, W_ORI*(rr.ang||0));
  /* ① 便宜路径：拿上一帧的解当种子（绝大多数帧够用，关节角也自然连续） */
  let r = ikSolveAsmA(tgt, ANIM.seed, 160, 0, axTgt, axIdx);
  if(!(r.e < NEAR.e && r.ang < NEAR.ang)){
    /* ② 兜底：换用【判据同款】的多起点求解 —— 实测"就近扰动"逃不出局部极小
       （同一个目标：就近扰动停在 30° 轴差，判据那套 12 起点/400 迭代能到 1.63°）。 */
    const r2 = bestAsmSeedsA(tgt, axTgt, ANIM.seed, 12, 0, 400);
    if(score(r2) < score(r)) r = r2;
  }
  // ★ 不管收没收敛都用这个解，否则 IK 一失败臂就冻住
  if(r.q && r.q.length === NJ) Q = r.q;
  ANIM.seed = Q.slice();
  CURVEPTS = fkAll(Q);
  const el2 = document.getElementById('animMsg');
  if(el2){
    const qTxt = CHAIN.filter(c=>c.movable).map((c,i)=>c.joint+' '+Q[i].toFixed(1)+'°').join('　');
    /* ⚠ 只在【真的跟不上】时才报：判据用 r.ok（很严的 4.6°），
       那会把 1.4° 的正常跟踪也标成警告，反而误导（踩过）。 */
    const badTrack = (r.e > 8) || ((r.ang||0) > 0.175);      // >8mm 或 >10°
    el2.innerHTML = st.msg + (badTrack ? '　<span style="color:#ffb454">⚠ 跟不上 ' +
        r.e.toFixed(0) + ' mm / 姿态 ' + ((r.ang||0)*180/Math.PI).toFixed(1) + '°</span>' : '') +
                    '<br><span style="color:#8fa0bf">关节角：'+qTxt+'</span>';
  }
  draw3D(); drawList();
  ANIM.t0 = now - el;
  requestAnimationFrame(animTick);
}
document.getElementById('btnAnim').onclick = () => { ANIM ? stopAnim() : startAnim(); };
/* #anim：自动播放（便于无头截取动画各阶段） */
window.addEventListener('DOMContentLoaded', () => {
  if(location.hash.indexOf('anim') >= 0) setTimeout(() => { if(MODEL) startAnim(); }, 400);
});

/* ---------------- 让每次模型更新后刷新数据面板 ---------------- */
const _updateAll = updateAll;
updateAll = function(){
  _updateAll(); analyzeTask();
  const y = document.getElementById('inYaw');
  if(y && MODEL) y.value = MODEL.base.yaw;
};
/* 安装 yaw：直接改臂的安装朝向（模型链里 base 的朝向） */
document.getElementById('inYaw').onchange = e => {
  if(!MODEL) return;
  MODEL.base.yaw = +e.target.value || 0;
  updateAll(); saveLocal();
};
/* 装配臂位置：OXYZ 下，直接改就重算并记住 */
['inCx','inCy','inCz'].forEach(id => {
  document.getElementById(id).onchange = () => {
    RULES.init.x = +document.getElementById('inCx').value || 0;
    RULES.init.y = +document.getElementById('inCy').value || 0;
    RULES.init.z = +document.getElementById('inCz').value || 0;
    if(CORE){ CORE.x = RULES.init.x; CORE.y = RULES.init.y; CORE.z = RULES.init.z; }
    updateAll(); saveLocal();
  };
});
/* 映射切换：核心在"前方"还是"侧面"
   ★ 改的是配置里的 frame.taskMap，然后按同一份实现重建换算对象。
     换算公式只有 04-config.js 的 mapObj() 一处，不再各写一份。 */
document.getElementById('selMap').onchange = e => {
  if (!CONFIG || !CONFIG.rules) return;                 // 该配置没有规则，忽略
  CONFIG.rules.frame.taskMap = (e.target.value === 'swap') ? 'swap' : 'noSwap';
  rebuildTaskMap();
  updateAll(); saveLocal();
};
/* 若用户从开始窗口选了"示例"，把规则侧设置也存下来（但不覆盖已有模型） */
document.getElementById('mDemo').addEventListener('click', () => {
  setTimeout(() => { try{ localStorage.setItem('armBuilderCfg', JSON.stringify(cfgObj())); }
                     catch(e){ warnNoLS(); } }, 0);
});