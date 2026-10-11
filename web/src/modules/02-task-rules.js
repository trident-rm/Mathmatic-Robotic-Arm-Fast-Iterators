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
let CORE = null;      // ★ 六边形（核心本体）的位姿：放置好之后【固定不动】
let QASM = null;      // ★ Q 轴组件：{ins 插入量, dz 抬升, dphi 绕P轴转角, qfix 绕base.Y转角}

/* Q 轴组件的几何（全部在 base 系下）：
     - 初始方向 = 核心的 e⃗（零点位形时 = -X_b）
     - ④ 抬起：绕【base 的 +Y 轴】转 90° ⇒ Q 轴从水平转到 +Z（往上）✔
     - ⑤ 末步：绕【base 的 X 轴】转 90°（不是再绕 Y 转 90°，那会变成绕 Y 180°）
     - Q 轴组件原点固定在核心前表面中心 E：
       插入是"能量单元沿 Q 轴滑进去"，不是核心在动 */
function qasmGeom(P, A){
  const F0 = coreFrame(P);
  let e = dirToBase(F0.e), u = dirToBase(F0.u), v = dirToBase(F0.v);
  const e0b = e.slice();                                   // 初始 Q 方向（插入沿它）
  let R = rotAxis([0,1,0], (A.lift||0)*D2R);               // ④ 绕 base +Y 抬起
  e = mulv(R,e); u = mulv(R,u); v = mulv(R,v);
  R = rotAxis([1,0,0], (A.qfix||0)*D2R);                   // ⑤ 绕 base X 旋转
  e = mulv(R,e); u = mulv(R,u); v = mulv(R,v);
  const O = taskToBase([P.x,P.y,P.z]);                     // 固定不动
  return {O, e, u, v, e0b};
}

function drawCoreOverlay(g,S,label){
  if(!CORE) return;
  const P=CORE, F0=coreFrame(P), Eb=taskToBase([P.x,P.y,P.z]);
  const Fu=dirToBase(F0.u), Fv=dirToBase(F0.v), Fe=dirToBase(F0.e);
  // ---- 六边形：固定不动 ----
  const R=150, hex=[];
  for(let k=0;k<6;k++){ const a=k*Math.PI/3;
    hex.push(addv(addv(Eb, mulS(Fu,R*Math.cos(a))), mulS(Fv,R*Math.sin(a)))); }
  g.strokeStyle='#8fd0ff'; g.lineWidth=2; g.beginPath();
  hex.forEach((p,i)=>{const s=S(p); i?g.lineTo(s[0],s[1]):g.moveTo(s[0],s[1]);});
  g.closePath(); g.stroke();
  g.globalAlpha=.12; g.fillStyle='#8fd0ff'; g.fill(); g.globalAlpha=1;
  label('核心本体(固定)', S(hex[4])[0]-10, S(hex[4])[1]+16, '#8fd0ff',
        "bold 11px system-ui,'Microsoft YaHei'");
  // 前表面法向 e⃗（虚线参考）
  const e1=S(addv(Eb, mulS(Fe,190)));
  g.strokeStyle='rgba(143,208,255,.55)'; g.lineWidth=1.4; g.setLineDash([6,4]);
  g.beginPath(); g.moveTo(S(Eb)[0],S(Eb)[1]); g.lineTo(e1[0],e1[1]); g.stroke(); g.setLineDash([]);
  // P 轴（六边形顶部；Q 轴组件绕它抬起）
  const top=addv(Eb, mulS(Fv,R)), p1=S(addv(top, mulS(Fu,80))), p2=S(addv(top, mulS(Fu,-80)));
  g.strokeStyle='#e879f9'; g.lineWidth=3;
  g.beginPath(); g.moveTo(p1[0],p1[1]); g.lineTo(p2[0],p2[1]); g.stroke();
  label('P轴', p2[0]-30, p2[1]+14, '#e879f9', "bold 11px system-ui,'Microsoft YaHei'");
  // ---- Q 轴组件：会动 ----
  const A = QASM || {ins:0,dz:0,dphi:0,qfix:0};
  const Q = qasmGeom(P, A);
  const q0=S(Q.O), q1=S(addv(Q.O, mulS(Q.e,110)));
  g.strokeStyle='#ffb454'; g.lineWidth=3;
  g.beginPath(); g.moveTo(q0[0],q0[1]); g.lineTo(q1[0],q1[1]); g.stroke();
  label('Q轴', q1[0]+5, q1[1]-5, '#ffb454', "bold 11px system-ui,'Microsoft YaHei'");
  label('E', q0[0]+6, q0[1]+15, '#8fd0ff', "bold 12px system-ui,'Microsoft YaHei'");
  // Q 轴方头（跟着 Q 轴一起走；qfix 时绕 base.Y 公转，所以整根轴会摆过去）
  const base=addv(Q.O, mulS(Q.e,88));
  const g1=S(addv(base, mulS(Q.u,24))), g2=S(addv(base, mulS(Q.u,-24)));
  g.strokeStyle='#ffd9a0'; g.lineWidth=4;
  g.beginPath(); g.moveTo(g1[0],g1[1]); g.lineTo(g2[0],g2[1]); g.stroke();
  if(A.qfix) label('绕base.Y '+A.qfix.toFixed(0)+'°', q1[0]+6, q1[1]+16, '#ffd9a0',
                   "bold 11px system-ui,'Microsoft YaHei'");
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
   序列（按你的口述）：① 臂移动到 E（真动画：目标从当前末端平滑移过去）
                     ② 沿 Q 轴方向插入（EE 沿 e⃗ 逼近 E）
                     ③ 放入后【沿 z 平移 100mm】（核心与臂一起上升）
                     ④ 以【P 轴 = 核心 Y′】为旋转轴抬起（φ 90°→0°）
                     ⑤ 绕【零点位形的 Q 轴】转 90°                            */
let ANIM = null;
const lerp = (a,b,t) => a+(b-a)*t;
const ease = t => t<0.5 ? 2*t*t : 1-Math.pow(-2*t+2,2)/2;
function animSteps(){
  const I = RULES.init;
  const K0 = {x:I.x, y:I.y, z:I.z, th:I.th, ph:I.ph, al:I.al};  // ★ 核心本体放置位姿：全场不动
  const A0 = {ins:0, dz:0, lift:0, qfix:0};
  const A  = o => Object.assign({}, A0, o);
  return [
    {d:3200, msg:'① 就位：臂移到 Q 轴前端外侧（夹爪 y 对准 Q 轴）',
     core:K0, a:A(), b:A(), fromHome:true},
    {d:2200, msg:'② 插入：从 Q 轴【前端】沿 Q 轴推进到末端（核心端面）',
     core:K0, a:A(), b:A({ins:1})},
    {d:2000, msg:'③ 放入后沿 z 平移 100mm',
     core:K0, a:A({ins:1}), b:A({ins:1, dz:100})},
    {d:2600, msg:'④ 抬起：绕【base 的 Y 轴】往上 90°（Q 轴转到 +Z）',
     core:K0, a:A({ins:1, dz:100}), b:A({ins:1, dz:100, lift:90})},
    {d:2600, msg:'⑤ 末步：绕【base 的 X 轴】旋转 90°',
     core:K0, a:A({ins:1, dz:100, lift:90}), b:A({ins:1, dz:100, lift:90, qfix:90})},
    {d:1200, msg:'⑥ 完成（六边形全程不动）', hold:true,
     core:K0, a:A({ins:1, dz:100, lift:90, qfix:90}), b:A({ins:1, dz:100, lift:90, qfix:90})}
  ];
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
  const A = {};
  for(const k of ['ins','dz','lift','qfix']) A[k] = lerp(st.a[k]||0, st.b[k]||0, t);
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
  CORE = st.core;                                  // 核心本体：不动
  QASM = asmAt(st, t);                             // Q 轴组件：动
  const G = qasmGeom(CORE, QASM);
  // 末端目标：② 从 Q 轴【前端外侧】沿 Q 轴进到端面（t=0 在外侧 ins 处 → t=1 到位）；
  //           ③ 之后跟着 z 平移一起升起
  const insLen = 220, standoff = 40;   // 插入行程 220mm；夹爪停在离 Q 末端 40mm 处
  const seat = addv(taskToBase([CORE.x,CORE.y,CORE.z]), [0,0,QASM.dz||0]);
  let tgt = addv(seat, mulS(G.e0b, standoff + (1 - (QASM.ins||0)) * insLen));
  if(st.fromHome){                                 // ① 从"当前末端"平滑移过去
    tgt = [lerp(ANIM.eeStart[0],tgt[0],t), lerp(ANIM.eeStart[1],tgt[1],t),
           lerp(ANIM.eeStart[2],tgt[2],t)];
  }
  const dir = mulS(G.e0b,-1);
  let r = ikSolve6(tgt, dir, ANIM.seed, 320, 1);            // 夹爪 y ∥ Q 轴
  for(let tries=0; !r.ok && r.e > 8 && tries < 2; tries++){  // 卡住就换种子再试两次
    const lim = qLimits();
    const seed2 = lim.map(([lo,hi]) => lo + Math.random()*(hi-lo));
    const r2 = ikSolve6(tgt, dir, seed2, 320, 1);
    if(r2.e < r.e) r = r2;
  }
  // ★ 不管收没收敛都用这个解，否则 IK 一失败臂就冻住（"动两下卡住"的根因）
  if(r.q && r.q.length === NJ) Q = r.q;
  ANIM.seed = Q.slice();
  CURVEPTS = fkAll(Q);
  const el2 = document.getElementById('animMsg');
  if(el2){
    const qTxt = CHAIN.filter(c=>c.movable).map((c,i)=>c.joint+' '+Q[i].toFixed(1)+'°').join('　');
    el2.innerHTML = st.msg + (r.ok?'':'　<span style="color:#ffb454">⚠ IK 差 '+r.e.toFixed(0)+' mm</span>') +
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