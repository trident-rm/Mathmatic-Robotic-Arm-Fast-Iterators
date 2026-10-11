/* ==================================================================
   ⛓ 连杆调参 · 任务适应性评估   （armLink.html 追加模块 · 纯追加）

   它回答的问题：**在关节限位一动不动的前提下，连杆（各段 offset）怎么改，
   三级任务域才更好做？**

   为什么不能只看上面那行「IK 复核 · 可解」：
     它判 ok 的容差是 位置<3mm 且 姿态<0.2rad(11.5°)，而且限位顶死也算 ok
     —— 实测基准臂在三级任务域上「可解 100%」，但解里 joint1/joint4 的
        限位裕度中位数是 0.0°（一半位姿顶死），等于"靠顶死凑出来的可解"。
     所以本模块换一套**严格判据**：
       位置 < eTol(1.5mm)  且  姿态 < aTol(3°)  且  最紧关节留 > mGate(5°) 裕度
     并且把"最紧关节裕度"本身当成要优化的量（满分 mCap=15°）。

   指标 → 单一分数（0~100）：
     score = 100 × mean_over_poses( 找到严格解 ? clamp(裕度/mCap, 0, 1) : 0 )

   硬约束（写死，不给改）：
     ① 关节限位**只读**：模块从头到尾不写 MODEL.links[i].limits，
        每次评估前后都比对限位签名，变了就在界面上报警（自检里也有断言）
     ② 杆长之和 ≤ lenCap（默认 = 基准臂的杆长之和，即"不许把臂变长"）
     ③ 各旋钮在 [lo, hi] 内（界面可调）

   —— 与上面原脚本的关系：只"读"它（MODEL / CHAIN / qLimits / fkAll / ikSolve6 /
      RULES / taskToBase / coreFrame / updateAll / setModel / saveLocal …），
      唯一"写"的地方是把改编后的 offset 写回 MODEL 后调 updateAll()（= 应用候选），
      以及把 analyzeTask 包一层、在数据面板末尾追加一段本模块的摘要。
   ================================================================== */
(function(){
"use strict";

/* ---------------- 数值小工具（lk 前缀，避免与上面重名） ---------------- */
const lkSub = (a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const lkCrs = (a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const lkDot = (a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const lkLen = a=>Math.hypot(a[0],a[1],a[2]);
const lkScl = (a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const lkNrm = a=>{ const n=lkLen(a)||1; return [a[0]/n,a[1]/n,a[2]/n]; };
const lkCl  = (v,a,b)=>Math.min(Math.max(v,a),b);
const lkMed = a=>{ if(!a.length) return 0; const s=a.slice().sort((x,y)=>x-y);
                   return s[Math.floor(s.length/2)]; };
const lkPct = (a,b)=> b ? 100*a/b : 0;
const lkAvg = a=> a.length ? a.reduce((x,y)=>x+y,0)/a.length : 0;
const lkEsc = s=>String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;')
                         .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
function lkRng(a){ return function(){ a|=0; a=a+0x6D2B79F5|0;
  let t=Math.imul(a^a>>>15,1|a); t=t+Math.imul(t^t>>>7,61|t)^t;
  return ((t^t>>>14)>>>0)/4294967296; }; }
/* Halton 低差异序列：位姿集是"分层"的，换位姿数时是**前缀**，指标才稳定 */
function lkHalton(i,b){ let f=1,r=0; while(i>0){ f/=b; r+=f*(i%b); i=Math.floor(i/b); } return r; }

const $ = id => document.getElementById(id);

/* ---------------- 状态 ---------------- */
const DEF = { nPose:12, budget:4, iters:220, eTol:3, aTol:3, mGate:5, mCap:15,
              seed:20260925, K:7, restarts:6, lenCap:0, wsN:1200 };
const S = { open:false, knobs:[], modelVals:[], base:null, baseVals:null,
            res:[], job:null, timer:0, stop:false, cfg:Object.assign({},DEF),
            poses:[], pool:[], applied:-1, limSig:'', armLen0:0, t0:0, builtKey:'' };

/* ---------------- 限位只读：签名比对 ---------------- */
function lkLimits(){
  return CHAIN.filter(c=>c.movable).map(c=>({joint:c.joint, name:c.name,
    limits: c.limits ? c.limits.slice() : null }));
}
function lkLimSig(){
  return lkLimits().map(o=>o.joint+'['+(o.limits?o.limits[0]+','+o.limits[1]:'不限')+']').join(' ');
}
function lkSegLens(){
  return CHAIN.map(c=>Math.hypot(c.offset[0],c.offset[1],c.offset[2]));
}
function lkArmLen(){ return lkSegLens().reduce((a,b)=>a+b,0); }

/* ---------------- 旋钮：由当前 MODEL 自动生成 ---------------- */
const AXC = ['x','y','z'];
function lkKnobsPreserve(){
  const old = {}; S.knobs.forEach(k=>{ old[k.li+'.'+k.c] = k; });
  const out = [];
  MODEL.links.forEach((L,i)=>{
    if(i === 0) return;                       // 第 1 段是臂安装位（整机原点），不是连杆
    if(L.posMode === 'origin') return;        // 绝对定位的段会牵动整臂，不参与
    for(let c=0;c<3;c++){
      const v0 = +L.pos[c] || 0, key = i+'.'+c, o = old[key];
      const r = Math.max(60, Math.abs(v0)*0.5);
      out.push({ li:i, c, key, name:L.name+'.'+AXC[c], v0,
                 lo: o ? o.lo : Math.round(v0-r), hi: o ? o.hi : Math.round(v0+r),
                 on: o ? o.on : Math.abs(v0) > 1e-9 });
    }
  });
  return out;
}
function lkReadModel(){ return S.knobs.map(k => +MODEL.links[k.li].pos[k.c] || 0); }
function lkWriteVals(vals){
  S.knobs.forEach((k,i)=>{ if(vals[i] === undefined) return;
    MODEL.links[k.li].pos[k.c] = +vals[i]; });
  buildChain();
}
const lkIdx = k => S.knobs.indexOf(k);

/* ---------------- 位姿集（三级任务域，分层采样，随种子确定） ---------------- */
function lkBuildPoses(cfg){
  const c = cfg || S.cfg;
  const L = RULES.levels[3], n = c.nPose;
  const rnd = lkRng((c.seed|0) ^ 0x9E3779B9);
  const off = [0,1,2,3,4,5].map(()=>rnd());
  const BS  = [2,3,5,7,11,13];
  S.poses = [];
  for(let i=1;i<=n;i++){
    const u = BS.map((b,k)=> (lkHalton(i,b)+off[k]) % 1);
    const P = { x:L.x[0]+u[0]*(L.x[1]-L.x[0]), y:L.y[0]+u[1]*(L.y[1]-L.y[0]),
                z:L.z[0]+u[2]*(L.z[1]-L.z[0]), th:L.th[0]+u[3]*(L.th[1]-L.th[0]),
                ph:L.ph[0]+u[4]*(L.ph[1]-L.ph[0]), al:L.al[0]+u[5]*(L.al[1]-L.al[0]) };
    const F0 = coreFrame(P);
    const eB = dirToBase(F0.e), uB = dirToBase(F0.u);
    const ey = lkScl(lkNrm(eB), -1);                 // 插入姿态：夹爪自身 y ∥ −e⃗（与上面面板同一约定）
    const x0 = lkNrm(lkSub(uB, lkScl(ey, lkDot(uB,ey))));
    S.poses.push({ P, Eb: taskToBase([P.x,P.y,P.z]), eB, uB, ey, x0, z0: lkCrs(x0,ey), z:P.z });
  }
  /* 起点池：每个位姿固定一串（与预算无关 ⇒ 任何候选都用同一批随机起点，公平） */
  const r2 = lkRng((c.seed|0) ^ 0x51ED2701), lim = qLimits();
  const NP = Math.max(12, c.budget + 4);
  S.pool = S.poses.map(()=>{
    const out = [new Array(NJ).fill(0)];             // 0 号 = 全零位（和界面显示位形一致）
    for(let k=1;k<NP;k++) out.push(lim.map(([lo,hi]) => lo + r2()*(hi-lo)));
    return out;
  });
}

/* 位姿集/起点池只在（位姿数·种子·预算·关节数）变了时重建；评估前统一 ensure 一次。
   ★ 用传进来的 cfg（而不是 S.cfg）：否则"[复核档位] 用 64 位姿"会被悄悄按当前位姿数算 */
function lkEnsurePoses(cfg){
  const c = cfg || S.cfg;
  const k = c.nPose+'|'+c.seed+'|'+c.budget+'|'+NJ;
  if(k !== S.builtKey){ lkBuildPoses(c); S.builtKey = k; }
}

/* ---------------- 单个位姿求解 + 严格判据 ---------------- */
function lkMinMargin(q, lim){
  let m = Infinity, who = -1;
  for(let i=0;i<q.length;i++){
    const d = Math.min(q[i]-lim[i][0], lim[i][1]-q[i]);
    if(d < m){ m = d; who = i; }
  }
  return {m, who};
}
function lkSolvePose(pi, cfg){
  const P = S.poses[pi], lim = qLimits();
  let found = false, bestM = -1e9, angGood = 0, who = -1, angBest = 1e9, eBest = 1e9;
  for(let s=0;s<cfg.budget;s++){
    const r = ikSolve6(P.Eb, P.ey, S.pool[pi][Math.min(s, S.pool[pi].length-1)], cfg.iters, 1);
    const ang = (r.ang || 0)*180/Math.PI;
    if(ang < angBest){ angBest = ang; }
    if(r.e < eBest) eBest = r.e;
    if(r.e < cfg.eTol && ang < cfg.aTol){
      const M = lkMinMargin(r.q, lim);
      if(M.m > bestM){ bestM = M.m; angGood = ang; who = M.who; found = true; }
      if(bestM >= cfg.mCap) break;                   // 满裕度了，不必再试
    }
  }
  return { i:pi, z:P.z, found, marg: found ? bestM : 0, ang: found ? angGood : angBest,
           angBest, eBest, who };
}

/* ---------------- 汇总成一条候选结果 ---------------- */
function lkAgg(per, cfg, tag, vals){
  const N = per.length;
  const foundA = per.filter(p=>p.found);
  const tight = new Array(NJ).fill(0);
  per.forEach(p=>{ if(p.found && p.who >= 0 && p.marg < cfg.mGate) tight[p.who]++; });
  const score = 100 * lkAvg(per.map(p => p.found ? Math.min(p.marg/cfg.mCap, 1) : 0));
  return { tag, vals: vals.slice(), n:N,
           found: foundA.length/N,
           pass : per.filter(p=>p.found && p.marg > cfg.mGate).length/N,
           score,
           margMed: lkMed(foundA.map(p=>p.marg)),
           margP25: (function(a){ if(!a.length) return 0; const s=a.slice().sort((x,y)=>x-y);
                                 return s[Math.floor(s.length*0.25)]; })(foundA.map(p=>p.marg)),
           angMed : lkMed(per.map(p=>p.ang)),
           eMed   : lkMed(per.map(p=>p.eBest)),
           per, tight,
           armLen : lkArmLen(), segs: lkSegLens(), limitOK: lkLimSig() === S.limSig };
}
/* 便宜的可达域核对（不碰全局 CLOUD/VOX；只用来确认"没把可达域搞坏"） */
function lkWorkspace(n){
  const lim = qLimits(), q = new Array(NJ), keys = new Set();
  const rng = lkRng(0x5EED01);
  let reach = 0, zmin = Infinity, zmax = -Infinity, rhomax = 0;
  for(let s=0;s<n;s++){
    for(let i=0;i<NJ;i++){ const L = lim[i]; q[i] = L[0] + rng()*(L[1]-L[0]); }
    const F = fkAll(q), p = F[F.length-1].t;
    keys.add(Math.floor(p[0]/50)+','+Math.floor(p[1]/50)+','+Math.floor(p[2]/50));
    const d = Math.hypot(p[0],p[1],p[2]); if(d > reach) reach = d;
    if(p[2] < zmin) zmin = p[2]; if(p[2] > zmax) zmax = p[2];
    const rh = Math.hypot(p[0],p[1]); if(rh > rhomax) rhomax = rh;
  }
  /* 三级任务域位置可达率（±1 体素，和上面面板同一判据） */
  let hit = 0; const M = 240, rp = lkRng(0xABCD01);
  for(let s=0;s<M;s++){
    const p = taskToBase([ rp()* (RULES.levels[3].x[1]-RULES.levels[3].x[0]) + RULES.levels[3].x[0],
                           rp()* (RULES.levels[3].y[1]-RULES.levels[3].y[0]) + RULES.levels[3].y[0],
                           rp()* (RULES.levels[3].z[1]-RULES.levels[3].z[0]) + RULES.levels[3].z[0] ]);
    const i0 = Math.floor(p[0]/50), j0 = Math.floor(p[1]/50), k0 = Math.floor(p[2]/50);
    let f = false;
    for(let a=-1;a<=1 && !f;a++) for(let b=-1;b<=1 && !f;b++) for(let c=-1;c<=1 && !f;c++)
      if(keys.has((i0+a)+','+(j0+b)+','+(k0+c))) f = true;
    if(f) hit++;
  }
  return { vox:keys.size, reach, zmin, zmax, rhomax, posRate: hit/M, wsN:n };
}

/* ---------------- 评估一个候选连杆 ---------------- */
function lkEval(vals, tag, cfg, opt){
  cfg = cfg || S.cfg; opt = opt || {};
  lkEnsurePoses(cfg);
  const keep = lkReadModel();                       // 记住"界面上当前这一套"
  let res = null;
  try{
    lkWriteVals(vals);
    const per = [];
    for(let i=0;i<S.poses.length;i++) per.push(lkSolvePose(i, cfg));
    res = lkAgg(per, cfg, tag, vals);
    if(opt.ws) Object.assign(res, lkWorkspace(cfg.wsN || DEF.wsN));
    res.ms = opt.t0 ? (performance.now() - opt.t0) : NaN;
  } finally {
    lkWriteVals(keep);                              // 无论成败都把模型还原
  }
  return res;
}

/* ---------------- 生成器：扫描 / 搜索 ---------------- */
function lkMakeSweeper(baseVals, cfg){
  const D = S.knobs.filter(k=>k.on);
  if(!D.length) return null;
  const list = [];
  for(const k of D){
    for(let t=0;t<cfg.K;t++){
      const x = cfg.K > 1 ? k.lo + (k.hi-k.lo)*t/(cfg.K-1) : k.v0;
      const v = baseVals.slice(); v[lkIdx(k)] = x;
      list.push({vals:v, tag:k.name+' = '+x.toFixed(0), knob:k, val:x});
    }
  }
  let i = 0;
  return { total:list.length, next:()=> i<list.length ? list[i++] : null, push(){}, kind:'sweep' };
}
/* 随机重启 + 坐标下降（pattern search）；约束（杆长之和上限 / 限位没被改）在这里挡掉 */
function lkMakeSearcher(baseVals, cfg){
  const D = S.knobs.filter(k=>k.on);
  if(!D.length) return null;
  const STEPS = [0.26, 0.11, 0.045];
  const rnd = lkRng((cfg.seed|0) ^ 0x1BADC0DE);
  const st = { mode:'start', left:cfg.restarts, cur:null, best:null, stepI:0, pi:0,
               improved:false, aw:false, anchor:null };
  let probes = [];
  const valid = res => res.limitOK !== false && res.armLen <= cfg.lenCap + 1e-6;
  const mkProbes = ()=> { const L=[]; for(const k of D) for(const s of [+1,-1]) L.push({k,s}); return L; };
  function next(){
    if(st.mode === 'start'){
      if(st.left <= 0) return null;
      st.left--;
      const v = baseVals.slice();
      for(const k of D) v[lkIdx(k)] = k.lo + rnd()*(k.hi-k.lo);
      st.aw = true;
      return { vals:v, tag:'随机起点' };
    }
    if(st.pi >= probes.length){
      if(st.improved){ st.improved = false; st.pi = 0; }        // 本轮有改进 → 同步长再来一轮
      else{
        st.stepI++; st.pi = 0;
        if(st.stepI >= STEPS.length){ st.mode = 'start'; st.cur = null; return next(); }
      }
    }
    const p = probes[st.pi], v = st.anchor.slice();
    v[lkIdx(p.k)] = lkCl(v[lkIdx(p.k)] + p.s*STEPS[st.stepI]*(p.k.hi-p.k.lo), p.k.lo, p.k.hi);
    return { vals:v, tag:'下降 '+p.k.name+(p.s>0?'+':'−') };
  }
  function push(res){
    if(st.aw){                                                  // 刚评完一个随机起点
      st.aw = false;
      if(valid(res)){
        if(!st.cur || res.score > st.cur.score) st.cur = {vals:res.vals, score:res.score};
        if(!st.best || res.score > st.best.score) st.best = {vals:res.vals, score:res.score, res};
        st.anchor = st.cur.vals.slice();
        st.mode = 'descend'; st.stepI = 0; st.pi = 0; st.improved = false; probes = mkProbes();
      } else st.mode = 'start';                                 // 起点不合法（太长）→ 换下一个
      return;
    }
    if(valid(res) && st.cur && res.score > st.cur.score + 1e-9){
      st.cur = {vals:res.vals, score:res.score};
      st.anchor = st.cur.vals.slice(); st.improved = true; st.pi = 0;
      if(!st.best || res.score > st.best.score) st.best = {vals:res.vals, score:res.score, res};
    } else st.pi++;
  }
  return { total: cfg.restarts + cfg.restarts*STEPS.length*D.length*2, next, push, kind:'search',
           bestOf: ()=> st.best };
}
function lkMakeSingle(vals, tag){
  let done = false;
  return { total:1, push(){}, kind:'single', next:()=> done ? null : (done = true, {vals:vals.slice(), tag}) };
}

/* ---------------- 任务调度：每 ~55ms 让一次，界面不卡 ---------------- */
function lkTick(){
  const J = S.job; if(!J){ return; }
  const t0 = performance.now();
  try{
    while(!S.stop && performance.now() - t0 < 55){
      const cand = J.gen.next();
      if(!cand){ return lkEndJob(false); }
      const t1 = performance.now();
      const res = lkEval(cand.vals, cand.tag, S.cfg, {t0:t1});
      if(cand.knob) res.point = {key:cand.knob.key, name:cand.knob.name, val:cand.val};
      if(cand.knob && J.points) J.points.push({key:cand.knob.key, name:cand.knob.name,
          lo:cand.knob.lo, hi:cand.knob.hi, val:cand.val, score:res.score,
          pass:res.pass, marg:res.margMed, ang:res.angMed, armLen:res.armLen});
      J.gen.push(res);
      J.n++;
      J.last = res;
      if(!res.limitOK){ J.limitOK = false; lkLimitAlarm(); }
      J.best = Math.max(J.best === undefined ? -1 : J.best, res.score);
      /* 只有"刷新了最好成绩"的结果（以及单点评估）才进结果表，免得几百行刷屏 */
      if(J.kind === 'single' || J.kind === 'scan' || J.kind === 'base' ||
         res.score >= J.best - 1e-9) lkAddRes(res, J, J.kind === 'sweep' ? res.tag : undefined);
      lkRenderProg();
    }
  }catch(e){
    lkEndJob(true); alert('连杆调参出错：' + (e && e.message));
    console.error(e); return;
  }
  if(!S.stop) S.timer = setTimeout(lkTick, 0); else lkEndJob(false);
}
function lkEndJob(err){
  clearTimeout(S.timer); S.timer = 0;
  const J = S.job; S.job = null; S.stop = false;
  lkWriteVals(S.modelVals);                       // 无条件还原界面上的模型
  if(J && J.gen.bestOf){
    const b = J.gen.bestOf();
    if(b && b.res) lkAddRes(b.res, null, '搜索最优');
  }
  if(J && J.points && J.points.length){ S.sweepPts = J.points; lkRenderSweep(J.points); }
  lkRenderProg(); lkRenderRes(); lkRenderBody(); lkDrawCmp();
  if(!err && J) lkNote(`本轮「${{sweep:'灵敏度扫描',search:'全局搜索',scan:'单参数扫描',base:'基准评估',verify:'复核'}[J.kind]||J.kind}」完成：` +
    `评估 ${J.n} 个候选，用时 ${((Date.now()-J.t0)/1000).toFixed(1)} s` +
    (J.limitOK === false ? '　⚠ 限位被改过！' : '　（限位全程未改动 ✓）'));
}
function lkStartJob(kind){
  if(!MODEL){ alert('先载入模型'); return; }
  if(S.job){ alert('正在跑，先点「⏹ 停止」'); return; }
  S.modelVals = lkReadModel();
  lkEnsurePoses();
  let gen = null;
  if(kind === 'sweep')  gen = lkMakeSweeper(S.modelVals, S.cfg);
  if(kind === 'search') gen = lkMakeSearcher(S.modelVals, S.cfg);
  if(kind === 'single') gen = lkMakeSingle(S.modelVals, '当前模型');
  if(!gen){ alert('没有勾选任何旋钮（第 2 段里勾上要调的杆）'); return; }
  if(!S.limSig) S.limSig = lkLimSig();
  S.job = { kind, gen, n:0, t0:Date.now(), total:gen.total, note:'',
            points: [], best:-1, limOK:true };
  S.stop = false;
  lkRenderProg(); lkRenderBody();
  lkTick();
}
function lkStopJob(){ if(S.job){ S.stop = true; } }

/* ---------------- 结果表 ---------------- */
function lkAddRes(res, J, tag){
  res.tag = tag || res.tag;
  S.res.push(res);
  if(S.res.length > 60) S.res.splice(0, S.res.length - 60);
  if(S.job) S.job.last = res;
  lkRenderRes();
  if(res.limitOK === false) lkLimitAlarm();
}
function lkLimitAlarm(){
  const f = $('lform'); if(!f || f.querySelector('#limBad')) return;
  const d = document.createElement('div');
  d.id = 'limBad'; d.className = 'warn';
  d.style.cssText = 'margin:6px 0;padding:4px 6px;border:1px solid #7a5a20;border-radius:6px';
  d.textContent = '⚠ 关节限位被改动了！本模块不允许改限位 —— 请点「↩ 恢复原始臂」。';
  f.insertBefore(d, f.firstChild);
}
function lkApplyRow(i){
  const r = S.res[i]; if(!r) return;
  const keep = lkReadModel();
  S.applied = i;
  lkWriteVals(r.vals);
  updateAll(); saveLocal();
  lkRenderRes(); lkRenderBody(); lkDrawCmp();
  lkNote(`已把「${r.tag}」应用到模型（限位未改动：${lkLimSig()}）`);
}
function lkRevert(){
  if(!S.base) return;
  setModel(JSON.parse(JSON.stringify(S.base)), true);
  S.knobs = lkKnobsPreserve();
  S.modelVals = lkReadModel();
  S.applied = -1;
  lkRenderBody(); lkRenderRes(); lkDrawCmp();
  lkNote('已恢复原始臂（连限位一起，逐字节回到快照）');
}

/* ---------------- 小结画布：基准 vs 当前（base 系 YZ 平面） ---------------- */
function lkDrawCmp(){
  const cv = $('lcmp'); if(!cv || !MODEL) return;
  const dpr = window.devicePixelRatio || 1;
  const r = cv.getBoundingClientRect(); const w = r.width||320, h = r.height||158;
  cv.width = Math.round(w*dpr); cv.height = Math.round(h*dpr);
  const g = cv.getContext('2d'); g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);
  const cur = fkAll(Q);
  let oldF = null;
  if(S.base){
    const keep = MODEL; MODEL = JSON.parse(JSON.stringify(S.base)); buildChain();
    oldF = fkAll(Q);
    MODEL = keep; buildChain();
  }
  const L = RULES.levels[3];
  const pz = [ (L.z[0]-RULES.baseH), (L.z[1]-RULES.baseH) ];       // 任务域 z（base 系）
  const py = MAP.swap ? [-L.y[1], -L.y[0]] : [L.y[0], L.y[1]];
  const ymin = Math.min(py[0]-60, -140), ymax = Math.max(py[1]+60, 540);
  const zmin = Math.min(pz[0]-80, -140), zmax = Math.max(pz[1]+80, 560);
  const sc = Math.min(w/(ymax-ymin), h/(zmax-zmin))*0.92;
  const X = y => w/2 + (y-(ymin+ymax)/2)*sc;
  const Y = z => h/2 - (z-(zmin+zmax)/2)*sc;
  /* 任务域（base 系 y-z 投影） */
  g.fillStyle = 'rgba(62,166,255,.10)'; g.strokeStyle = 'rgba(62,166,255,.55)';
  g.lineWidth = 1; g.setLineDash([3,3]);
  g.fillRect(X(py[0]), Y(pz[1]), (py[1]-py[0])*sc, (pz[1]-pz[0])*sc);
  g.strokeRect(X(py[0]), Y(pz[1]), (py[1]-py[0])*sc, (pz[1]-pz[0])*sc);
  g.setLineDash([]);
  const skel = (F, color, lw, dash)=>{ g.strokeStyle = color; g.lineWidth = lw;
    g.setLineDash(dash||[]); g.beginPath();
    g.moveTo(X(0), Y(0));
    F.forEach((f,i)=>{ if(i===0) return; g.lineTo(X(f.t[1]), Y(f.t[2])); });
    g.stroke(); g.setLineDash([]); };
  if(oldF) skel(oldF, 'rgba(150,160,180,.75)', 1.5, [4,3]);
  skel(cur, '#e8933a', 2);
  g.fillStyle = '#c9d6ee'; g.font = "10px system-ui,'Microsoft YaHei'";
  g.fillText('基准（虚线） vs 当前（橙）· base 系 YZ 投影', 6, 12);
  g.fillStyle = 'rgba(62,166,255,.85)'; g.fillText('蓝框 = 三级任务域 (y,z)', 6, h-6);
}

/* ---------------- 和「当前配置里的原臂」的差异（防止把候选当成原臂） ---------------- */
/* ★ 权威定义是【当前配置】里的模型（configModel），不是 demoModel()。
   配置层改造后 demoModel() = 当前模型（点"示例"时用来把模型重置回配置），
   拿它当基准会让差异永远是空的 —— 那就正好丢掉了这个防呆功能。
   注意 configModel(id) 返回的是深拷贝，不是活对象。 */
function lkDemoDiff(){
  let d = null;
  try{
    d = (typeof configModel === 'function')
        ? configModel(typeof activeConfigId === 'function' ? activeConfigId() : null)
        : demoModel();
  }catch(e){ return null; }
  if(!MODEL || !d) return null;
  const out = [];
  const b = MODEL.base || {}, db = d.base || {};
  if(+b.yaw !== +db.yaw) out.push('安装yaw ' + db.yaw + '→' + b.yaw);
  (db.xyz || []).forEach((v,i)=>{ const w = (b.xyz||[])[i];
    if(Math.abs((+w||0) - (+v||0)) > 1e-9) out.push('base.' + AXC[i] + ' ' + v + '→' + (+w)); });
  if(MODEL.links.length !== d.links.length) out.push('链段数 ' + d.links.length + '→' + MODEL.links.length);
  const n = Math.min(MODEL.links.length, d.links.length);
  for(let i=0;i<n;i++){
    const a = MODEL.links[i], q = d.links[i];
    if(a.name !== q.name) out.push('第' + (i+1) + '段名字 ' + q.name + '→' + a.name);
    for(let c=0;c<3;c++){ const x = +a.pos[c]||0, y = +q.pos[c]||0;
      if(Math.abs(x - y) > 1e-9) out.push(q.name + '.' + AXC[c] + ' ' + y + '→' + x); }
    const la = a.limits ? a.limits.join(',') : '不限', lb = q.limits ? q.limits.join(',') : '不限';
    if(la !== lb) out.push(q.name + ' 限位 ' + lb + '→' + la);
    if(a.axis !== q.axis || a.type !== q.type) out.push(q.name + ' 轴/类型 变了');
    if(JSON.stringify(a.axes) !== JSON.stringify(q.axes)) out.push(q.name + ' 坐标系朝向 变了');
  }
  return out;
}
function lkDemoLine(max){
  const dif = lkDemoDiff();
  if(!dif) return '';
  /* 基准的名字跟着配置走 —— 换构型后不该还写"同济示例模型" */
  let base = '配置原臂';
  try{
    if(typeof CONFIG !== 'undefined' && CONFIG && CONFIG.label) base = CONFIG.label;
  }catch(e){}
  if(!dif.length) return '<span class="good">✓ 当前模型 = ' + lkEsc(base) + '（原版，一字未改）</span>';
  const m = max === undefined ? 5 : max;
  return '<span class="warn">⚠ 当前模型 ≠ ' + lkEsc(base) + '（' + dif.length + ' 处不同）</span>：' +
    dif.slice(0,m).map(lkEsc).join(' · ') + (dif.length > m ? ' …' : '') +
    '<br>想回原版：顶栏 <b>示例</b>（重新载入原版并覆盖存档），或本面板 <b>↩ 恢复原始臂</b>' +
    '（恢复的是<b>打开本面板时</b>那一份 —— 若那时模型已经被改过，恢复出来的还是改过的）';
}

/* ---------------- 面板 ---------------- */
function lkNote(t){ S.note = t; const e = $('lnote'); if(e) e.textContent = t; }
function lkNum(id, v, step, w){ return `<input type="number" id="${id}" value="${v}" step="${step||1}" style="width:${w||58}px">`; }

function lkRenderBody(){
  const f = $('lform'); if(!f) return;
  if(!MODEL){ f.innerHTML = '<div class="note">先用「示例 / 读取文件」载入模型，再用连杆调参。</div>'; return; }
  const C = S.cfg;
  const D = S.knobs.filter(k=>k.on).length;
  const rows = S.knobs.map((k,i)=>
    `<tr class="${k.on?'':'off'}">
       <td><input type="checkbox" data-k="${i}" class="kOn" ${k.on?'checked':''}></td>
       <td class="kn">${k.name}</td>
       <td class="num">${k.v0.toFixed(k.v0%1?1:0)}</td>
       <td class="num"><input type="number" data-k="${i}" class="kLo" value="${k.lo}" step="10"></td>
       <td class="num"><input type="number" data-k="${i}" class="kHi" value="${k.hi}" step="10"></td>
       <td class="num"><input type="number" data-k="${i}" class="kCur" value="${S.modelVals[i]!==undefined?S.modelVals[i].toFixed(1):k.v0.toFixed(1)}" step="5"></td>
       <td><button class="mini kScan" data-k="${i}">⟲</button></td>
     </tr>`).join('');
  f.innerHTML = `
  <div class="sect">⛓ 连杆调参 —— 只改杆，<span class="warn">不动限位</span></div>
  <div class="note">${lkDemoLine(5)}</div>
  <div class="note">（在别处改过模型？把本面板关掉再打开，就会重新读一遍基准。<b>第 5 区点一行 = 应用 + 存进浏览器本地存档</b>，刷新后仍生效。）<br>判据比上面那行「IK 复核」严：位置 &lt;${C.eTol}mm、姿态 &lt;${C.aTol}°、
    <b>且最紧关节留 &gt;${C.mGate}° 裕度</b>。分数 = 各位姿裕度的平均（满分 ${C.mCap}°）。<br>
    限位（只读）：<b>${lkLimSig()}</b></div>

  <div class="sect">1 · 目标与预算</div>
  <div class="fld">
    <label>位姿数</label><select id="lNPose">
      ${[8,12,24,48].map(v=>`<option value="${v}" ${C.nPose===v?'selected':''}>${v}</option>`).join('')}</select>
    <label>每起点预算</label><select id="lBudget">
      ${[2,4,6,10].map(v=>`<option value="${v}" ${C.budget===v?'selected':''}>${v}</option>`).join('')}</select>
    <label>IK 迭代</label><select id="lIters">
      ${[160,220,320,450].map(v=>`<option value="${v}" ${C.iters===v?'selected':''}>${v}</option>`).join('')}</select>
  </div>
  <div class="fld">
    <label>位置容差</label>${lkNum('lETol',C.eTol,0.5,52)}<span class="note">mm</span>
    <label>姿态容差</label>${lkNum('lATol',C.aTol,1,52)}<span class="note">°</span>
    <label>裕度门槛</label>${lkNum('lMGate',C.mGate,1,52)}<span class="note">°</span>
    <label>满分裕度</label>${lkNum('lMCap',C.mCap,5,52)}<span class="note">°</span>
  </div>
  <div class="fld">
    <label>杆长之和 ≤</label>${lkNum('lLenCap',C.lenCap||S.armLen0,10,72)}<span class="note">mm（基准 ${S.armLen0.toFixed(0)}）</span>
    <label>随机种子</label>${lkNum('lSeed',C.seed,1,84)}
  </div>
  <div class="fld">
    <button id="lEvalBase">① 评估基准</button>
    <button id="lClear">🧹 清空结果</button>
    <span class="note">已勾选 ${D} 个旋钮</span>
  </div>

  <div class="sect">2 · 旋钮（第 1 段=臂安装位，不在此列；勾上才参与）</div>
  <div class="fld">
    <button id="lOnNonzero" class="mini">只勾非零分量</button>
    <button id="lOnLen" class="mini">只勾"杆长方向"</button>
    <button id="lOnAll" class="mini">全部</button>
    <button id="lOnNone" class="mini">全不勾</button>
    <label>范围 ±</label><input type="number" id="lScale" value="50" step="10" style="width:48px"><span class="note">%</span>
    <button id="lSetRange" class="mini">按比例设范围</button>
  </div>
  <table><thead><tr><th>用</th><th>链段·分量</th><th class="num">基准</th><th class="num">下界</th>
    <th class="num">上界</th><th class="num">当前</th><th>扫</th></tr></thead><tbody>${rows}</tbody></table>

  <div class="sect">3 · 跑</div>
  <div class="fld">
    <button id="lSweep" class="primary">📈 灵敏度扫描</button>
    <label>点数</label><select id="lK">${[5,7,9].map(v=>`<option value="${v}" ${C.K===v?'selected':''}>${v}</option>`).join('')}</select>
  </div>
  <div class="fld">
    <button id="lSearch" class="primary">🔍 全局搜索</button>
    <label title="随机起点越多搜得越狠（每个起点约 37 次评估）；进度条会给预计剩余时间">随机起点</label><select id="lRestarts">${[3,6,12,20].map(v=>`<option value="${v}" ${C.restarts===v?'selected':''}>${v}</option>`).join('')}</select>
    <button id="lStop" class="danger">⏹ 停止</button>
  </div>
  <div class="fld">
    <button id="lVerify">✅ 复核当前模型（用当前设置重算）</button>
    <button id="lRevert" title="回到打开本面板时的模型快照（含限位）">↩ 恢复原始臂</button>
    <button id="lExport">📤 导出候选 JSON</button>
  </div>
  <div id="lprog"></div>

  <div class="sect">4 · 对比（基准 vs 当前模型）</div>
  <div class="cmp"><canvas id="lcmp"></canvas>
    <div class="lst" id="lsegs"></div></div>

  <div id="lsw"></div>

  <div class="sect">5 · 结果（点一行 = 把该连杆方案应用到模型）</div>
  <div id="lres"></div>
  <div id="lnote" class="note" style="margin-top:6px">${lkEsc(S.note||'')}</div>`;
  /* ---- 接线 ---- */
  const num = (id,key,min,max)=>{
    const e = $(id); if(!e) return;
    e.onchange = ()=>{ let v = parseFloat(e.value); if(isNaN(v)) v = S.cfg[key];
      S.cfg[key] = (min!==undefined) ? lkCl(v,min,max) : v; e.value = S.cfg[key];
      lkBuildPoses(); if(key==='nPose'||key==='budget'||key==='seed') lkRenderBody(); };
  };
  num('lETol','eTol',0.05,50); num('lATol','aTol',0.1,45);
  num('lMGate','mGate',0,60); num('lMCap','mCap',1,90);
  num('lLenCap','lenCap',10,5000); num('lSeed','seed');
  const sel = (id,key,cast)=>{ const e = $(id); if(!e) return;
    e.onchange = ()=>{ S.cfg[key] = cast(e.value); lkBuildPoses(); lkRenderBody(); }; };
  sel('lNPose','nPose',v=>+v); sel('lBudget','budget',v=>+v);
  sel('lIters','iters',v=>+v); sel('lK','K',v=>+v); sel('lRestarts','restarts',v=>+v);
  S.knobs.forEach((k,i)=>{
    const ck = f.querySelector(`.kOn[data-k="${i}"]`);
    if(ck) ck.onchange = ()=>{ k.on = ck.checked; lkRenderBody(); };
    const lo = f.querySelector(`.kLo[data-k="${i}"]`);
    if(lo) lo.onchange = ()=>{ k.lo = parseFloat(lo.value); lkRenderBody(); };
    const hi = f.querySelector(`.kHi[data-k="${i}"]`);
    if(hi) hi.onchange = ()=>{ k.hi = parseFloat(hi.value); lkRenderBody(); };
    const cu = f.querySelector(`.kCur[data-k="${i}"]`);
    if(cu) cu.onchange = ()=>{
      MODEL.links[k.li].pos[k.c] = parseFloat(cu.value)||0;
      updateAll(); saveLocal(); S.modelVals = lkReadModel(); lkDrawCmp(); lkRenderBody(); };
    const sc = f.querySelector(`.kScan[data-k="${i}"]`);
    if(sc) sc.onclick = ()=> lkScanOne(i);
  });
  const B = (id,fn)=>{ const e = $(id); if(e) e.onclick = fn; };
  B('lEvalBase', ()=>{ S.modelVals = lkReadModel(); S.limSig = lkLimSig();
      S.job = {kind:'base', gen:lkMakeSingle(S.modelVals,'基准（当前杆）'), n:0, t0:Date.now(), total:1};
      S.stop = false; lkTick(); });
  B('lClear', ()=>{ S.res = []; S.applied = -1; lkRenderRes(); lkNote('结果已清空'); });
  B('lSweep', ()=> lkStartJob('sweep'));
  B('lSearch', ()=> lkStartJob('search'));
  B('lStop', ()=> lkStopJob());
  B('lVerify', ()=>{ S.modelVals = lkReadModel(); S.limSig = lkLimSig();
      S.job = {kind:'verify', gen:lkMakeSingle(S.modelVals,'复核（当前设置）'), n:0, t0:Date.now(), total:1};
      S.stop = false; lkTick(); });
  B('lRevert', ()=> lkRevert());
  B('lExport', ()=> lkExport());
  B('lOnNonzero', ()=>{ S.knobs.forEach(k=>k.on = Math.abs(k.v0)>1e-9); lkRenderBody(); });
  B('lOnAll', ()=>{ S.knobs.forEach(k=>k.on = true); lkRenderBody(); });
  B('lOnNone', ()=>{ S.knobs.forEach(k=>k.on = false); lkRenderBody(); });
  B('lOnLen', ()=>{
    const big = {}; MODEL.links.forEach((L,i)=>{ let bi=-1, bv=0;
      for(let c=0;c<3;c++){ const v = Math.abs(+L.pos[c]||0); if(v>bv){bv=v;bi=c;} }
      if(bi>=0) big[i+'.'+bi] = true; });
    S.knobs.forEach(k=>k.on = !!big[k.key]); lkRenderBody(); });
  B('lSetRange', ()=>{ const p = parseFloat($('lScale').value)||50;
      S.knobs.forEach(k=>{ const r = Math.max(20, Math.abs(k.v0)*p/100);
        k.lo = Math.round(k.v0-r); k.hi = Math.round(k.v0+r);
        if(!k.v0){ k.lo = -Math.round(r); k.hi = Math.round(r); } });
      lkRenderBody(); });
  lkRenderProg(); lkRenderRes(); lkDrawCmp(); lkRenderSegs();
  if(S.sweepPts && S.sweepPts.length) lkRenderSweep(S.sweepPts);
}
/* 单个旋钮单独扫（不等其它旋钮） */
function lkScanOne(i){
  const k = S.knobs[i]; if(!k || S.job) return;
  S.modelVals = lkReadModel(); S.limSig = lkLimSig(); lkBuildPoses();
  const list = [];
  const sub = Object.assign({}, S.cfg, {K:5});
  for(let t=0;t<5;t++){
    const x = k.lo + (k.hi-k.lo)*t/4, v = S.modelVals.slice(); v[i] = x;
    list.push({vals:v, tag:k.name+' = '+x.toFixed(0)});
  }
  let p = 0;
  S.job = {kind:'scan', total:list.length, n:0, t0:Date.now(),
           gen:{ kind:'scan', total:list.length, push(){}, next:()=> p<list.length ? list[p++] : null }};
  S.stop = false; lkTick();
}
function lkRenderProg(){
  const e = $('lprog'); if(!e) return;
  const J = S.job;
  if(!J){ e.innerHTML = S.res.length ? `<div class="note">共 ${S.res.length} 条结果</div>` : ''; return; }
  const p = Math.min(1, J.total ? J.n/J.total : 0);
  const dt = (Date.now()-J.t0)/1000;
  const eta = J.n ? (dt/J.n*Math.max(0,J.total-J.n)) : 0;
  e.innerHTML = `<div class="bar"><i style="width:${(100*p).toFixed(1)}%"></i></div>
    <div class="note">${J.kind==='sweep'?'灵敏度扫描':J.kind==='search'?'全局搜索':'单点评估'}
      · 已评估 <b>${J.n}</b>${J.total?'/'+J.total:''} · ${dt.toFixed(0)}s${J.n&&eta>3?'（约剩 '+eta.toFixed(0)+'s）':''}
      ${S.job&&S.job.last?'· 最新：'+lkEsc(S.res.length?S.res[S.res.length-1].tag:''):''}</div>`;
}
function lkRowCls(r,i){
  return (S.applied===i?' cur':'') + (i===lkBestRow()?' best':'');
}
function lkBestRow(){
  let bi = -1, bs = -1;
  S.res.forEach((r,i)=>{ if(r.score > bs){ bs = r.score; bi = i; } });
  return bi;
}
function lkRenderRes(){
  const e = $('lres'); if(!e) return;
  if(!S.res.length){ e.innerHTML = '<div class="note">还没有结果。点「① 评估基准」或直接「📈 灵敏度扫描」。</div>'; return; }
  const best = lkBestRow(), base = S.res[0];
  const rows = S.res.map((r,i)=>{
    const d = (r.score - base.score);
    const cls = r.score > base.score + 0.01 ? 'good' : (r.score < base.score - 0.01 ? 'bad' : '');
    return `<tr class="rowc${lkRowCls(r,i)}" data-r="${i}" title="点击 = 应用这套杆">
      <td>${lkEsc(r.tag)}</td>
      <td class="num ${cls}">${r.score.toFixed(1)}${i?` <span class="note">${d>=0?'+':''}${d.toFixed(1)}</span>`:''}</td>
      <td class="num">${lkPct(r.pass,1).toFixed(0)}%</td>
      <td class="num">${r.margMed.toFixed(1)}°</td>
      <td class="num">${r.angMed.toFixed(1)}°</td>
      <td class="num">${r.found*100 > 99.9 ? '' : (r.found*100).toFixed(0)+'%'}</td>
      <td class="num">${r.armLen.toFixed(0)}</td>
      <td class="num">${S.knobs.filter((k,i2)=>k.on).map(k=>{
          const v = r.vals[lkIdx(k)]; return Math.abs(v-k.v0)>0.5 ? k.name.split('.')[0]+'.'+AXC[k.c]+'='+v.toFixed(0) : null;
        }).filter(Boolean).join(' ')}</td></tr>`;
  }).join('');
  e.innerHTML = `<table><thead><tr><th>方案</th><th class="num">分数</th><th class="num">合格率</th>
      <th class="num">裕度中位</th><th class="num">姿态中位</th><th class="num">未解出</th>
      <th class="num">杆长和</th><th>相对基准的改动</th></tr></thead>
    <tbody>${rows}</tbody></table>
    <div class="note">分数 = 各位姿"最紧关节裕度/${S.cfg.mCap}°"的平均（解不出来记 0）；
      合格率 = 位置&lt;${S.cfg.eTol}mm 且 姿态&lt;${S.cfg.aTol}° 且 裕度&gt;${S.cfg.mGate}° 的位姿占比。
      ${best>=0?'<br>当前最好 = 「'+lkEsc(S.res[best].tag)+'」（分数 '+S.res[best].score.toFixed(1)+'）；点该行即可应用。':''}</div>`;
  e.querySelectorAll('tr.rowc').forEach(tr=>{ tr.onclick = ()=> lkApplyRow(+tr.dataset.r); });
}
/* 灵敏度曲线：每个旋钮一行 —— 值 + 分数条（★ 标出该旋钮最好的那一档） */
function lkRenderSweep(pts){
  const e = $('lsw'); if(!e) return;
  const byKnob = new Map();
  pts.forEach(p=>{ if(!byKnob.has(p.key)) byKnob.set(p.key, {name:p.name, lo:p.lo, hi:p.hi, a:[]});
                   byKnob.get(p.key).a.push(p); });
  let html = `<div class="note">灵敏度扫描：横条长度 = 分数（0~100），★ = 该杆最好的取值。
    只看某一根杆动的时候分数怎么走 —— 平的就是没关系，有峰的就是该调的。</div>`;
  const base = S.res.length ? S.res[0].score : 0;
  byKnob.forEach(g=>{
    const mx = Math.max(1, ...g.a.map(p=>p.score));
    let bi = 0; g.a.forEach((p,i)=>{ if(p.score > g.a[bi].score) bi = i; });
    html += `<div style="margin:5px 0 2px"><b>${lkEsc(g.name)}</b>
      <span class="note">（基准 ${(g.a[0].score).toFixed(1)} → 最好 ${g.a[bi].score.toFixed(1)}
      @ ${g.a[bi].val.toFixed(0)}）</span></div><table>`;
    g.a.forEach((p,i)=>{
      const w = mx > 0 ? Math.max(1, 100*p.score/mx) : 0;
      const cls = p.score > base + 0.5 ? 'good' : (p.score < base - 0.5 ? 'bad' : '');
      html += `<tr><td class="num" style="width:52px">${p.val.toFixed(0)}</td>
        <td style="width:46%"><i style="display:block;height:7px;border-radius:3px;background:#2f6fb0;width:${w}%"></i></td>
        <td class="num ${cls}">${p.score.toFixed(1)}${i===bi?' ★':''}</td>
        <td class="note">合格 ${(100*p.pass).toFixed(0)}% · 裕度 ${p.marg.toFixed(1)}° · 杆长和 ${p.armLen.toFixed(0)}</td></tr>`;
    });
    html += '</table>';
  });
  e.innerHTML = html;
}
function lkRenderSegs(){
  const e = $('lsegs'); if(!e) return;
  if(!MODEL){ e.innerHTML = ''; return; }
  const cand = lkSegLens();
  let base = null;
  if(S.base){
    const keep = MODEL; MODEL = JSON.parse(JSON.stringify(S.base)); buildChain();
    base = lkSegLens(); MODEL = keep; buildChain();
  }
  e.innerHTML = '<b style="color:#c9d6ee">各段杆长 mm</b><br>' +
    cand.map((v,i)=>{
      const b = base ? base[i] : v, d = v-b;
      const nm = CHAIN[i] ? CHAIN[i].name : ('seg'+i);
      return `${nm}: <b>${v.toFixed(1)}</b>` + (Math.abs(d)>0.5 ? ` <span class="${d<0?'good':'bad'}">(${d>0?'+':''}${d.toFixed(0)})</span>` : '');
    }).join('<br>') +
    `<br>杆长之和 <b>${cand.reduce((a,b)=>a+b,0).toFixed(1)}</b>` +
    (base ? ` <span class="note">(基准 ${base.reduce((a,b)=>a+b,0).toFixed(1)})</span>` : '');
}

/* ---------------- 导出 ---------------- */
function lkDownload(name, text){
  const b = new Blob([text], {type:'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(b); a.download = name;
  document.body.appendChild(a); a.click();
  setTimeout(()=>{ URL.revokeObjectURL(a.href); a.remove(); }, 400);
}
function lkExportPayload(){
  const best = lkBestRow();
  return {
    tool:'armLink.html · 连杆调参', when:new Date().toISOString(),
    rule:'只改连杆 offset；关节限位只读',
    limits: lkLimits(),
    cfg: S.cfg, knobs: S.knobs.map(k=>({name:k.name, li:k.li, c:k.c, v0:k.v0, lo:k.lo, hi:k.hi, on:k.on})),
    baselineArmLen: S.armLen0,
    results: S.res.map(r=>({ tag:r.tag, score:+r.score.toFixed(3), pass:+(100*r.pass).toFixed(1),
      margMed:+r.margMed.toFixed(2), angMed:+r.angMed.toFixed(2), found:+(100*r.found).toFixed(1),
      armLen:+r.armLen.toFixed(2), segs:r.segs.map(v=>+v.toFixed(2)),
      vals:S.knobs.map((k,i)=>[k.name, +r.vals[i].toFixed(2)]),
      chan: S.knobs.map((k,i)=>({key:k.name, v0:+k.v0.toFixed(2), v:+r.vals[i].toFixed(2)})).filter(o=>Math.abs(o.v-o.v0)>0.01) })),
    best: best>=0 ? {tag:S.res[best].tag, vals:S.knobs.map((k,i)=>[k.name, +S.res[best].vals[i].toFixed(2)])} : null
  };
}
function lkExport(){
  if(!S.res.length){ alert('还没有结果'); return; }
  lkDownload('armLink_候选_'+new Date().toISOString().slice(0,10)+'.json',
             JSON.stringify(lkExportPayload(), null, 1));
  lkNote('已导出候选 JSON（含限位只读快照与每套杆的数值）');
}

/* ---------------- 数据面板里追加一小段摘要 ---------------- */
function lkPanelSection(){
  const el = $('dataBody'); if(!el || !MODEL) return;
  const old = el.querySelector('#linkSect'); if(old) old.remove();
  const cand = lkSegLens();
  let base = null;
  if(S.base){ const keep = MODEL; MODEL = JSON.parse(JSON.stringify(S.base)); buildChain();
              base = lkSegLens(); MODEL = keep; buildChain(); }
  const d = cand.reduce((a,b)=>a+b,0) - (base ? base.reduce((a,b)=>a+b,0) : cand.reduce((a,b)=>a+b,0));
  const div = document.createElement('div');
  div.id = 'linkSect';
  div.innerHTML = `
    <div class="sect">连杆（当前 / 基准）</div>
    <div style="color:#5d6b88;font-size:10.5px;line-height:1.6;margin:2px 0 3px">${lkDemoLine(6)}</div>
    <div class="grid">
      <span>各段杆长</span><b>${cand.map(v=>v.toFixed(0)).join(' / ')}</b>
      <span>杆长之和</span><b class="${Math.abs(d)<0.5?'':'bad'}">${cand.reduce((a,b)=>a+b,0).toFixed(1)} mm${Math.abs(d)<0.5?'':'（'+(d>0?'+':'')+d.toFixed(0)+'）'}</b>
      <span>关节限位</span><b>未改动 ✓</b>
      <span>连杆调参</span><b>${S.res.length? S.res.length+' 条结果，最好 '+
          (function(){const b=lkBestRow(); return b>=0?S.res[b].score.toFixed(1):'—';})() : '未跑（点右上「⛓ 连杆调参」）'}</b>
    </div>
    <div style="color:#5d6b88;font-size:10.5px;margin-top:2px">
      ⚠ 上面那行「IK 复核 · 可解」用的容差很宽（姿态 11.5° 都算 ok，限位顶死也算），
      所以基准臂也能显示 ~100%；连杆调参用的是严格判据（位置 ${S.cfg.eTol}mm / 姿态 ${S.cfg.aTol}° /
      <b>关节裕度 &gt;${S.cfg.mGate}°</b>），那才分得出差别。</div>`;
  el.appendChild(div);
}
/* 包一层 analyzeTask：它重建 dataBody 之后，我们追加自己的段 */
const _lk_analyzeTask = analyzeTask;
analyzeTask = function(){ _lk_analyzeTask.apply(this, arguments); try{ lkPanelSection(); }catch(e){} };

/* ---------------- 打开 / 关闭面板 ---------------- */
/* lkInit：把"当前模型"认成基准（快照 + 旋钮 + 限位签名 + 基准臂长）——面板与无头自检共用 */
function lkInit(rebase){
  if(!MODEL) return false;
  if(!S.base || rebase){
    S.base = JSON.parse(JSON.stringify(MODEL));
    S.armLen0 = lkArmLen();
  }
  S.knobs = lkKnobsPreserve();
  S.modelVals = lkReadModel();
  S.limSig = lkLimSig();
  if(!S.armLen0) S.armLen0 = lkArmLen();
  if(!S.cfg.lenCap) S.cfg.lenCap = Math.round(S.armLen0*10)/10;
  return true;
}
function lkOpen(){
  const f = $('lform'); if(!f) return;
  const on = f.classList.toggle('on');
  S.open = on;
  if(!on) return;
  if(!MODEL){ lkRenderBody(); return; }
  lkInit(false);
  lkRenderBody();
  if(!S.res.length){                            // 顺手把基准评一次
    S.limSig = lkLimSig();
    S.job = {kind:'base', gen:lkMakeSingle(S.modelVals,'基准（原始杆）'), n:0, t0:Date.now(), total:1};
    S.stop = false; lkTick();
  }
}

/* ---------------- 同步整轮（无头自检 / 离线搜索用；界面走 lkTick 的切片版） ---------------- */
function lkRunSync(kind, cfg){
  if(!MODEL) return null;
  if(cfg) Object.assign(S.cfg, cfg);
  lkInit(false);
  S.modelVals = lkReadModel();
  lkEnsurePoses();
  const gen = kind === 'sweep' ? lkMakeSweeper(S.modelVals, S.cfg)
            : kind === 'search'? lkMakeSearcher(S.modelVals, S.cfg)
            : lkMakeSingle(S.modelVals, '同步单点');
  if(!gen) return null;
  const t0 = Date.now(), pts = [];
  let n = 0, limOK = true;
  while(true){
    const cand = gen.next(); if(!cand) break;
    const res = lkEval(cand.vals, cand.tag, S.cfg, {t0: performance.now()});
    if(!res.limitOK) limOK = false;
    if(cand.knob) pts.push({key:cand.knob.key, name:cand.knob.name, lo:cand.knob.lo, hi:cand.knob.hi,
      val:cand.val, score:res.score, pass:res.pass, marg:res.margMed, ang:res.angMed, armLen:res.armLen});
    gen.last = res;
    gen.push(res); n++;
  }
  lkWriteVals(S.modelVals);
  const b = gen.bestOf ? gen.bestOf() : null;
  if(gen.kind !== 'sweep'){                       // 和界面一致：单点/搜索最优都进结果表
    if(b && b.res) lkAddRes(b.res, null, '搜索最优');
    else if(gen.kind === 'single') lkAddRes(gen.last, null, '单点评估');
  }
  return { n, ms: Date.now()-t0, points: pts, limitOK: limOK && (lkLimSig() === S.limSig),
           best: b ? b.res : null };
}
/* ---------------- 对外 API（无头自检用） ---------------- */
window.LINK = {
  get S(){ return S; },
  get cfg(){ return S.cfg; },
  init: lkInit,
  knobs: ()=>S.knobs, poses: ()=>S.poses, res: ()=>S.res,
  limits: lkLimits, limSig: lkLimSig, segLens: lkSegLens, armLen: lkArmLen,
  readModel: lkReadModel, writeVals: lkWriteVals,
  buildPoses: lkBuildPoses, solvePose: lkSolvePose, eval: lkEval, workspace: lkWorkspace,
  runSync: lkRunSync, sweepSync: (cfg)=>lkRunSync('sweep', cfg), searchSync: (cfg)=>lkRunSync('search', cfg),
  sweep: ()=>lkStartJob('sweep'), search: ()=>lkStartJob('search'), stop: lkStopJob,
  open: lkOpen, apply: lkApplyRow, revert: lkRevert, export: lkExportPayload,
  bestRow: lkBestRow, snapshot: ()=>JSON.parse(JSON.stringify(S.base||MODEL)),
  demoDiff: lkDemoDiff, demoLine: lkDemoLine, demoModel: ()=>demoModel(),
  busy: ()=>!!S.job
};

/* ---------------- 接线 ---------------- */
function lkBind(){
  const b = $('btnLink'); if(b) b.onclick = lkOpen;
  if(location.hash.indexOf('link') >= 0){
    setTimeout(()=>{ if(!$('lform').classList.contains('on')) lkOpen(); }, 80);
  }
}
if(document.readyState === 'loading') document.addEventListener('DOMContentLoaded', lkBind);
else lkBind();
})();
