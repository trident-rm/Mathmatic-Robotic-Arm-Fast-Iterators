"use strict";
/* ==================================================================
   机械臂建模器（单文件、零依赖、不需要 MATLAB）
   - 模型：MODEL = { name, base:{xyz,yaw}, links:[ ... ] }
   - 每个 link：{ name, joint, type, posMode:'origin'|'parent', pos:[x,y,z](mm),
                  axes:{x,y,z}（新坐标系三轴对应父系的哪个轴）, axis（自身转轴）,
                  limits:[lo,hi]|null }
   - 正解：T_i = T_{i-1} · T_fixed(Rf, offset) · Rot(axis, q_i)
   ================================================================== */
const D2R = Math.PI/180;
const I3  = [[1,0,0],[0,1,0],[0,0,1]];
const AXES = ['+X','-X','+Y','-Y','+Z','-Z'];

function axisVec(s){ return ({'+X':[1,0,0],'-X':[-1,0,0],'+Y':[0,1,0],
                              '-Y':[0,-1,0],'+Z':[0,0,1],'-Z':[0,0,-1]})[s] || [0,0,1]; }
function axisLabel(v){ const a=[Math.abs(v[0]),Math.abs(v[1]),Math.abs(v[2])];
  const i=a[0]>=a[1]&&a[0]>=a[2]?0:(a[1]>=a[2]?1:2);
  return (v[i]<0?'-':'+')+'XYZ'[i]; }
function mul3(A,B){ const C=[[0,0,0],[0,0,0],[0,0,0]];
  for(let i=0;i<3;i++)for(let j=0;j<3;j++){let s=0;for(let k=0;k<3;k++)s+=A[i][k]*B[k][j];C[i][j]=s;} return C; }
function mulv(A,v){ return [A[0][0]*v[0]+A[0][1]*v[1]+A[0][2]*v[2],
                            A[1][0]*v[0]+A[1][1]*v[1]+A[1][2]*v[2],
                            A[2][0]*v[0]+A[2][1]*v[1]+A[2][2]*v[2]]; }
function addv(a,b){ return [a[0]+b[0],a[1]+b[1],a[2]+b[2]]; }
function subv(a,b){ return [a[0]-b[0],a[1]-b[1],a[2]-b[2]]; }
function T3(A){ return [[A[0][0],A[1][0],A[2][0]],[A[0][1],A[1][1],A[2][1]],[A[0][2],A[1][2],A[2][2]]]; }
function det3(A){ return A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
                       - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
                       + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]); }
function rotAxis(u,th){ const x=u[0],y=u[1],z=u[2],c=Math.cos(th),s=Math.sin(th),C=1-c;
  return [[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]]; }
function rotZdeg(d){ const a=d*D2R,c=Math.cos(a),s=Math.sin(a);
  return [[c,-s,0],[s,c,0],[0,0,1]]; }

/* ---------------- 示例模型（= 同济臂，7 段） ---------------- */
/* ---------------- 示例 / 默认臂型 ----------------
   ★ 这里【不再】写死同济臂的杆长与限位 —— 它已搬到 web/src/configs/tongji.js。
     demoModel() 保留为兼容入口：读当前配置里的模型。
     （历史上这里是第二份臂型定义，和 MATLAB 的 armSpec.m 各写一份，
       改一处忘一处是常态。现在两边都以配置文件为准。） */
function demoModel(){
  if (MODEL) return JSON.parse(JSON.stringify(MODEL));
  if (typeof CONFIG !== 'undefined' && CONFIG && CONFIG.model)
    return JSON.parse(JSON.stringify(CONFIG.model));
  throw new Error('还没有加载任何配置：demoModel() 无模型可取');
}

/* ---------------- 由 MODEL 生成可计算的链 ---------------- */
let MODEL = null, CHAIN = [], NJ = 0, Q = [], ACTIVE = -1, SHOW_FRAMES = true;
let CLOUD = [], STATS = {}, CURVEPTS = null;
let ARMSTATS = [];              // 每条臂的可达集与统计（多臂用）

/* ---------------- 底盘碰撞箱 / 伸展尺寸：规则侧状态 ----------------
   ★ 默认值来自配置（CONFIG.chassis），引擎不再自带同济的车体尺寸。
     没配置时这些对象为空，尺寸校核那块界面会自己让开。
   CHASSIS.pos 默认 base 系；zDir=−1 ⇒ 从中心点往 −Z 长 size[2]
   ⇒ 中心点就是碰撞箱【上表面中心】，底面中心 = 接地中心                        */
let CHASSIS = { posMode:'base', pos:[0,0,0], size:[1,1,1], zDir:-1 };
let SIZER   = { lwh:[0,0,0], ballD:0, ballH:0 };
let SHOWBOX = { chassis:true, lwh:true, ball:true };
let PANEL_REST = '';            // 数据面板里除"底盘/尺寸"段以外的 HTML（改底盘参数时只重画尺寸段）

/* ---------------- 多臂支持 ----------------
   一个构型可以有 N 条臂（N=1 就是原来那条）。
   约定：
     · MODEL / CHAIN / NJ / Q 仍然指向【第 0 条臂】，老代码全部照旧能用
     · CHAIN 是【所有臂首尾相接】的扁平数组，Q 也是扁平的
       ⇒ qLimits() / ikSolve6() 这些"按关节序号"的函数天然支持多臂，
         第 a 条臂的关节就是 [arm.j0, arm.j0+arm.nJ)
     · ARMS[a] 记录每条臂的元信息与它在扁平数组里的区间
   ------------------------------------------------------------------ */
let ARMS = [];                  // 每条臂：{name, model, yaw, c0, cN, j0, nJ, ee, xyz}
function armCount(){ return ARMS.length || 1; }
function armOfJointQI(qi){       // 扁平关节序号 -> 臂序号（-1 = 找不到）
  for(let a=0;a<ARMS.length;a++){
    const A = ARMS[a];
    if(qi >= A.j0 && qi < A.j0 + A.nJ) return a;
  }
  return -1;
}
/* 取某条臂的模型（不传就用第 0 条） */
function armModel(a){ a = a || 0; return (ARMS[a] && ARMS[a].model) || MODEL; }

/* 把一条臂的 link 追加进 CHAIN */
function appendArm(a, arm, R0, t0, out){
  let R = R0.slice(), t = t0.slice();
  let nj = 0;
  const c0 = out.length;
  arm.links.forEach((L, i) => {
    let off;
    if(L.posMode === 'origin' && i > 0){
      // 在整机原点系里给的绝对位置 → 换算成相对父系的偏移
      off = mulv(T3(R), subv(L.pos, t));
    } else {
      off = L.pos.slice();
    }
    const Rf = frameRot(L.axes);
    const movable = L.type !== 'fixed';
    out.push({name:L.name, joint:L.joint, type:L.type, movable, offset:off, Rf,
              axis:axisVec(L.axis), axLabel:L.axis, arm:a,
              limits:L.limits ? L.limits.slice() : null});
    if(movable) nj++;
    t = addv(mulv(R, off), t);
    R = mul3(R, Rf);
  });
  return {c0, cN: out.length, nJ: nj};
}

function buildChain(modelOrList){
  /* 参数三种写法：
       buildChain()            用【当前配置】的臂列表（界面走这条）
       buildChain(model)       用这一条臂（无头自检、临时试算走这条）
       buildChain([m0, m1, …]) 用这组臂（多臂）
     ★ 引擎函数不该偷偷去读全局配置：不传参数时才回落到配置，
       这样"给我一条臂，把链建出来"是显式可测的，不会出现
       "我设了 MODEL 却没生效"这种隐蔽耦合（t12 就是被这个坑挂掉的）。 */
  let list;
  if(Array.isArray(modelOrList)){
    list = modelOrList;
  } else if(modelOrList && Array.isArray(modelOrList.links)){
    list = [modelOrList];
  } else {
    const arms = (typeof activeArms === 'function') ? activeArms() : null;
    list = (arms && arms.length) ? arms : (MODEL ? [MODEL] : []);
  }
  const flat = [];
  ARMS = [];
  let j0 = 0;
  list.forEach((arm, a) => {
    const R0 = rotZdeg(arm.base.yaw), t0 = arm.base.xyz.slice();
    const m = appendArm(a, arm, R0, t0, flat);
    ARMS.push({name: arm.name, model: arm, yaw: arm.base.yaw, xyz: arm.base.xyz.slice(),
               c0: m.c0, cN: m.cN, j0: j0, nJ: m.nJ, ee: m.cN - 1});
    j0 += m.nJ;
  });
  CHAIN = flat;
  NJ = j0;
  /* 第 0 条臂继续用原来的名字，老代码（与自检）不受影响 */
  if(list.length) MODEL = list[0];
  syncQ();
}
function frameRot(ax){
  const c0 = axisVec(ax.x), c1 = axisVec(ax.y), c2 = axisVec(ax.z);
  return [[c0[0],c1[0],c2[0]],[c0[1],c1[1],c2[1]],[c0[2],c1[2],c2[2]]];
}
function frameOK(ax){
  const R = frameRot(ax);
  const det = det3(R);
  const dup = (ax.x===ax.y)||(ax.y===ax.z)||(ax.x===ax.z);
  return {ok: !dup && Math.abs(det-1) < 1e-9, det, dup, R};
}
function qLimits(){
  return CHAIN.filter(c=>c.movable).map(c => {
    const l = c.limits ? [Math.min(...c.limits), Math.max(...c.limits)] : [-180,180];
    return [l[0] - LIM_WIDEN, l[1] + LIM_WIDEN];     // LIM_WIDEN 用于"放宽限位能救回多少"的试验
  });
}
let LIM_WIDEN = 0;
function syncQ(){                       // 关节数变化就重置，否则保留用户拖动值
  const lim = qLimits();
  if(Q.length !== lim.length){ Q = lim.map(([lo,hi]) => (lo<=0&&0<=hi) ? 0 : (lo+hi)/2); }
  else { Q = Q.map((v,i) => Math.min(Math.max(v, lim[i][0]), lim[i][1])); }
}

/* ---------------- 正解 ----------------
   fkAllArm(q, a) 算【第 a 条臂】：返回该臂的坐标系序列，第 0 个是整机原点 O。
   fkAll(q)       保持老语义 = 第 0 条臂（所有老代码与自检不受影响）。
   fkAllArms(q)   所有臂拼起来，每项多一个 arm 字段（用于多臂取样与显示）。
   q 是【扁平】关节角数组，第 a 条臂用 q[j0 .. j0+nJ)。                    */
function fkAllArm(q, a){
  a = a || 0;
  const A = ARMS[a];
  if(!A) throw new Error('没有第 ' + a + ' 条臂');
  const arm = A.model;
  let R = rotZdeg(arm.base.yaw), t = arm.base.xyz.slice();
  const out = [{name:'O(整机原点)', joint:'', R:R, t:t, axis:[0,0,0],
                movable:false, qi:-1, arm:a}];
  let qi = A.j0;
  for(let ci = A.c0; ci < A.cN; ci++){
    const c = CHAIN[ci];
    let Rj = I3;
    if(c.movable){ Rj = rotAxis(c.axis, (q[qi]||0)*D2R); qi++; }
    const tNew = addv(mulv(R, c.offset), t);   // 用【更新前】的 R
    R = mul3(mul3(R, c.Rf), Rj);
    t = tNew;
    out.push({name:c.name, joint:c.joint, R:R, t:t, axis:c.axis, movable:c.movable,
              qi:qi-1, arm:a});
  }
  return out;
}
function fkAll(q){ return fkAllArm(q, 0); }

/* 所有臂的正解拼接（第 0 条臂的 O 只留一份，避免整机原点重复出现） */
function fkAllArms(q){
  const out = [];
  for(let a=0;a<armCount();a++){
    const F = fkAllArm(q, a);
    if(a === 0) out.push(...F);
    else        out.push(...F.slice(1));
  }
  return out;
}
/* 某条臂的末端位姿（不传 a 就是第 0 条） */
function armEnd(q, a){
  const A = ARMS[a || 0];
  if(!A) return null;
  const F = fkAllArm(q, a || 0);
  return F[F.length - 1];
}

/* ---------------- 采样 + 体素化（全部在浏览器里算） ----------------
   sampleReach(N) 是【纯计算】：不碰 DOM、不读界面，只把结果算出来返回。
   computeFeasible() 只负责"从界面读采样数 -> 调它 -> 写回全局"。
   这样无头自检（Node 沙箱）可以直接调 sampleReach 验证多臂统计，
   不必依赖界面控件。                                                */
function sampleReach(N){
  N = N || 30000;
  const lim = qLimits();
  const q = new Array(NJ);
  const nA = armCount();
  const pts = new Array(N);          // 主点云：第 0 条臂（老语义不变）
  const ptsArm = [];                 // 多臂：每条臂一条点云
  for(let a=0;a<nA;a++) ptsArm.push([]);
  for(let s=0;s<N;s++){
    for(let i=0;i<NJ;i++){ const [lo,hi]=lim[i]; q[i] = lo + Math.random()*(hi-lo); }
    /* 一次采样把所有臂的末端都取出来 —— 同一组 q，各臂各自算，
       所以"协同"是天然成立的（两条臂共用同一时刻的关节角向量）。
       臂间碰撞/干涉不在本工具范围内（见 README 已知边界）。 */
    for(let a=0;a<nA;a++){
      const p = armEnd(q, a).t.slice();
      ptsArm[a].push(p);
      if(a === 0) pts[s] = p;
    }
  }
  const vs = 50, keys = new Set();
  let rmin = Infinity, rmax = 0, zmin = Infinity, zmax = -Infinity, rhomin = Infinity, rhomax = 0;
  for(const p of pts){
    keys.add(Math.floor(p[0]/vs)+','+Math.floor(p[1]/vs)+','+Math.floor(p[2]/vs));
    const r = Math.hypot(p[0],p[1],p[2]); if(r<rmin) rmin=r; if(r>rmax) rmax=r;
    const rh = Math.hypot(p[0],p[1]);     if(rh<rhomin) rhomin=rh; if(rh>rhomax) rhomax=rh;
    if(p[2]<zmin) zmin=p[2]; if(p[2]>zmax) zmax=p[2];
  }
  const step = Math.max(1, Math.floor(N/3000));
  const cloud = []; for(let i=0;i<N;i+=step) cloud.push(pts[i]);
  /* 每条臂的可达集与统计（多臂时用于分别显示/比对） */
  const armStats = ptsArm.map((P, a) => {
    const kk = new Set();
    let aRmax = 0, aZmin = Infinity, aZmax = -Infinity;
    for(const p of P){
      kk.add(Math.floor(p[0]/vs)+','+Math.floor(p[1]/vs)+','+Math.floor(p[2]/vs));
      aRmax = Math.max(aRmax, Math.hypot(p[0],p[1],p[2]));
      aZmin = Math.min(aZmin, p[2]); aZmax = Math.max(aZmax, p[2]);
    }
    /* keys = 这条臂自己的可达体素集合。
       ★ 任务域校核要按臂判断"这个任务点落不落在这条臂的可达集里"，
         所以每条臂必须带自己的 keys；原来只有全局 VOX（= 臂 0 的）。 */
    return {name:(ARMS[a]||{}).name || ('臂'+a), n:P.length, voxels:kk.size, keys:kk,
            rmax:aRmax, zmin:aZmin, zmax:aZmax,
            cloud: P.filter((_, i) => i % step === 0)};
  });
  return {N, vs, keys, cloud, armStats,
          stats:{n:N, voxels:keys.size, rmin, rmax, rhomin, rhomax, zmin, zmax,
                 nJoint:NJ, nArm:nA}};
}
function computeFeasible(){
  const N = parseInt(document.getElementById('selN').value, 10) || 30000;
  const R = sampleReach(N);
  CLOUD = R.cloud;
  ARMSTATS = R.armStats;
  VOX = R.keys;                     // 可达集体素（供任务域判定用）
  STATS = R.stats;
}
/* ---------------- 模型更新总入口 ---------------- */
function updateAll(){
  buildChain();
  const t0 = performance.now();
  computeFeasible();
  const dt = performance.now() - t0;
  CURVEPTS = fkAll(Q);
  renderHeader(dt);
  draw3D(); drawList();
}
/* 重建"旋转角度"面板。
   ★ 为什么必须单独有这么一步：updateAll() 里的 buildChain() 只更新【引擎】
     （CHAIN / NJ / qLimits），而右侧那个面板是【一次性渲染出来的 DOM】——
     滑条的范围、数字框的 max、关节名、行数，全是渲染那一刻的快照。
     谁改了链结构（加段/删段/改限位），谁就要把它重画一遍；
     不重画的表现是"引擎里已经变了、界面上还是老的"：
       · 改限位 → 滑条范围还是旧限位
       · 加一段 → 面板里根本不出现这条关节
       · 删一段 → 面板里那条关节还留着
     （06-ui 里 renderAngles() 会重建整个 #qbody 并重接事件。） */
function refreshAnglesUI(){
  if(typeof renderAngles === 'function') renderAngles();
}
function renderHeader(dt){
  document.getElementById('title').textContent = MODEL.name || '未命名';
  document.getElementById('model').textContent =
      '纯杆+关节 · 无碰撞几何 · 全部在浏览器内计算（无需 MATLAB）';
  const s = STATS;
  document.getElementById('chips').innerHTML = [
    chip('关节数', NJ),
    chip('采样', s.n.toLocaleString()),
    chip('末端到原点', Math.round(s.rmin)+'~'+Math.round(s.rmax)+' mm'),
    chip('距 Z 轴', Math.round(s.rhomin)+'~'+Math.round(s.rhomax)+' mm'),
    chip('高度 z', Math.round(s.zmin)+'~'+Math.round(s.zmax)+' mm'),
    chip('体素 50mm', s.voxels),
    chip('计算耗时', dt.toFixed(0)+' ms')
  ].join('');
  document.getElementById('lgJoints').textContent = CHAIN.filter(c=>c.movable)
      .map(c=>c.joint).join('　');
}
const chip = (k,v) => `<span class="chip">${k} <b>${v}</b></span>`;

/* ==================================================================
   右侧：模型编辑面板
   ================================================================== */
function drawList(){
  const list = document.getElementById('list');
  list.innerHTML = '';
  const F0 = fkAll(Q);                       // 当前位形下的各系原点（用于显示位置）
  MODEL.links.forEach((L, i) => {
    const c = CHAIN[i], o = F0[i+1].t;
    const lim = c.limits ? (c.limits[0]+'~'+c.limits[1]+'°') : (c.movable ? '不限' : '—');
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML =
      `<span class="idx">${i+1}</span>
       <span class="nm"><b>${L.name}</b>
         <span>${L.joint} · ${c.type==='fixed'?'固定':'转动'} ${c.movable?c.axLabel:''} ${lim}</span>
         <span>原点 [${o.map(v=>v.toFixed(1)).join(', ')}] mm</span></span>
       <button data-i="${i}" class="ed">编辑</button>
       <button data-i="${i}" class="del danger">删除</button>`;
    list.appendChild(row);
  });
  list.querySelectorAll('button.ed').forEach(b => b.onclick = () => openForm(+b.dataset.i));
  list.querySelectorAll('button.del').forEach(b => b.onclick = () => {
    const i = +b.dataset.i, n = MODEL.links.length - i;
    const msg = (n > 1) ? `删除「${MODEL.links[i].name}」及其之后的 ${n-1} 个坐标系？`
                        : `删除「${MODEL.links[i].name}」？`;
    if(confirm(msg)){ MODEL.links.splice(i); closeForm(); updateAll(); refreshAnglesUI(); saveLocal(); }
  });
}

let EDIT = -1;      // -1 = 新增
function openForm(i){
  EDIT = (i === undefined) ? -1 : i;
  const isNew = EDIT < 0;
  const L = isNew
    ? {name:'link'+(MODEL.links.length+1), joint:'joint'+(MODEL.links.length+1),
       type:'revolute', posMode:(MODEL.links.length? 'parent':'origin'), pos:[0,0,0],
       axes:{x:'+X',y:'+Y',z:'+Z'}, axis:'+Z', limits:null}
    : JSON.parse(JSON.stringify(MODEL.links[EDIT]));
  const first = (MODEL.links.length === 0);
  const parentName = first ? '整机原点 O' : MODEL.links[MODEL.links.length-1].name;
  const f = document.getElementById('form');
  f.className = 'on';
  f.innerHTML = `
    <div class="fld" style="justify-content:space-between">
      <b>${isNew?'建立新坐标系':'编辑：'+L.name}</b>
      <span class="note">${first?'第一个坐标系':(isNew?'会接在链末':'')} · 父坐标系 = ${parentName}</span>
    </div>
    <div class="fld"><label>名称</label><input type="text" id="fName" value="${L.name}">
      <label>关节名</label><input type="text" id="fJoint" value="${L.joint}"></div>
    <div class="fld"><label>类型</label>
      <label class="rad"><input type="radio" name="fType" value="revolute" ${L.type!=='fixed'?'checked':''}>转动</label>
      <label class="rad"><input type="radio" name="fType" value="fixed" ${L.type==='fixed'?'checked':''}>固定（无自由度）</label></div>

    <div class="fld"><label>位置</label>
      <label class="rad"><input type="radio" name="fPos" value="parent" ${L.posMode==='parent'?'checked':''}>相对上一坐标系</label>
      <label class="rad"><input type="radio" name="fPos" value="origin" ${L.posMode==='origin'?'checked':''}>相对整机原点</label>
      ${first?'<span class="note">（base 默认在原点）</span>':''}</div>
    <div class="fld"><label></label>
      X <input type="number" id="fPx" step="0.01" value="${L.pos[0]}">
      Y <input type="number" id="fPy" step="0.01" value="${L.pos[1]}">
      Z <input type="number" id="fPz" step="0.01" value="${L.pos[2]}"> mm
      <span class="note" id="posNote"></span></div>

    <div class="fld"><label>坐标系变换</label><span class="note">新坐标系的三个轴分别对齐父系的哪个轴</span></div>
    <div class="fld"><label></label>
      新 X → ${sel('fAx',L.axes.x)}　新 Y → ${sel('fAy',L.axes.y)}　新 Z → ${sel('fAz',L.axes.z)}
      <span id="rotMsg" class="note"></span></div>

    <div class="fld"><label>转动轴</label>${sel('fAxis',L.axis)}
      <span class="note">（在新坐标系自身下）</span></div>
    <div class="fld"><label>限位角</label>
      下限 <input type="number" id="fLo" step="1" value="${L.limits?L.limits[0]:''}">
      上限 <input type="number" id="fHi" step="1" value="${L.limits?L.limits[1]:''}"> 度
      <span class="note">留空 = 不限位</span></div>

    <div class="fld" style="justify-content:flex-end;gap:8px">
      <button id="fCancel">取消</button>
      <button id="fOK" class="primary">${isNew?'确认建立':'保存修改'}</button></div>`;
  ['fAx','fAy','fAz'].forEach(id => document.getElementById(id).onchange = checkRot);
  document.getElementById('fCancel').onclick = closeForm;
  document.getElementById('fOK').onclick = submitForm;
  checkRot();
  function updPosNote(){
    const m = f.querySelector('input[name=fPos]:checked').value;
    document.getElementById('posNote').textContent =
      m === 'origin' ? '（在整机原点系 O 下的绝对坐标，零位时）' : '（在上一个坐标系下的位移）';
  }
  f.querySelectorAll('input[name=fPos]').forEach(r => r.onchange = updPosNote);
  updPosNote();
}
function sel(id, cur){
  return `<select id="${id}">${AXES.map(a=>`<option ${a===cur?'selected':''}>${a}</option>`).join('')}</select>`;
}
function checkRot(){
  const ax = {x:document.getElementById('fAx').value,
              y:document.getElementById('fAy').value,
              z:document.getElementById('fAz').value};
  const r = frameOK(ax), el = document.getElementById('rotMsg');
  if(r.dup){ el.className = 'badmsg'; el.textContent = '✗ 三个轴不能重复'; }
  else if(Math.abs(r.det-1) > 1e-9){
    el.className = 'badmsg';
    el.textContent = '✗ 不是右手系（det=' + r.det.toFixed(0) + '），请调整';
  } else { el.className = 'okmsg'; el.textContent = '✓ 右手系'; }
}
function submitForm(){
  const f = document.getElementById('form');
  const ax = {x:document.getElementById('fAx').value,
              y:document.getElementById('fAy').value,
              z:document.getElementById('fAz').value};
  if(!frameOK(ax).ok){ alert('坐标系变换不是右手系（或有重复轴），请修正'); return; }
  const loS = document.getElementById('fLo').value.trim();
  const hiS = document.getElementById('fHi').value.trim();
  const type = f.querySelector('input[name=fType]:checked').value;
  const L = {
    name: document.getElementById('fName').value.trim() || 'link',
    joint: document.getElementById('fJoint').value.trim() || 'joint',
    type,
    posMode: f.querySelector('input[name=fPos]:checked').value,
    pos: [ +document.getElementById('fPx').value || 0,
           +document.getElementById('fPy').value || 0,
           +document.getElementById('fPz').value || 0 ],
    axes: ax,
    axis: document.getElementById('fAxis').value,
    limits: (type === 'fixed' || loS === '' || hiS === '') ? null
            : [Math.min(+loS,+hiS), Math.max(+loS,+hiS)]
  };
  if(EDIT < 0) MODEL.links.push(L); else MODEL.links[EDIT] = L;
  closeForm(); updateAll(); refreshAnglesUI(); saveLocal();
}
function closeForm(){ document.getElementById('form').className = ''; EDIT = -1; }

/* ---------------- 存取 ---------------- */
let LS_BLOCKED = false;      // 有些浏览器（如 macOS Safari 打开本地文件）会禁用 localStorage
function warnNoLS(){
  if(LS_BLOCKED) return;
  LS_BLOCKED = true;
  const t = document.getElementById('tools');
  if(!t || document.getElementById('lsWarn')) return;
  const s = document.createElement('span');
  s.id = 'lsWarn'; s.className = 'chip';
  s.style.cssText = 'border-color:#7a5a20;color:#ffb454';
  s.textContent = '⚠ 浏览器禁用了本地存档，请用「保存 JSON」保存模型';
  t.insertBefore(s, t.firstChild);
}
/* 模型的【规范化】序列化：对象按 key 排序，用于"两份模型是不是同一份"的比对。
   ★ 必须只有这一个口径：存档端与读取端各写各的（一个排序一个不排），
     比出来的结果必然是"不相等" —— 踩过，判定恒假、看着像功能没生效。
   ★ 另外固定忽略 vehicle：那是【装配参数】（装在哪、往哪插），
     是界面运行起来之后挂到模型上的，配置里那条臂本来就没有这个块。
     不忽略的话"这份模型是不是当前配置那条臂"永远判否。 */
function canonModel(m){
  if(!m) return '';
  return JSON.stringify(m, (k, v) => {
    if(k === 'vehicle') return undefined;                 // 顶层 vehicle 不参与比对
    return (v && typeof v === 'object' && !Array.isArray(v))
      ? Object.keys(v).sort().reduce((o, kk) => (o[kk] = v[kk], o), {})
      : v;
  });
}
/* ---------------- 本地存档 ----------------
   ★ 每条构型各存一份：模型 `armBuilderModel:<configId>`、设置 `armBuilderCfg:<configId>`。
     为什么必须分开存：一套构型下的改动，不该被"切到另一套构型看一眼"冲掉 ——
     原来只有一份，切走那一下就把存档覆盖成新构型，切回来改动就没了。

   ★ 设置里其实混了两类东西，不能都按构型分（分错了会互相打架）：
       按构型分的「设置」：装配位 init / 映射 taskMap / 底盘箱 chassis / 底盘高 baseH
                          / 采样数 nSample / 安装 yaw / 关节角 q
         —— 换个构型这些本来就该各是各的（车不一样、规则不一样）
       全局的「视图偏好」：分区折叠 SECT_OPEN（另有自己的键）、左右栏分隔条 splitPct、
                          显示开关 showbox
         —— 这是"怎么看"，与哪条臂无关，切构型不该把它重置

   ★ 恢复顺序（踩过两次）：加载时【先设置、再模型、最后关节角】。
     setModel() 会把 Q 清空（换了模型旧角度没意义），所以关节角必须最后恢复。 */
const LS_MODEL_PREFIX = 'armBuilderModel';
const LS_CFG_PREFIX = 'armBuilderCfg';
const LS_VIEW_KEY = 'armBuilderView';
const LS_MODEL_OLD = 'armBuilderModel';        // 旧版：不带后缀的那两个键
const LS_CFG_OLD = 'armBuilderCfg';
function lsModelKey(cfgId){
  const id = cfgId || ((typeof activeConfigId === 'function') ? activeConfigId() : null);
  return id ? (LS_MODEL_PREFIX + ':' + id) : LS_MODEL_OLD;
}
function lsCfgKey(cfgId){
  const id = cfgId || ((typeof activeConfigId === 'function') ? activeConfigId() : null);
  return id ? (LS_CFG_PREFIX + ':' + id) : LS_CFG_OLD;
}
/* 上次用的是哪个配置？
   ★ 不能靠"扫一遍 armBuilderCfg:* 取第一个" —— 每条构型都有自己的存档，
     遍历顺序是插入顺序，取到的很可能是最早那条，不是最后用的那条（踩过）。
     所以"上次用谁"单独记在全局视图那块（armBuilderView.configId）。 */
function savedConfigId(){
  try{
    const v = localStorage.getItem(LS_VIEW_KEY);
    if(v){ const id = JSON.parse(v).configId; if(id) return id; }
  }catch(e){}
  try{                                    // 旧版：不带后缀的那个键里记着
    const s = localStorage.getItem(LS_CFG_OLD);
    if(s) return JSON.parse(s).configId || null;
  }catch(e){}
  return null;
}
/* 一次性迁移：旧版把模型与设置存在【不带后缀】的键里。
   只有当那份存档确实属于当前配置时才认领 —— 否则会拿甲的模型去套乙的配置
   （这正是要修的那类错位）。迁移只做一次，之后它归当前配置所有。 */
function migrateOldArchive(){
  if(!(typeof localStorage !== 'undefined') || LS_BLOCKED) return;
  let old = null, oldCfg = null;
  try{ old = localStorage.getItem(LS_MODEL_OLD); oldCfg = localStorage.getItem(LS_CFG_OLD); }
  catch(e){ return; }
  if(!old) return;
  const cfgId = (typeof activeConfigId === 'function') ? activeConfigId() : null;
  if(!cfgId) return;
  let c = null, m = null;
  try{ c = oldCfg ? JSON.parse(oldCfg) : null; }catch(e){}
  try{ m = JSON.parse(old); }catch(e){ return; }
  const mine = (c && c.configId === cfgId) ||
               ((c && c.modelCfg) ? canonModel(m) === c.modelCfg : false);
  if(!mine) return;                       // 不是我的，留着不动（用户切回去还能用）
  try{
    if(!localStorage.getItem(lsModelKey(cfgId))) localStorage.setItem(lsModelKey(cfgId), old);
    if(oldCfg && !localStorage.getItem(lsCfgKey(cfgId))) localStorage.setItem(lsCfgKey(cfgId), oldCfg);
    localStorage.removeItem(LS_MODEL_OLD);
    localStorage.removeItem(LS_CFG_OLD);
  }catch(e){}
}
/* 按构型分的设置快照（模型之外的一切） */
function cfgObj(){                       // 规则侧设置的快照（模型之外的一切）
  return { init: RULES ? RULES.init : null,
           taskMap: MAP ? MAP.taskMap : null,      // ★ 任务系映射（原来是布尔 swap）
           baseH: RULES ? RULES.baseH : null,
           nSample: document.getElementById('selN').value,
           yaw: MODEL ? MODEL.base.yaw : -90,
           /* ★ 关节角也要存：原来只存"模型"，拖过的角度刷新就归零了。
              存的是扁平向量，载入时按关节数核对（对不上就整段忽略，不能硬套）。 */
           q: (typeof Q !== 'undefined' && Array.isArray(Q)) ? Q.slice() : null,
           chassis: CHASSIS, chassisV: CHASSIS_V,      // ★ 底盘箱（带默认值版本号）
           sizer: SIZER,                               // ★ 伸展尺寸
           configId: (typeof activeConfigId === 'function') ? activeConfigId() : null,  // ★ 当前配置
           /* ★ 配置里【原始】那条臂的快照（规范化序列化，与读取端同口径 canonModel）。
              用途：旧版存档迁移时判断"这份存档是不是当前配置这条臂的"。 */
           modelCfg: canonModel(CONFIG && CONFIG.model),
           t: Date.now() };                      // 存档时间（界面上的"已保存"提示用）
}
/* 全局视图偏好（与哪条构型无关）。configId 一并记在这里 ——
   它是"上次用的是哪条构型"，属于全局状态，不属于任何一条构型的存档。 */
function viewObj(){
  return { splitPct: SPLIT_PCT, showbox: SHOWBOX,
           configId: (typeof activeConfigId === 'function') ? activeConfigId() : null,
           t: Date.now() };
}
function saveSettings(){
  try{ localStorage.setItem(lsCfgKey(), JSON.stringify(cfgObj())); }
  catch(e){ warnNoLS(); }
}
/* ★ 换构型期间的落盘闸门。
   切构型是"卸下旧臂 → 装上新臂 → 恢复它的存档"三步，中间有一小段
   两边都不对的状态（模型还是旧的、角度已清空…）。这段里任何一次 saveLocal()
   都会把【新构型那条键】写坏 —— 踩过：切到 B 再切回 A，A 的关节角变成全零，
   因为 A 的存档在切换途中被 B 的空角度覆盖了。
   所以切换期间把落盘压住，最后由切换流程自己在"状态齐了"之后写一次。 */
let LS_HOLD = 0;
function holdSave(on){ LS_HOLD += on ? 1 : -1; if(LS_HOLD < 0) LS_HOLD = 0; }
function saveLocal(){
  if(LS_HOLD > 0) return;                  // 切换途中：不落盘（见上）
  try{
    localStorage.setItem(lsModelKey(), JSON.stringify(MODEL));
    // ★ 规则侧设置也要存：装配臂位置 / 映射 / 底盘高 / 采样数 / 底盘箱 / 伸展尺寸 / 关节角
    localStorage.setItem(lsCfgKey(), JSON.stringify(cfgObj()));
    localStorage.setItem(LS_VIEW_KEY, JSON.stringify(viewObj()));
    if(typeof markSaved === 'function') markSaved();
  }catch(e){ warnNoLS(); }
}
function loadLocal(){
  try{
    const s = localStorage.getItem(lsModelKey());
    if(s) return JSON.parse(s);
    /* 还没有这条构型的存档：退回旧版那个不带后缀的键（迁移前的兜底） */
    const o = localStorage.getItem(LS_MODEL_OLD);
    return o ? JSON.parse(o) : null;
  }
  catch(e){ warnNoLS(); return null; }
}
/* 恢复上次的设置。分两段：
     ① 按构型的设置（只在这条构型自己的存档里，且 configId 要对得上）
     ② 全局视图偏好（分隔条 / 显示开关）—— 与构型无关，永远恢复
   ★ 关节角留到最后：调用方在 setModel() 之后必须再调一次 loadCfg()，
     因为 setModel() 会清空 Q（换了模型，旧角度没意义）。 */
function loadCfg(opts){
  const qOnly = !!(opts && opts.qOnly);
  try{
    if(!qOnly){
      const v = localStorage.getItem(LS_VIEW_KEY);
      if(v){
        const o = JSON.parse(v);
        if(typeof o.splitPct === 'number'){ SPLIT_PCT = o.splitPct; applySplit(); }
        if(o.showbox) Object.assign(SHOWBOX, o.showbox);
      }
    }
    let s = localStorage.getItem(lsCfgKey());
    if(!s) s = localStorage.getItem(LS_CFG_OLD);      // 迁移前的兜底
    if(!s) return;
    const c = JSON.parse(s);

    /* ★ 这份设置属于哪个构型？不属于当前构型就整段忽略 ——
       否则拿同济的装配位/底盘箱去套新构型，数字看着正常其实全错。
       （恢复顺序踩过：先恢复设置、再上模型，最后才恢复关节角。） */
    if(c.configId && typeof activeConfigId === 'function' && c.configId !== activeConfigId()){
      return;
    }
    if(!qOnly){
      if(c.init && RULES) Object.assign(RULES.init, c.init);
      if(c.taskMap && MAP){ MAP.taskMap = c.taskMap; if(RULES && RULES.frame) RULES.frame.taskMap = c.taskMap; }
      else if(typeof c.swap === 'boolean' && MAP) MAP.taskMap = c.swap ? 'swap' : 'noSwap';
      if(c.baseH && RULES) RULES.baseH = c.baseH;
      if(c.nSample) document.getElementById('selN').value = c.nSample;
      /* ★ 底盘箱要带版本号：改动 CHASSIS 默认值后，老存档不能把新默认值盖回去 */
      if(c.chassis && c.chassisV === CHASSIS_V) Object.assign(CHASSIS, c.chassis);
      if(c.sizer)   Object.assign(SIZER,   c.sizer);
      /* 映射可能被存档改过 ⇒ 重建换算对象（实现只有一份，见 04-config.js） */
      if(typeof rebuildTaskMap === 'function' && MAP) rebuildTaskMap();
      syncBoxChecks();
    }
    /* ★ 关节角：拖过的角度刷新后要还在。
       只在"关节数对得上"时套用 —— 存档可能属于另一条臂，
       硬套会把不相干的数塞进关节里（比归零更糟）。 */
    if(Array.isArray(c.q) && c.q.length === NJ){
      const lim = qLimits();
      Q = c.q.map((v, i) => Math.min(Math.max(+v || 0, lim[i][0]), lim[i][1]));
      CURVEPTS = fkAll(Q);
      if(typeof syncQReadout === 'function') syncQReadout();
    }
  }catch(e){ warnNoLS(); }
}
/* 把当前设置回填到控件上。★ 没有规则/尺寸的配置（rules:null）就跳过对应控件 */
function syncCfgInputs(){
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.value = v; };
  if (RULES && RULES.init) {
    set('inCx', RULES.init.x); set('inCy', RULES.init.y); set('inCz', RULES.init.z);
  }
  set('selMap', (MAP && MAP.taskMap === 'noSwap') ? 'side' : 'swap');
  if (MODEL) set('inYaw', MODEL.base.yaw);
}
/* 存档属于哪个配置？（没存过就 null）
   ★ 只有这一个定义（在"本地存档"那一节里）。这里原来还有一份重复的，
     两份各写各的键名，改了一处另一处不动 —— 典型的"改一处忘一处"。 */
/* 这份存档能套到当前配置上吗？
   ★ 判据是【结构】，不是【数值】。
     存档按 configId 分键（armBuilderModel:<id>），"它属于谁"由键保证；
     而杆长/限位本来就该跟配置不一样 —— 那正是要保存的东西。
     所以只拦结构性不符：
       · 段名对不上（配置源码里加了/删了/改了名的段）⇒ 旧存档套不上，别硬套
       · 关节数不一样 ⇒ 关节角向量长度对不上，套了会错位
     踩过：早先拿 canonModel 逐值比对，结果是"我改过的模型一律判成不属于本构型"，
     切走再切回来改动就没了 —— 正好把要保存的东西当成异常拦掉了。 */
function modelFitsConfig(m){
  if(!m || !Array.isArray(m.links)) return false;
  const cfg = (CONFIG && CONFIG.model) || null;
  if(!cfg || !Array.isArray(cfg.links)) return true;      // 没有配置可比 ⇒ 放行
  const name = L => (L && L.name) || '';
  if(m.links.length !== cfg.links.length) return false;
  for(let i = 0; i < cfg.links.length; i++){
    if(name(m.links[i]) !== name(cfg.links[i])) return false;
  }
  return true;
}
function setModel(m, save){
  if(!m || !Array.isArray(m.links)){ alert('模型格式不对'); return; }
  MODEL = m; Q = []; closeForm();
  /* ★ buildChain 是从"当前配置的臂列表"取模型的，所以手动换模型必须同步那一份，
     否则页面显示的是旧模型（配置层引入后新增的坑）。 */
  if(CONFIG) CONFIG.__arms = [m];
  document.getElementById('modal').classList.add('hidden');
  updateAll(); if(save !== false) saveLocal();
  refreshAnglesUI();                                        // 06-ui：刷新"旋转角度"滑条
  if(typeof renderVehicle === 'function') renderVehicle(); // 06-ui：刷新"装配参数"
}

/* ==================================================================
   三维显示
   ================================================================== */
let cam = {yaw:-0.9, pitch:0.42, zoom:1};
const FX=['#ff5f5f','#5fff8f','#5f9fff'], FXN=['X','Y','Z'];
function fitCanvas(c){ const r=c.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  c.width=Math.max(1,Math.round(r.width*dpr)); c.height=Math.max(1,Math.round(r.height*dpr));
  const g=c.getContext('2d'); g.setTransform(dpr,0,0,dpr,0,0); return {w:r.width,h:r.height}; }
function proj(p){ const a=cam.yaw,b=cam.pitch;
  const x1=p[0]*Math.cos(a)-p[1]*Math.sin(a), y1=p[0]*Math.sin(a)+p[1]*Math.cos(a), z1=p[2];
  return [x1, -y1*Math.sin(b)+z1*Math.cos(b), y1*Math.cos(b)+z1*Math.sin(b)]; }

function draw3D(){
  const c = document.getElementById('c3d'); if(!MODEL) return;
  const {w,h} = fitCanvas(c); const g = c.getContext('2d'); g.clearRect(0,0,w,h);
  const F = fkAll(Q);

  let pts = CLOUD.slice();
  if(!pts.length) pts = F.map(f=>f.t);
  let cx=0,cy=0,cz=0; for(const p of pts){cx+=p[0];cy+=p[1];cz+=p[2];}
  const n=Math.max(1,pts.length); cx/=n;cy/=n;cz/=n;
  let rad=1; for(const p of pts) rad=Math.max(rad, Math.hypot(p[0]-cx,p[1]-cy,p[2]-cz));
  if(rad<50) rad=300;
  const sc = Math.min(w,h)*0.42/rad*cam.zoom;
  const S = p => { const q=proj([p[0]-cx,p[1]-cy,p[2]-cz]); return [w/2+q[0]*sc, h/2-q[1]*sc, q[2]]; };

  const label = (txt,x,y,color,font)=>{ g.font=font||"11px system-ui,'Microsoft YaHei'";
    g.lineWidth=3; g.strokeStyle='rgba(10,14,24,.85)'; g.strokeText(txt,x,y);
    g.fillStyle=color; g.fillText(txt,x,y); };
  const triad = (o,R,len,lw,names,font)=>{ for(let a=0;a<3;a++){
      const d=[R[0][a],R[1][a],R[2][a]];
      const p0=S(o), p1=S([o[0]+d[0]*len,o[1]+d[1]*len,o[2]+d[2]*len]);
      g.strokeStyle=FX[a]; g.lineWidth=lw;
      g.beginPath(); g.moveTo(p0[0],p0[1]); g.lineTo(p1[0],p1[1]); g.stroke();
      if(names) label(FXN[a], p1[0]+3, p1[1]-3, FX[a], font); } };

  // 地面网格
  g.strokeStyle='rgba(255,255,255,.05)'; g.lineWidth=1;
  const E=Math.ceil(rad/200)*200;
  for(let v=-E;v<=E;v+=200){
    const a1=S([v+cx,-E+cy,-cz]),a2=S([v+cx,E+cy,-cz]);
    const b1=S([-E+cx,v+cy,-cz]),b2=S([E+cx,v+cy,-cz]);
    g.beginPath(); g.moveTo(a1[0],a1[1]); g.lineTo(a2[0],a2[1]);
    g.moveTo(b1[0],b1[1]); g.lineTo(b2[0],b2[1]); g.stroke();
  }
  // 底盘碰撞箱 / 长宽高框 / 限制球（画在点云与骨架【下层】，避免盖住臂）
  if(typeof drawSizeOverlay === 'function') drawSizeOverlay(g, S, label);
  // 点云
  if(CLOUD.length){
    const q=CLOUD.map(p=>S(p)).sort((A,B)=>B[2]-A[2]);
    g.globalAlpha=.75; g.fillStyle='#3ea6ff';
    for(const s of q){ g.beginPath(); g.arc(s[0],s[1],1.7,0,6.2832); g.fill(); }
    g.globalAlpha=1;
  }
  // 杆 = 相邻坐标系连线
  if(F.length>1){
    g.strokeStyle='#e8933a'; g.lineWidth=7; g.lineJoin='round'; g.lineCap='round';
    g.beginPath(); F.forEach((f,i)=>{const s=S(f.t); i?g.lineTo(s[0],s[1]):g.moveTo(s[0],s[1]);});
    g.stroke(); g.lineCap='butt';
  }
  // 各坐标系
  for(let i=1;i<F.length;i++){
    const f=F[i];
    const isO = Math.hypot(f.t[0],f.t[1],f.t[2])<1e-6 &&
                Math.abs(f.R[0][0]-1)<1e-9 && Math.abs(f.R[1][1]-1)<1e-9 && Math.abs(f.R[2][2]-1)<1e-9;
    if(SHOW_FRAMES && !isO) triad(f.t,f.R,95,1.8,true,"bold 12px system-ui,'Microsoft YaHei'");
    const s=S(f.t); label(f.name, s[0]+7, s[1]+15, '#c9d6ee', "11px system-ui,'Microsoft YaHei'");
  }
  // 原点坐标系（常显）
  triad([0,0,0], I3, 170, 2.6, true, "bold 12px system-ui,'Microsoft YaHei'");
  const so=S([0,0,0]); label('O 原点', so[0]+7, so[1]-7, '#ffffff', "bold 12px system-ui,'Microsoft YaHei'");
  // 关节轴（最后画、更长）
  for(let i=1;i<F.length;i++){
    const f=F[i]; if(!f.movable) continue;
    const nn=Math.hypot(...f.axis); if(nn<1e-12) continue;
    const d=mulv(f.R,[f.axis[0]/nn,f.axis[1]/nn,f.axis[2]/nn]);
    const L2=210, p0=S(f.t), p1=S([f.t[0]+d[0]*L2,f.t[1]+d[1]*L2,f.t[2]+d[2]*L2]);
    const hot=(f.qi===ACTIVE);
    g.strokeStyle=hot?'#ffe066':'#e879f9'; g.lineWidth=hot?4:3;
    g.beginPath(); g.moveTo(p0[0],p0[1]); g.lineTo(p1[0],p1[1]); g.stroke();
    const ang=Math.atan2(p1[1]-p0[1],p1[0]-p0[0]);
    g.beginPath(); g.moveTo(p1[0],p1[1]);
    g.lineTo(p1[0]-11*Math.cos(ang-.4),p1[1]-11*Math.sin(ang-.4));
    g.lineTo(p1[0]-11*Math.cos(ang+.4),p1[1]-11*Math.sin(ang+.4));
    g.closePath(); g.fillStyle=g.strokeStyle; g.fill();
    label(f.joint, p1[0]+5, p1[1]+13, g.strokeStyle, "bold 11px system-ui,'Microsoft YaHei'");
  }
  // 末端
  const ee=F[F.length-1].t, se=S(ee);
  g.fillStyle='#4ade80'; g.beginPath(); g.arc(se[0],se[1],5,0,6.2832); g.fill();

  if(typeof drawCoreOverlay === 'function') drawCoreOverlay(g, S, label);   // 科技核心示意
  if(typeof drawJointArcs === 'function') drawJointArcs(g, S, F, label);    // 各关节实际转角

  document.getElementById('camhint').textContent =
    'yaw '+Math.round(-cam.yaw/D2R)+'°  pitch '+Math.round(cam.pitch/D2R)+'°  zoom '+cam.zoom.toFixed(2);
  /* 链结构：单臂时就是原来那一行；多臂时每条臂各一行，用臂名开头 */
  const chainHTML = () => {
    const one = (a) => {
      const F = fkAllArm(Q, a);
      return F.map((f, i) => {
        if (i === 0) return a === 0 ? '<b>O(整机原点)</b>' : '';
        const c = CHAIN[ARMS[a].c0 + i - 1];
        const lim = c.movable ? (c.limits ? c.limits[0] + '~' + c.limits[1] + '°' : '不限') : '固定';
        return (i === 1 && a > 0 ? '' : ' → ') + '<b>' + f.name + '</b>' +
               '<span class="fx"> [' + c.joint + ': ' +
               (c.movable ? c.axLabel + ' ' + lim : 'fixed') + ']</span>';
      }).join('');
    };
    if (armCount() === 1) return '链：' + one(0);
    let h = '';
    for (let a = 0; a < armCount(); a++) {
      h += '<div style="margin-bottom:2px">' +
           '<b style="color:#8fd0ff">' + (ARMS[a].name || ('臂' + a)) + '</b>：' + one(a) + '</div>';
    }
    return h;
  };
  document.getElementById('chain').innerHTML = chainHTML();
}

/* ---------------- 交互 ---------------- */
window.addEventListener('resize', ()=>{ draw3D(); });
(function(){
  let drag=null; const c=document.getElementById('c3d');
  c.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY};e.preventDefault();});
  window.addEventListener('mouseup',()=>drag=null);
  window.addEventListener('mousemove',e=>{ if(!drag) return;
    cam.yaw -= (e.clientX-drag.x)*0.008;
    cam.pitch = Math.max(-1.45,Math.min(1.45,cam.pitch+(e.clientY-drag.y)*0.008));
    drag={x:e.clientX,y:e.clientY}; draw3D(); });
  c.addEventListener('wheel',e=>{ e.preventDefault();
    cam.zoom=Math.max(.25,Math.min(6,cam.zoom*(e.deltaY>0?.92:1.08))); draw3D(); },{passive:false});
})();
document.getElementById('cbFrames').onchange = e => { SHOW_FRAMES = e.target.checked; draw3D(); };
document.getElementById('selN').onchange = () => updateAll();
document.getElementById('btnAdd').onclick  = () => openForm();
document.getElementById('btnDemo').onclick = () => { if(confirm('载入示例模型？当前模型会被替换')) setModel(demoModel()); };
document.getElementById('btnSave').onclick = () => {
  const blob = new Blob([JSON.stringify(MODEL,null,2)],{type:'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = (MODEL.name||'arm').replace(/[\\/:*?"<>|]/g,'_') + '.json';
  a.click();
};
document.getElementById('btnLoad').onclick = () => document.getElementById('fileIn').click();
document.getElementById('fileIn').onchange = e => {
  const f=e.target.files[0]; if(!f) return;
  const r=new FileReader();
  r.onload = () => { try{ setModel(JSON.parse(r.result)); }catch(err){ alert('文件解析失败：'+err.message); } };
  r.readAsText(f,'utf-8'); e.target.value='';
};
/* 开始弹窗 */
document.getElementById('mDemo').onclick = () => setModel(demoModel());
document.getElementById('mLoad').onclick = () => document.getElementById('fileIn').click();
document.getElementById('mLast').onclick = () => {
  const m = loadLocal(); if(m) setModel(m); else alert('本地还没有保存过模型');
};

/* 自检钩子：#demo 直接载入示例；#dump 把浏览器算出来的各坐标系数据铺满屏幕（供无头截图核对） */
window.addEventListener('DOMContentLoaded', () => {
  /* ★ 先把配置装上 —— 模型 / 规则 / 车体尺寸全部来自配置，引擎不自带。
     存档里记着"上次用的是哪个配置"就把它装上（原来固定回默认配置，
     表现是"换了构型、重开又变回去了"）。 */
  const wantCfg = savedConfigId();
  applyConfig(wantCfg && CONFIGS[wantCfg] ? wantCfg : DEFAULT_CONFIG_ID);
  migrateOldArchive();                       // 旧版那份不带后缀的存档，认领到当前构型名下
  initConfigSelector();

  loadCfg();                                 // 先恢复"设置"（位置/映射/采样数/开关）
  /* ★ 顺序要紧：模型在【后】，关节角在【最后】。
     两个坑叠在一起，都踩过：
       ① setModel() 会把 Q 清空（换了模型，旧角度没意义）⇒ 角度必须最后恢复；
       ② setModel() 内部还会 saveLocal() 一次 —— 那一下写的是"角度还没恢复"的中间状态，
          会把存档里的关节角覆盖成 0。所以这段必须先用 holdSave() 把落盘压住，
          等关节角恢复完再统一写一次。 */
  const last = loadLocal();                  // ★ 读的是【当前构型自己那份】存档
  document.getElementById('mLast').style.display = last ? '' : 'none';
  const h = location.hash;
  const fits = modelFitsConfig(last);        // 存档的模型对不对得上当前配置
  holdSave(true);
  try{
    if(h.indexOf('demosave') >= 0){          // 载入示例并【存档】（等同于在弹窗里点"使用示例"）
      setModel(demoModel());
    } else if(h.indexOf('demo') >= 0 || h.indexOf('dump') >= 0 || h.indexOf('anim') >= 0){
      setModel(demoModel(), false);          // 不改写本机保存的模型
      if(h.indexOf('dump') >= 0) doDump();
    } else if(fits){
      setModel(last);                        // ★ 有存档就直接进去，不再每次弹窗
    } else if(last){
      /* 存档对不上当前构型：不套上去，但也不删 */
      const t = document.getElementById('tools');
      if(t && !document.getElementById('cfgWarn')){
        const s = document.createElement('span');
        s.id = 'cfgWarn'; s.className = 'chip';
        s.style.cssText = 'border-color:#7a5a20;color:#ffb454';
        s.textContent = '⚠ 本地存档对不上当前构型，已按配置载入';
        t.insertBefore(s, t.firstChild);
      }
    }
    if(fits) loadCfg({ qOnly: true });       // 模型定了，再恢复关节角
  }finally{
    holdSave(false);
  }
  saveLocal();                               // 状态齐了，落一次盘（顶栏"已保存"也跟着亮）
  syncCfgInputs();
  if(typeof renderAngles === 'function') renderAngles();
  if(typeof renderVehicle === 'function') renderVehicle();
  if(typeof markSaved === 'function') markSaved(true);   // 顶栏"已保存"的状态（不闪）
});
function doDump(){
  const F = fkAll(Q);
  const lines = ['MODEL = ' + MODEL.name, 'q(deg) = ' + Q.join(', '), ''];
  for(const f of F){
    const ax = v => mulv(f.R, v).map(x => (x>=0?' ':'') + x.toFixed(5)).join('  ');
    lines.push(f.name + '  (joint: ' + (f.joint||'-') + ')');
    lines.push('  o = ' + f.t.map(x => (x>=0?' ':'') + x.toFixed(2)).join('  '));
    lines.push('  X = ' + ax([1,0,0]));
    lines.push('  Y = ' + ax([0,1,0]));
    lines.push('  Z = ' + ax([0,0,1]));
    lines.push('');
  }
  document.body.innerHTML = '<pre style="margin:0;padding:16px;background:#0f1420;color:#e6ecf7;'+
    'font:15px/1.5 Consolas,monospace;white-space:pre">' + lines.join('\n') + '</pre>';
}