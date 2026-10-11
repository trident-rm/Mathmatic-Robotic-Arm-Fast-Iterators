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

/* ---------------- 底盘碰撞箱 / 伸展尺寸：规则侧状态 ----------------
   ★ 默认值来自配置（CONFIG.chassis），引擎不再自带同济的车体尺寸。
     没配置时这些对象为空，尺寸校核那块界面会自己让开。
   CHASSIS.pos 默认 base 系；zDir=−1 ⇒ 从中心点往 −Z 长 size[2]
   ⇒ 中心点就是碰撞箱【上表面中心】，底面中心 = 接地中心                        */
let CHASSIS = { posMode:'base', pos:[0,0,0], size:[1,1,1], zDir:-1 };
let SIZER   = { lwh:[0,0,0], ballD:0, ballH:0 };
let SHOWBOX = { chassis:true, lwh:true, ball:true };
let PANEL_REST = '';            // 数据面板里除"底盘/尺寸"段以外的 HTML（改底盘参数时只重画尺寸段）

function buildChain(){
  CHAIN = []; NJ = 0;
  let R = rotZdeg(MODEL.base.yaw), t = MODEL.base.xyz.slice();   // 父系 = 整机原点系
  MODEL.links.forEach((L, i) => {
    let off;
    if(L.posMode === 'origin' && i > 0){
      // 在整机原点系里给的绝对位置 → 换算成相对父系的偏移
      off = mulv(T3(R), subv(L.pos, t));
    } else {
      off = L.pos.slice();
    }
    const Rf = frameRot(L.axes);
    const movable = L.type !== 'fixed';
    CHAIN.push({name:L.name, joint:L.joint, type:L.type, movable, offset:off, Rf,
                axis:axisVec(L.axis), axLabel:L.axis,
                limits:L.limits ? L.limits.slice() : null});
    if(movable) NJ++;
    t = addv(mulv(R, off), t);
    R = mul3(R, Rf);
  });
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

/* ---------------- 正解 ---------------- */
function fkAll(q){
  let R = rotZdeg(MODEL.base.yaw), t = MODEL.base.xyz.slice();
  const out = [{name:'O(整机原点)', joint:'', R:R, t:t, axis:[0,0,0], movable:false, qi:-1}];
  let qi = 0;
  for(const c of CHAIN){
    let Rj = I3;
    if(c.movable){ Rj = rotAxis(c.axis, (q[qi]||0)*D2R); qi++; }
    const tNew = addv(mulv(R, c.offset), t);   // 用【更新前】的 R
    R = mul3(mul3(R, c.Rf), Rj);
    t = tNew;
    out.push({name:c.name, joint:c.joint, R:R, t:t, axis:c.axis, movable:c.movable, qi:qi-1});
  }
  return out;
}

/* ---------------- 采样 + 体素化（全部在浏览器里算） ---------------- */
function computeFeasible(){
  const N = parseInt(document.getElementById('selN').value, 10) || 30000;
  const lim = qLimits();
  const q = new Array(NJ);
  const pts = new Array(N);
  for(let s=0;s<N;s++){
    for(let i=0;i<NJ;i++){ const [lo,hi]=lim[i]; q[i] = lo + Math.random()*(hi-lo); }
    const F = fkAll(q);
    pts[s] = F[F.length-1].t.slice();
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
  CLOUD = []; for(let i=0;i<N;i+=step) CLOUD.push(pts[i]);
  VOX = keys;                       // 可达集体素（供任务域判定用）
  STATS = {n:N, voxels:keys.size, rmin, rmax, rhomin, rhomax, zmin, zmax, nJoint:NJ};
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
    if(confirm(msg)){ MODEL.links.splice(i); closeForm(); updateAll(); saveLocal(); }
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
  closeForm(); updateAll(); saveLocal();
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
function cfgObj(){                       // 规则侧设置的快照（模型之外的一切）
  return { init: RULES ? RULES.init : null,
           taskMap: MAP ? MAP.taskMap : null,      // ★ 任务系映射（原来是布尔 swap）
           baseH: RULES ? RULES.baseH : null,
           nSample: document.getElementById('selN').value,
           yaw: MODEL ? MODEL.base.yaw : -90,
           chassis: CHASSIS, chassisV: CHASSIS_V,      // ★ 底盘箱（带默认值版本号）
           sizer: SIZER, showbox: SHOWBOX,             // ★ 伸展尺寸 / 显示开关
           splitPct: SPLIT_PCT,                        // ★ 右栏上下分隔条位置
           configId: (typeof activeConfigId === 'function') ? activeConfigId() : null };  // ★ 当前配置
}
function saveLocal(){
  try{
    localStorage.setItem('armBuilderModel', JSON.stringify(MODEL));
    // ★ 规则侧设置也要存：装配臂位置 / 映射 / 底盘高 / 采样数 / 底盘箱 / 伸展尺寸
    localStorage.setItem('armBuilderCfg', JSON.stringify(cfgObj()));
  }catch(e){ warnNoLS(); }
}
function loadLocal(){
  try{ const s = localStorage.getItem('armBuilderModel'); return s?JSON.parse(s):null; }
  catch(e){ warnNoLS(); return null; }
}
/* 恢复上次的规则侧设置（否则刷新一下就回到默认） */
function loadCfg(){
  try{
    const s = localStorage.getItem('armBuilderCfg'); if(!s) return;
    const c = JSON.parse(s);

    /* ★ 存档属于哪个配置？不属于当前配置就整段忽略 ——
       否则拿同济的存档去套新构型，会得到看着正常其实全错的数 */
    if(c.configId && typeof activeConfigId === 'function' && c.configId !== activeConfigId()){
      return;
    }
    if(c.init && RULES) Object.assign(RULES.init, c.init);
    if(c.taskMap && MAP) MAP.taskMap = c.taskMap;        // ★ 原 swap:true/false
    else if(typeof c.swap === 'boolean' && MAP) MAP.taskMap = c.swap ? 'swap' : 'noSwap';
    if(c.baseH && RULES) RULES.baseH = c.baseH;
    if(c.nSample) document.getElementById('selN').value = c.nSample;
    /* ★ 底盘箱要带版本号：改动 CHASSIS 默认值后，老存档不能把新默认值盖回去 */
    if(c.chassis && c.chassisV === CHASSIS_V) Object.assign(CHASSIS, c.chassis);
    if(c.sizer)   Object.assign(SIZER,   c.sizer);
    if(c.showbox) Object.assign(SHOWBOX, c.showbox);
    if(typeof c.splitPct === 'number'){ SPLIT_PCT = c.splitPct; applySplit(); }
    /* 映射可能被存档改过 ⇒ 重建换算对象（实现只有一份，见 04-config.js） */
    if(typeof rebuildTaskMap === 'function' && MAP) rebuildTaskMap();
    syncBoxChecks();
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
function setModel(m, save){
  if(!m || !Array.isArray(m.links)){ alert('模型格式不对'); return; }
  MODEL = m; Q = []; closeForm();
  document.getElementById('modal').classList.add('hidden');
  updateAll(); if(save !== false) saveLocal();
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
  document.getElementById('chain').innerHTML = '链：' + F.map((f,i)=>{
    if(i===0) return '<b>O(整机原点)</b>';
    const c=CHAIN[i-1];
    const lim = c.movable ? (c.limits ? c.limits[0]+'~'+c.limits[1]+'°' : '不限') : '固定';
    return ' → <b>'+f.name+'</b><span class="fx"> ['+c.joint+': '+ (c.movable?c.axLabel+' '+lim : 'fixed') +']</span>';
  }).join('');
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
  /* ★ 先把配置装上 —— 模型 / 规则 / 车体尺寸全部来自配置，引擎不自带 */
  let cfgId = null;
  try{ const s = localStorage.getItem('armBuilderCfg'); if(s) cfgId = JSON.parse(s).configId; }catch(e){}
  applyConfig(cfgId && CONFIGS[cfgId] ? cfgId : DEFAULT_CONFIG_ID);
  initConfigSelector();

  loadCfg();                                 // 再恢复上次的设置（位置/映射/采样数）
  const last = loadLocal();
  document.getElementById('mLast').style.display = last ? '' : 'none';
  const h = location.hash;
  if(h.indexOf('demosave') >= 0){            // 载入示例并【存档】（等同于在弹窗里点"使用示例"）
    setModel(demoModel());
  } else if(h.indexOf('demo') >= 0 || h.indexOf('dump') >= 0 || h.indexOf('anim') >= 0){
    setModel(demoModel(), false);            // 不改写本机保存的模型
    if(h.indexOf('dump') >= 0) doDump();
  } else if(last){
    setModel(last);                          // ★ 有存档就直接进去，不再每次弹窗
  }
  syncCfgInputs();
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