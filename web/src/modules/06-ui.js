/* ==================================================================
   06-ui.js —— 界面骨架：折叠面板 / 勾选下拉 / 旋转角度表 / 分隔条

   为什么单独一个模块：
     01~03 是"算"的（运动学 / 任务域 / 底盘尺寸），04~05 是配置层。
     这里是纯粹的"摆界面"，改版时基本只动这个文件，不碰算法。

   对外只暴露两个函数给其它模块用：
     renderAngles()   重画"旋转角度"表（updateAll / 拖动关节时调）
     syncVisMenu()    把 SHOWBOX / SHOW_FRAMES 的状态回填到勾选下拉
   ==================================================================================== */
'use strict';

/* ---------------- 可折叠分区 ----------------
   三个分区（模型链 / 旋转角度 / 可达域）各自有 .sect-hd + .sect-bd。
   状态存 localStorage：这是纯界面偏好，不属于模型。 */
const SECTS = ['List', 'Q', 'Veh', 'Data'];
let SECT_OPEN = { List: true, Q: true, Veh: true, Data: true };
/* 旋转角度是唯一"给足高度"的块：它的滑条要能一次看全、方便反复拖。
   其余几块都是长内容，各占一点就行 —— 想细看就折叠别的、把它撑开。 */
const SECT_ROOMY = { Q: 340 };

function setSectOpen(key, open, save) {
  SECT_OPEN[key] = !!open;
  const hd = document.getElementById('hd' + key);
  const bd = document.getElementById('bd' + key);
  if (hd) hd.classList.toggle('collapsed', !open);
  if (bd) {
    bd.classList.toggle('hidden', !open);
    /* 旋转角度是唯一"按内容给高度"的块：展开时把它放开，
       这样它的滑条能一次看全（其余几块是长内容，靠滚动）。 */
    if (SECT_ROOMY[key]) bd.style.maxHeight = open ? SECT_ROOMY[key] + 'px' : '';
  }
  syncSplitHandles();
  if (save !== false) saveSectPrefs();
}
function saveSectPrefs() {
  try { localStorage.setItem('armBuilderSect', JSON.stringify(SECT_OPEN)); } catch (e) {}
}
function loadSectPrefs() {
  try {
    const s = localStorage.getItem('armBuilderSect');
    if (s) { const o = JSON.parse(s); SECTS.forEach(k => { if (typeof o[k] === 'boolean') SECT_OPEN[k] = o[k]; }); }
  } catch (e) {}
  SECTS.forEach(k => setSectOpen(k, SECT_OPEN[k], false));
}
function initCollapse() {
  SECTS.forEach(k => {
    const hd = document.getElementById('hd' + k);
    if (!hd) return;
    hd.addEventListener('click', e => {
      /* 点标题里的按钮（比如 ＋建立坐标系）不该折叠 */
      if (e.target.closest('button')) return;
      setSectOpen(k, !SECT_OPEN[k]);
    });
  });
  loadSectPrefs();
}

/* ---------------- 分隔条 ----------------
   只有【同时展开】的两个分区之间那条才有用。
   #split  在 模型链 / 旋转角度 之间
   #split2 在 旋转角度 / 可达域 之间
   拖动改的是上下两块的 flex 比例（不写死 px），窗口改高时比例自动跟着走。 */
let SPLIT_PCT = null;
const SPLIT_DEF = .5;
function setSplitPct(p) {
  p = Math.max(.16, Math.min(.84, p));
  const a = document.getElementById('bdList'), b = document.getElementById('bdData');
  if (a) a.style.flex = '1 1 ' + (p * 100).toFixed(2) + '%';
  if (b) b.style.flex = '1 1 ' + ((1 - p) * 100).toFixed(2) + '%';
  SPLIT_PCT = p;
}function applySplit() { setSplitPct(SPLIT_PCT == null ? SPLIT_DEF : SPLIT_PCT); }
function syncSplitHandles() {
  const o = k => SECT_OPEN[k];
  const show = (id, on) => { const el = document.getElementById(id); if (el) el.style.display = on ? '' : 'none'; };
  /* 一条分隔条在它上下相邻的"两块里至少有一块展开"时才有意义 */
  show('split',  o('List') && (o('Q') || o('Veh') || o('Data')));
  show('split2', (o('List') || o('Q')) && (o('Veh') || o('Data')));
  show('split3', (o('List') || o('Q') || o('Veh')) && o('Data'));
}function initSplit() {
  const sp = document.getElementById('split');
  if (!sp) return;
  const a = document.getElementById('bdList'), b = document.getElementById('bdData');
  let drag = null;
  sp.addEventListener('mousedown', e => {
    if (!a || !b) return;
    const ra = a.getBoundingClientRect(), rb = b.getBoundingClientRect();
    drag = { y: e.clientY, h: ra.height, tot: ra.height + rb.height };
    sp.classList.add('on'); document.body.style.cursor = 'ns-resize';
    e.preventDefault();
  });
  window.addEventListener('mousemove', e => {
    if (!drag) return;
    setSplitPct((drag.h + (e.clientY - drag.y)) / Math.max(1, drag.tot));
  });
  window.addEventListener('mouseup', () => {
    if (!drag) return;
    drag = null; sp.classList.remove('on'); document.body.style.cursor = '';
    saveLocal();
  });
  sp.addEventListener('dblclick', () => { SPLIT_PCT = SPLIT_DEF; applySplit(); saveLocal(); });
  applySplit();
}

/* ---------------- 勾选下拉（坐标架 / 底盘框 / 长宽高框 / 限制球）----------------
   把原来顶栏那四个独立勾选框收进一个菜单，省地方。
   底下的 DOM 元素（cbFrames/cbChassis/cbLWH/cbBall）保持不变 ——
   03 模块里那套 SHOWBOX 联动逻辑一行都不用改。 */
function syncVisMenu() {
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.checked = !!v; };
  set('cbFrames', SHOW_FRAMES);
  set('cbChassis', SHOWBOX.chassis !== false);
  set('cbLWH', SHOWBOX.lwh !== false);
  set('cbBall', SHOWBOX.ball !== false);
}
function initVisMenu() {
  const btn = document.getElementById('btnVis');
  const pop = document.getElementById('visMenu');
  if (!btn || !pop) return;
  /* 菜单贴到按钮正下方 */
  const place = () => {
    const r = btn.getBoundingClientRect();
    pop.style.left = Math.round(r.left) + 'px';
    pop.style.top = Math.round(r.bottom + 4) + 'px';
  };
  btn.addEventListener('click', e => {
    e.stopPropagation();
    if (pop.classList.contains('on')) { pop.classList.remove('on'); return; }
    place(); pop.classList.add('on');
  });
  /* 点别处 / 滚动就收起来 */
  document.addEventListener('click', e => {
    if (!pop.contains(e.target) && e.target !== btn) pop.classList.remove('on');
  });
  window.addEventListener('resize', () => { if (pop.classList.contains('on')) place(); });
  window.addEventListener('scroll', () => pop.classList.remove('on'), true);
  syncVisMenu();
}

/* ---------------- 旋转角度：每个关节一条滑条 ----------------
   用户要的是"能手动拉条看不同角度下的情况"，不只是看数字。
   所以这里每个关节一行：
       [关节名 轴向]  [滑条（范围=该关节限位）]  [数字框]  [↺]
   拖动 => 改 Q[i] => 重算正解 + 可达集 + 数据面板，三维视图跟着动。
   范围就是该关节的限位；不限位的关节给 ±180°。

   性能：滑条 input 事件很密，所以用 rAF 合并 —— 一帧最多算一次，
   拖动不会把主线程堵死。 */
let Q_RAF = null;
/* ---------------- 关节角度的"网格" ----------------
   滑条只能取 min + k·step 这类网格上的值，而"我们算出来的网格点"和
   "浏览器自己吸附出来的网格点"一旦差一格，就会出现
   "引擎里 77、滑条上 76.9"这种两边对不上的状态（踩过）。
   ★ 所以网格【由我们自己定】，并且只此一处：
       步长 = 1°（度数上最自然，也和数字框的 step=1 一致）
       网格点 = lo 起、每 1° 一个，最后补上 hi
     写 value 之前先吸附到它；拖回来的值也吸附到它。
   这样引擎、滑条、数字框、读数永远是同一个数。 */
function angleGrid(lo, hi){
  const step = 1;
  const n = Math.max(0, Math.ceil((hi - lo) / step - 1e-9));
  return { lo: lo, hi: hi, step: step, n: n };
}
function snapToGrid(v, g){
  if (!g) return +v || 0;
  if (!(v >= g.lo)) v = g.lo;                     // NaN 也走这条
  if (v > g.hi) v = g.hi;
  return Math.min(g.hi, Math.round(g.lo + Math.round((v - g.lo) / g.step) * g.step));
}
/* 第 i 个关节的网格（找不到面板元素时按限位现算一个） */
function jointGrid(i, lo, hi){
  const el = document.getElementById('qsl' + i);
  if (el) {
    const step = +el.step;
    if (isFinite(step) && step > 0) {
      return { lo: +el.min, hi: +el.max, step: step,
               n: Math.max(0, Math.ceil((+el.max - +el.min) / step - 1e-9)) };
    }
  }
  return angleGrid(lo, hi);
}
function onChangeJoint(i, v) {
  /* ★ 以"网格上的那个点"为准：传进来的值先吸附到网格，再写回控件与引擎。
     不这么做的话，控件会按自己的规则再吸附一次，两边就差一格。 */
  const lim = (typeof qLimits === 'function') ? qLimits()[i] : null;
  let val = +v;
  if (lim) val = snapToGrid(val, jointGrid(i, lim[0], lim[1]));
  Q[i] = val;
  const num = document.getElementById('qnum' + i);
  if (num && +num.value !== val) num.value = val.toFixed(1);
  const sl0 = document.getElementById('qsl' + i);
  if (sl0 && +sl0.value !== val) sl0.value = val;
  if (Q_RAF) return;
  Q_RAF = requestAnimationFrame(() => {
    Q_RAF = null;
    refreshAfterJoint();
  });
}
function refreshAfterJoint() {
  if (typeof CURVEPTS !== 'undefined') CURVEPTS = fkAll(Q);
  if (typeof draw3D === 'function') draw3D();
  if (typeof analyzeTask === 'function') analyzeTask();
  if (typeof syncQReadout === 'function') syncQReadout();
  if (typeof saveLocal === 'function') saveLocal();
}

/* 只更新表里的数字，不重建 DOM —— 拖动时不能把正在拖的元素换掉 */
function syncQReadout() {
  const nA = (typeof armCount === 'function') ? armCount() : 1;
  let flat = 0;
  for (let a = 0; a < nA; a++) {
    const A = (typeof ARMS !== 'undefined' && ARMS[a]) ? ARMS[a] : null;
    if (!A) continue;
    for (let ci = A.c0; ci < A.cN; ci++) {
      if (!CHAIN[ci].movable) continue;
      const el = document.getElementById('qv' + flat);
      if (el) el.textContent = (Q[flat] === undefined ? '-' : (+Q[flat]).toFixed(1)) + '°';
      flat++;
    }
  }
}

function renderAngles() {
  const box = document.getElementById('qbody');
  if (!box) return;
  if (!CHAIN || !CHAIN.length) {
    box.innerHTML = '<div style="color:#5d6b88;padding:4px 0">还没有模型</div>';
    return;
  }
  const nA = (typeof armCount === 'function') ? armCount() : 1;
  const rows = [];
  let flat = 0;
  for (let a = 0; a < nA; a++) {
    const A = (typeof ARMS !== 'undefined' && ARMS[a]) ? ARMS[a] : null;
    if (!A) continue;
    if (nA > 1) {
      rows.push('<div class="qarm">' + (A.name || ('臂' + a)) + '</div>');
    }
    for (let ci = A.c0; ci < A.cN; ci++) {
      const c = CHAIN[ci];
      if (!c.movable) continue;
      const i = flat;
      const lo = c.limits ? Math.min(c.limits[0], c.limits[1]) : -180;
      const hi = c.limits ? Math.max(c.limits[0], c.limits[1]) : 180;
      /* ★ 步长固定 1°（与数字框的 step=1 一致）。
         原来用 (hi−lo)/200（joint1 = 0.61°、joint2 = 1.45°），网格是"歪"的：
         写一个不在网格上的 value 会被浏览器自己吸附，和引擎里的值差一格。
         1° 这个网格是自洽的，且 200 个刻度在 150px 宽的条上够用。 */
      const grid = angleGrid(lo, hi);
      const stepv = grid.step;
      const val = snapToGrid((Q[i] === undefined) ? 0 : Q[i], grid);
      const limTxt = c.limits ? (lo + ' ~ ' + hi + '°') : '不限（±180°）';
      rows.push(
        '<div class="qrow" title="限位 ' + limTxt + '">' +
          '<div class="qhd"><span class="qnm">' + c.joint +
            '</span><span class="qax">' + (c.axLabel || '') + '</span>' +
            '<span class="qv" id="qv' + i + '">' + val.toFixed(1) + '°</span></div>' +
          '<div class="qs"><input type="range" id="qsl' + i + '" min="' + lo + '" max="' + hi +
            '" step="' + stepv + '" value="' + val + '" title="限位 ' + limTxt + '">' +
            '<input type="number" id="qnum' + i + '" min="' + lo + '" max="' + hi +
            '" step="1" value="' + val.toFixed(1) + '" title="直接输入角度">' +
            '<button class="qrs" data-qrs="' + i + '" title="回到零位">↺</button></div>' +
        '</div>');
      flat++;
    }
  }
  if (!flat) { box.innerHTML = '<div style="color:#5d6b88;padding:4px 0">这条链没有可动关节</div>'; return; }
  box.innerHTML = rows.join('');
  wireAngleControls();
}
function wireAngleControls() {
  const box = document.getElementById('qbody');
  if (!box || box.dataset.wired === '1') { /* 每次重建都要重接 */ }
  box.querySelectorAll('input[type=range]').forEach(el => {
    el.addEventListener('input', () => onChangeJoint(+el.id.slice(3), +el.value));
  });
  box.querySelectorAll('input[type=number]').forEach(el => {
    el.addEventListener('change', () => onChangeJoint(+el.id.slice(4), +el.value || 0));
  });
  box.querySelectorAll('[data-qrs]').forEach(el => {
    el.addEventListener('click', () => onChangeJoint(+el.dataset.qrs, 0));
  });
}

/* ---------------- 全部关节归零 ----------------
   对应原来「🎛 关节」面板里的"重置" */
function resetAllJoints() {
  for (let i = 0; i < NJ; i++) Q[i] = 0;
  const lim = qLimits();
  for (let i = 0; i < NJ; i++) Q[i] = Math.min(Math.max(0, lim[i][0]), lim[i][1]);
  renderAngles();
  refreshAfterJoint();
}

/* ---------------- 装配位置 / 夹爪轴（进 JSON） ----------------
   这两项不是几何，但换了就必须一起说清楚，否则同一份臂型的结论会变。
   所以作为 MODEL.vehicle 存进模型 JSON，随「保存 JSON / 读取文件」一起走。

   向后兼容：老 JSON 没有 vehicle 字段时，从 RULES.init 兜一个默认值，
   并在第一次保存时补上。 */
const AXIS6 = ['+X', '-X', '+Y', '-Y', '+Z', '-Z'];

function vehicleOf(m) {
  const mm = m || MODEL;
  if (mm && mm.vehicle) return mm.vehicle;
  /* 兜底：从规则侧的装配位初始化（存在但没写进模型） */
  const init = (typeof RULES !== 'undefined' && RULES && RULES.init) ? RULES.init : null;
  return {
    asm: init ? { x: init.x, y: init.y, z: init.z } : { x: 0, y: 0, z: 0 },
    insertAxis: (RULES && RULES.act && RULES.act.insertAxis) || 'y',
    gripAxis: (RULES && RULES.act && RULES.act.gripAxis) || 'x'
  };
}
function setVehicle(patch) {
  if (!MODEL) return;
  const v = vehicleOf();
  MODEL.vehicle = v;
  if (patch && patch.asm) v.asm = patch.asm;
  if (patch && patch.insertAxis) v.insertAxis = patch.insertAxis;
  if (patch && patch.gripAxis) v.gripAxis = patch.gripAxis;
  /* 同步给规则侧（任务域换算要用装配位） */
  if (typeof RULES !== 'undefined' && RULES && RULES.init && v.asm) {
    RULES.init.x = +v.asm.x || 0; RULES.init.y = +v.asm.y || 0; RULES.init.z = +v.asm.z || 0;
  }
  if (typeof RULES !== 'undefined' && RULES && RULES.act) {
    RULES.act.insertAxis = v.insertAxis;
    RULES.act.gripAxis = v.gripAxis;
  }
  const set = (id, val) => { const el = document.getElementById(id); if (el) el.value = val; };
  if (v.asm) { set('inCx', v.asm.x); set('inCy', v.asm.y); set('inCz', v.asm.z); }
}

function renderVehicle() {
  const box = document.getElementById('vehBody');
  if (!box) return;
  if (!MODEL) { box.innerHTML = '<div style="color:#5d6b88">还没有模型</div>'; return; }
  const v = vehicleOf();
  MODEL.vehicle = v;                      // 认下来，保存时就会带上
  const num = (id, val, stepv) =>
    '<input type="number" id="' + id + '" step="' + stepv + '" value="' + (val || 0) +
    '" style="width:62px">';
  box.innerHTML =
    '<div class="vehrow"><span class="k1">装配臂位置</span>' +
      num('vhX', v.asm.x, 10) + num('vhY', v.asm.y, 10) + num('vhZ', v.asm.z, 10) +
      '<span class="u">mm（世界系 OXYZ）</span></div>' +
    '<div class="vehrow"><span class="k1">夹爪插入轴</span>' +
      '<select id="vhIns">' + AXIS6.map(a =>
        '<option ' + (v.insertAxis === a.toLowerCase().replace('+', '') ? 'selected' : '') +
        '>' + a + '</option>').join('') + '</select>' +
      '<span class="k1" style="margin-left:10px">夹紧轴</span>' +
      '<select id="vhGrip">' + ['x', 'y', 'z'].map(a =>
        '<option ' + (v.gripAxis === a ? 'selected' : '') + '>' + a.toUpperCase() + '</option>').join('') +
      '</select></div>' +
    '<div class="vehnote">这两项随「保存 JSON」一起存，读取时自动带回来</div>';
  wireVehicle();
}
function wireVehicle() {
  const g = id => document.getElementById(id);
  const push = () => setVehicle({ asm: {
    x: +g('vhX').value || 0, y: +g('vhY').value || 0, z: +g('vhZ').value || 0 } });
  ['vhX', 'vhY', 'vhZ'].forEach(id => { const el = g(id); if (el) el.onchange = push; });
  if (g('vhIns')) g('vhIns').onchange = e =>
    setVehicle({ insertAxis: e.target.value.toLowerCase().replace('+', '') });
  if (g('vhGrip')) g('vhGrip').onchange = e =>
    setVehicle({ gripAxis: e.target.value.toLowerCase() });
}
function initVehicle() {
  const b = document.getElementById('btnQReset');
  if (b) b.onclick = e => { e.stopPropagation(); resetAllJoints(); };
}

/* ---------------- 「已保存」提示 ----------------
   每次 saveLocal() 都会调这里。为什么要有它：用户问过"到底有没有实时保存" ——
   看不见的机制等于不存在（这次报的第二个问题就是这么来的）。
   行为：
     · 第一次（载入存档时）只把状态写出来，不闪
     · 之后每次改动闪一下并显示时间
   元素是懒创建的，模板里不用留位置；样式在 .chip 那条规则旁边。 */
let SAVED_HIDE = null;
function markSaved(quiet) {
  const t = document.getElementById('tools');
  if (!t) return;
  let el = document.getElementById('savedChip');
  if (!el) {
    el = document.createElement('span');
    el.id = 'savedChip'; el.className = 'chip';
    el.title = '模型、关节角与设置都存在浏览器本地；刷新/重开还在。' +
               '换构型时各存各的，互不覆盖。「保存 JSON」是另存成文件。';
    t.insertBefore(el, t.firstChild);
  }
  const d = new Date();
  const hm = ('0' + d.getHours()).slice(-2) + ':' + ('0' + d.getMinutes()).slice(-2) +
             ':' + ('0' + d.getSeconds()).slice(-2);
  el.textContent = '🕒 已保存 ' + hm;
  if (quiet) return;
  el.classList.add('flash');
  if (SAVED_HIDE) clearTimeout(SAVED_HIDE);
  SAVED_HIDE = setTimeout(() => el.classList.remove('flash'), 700);
}

/* ---------------- 接线 ---------------- */
initCollapse();
initSplit();
initVisMenu();
initVehicle();
