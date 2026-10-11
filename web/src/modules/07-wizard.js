/* ==================================================================
   07-wizard.js —— 新建向导

   「新建」按钮的流程（按用户规范）：
     ① 单臂 / 双臂
     ② base_yaw（双臂时每条臂各一个）
     ③ 依次搭建每个 joint 与 link
     ④ 装配臂相对世界坐标系原点的 xyz
     ⑤ 夹爪往哪个轴插入（可选）

   设计取舍：
   · 向导只产出【一份新的 MODEL/ARMS】，不改引擎。所以它本质是个"表单 -> 数据"的转换器。
   · 每一步先校验再放行：② 要有 yaw；③ 至少要有一段。
   · 第 ④ 步写进"装配臂位置"（任务域侧，cfg.init / 顶栏那三个输入框）；
     第 ⑤ 步写进 CONFIG 的夹爪轴约定（RULES.act.insertAxis / gripAxis）。
     两者都不是几何量，所以不进 links。

   ★ 为什么装配位置与夹爪轴放在这里问：
     规则手册的位姿范围是相对世界坐标系(OXYZ)的，而臂装在车上；
     不知道装在哪、往哪个轴插，任务域校核就没法做。
==================================================================================== */
'use strict';

/* ---------------- 草稿 ---------------- */
const WIZ = {
  on: false, step: 0,
  nArm: 1,                       // 1 或 2
  arms: [],                      // [{name, yaw, links:[{...}]}]
  asm: { x: -500, y: -200, z: 800 },   // 装配臂位置（世界系 mm）
  insertAxis: 'y',               // 夹爪往哪个轴插入
  gripAxis: 'x'                  // 夹爪自身沿哪个轴夹紧
};

const WIZ_STEPS = ['臂数', '安装基准', '关节与连杆', '装配位置', '夹爪轴'];
const AXIS_OPTS = ['+X', '-X', '+Y', '-Y', '+Z', '-Z'];
const ID_AXES = { x: '+X', y: '+Y', z: '+Z' };

function wizDefaultLinks(n) {
  const out = [];
  for (let i = 1; i <= n; i++) {
    out.push({ name: 'link' + i, joint: 'joint' + i,
               pos: [0, 0, i === 1 ? 100 : 150], axis: '+X',
               axes: { x: '+X', y: '+Y', z: '+Z' },      // 新坐标系三轴对齐父系的哪个轴
               rot: [], limits: [-90, 90], movable: true });
  }
  return out;
}
function wizReset() {
  WIZ.step = 0;
  WIZ.nArm = 1;
  WIZ.arms = [{ name: '臂1', yaw: -90, links: wizDefaultLinks(3) }];
  WIZ.asm = { x: -500, y: -200, z: 800 };
  WIZ.insertAxis = 'y';
  WIZ.gripAxis = 'x';
}
function wizEnsureArms() {
  while (WIZ.arms.length < WIZ.nArm) {
    WIZ.arms.push({ name: '臂' + (WIZ.arms.length + 1), yaw: -90, links: wizDefaultLinks(3) });
  }
  WIZ.arms.length = WIZ.nArm;
}

/* ---------------- 草稿 -> 臂模型 ---------------- */
function wizBuildArm(a) {
  const links = [{
    name: 'base_link', joint: 'base_joint', type: 'revolute', posMode: 'origin',
    pos: [0, 0, 0], axes: ID_AXES, axis: '+Z', limits: null
  }];
  a.links.forEach(L => {
    const nm = (L.name || '').trim() || 'link';
    links.push({
      name: nm, joint: (L.joint || '').trim() || (nm + '_j'),
      type: L.movable === false ? 'fixed' : 'revolute', posMode: 'parent',
      pos: [+L.pos[0] || 0, +L.pos[1] || 0, +L.pos[2] || 0],
      /* 新坐标系三轴对齐父系的哪个轴；没填就按 ID（与父系同向） */
      axes: L.axes || ID_AXES,
      axis: L.axis || '+X',
      limits: (L.movable === false || !L.limits) ? null : [+L.limits[0], +L.limits[1]],
      rot: (Array.isArray(L.rot) && L.rot.length) ? L.rot : undefined
    });
  });
  /* 末端：固定在最后一段上的工具（长度后续可在模型链里改） */
  links.push({ name: 'tip', joint: 'tip_fix', type: 'fixed', posMode: 'parent',
               pos: [0, 0, 0], axes: ID_AXES, axis: '+Z', limits: null });
  return { name: a.name || '臂', base: { xyz: [0, 0, 0], yaw: +a.yaw || 0 }, links };
}

/* ---------------- 各步的渲染 ---------------- */
function wizStepHTML() {
  const s = WIZ.step;
  if (s === 0) {
    return `<h3>这条链有几条臂？</h3>
      <p class="sub">双臂的两条臂各自有独立的基座与安装朝向，关节数也可以不同。</p>
      <div class="two">
        <div class="pick ${WIZ.nArm === 1 ? 'on' : ''}" data-arm="1">
          <b>单臂</b><span>一条链，最常用</span></div>
        <div class="pick ${WIZ.nArm === 2 ? 'on' : ''}" data-arm="2">
          <b>双臂</b><span>两条链，可各自定位</span></div>
      </div>`;
  }
  if (s === 1) {
    return `<h3>安装基准</h3>
      <p class="sub">臂本体装在车上的朝向（绕竖直轴）。这决定整车系与臂基座系的夹角。</p>
      ${WIZ.arms.map((a, i) => `
        <div class="segrow">
          <div class="t">${WIZ.arms.length > 1 ? (a.name || ('臂' + (i + 1))) : '安装朝向'}</div>
          <div class="f" style="grid-template-columns:auto 1fr">
            <label>base_yaw (°)</label>
            <input type="number" step="15" data-yaw="${i}" value="${a.yaw}">
          </div>
        </div>`).join('')}`;
  }
  if (s === 2) {
    return `<h3>依次搭建 joint 与 link</h3>
      <p class="sub">每行是一段：相对上一坐标系的位置(offset) + 转动轴 + 限位。留空限位=不限。</p>
      ${WIZ.arms.map((a, ai) => `
        <div class="segrow">
          <div class="t" style="display:flex;align-items:center">
            ${a.name || ('臂' + (ai + 1))}
            <button class="del" data-add="${ai}">＋ 加一段</button>
          </div>
          <div id="wlinks${ai}">${a.links.map((L, li) => wizLinkRow(ai, li, L)).join('')}</div>
        </div>`).join('')}`;
  }
  if (s === 3) {
    return `<h3>装配臂相对世界坐标系原点的位置</h3>
      <p class="sub">规则手册的位姿范围定义在战场地面系(OXYZ)下；这里填臂在其中的位置。</p>
      <div class="segrow">
        <div class="f">
          <label>x (mm)</label><input type="number" step="10" id="wAsmX" value="${WIZ.asm.x}">
          <label>y (mm)</label><input type="number" step="10" id="wAsmY" value="${WIZ.asm.y}">
          <label>z (mm)</label><input type="number" step="10" id="wAsmZ" value="${WIZ.asm.z}">
        </div>
      </div>`;
  }
  return `<h3>夹爪往哪个轴插入</h3>
    <p class="sub">插入方向用于姿态约束（IK 复核）；夹紧轴是夹爪自身张开/闭合的方向。</p>
    <div class="segrow">
      <div class="f" style="grid-template-columns:auto 1fr">
        <label>插入轴</label>
        <select id="wIns">${AXIS_OPTS.map(a =>
          `<option value="${a.toLowerCase().replace('+','').replace('-','-')}" ${WIZ.insertAxis === a.toLowerCase().replace('+','').replace('-','-') ? 'selected' : ''}>${a}</option>`).join('')}</select>
        <label>夹紧轴</label>
        <select id="wGrip">${['x', 'y', 'z'].map(a =>
          `<option value="${a}" ${WIZ.gripAxis === a ? 'selected' : ''}>${a.toUpperCase()}</option>`).join('')}</select>
      </div>
    </div>
    <div class="segrow" style="color:#5d6b88;font-size:11px">
      确认后会新建模型，并覆盖当前模型与本地存档。
    </div>`;
}

function wizLinkRow(ai, li, L) {
  const lim = L.limits ? L.limits : ['', ''];
  const ax = L.axes || ID_AXES;
  const axSel = (key, tag) =>
    `<select data-lax="${ai}:${li}:${key}" title="新坐标系的 ${tag} 轴对齐父系的哪个轴">` +
    AXIS_OPTS.map(a => `<option ${ax[key] === a ? 'selected' : ''}>${a}</option>`).join('') +
    `</select>`;
  return `<div class="segrow" style="padding:6px 8px;margin-bottom:6px">
    <div class="f">
      <label>#${li + 1}</label>
      <input type="text" data-ln="${ai}:${li}" value="${L.name || ''}" placeholder="link 名">
      <input type="number" data-lp0="${ai}:${li}" value="${L.pos[0]}" step="10" title="offset x (mm)">
      <input type="number" data-lp1="${ai}:${li}" value="${L.pos[1]}" step="10" title="offset y (mm)">
      <input type="number" data-lp2="${ai}:${li}" value="${L.pos[2]}" step="10" title="offset z (mm)">
      <button class="del" data-del="${ai}:${li}" title="删掉这一段">✕</button>
    </div>
    <div class="f" style="margin-top:5px">
      <label title="新坐标系 X/Y/Z 分别对齐父系的哪个轴">坐标系</label>
      ${axSel('x', 'X')}${axSel('y', 'Y')}${axSel('z', 'Z')}
      <span></span>
    </div>
    <div class="f" style="margin-top:5px">
      <label>转动轴</label>
      <select data-la="${ai}:${li}" title="绕新坐标系自身的哪个轴转动">${AXIS_OPTS.map(a =>
        `<option ${L.axis === a ? 'selected' : ''}>${a}</option>`).join('')}</select>
      <input type="number" data-ll0="${ai}:${li}" value="${lim[0]}" step="5" title="下限 (°)，留空=不限">
      <input type="number" data-ll1="${ai}:${li}" value="${lim[1]}" step="5" title="上限 (°)，留空=不限">
      <span></span>
    </div>
  </div>`;
}

/* ---------------- 表单 -> 草稿（每次切换步骤前收一次） ---------------- */
function wizCollect() {
  const q = sel => document.querySelector(sel);
  document.querySelectorAll('[data-yaw]').forEach(el => {
    const i = +el.dataset.yaw; if (WIZ.arms[i]) WIZ.arms[i].yaw = +el.value || 0;
  });
  document.querySelectorAll('[data-ln]').forEach(el => {
    const [ai, li] = el.dataset.ln.split(':').map(Number);
    if (WIZ.arms[ai] && WIZ.arms[ai].links[li]) WIZ.arms[ai].links[li].name = el.value;
  });
  for (const k of ['0', '1', '2']) {
    document.querySelectorAll('[data-lp' + k + ']').forEach(el => {
      const [ai, li] = el.dataset['lp' + k].split(':').map(Number);
      if (WIZ.arms[ai] && WIZ.arms[ai].links[li]) WIZ.arms[ai].links[li].pos[k] = +el.value || 0;
    });
  }
  document.querySelectorAll('[data-la]').forEach(el => {
    const [ai, li] = el.dataset.la.split(':').map(Number);
    if (WIZ.arms[ai] && WIZ.arms[ai].links[li]) WIZ.arms[ai].links[li].axis = el.value;
  });
  /* 新坐标系三轴对齐父系的哪个轴（data-lax="臂:段:轴"） */
  document.querySelectorAll('[data-lax]').forEach(el => {
    const [ai, li, key] = el.dataset.lax.split(':');
    const L = WIZ.arms[+ai] && WIZ.arms[+ai].links[+li]; if (!L) return;
    if (!L.axes) L.axes = { x: '+X', y: '+Y', z: '+Z' };
    L.axes[key] = el.value;
  });
  for (const k of ['0', '1']) {
    document.querySelectorAll('[data-ll' + k + ']').forEach(el => {
      const [ai, li] = el.dataset['ll' + k].split(':').map(Number);
      const L = WIZ.arms[ai] && WIZ.arms[ai].links[li]; if (!L) return;
      if (!L.limits) L.limits = [null, null];
      L.limits[k] = (el.value === '') ? null : +el.value;
    });
  }
  const g = id => document.getElementById(id);
  if (g('wAsmX')) WIZ.asm = { x: +g('wAsmX').value || 0, y: +g('wAsmY').value || 0, z: +g('wAsmZ').value || 0 };
  if (g('wIns')) WIZ.insertAxis = g('wIns').value;
  if (g('wGrip')) WIZ.gripAxis = g('wGrip').value;
  /* 收完之后把空限位规整一下：两端都空 ⇒ 不限位 */
  WIZ.arms.forEach(a => a.links.forEach(L => {
    if (L.limits && L.limits[0] === null && L.limits[1] === null) L.limits = null;
  }));
}

/* ---------------- 校验 ---------------- */
function wizValidate(step) {
  if (step === 2) {
    for (let i = 0; i < WIZ.arms.length; i++) {
      const a = WIZ.arms[i];
      const who = a.name || ('臂' + (i + 1));
      if (!a.links.length) return who + ' 至少要有一段';
      for (let j = 0; j < a.links.length; j++) {
        const L = a.links[j];
        const nm = (L.name || '').trim();
        if (!nm) return who + ' 第 ' + (j + 1) + ' 段还没起名字';
        /* 坐标系必须是右手系：三轴不能重复、行列式要为 +1。
           用引擎里的 frameOK() 判，判据只有一份。 */
        if (L.axes && typeof frameOK === 'function') {
          const r = frameOK(L.axes);
          if (r.dup) return who + ' 第 ' + (j + 1) + ' 段：坐标系的三个轴重复了';
          if (Math.abs(r.det - 1) > 1e-9) {
            return who + ' 第 ' + (j + 1) + ' 段：坐标系不是右手系（det=' +
                   r.det.toFixed(0) + '，应为 +1）';
          }
        }
      }
    }
  }
  return '';
}

/* ---------------- 渲染 / 导航 ---------------- */
function wizRender() {
  const S = document.getElementById('wSteps'), B = document.getElementById('wBody');
  if (!S || !B) return;
  S.innerHTML = WIZ_STEPS.map((t, i) =>
    '<span class="' + (i === WIZ.step ? 'on' : (i < WIZ.step ? 'done' : '')) + '">' +
    (i + 1) + '. ' + t + '</span>').join('');
  B.innerHTML = wizStepHTML();
  B.scrollTop = 0;
  const prev = document.getElementById('wPrev'), next = document.getElementById('wNext');
  if (prev) prev.disabled = (WIZ.step === 0);
  if (next) next.textContent = (WIZ.step === WIZ_STEPS.length - 1) ? '完成并新建' : '下一步';
  wizWire();
}
function wizWire() {
  const B = document.getElementById('wBody');
  if (!B) return;
  B.querySelectorAll('.pick').forEach(el => el.onclick = () => {
    WIZ.nArm = +el.dataset.arm; wizEnsureArms(); wizRender();
  });
  B.querySelectorAll('[data-add]').forEach(el => el.onclick = () => {
    wizCollect();
    const ai = +el.dataset.add;
    WIZ.arms[ai].links.push({ name: 'link' + (WIZ.arms[ai].links.length + 1),
      joint: 'joint' + (WIZ.arms[ai].links.length + 1),
      pos: [0, 0, 150], axis: '+X', limits: [-90, 90], movable: true });
    wizRender();
  });
  B.querySelectorAll('[data-del]').forEach(el => el.onclick = () => {
    wizCollect();
    const [ai, li] = el.dataset.del.split(':').map(Number);
    WIZ.arms[ai].links.splice(li, 1);
    if (!WIZ.arms[ai].links.length) WIZ.arms[ai].links = wizDefaultLinks(1);
    wizRender();
  });
}
function wizOpen() {
  wizReset(); WIZ.on = true;
  const w = document.getElementById('wiz');
  if (w) w.classList.add('on');
  document.getElementById('modal').classList.add('hidden');
  wizRender();
}
function wizClose() {
  WIZ.on = false;
  const w = document.getElementById('wiz');
  if (w) w.classList.remove('on');
}
function wizNext() {
  wizCollect();
  const bad = wizValidate(WIZ.step);
  if (bad) { alert(bad); return; }
  if (WIZ.step < WIZ_STEPS.length - 1) { WIZ.step++; wizRender(); return; }
  wizFinish();
}
function wizPrev() { wizCollect(); if (WIZ.step > 0) { WIZ.step--; wizRender(); } }

/* ---------------- 收尾：把草稿变成真正的模型 ---------------- */
function wizFinish() {
  wizCollect();
  const built = WIZ.arms.map(wizBuildArm);
  if (!CONFIG) { alert('还没有配置'); return; }

  /* ★ 向导只产出"臂型"，不产出规则与车体尺寸。
     所以这里把当前配置【复制一份】、只换掉臂，并把 rules / chassis 清成 null ——
     留着上一个构型的规则是错的：拿同济的规则去评一条全新的臂，
     算出来的数字不对应任何东西，而且（改之前）会直接在 analyzeTask 里抛异常。
     规范与尺寸要单独配：在 web/src/configs/ 里写一份 rules / chassis。 */
  const id = CONFIG.id + '_wiz';
  const clone = JSON.parse(JSON.stringify(
    Object.assign({}, CONFIG, { __arms: undefined, model: undefined, arms: undefined })));
  clone.label = (CONFIG.label || CONFIG.id) + ' · 新建';
  clone.model = built[0];
  if (built.length > 1) clone.arms = built; else delete clone.arms;
  clone.rules = null;
  clone.chassis = null;
  defineConfig(id, clone);

  /* 装配位置 / 夹爪轴：没有规则时无处可写，就在 CONFIG 上留一份，
     后续给它加 rules 时可以拿来填。 */
  /* ★ 装配位置与夹爪轴写进【模型】的 vehicle 块（随 JSON 走）。
     之前只挂在配置上，导出/读取时带不走 —— 用户指出后改成进模型。 */
  const veh = { asm: { x: WIZ.asm.x, y: WIZ.asm.y, z: WIZ.asm.z },
                insertAxis: WIZ.insertAxis, gripAxis: WIZ.gripAxis };
  built.forEach(m => { m.vehicle = JSON.parse(JSON.stringify(veh)); });
  clone.asmInit = veh.asm;                 // 保留旧的挂法，兼容已有引用
  clone.gripper = { insertAxis: veh.insertAxis, gripAxis: veh.gripAxis };

  applyConfig(id);

  const set = (id2, v) => { const el = document.getElementById(id2); if (el) el.value = v; };
  set('inCx', WIZ.asm.x); set('inCy', WIZ.asm.y); set('inCz', WIZ.asm.z);

  wizClose();
  updateAll();
  if (typeof renderAngles === 'function') renderAngles();
  if (typeof renderVehicle === 'function') renderVehicle();
  if (typeof initConfigSelector === 'function') initConfigSelector();
  saveLocal();
}

/* ---------------- 接线 ---------------- */
function initWizard() {
  const b = id => document.getElementById(id);
  if (b('mNew'))  b('mNew').onclick  = () => wizOpen();
  if (b('btnNew')) b('btnNew').onclick = () => wizOpen();
  if (b('wNext')) b('wNext').onclick = () => wizNext();
  if (b('wPrev')) b('wPrev').onclick = () => wizPrev();
  if (b('wCancel')) b('wCancel').onclick = () => wizClose();
  const w = b('wiz');
  if (w) w.addEventListener('click', e => { if (e.target === w) wizClose(); });
  window.addEventListener('keydown', e => {
    if (!WIZ.on) return;
    if (e.key === 'Escape') wizClose();
  });
}
initWizard();
