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
const SECTS = ['List', 'Q', 'Data'];
let SECT_OPEN = { List: true, Q: true, Data: true };

function setSectOpen(key, open, save) {
  SECT_OPEN[key] = !!open;
  const hd = document.getElementById('hd' + key);
  const bd = document.getElementById('bd' + key);
  if (hd) hd.classList.toggle('collapsed', !open);
  if (bd) bd.classList.toggle('hidden', !open);
  /* 折叠后相邻的分隔条没意义了，一起藏掉 */
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
}
function applySplit() { setSplitPct(SPLIT_PCT == null ? SPLIT_DEF : SPLIT_PCT); }
function syncSplitHandles() {
  const l = SECT_OPEN.List, q = SECT_OPEN.Q, d = SECT_OPEN.Data;
  const s1 = document.getElementById('split');
  const s2 = document.getElementById('split2');
  if (s1) s1.style.display = (l && (q || d)) ? '' : 'none';
  if (s2) s2.style.display = (q && d) ? '' : 'none';
}
function initSplit() {
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

/* ---------------- 旋转角度表 ----------------
   实时显示每个关节的当前角度。拖动关节滑条 / 播放动画 / 改模型都会刷新。
   多臂时按臂分组显示。 */
function renderAngles() {
  const box = document.getElementById('qbody');
  if (!box) return;
  if (!CHAIN || !CHAIN.length) { box.innerHTML = '<div class="one" style="color:#5d6b88">还没有模型</div>'; return; }

  const rows = [];
  const nA = (typeof armCount === 'function') ? armCount() : 1;
  const jointRow = (c, qi64) => {
    const v = Q[qi64];
    const lim = c.limits ? (c.limits[0] + '~' + c.limits[1] + '°') : '不限';
    const hot = (typeof ACTIVE !== 'undefined' && ACTIVE === qi64) ? ' class="hot"' : '';
    return '<tr' + hot + '><td class="nm">' + c.joint +
           ' <span class="axc">' + (c.axLabel || '') + '</span></td>' +
           '<td class="v">' + (v === undefined ? '-' : (+v).toFixed(1)) + '°</td>' +
           '<td class="axc" style="text-align:right;font-size:10.5px">' + lim + '</td></tr>';
  };
  let flat = 0;
  for (let a = 0; a < nA; a++) {
    const A = (typeof ARMS !== 'undefined' && ARMS[a]) ? ARMS[a] : null;
    if (!A) continue;
    if (nA > 1) {
      rows.push('<tr><td colspan="3" style="color:#8fd0ff;font-weight:600;padding-top:7px">' +
                (A.name || ('臂' + a)) + '</td></tr>');
    }
    for (let ci = A.c0; ci < A.cN; ci++) {
      const c = CHAIN[ci];
      if (!c.movable) continue;
      rows.push(jointRow(c, flat));
      flat++;
    }
  }
  if (!rows.length) { box.innerHTML = '<div class="one" style="color:#5d6b88">这条链没有可动关节</div>'; return; }
  box.innerHTML = '<table>' + rows.join('') + '</table>';
}

/* ---------------- 接线 ---------------- */
initCollapse();
initSplit();
initVisMenu();
