/* ==================================================================
   04-config.js —— 配置层（引擎与"具体哪条臂 / 哪套规则"之间的唯一接口）

   为什么要有这一层：
    这个仓库长期只有一条臂（同济示例臂）。杆长、比赛规则、车体尺寸
    全散在 01/02/03 三个模块里写死，于是：
      · 换一条臂来算，规则还是同济的，算出来的数没有意义
      · 想加新构型只能改引擎 → 很容易把原来那条臂弄坏
    所以把「具体是哪条臂 / 哪套规则」全部搬到配置里，引擎只认 CONFIG。

   一个配置长这样：

     defineConfig('tongji', {
       label:   '同济工程机械臂（示例）',
       model:   { name, base:{xyz,yaw}, links:[...] },     // 臂本身
       rules:   { frame:{...}, levels:{...}, ... } | null,  // 竞赛规则（可缺省）
       chassis: { posMode,pos,size,zDir, lwh, ballD,ballH } | null,  // 车体尺寸
       taskMap: 'auto' | fn                                 // 任务域 <-> base 的换算
     });

   ★ 关键：任务域换算不再手写，而是【从 model.base.yaw 推出来】。
     手写就是 t7 那个 bug 的来源（网页用 base 系、MATLAB 用另一套，差了 90°）。
==================================================================================== */
'use strict';

/* CONFIGS / CONFIG 是本模块自己的；RULES / MAP / MODEL / CHASSIS / SIZER
   的可变声明在 01 与 02 里（它们先加载）。这里【不重复声明】，
   否则同一个 <script> 作用域里 let 重名会直接 SyntaxError。 */
let CONFIG = null;                 // 当前生效的配置
let APPLIED_CONFIG_ID = null;      // 已 apply 的配置 id
const CONFIGS = {};                // id -> 配置
let CONFIG_ORDER = [];
let DEFAULT_CONFIG_ID = 'tongji';  // 默认配置（configs/*.js 可覆盖）

function defineConfig(id, cfg) {
  cfg.id = id;
  CONFIGS[id] = cfg;
  if (!CONFIG_ORDER.includes(id)) CONFIG_ORDER.push(id);
  return cfg;
}
function listConfigs() { return CONFIG_ORDER.slice(); }
function activeConfigId() { return APPLIED_CONFIG_ID; }

/* ---------------- 任务域 <-> base 的换算 ----------------
   这是【唯一】实现，引擎与 MATLAB 都必须用它，不许再各写一份
   （t7 那个"0/200 可解"就是各写一份造成的）。

   frame = { yaw, baseH, taskMap }
     yaw     : 臂本体装车上的朝向（应等于 model.base.yaw）
     baseH   : base 原点离任务系地面的高度 mm
     taskMap : 任务系 → 整车系的轴向约定
               'swap'   : base.X = -OXYZ.X, base.Y = -OXYZ.Y
               'noSwap' : base.X = -OXYZ.Y, base.Y = +OXYZ.X

   两个映射【不是】两个 yaw 角度 —— 它们是两套轴向约定。
   实现上都在 xy 平面内，所以统一写成 Rz(φ) + 抬升，φ 由 taskMap 决定：
     'swap' → φ = 180°     'noSwap' → φ = 90°
   取逆时用 Rz(−φ)。                                          */
const TASKMAP_PHI = { swap: 180, noSwap: 90 };

function mapObj(frame) {
  frame = frame || {};
  const h = +frame.baseH || 0;
  const kind = frame.taskMap === 'noSwap' ? 'noSwap' : 'swap';
  const phi = TASKMAP_PHI[kind] * D2R;
  const c = Math.cos(phi), s = Math.sin(phi);

  /* base = Rz(φ)·(OXYZ − [0,0,baseH]) ；反变换 Rz(−φ) 再抬回去 */
  const fwd = v => [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
  const inv = v => [c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]];

  return {
    yaw: +frame.yaw || 0, baseH: h, taskMap: kind, phiDeg: TASKMAP_PHI[kind],
    toBase: p => { const q = fwd([p[0], p[1], p[2] - h]); return [q[0], q[1], q[2]]; },
    toTask: p => { const q = inv(p); return [q[0], q[1], q[2] + h]; },
    dirToBase: d => fwd(d),
    dirToTask: d => inv(d),
    describe: () => kind + ' (Rz(' + TASKMAP_PHI[kind] + '°), 抬高 ' + h + 'mm)'
  };
}

/* 重新按当前 frame 造一个 taskMap 对象（改映射后调，不重新 apply 整个配置） */
function rebuildTaskMap() {
  if (!CONFIG || !CONFIG.rules) return null;
  MAP = mapObj(CONFIG.rules.frame);
  return MAP;
}

/* ---------------- 应用一个配置 ---------------- */
function applyConfig(id) {
  const cfg = CONFIGS[id];
  if (!cfg) throw new Error('没有这个配置：' + id);
  CONFIG = cfg;
  APPLIED_CONFIG_ID = id;

  /* 1) 臂型 → MODEL（引擎只从这里拿模型，不再自带示例臂） */
  if (cfg.model) {
    MODEL = JSON.parse(JSON.stringify(cfg.model));
  }
  /* 安装朝向以 rules.frame.yaw 为准（两者本来就该是同一个量，写两处容易走偏） */
  if (MODEL && cfg.rules && cfg.rules.frame && typeof cfg.rules.frame.yaw === 'number') {
    MODEL.base.yaw = cfg.rules.frame.yaw;
  }
  if (MODEL) buildChain();

  /* 2) 竞赛规则 */
  if (cfg.rules) {
    RULES = JSON.parse(JSON.stringify(cfg.rules));
    if (RULES.frame) {
      RULES.baseH = RULES.frame.baseH;
      MAP = mapObj(RULES.frame);
    } else {
      MAP = null;
    }
  } else {
    RULES = null;          // 没有规则：任务域那块界面自己会隐藏/提示
    MAP = null;
  }

  /* 3) 车体 / 尺寸规则。没配就回到"空"（尺寸校核那块会自己让开） */
  if (cfg.chassis) {
    const c = cfg.chassis;
    CHASSIS   = JSON.parse(JSON.stringify(c.chassis || {}));
    SIZER     = JSON.parse(JSON.stringify(c.sizer || {}));
    CHASSIS_V = (c.v === undefined ? CHASSIS_V : c.v);
  } else {
    CHASSIS = { posMode:'base', pos:[0,0,0], size:[1,1,1], zDir:-1 };
    SIZER   = { lwh:[0,0,0], ballD:0, ballH:0 };
  }

  /* 4) 按配置刷新能刷的界面（引擎不认识的控件就不动） */
  syncConfigUI();
  return cfg;
}

/* 配置切换后把界面同步一下。找不到的控件直接跳过 —— 配置可以不带规则/车体。 */
function syncConfigUI() {
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.value = v; };
  const txt = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };

  if (MODEL) txt('model', MODEL.name + '（' + MODEL.links.length + ' 段，' +
                         '安装 yaw ' + (MODEL.base && MODEL.base.yaw) + '°）');
  set('selMap', (MAP && MAP.taskMap === 'noSwap') ? 'side' : 'swap');

  const sel = document.getElementById('selConfig');
  if (sel) {
    sel.innerHTML = listConfigs()
      .map(cid => '<option value="' + cid + '"' + (cid === activeConfigId() ? ' selected' : '') +
                  '>' + (CONFIGS[cid].label || cid) + '</option>').join('');
  }
  /* 没有竞赛规则时，把依赖规则的面板标出来 */
  const noRule = document.getElementById('noRuleWarn');
  if (noRule) noRule.style.display = (RULES && RULES.levels) ? 'none' : '';
}

/* 把可选配置列表填进下拉框（DOM 就绪后调） */
function initConfigSelector() {
  const sel = document.getElementById('selConfig');
  if (!sel) return;
  sel.innerHTML = listConfigs()
    .map(cid => '<option value="' + cid + '"' + (cid === activeConfigId() ? ' selected' : '') +
                '>' + (CONFIGS[cid].label || cid) + '</option>').join('');
  sel.onchange = () => {
    if (!confirm('切换到「' + CONFIGS[sel.value].label + '」？当前模型会被替换')) {
      sel.value = activeConfigId(); return;
    }
    applyConfig(sel.value);
    updateAll();
  };
}

/* 取某个配置里的臂型（深拷贝）。
   注意与 demoModel() 的区别：
     configModel('tongji')  → 配置里【原始】的同济臂，永远不变
     demoModel()            → 【当前】模型（点"示例"按钮时用来把当前模型重置回配置）
   要拿"那条示例臂本身"（导出、对表、回归）一律用这个。 */
function configModel(id) {
  const cfg = CONFIGS[id || DEFAULT_CONFIG_ID];
  if (!cfg || !cfg.model) return null;
  return JSON.parse(JSON.stringify(cfg.model));
}

/* ---------------- 启动时把默认配置装上 ----------------
   由 05-bootstrap.js 在所有模块与配置文件都加载完之后调用。
   ★ 必须放在最后：applyConfig 会用到 MODEL/RULES/MAP/CHASSIS/SIZER，
     而它们的声明在前面几个模块里（同一个 <script> 作用域，TDZ）。 */
function initDefaultConfig() {
  const ids = listConfigs();
  if (!ids.length) throw new Error('没有任何配置：web/src/configs/ 是不是空了？');
  const id = ids.includes(DEFAULT_CONFIG_ID) ? DEFAULT_CONFIG_ID : ids[0];
  applyConfig(id);
  return id;
}
