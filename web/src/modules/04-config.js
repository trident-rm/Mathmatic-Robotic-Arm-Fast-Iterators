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
    /* off：装配臂在 OXYZ 下的【水平位置】({x,y})，可省略。
       ★ 为什么高度不从这里来：高度一直是 baseH（frame 的一部分，=170），
         这条约定从最早就是这样，改了会让所有历史结论变数。
         所以这里只承担"多臂左右分开装"带来的水平偏移；
         不传 off 时行为与以前【逐字相同】（只减 baseH）。
       ★ 多臂必须按臂传 off —— 两条臂装的位置不同，
         同一个任务点在各自 base 系里的坐标就不同。 */
    toBase: (p, off) => {
      const q = fwd([p[0] - (+(off && off.x) || 0),
                     p[1] - (+(off && off.y) || 0),
                     p[2] - h]);
      return [q[0], q[1], q[2]];
    },
    toTask: (p, off) => {
      const q = inv(p);
      return [q[0] + (+(off && off.x) || 0),
              q[1] + (+(off && off.y) || 0),
              q[2] + h];
    },
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

  /* 1) 臂型 → MODEL（引擎只从这里拿模型，不再自带示例臂）
       支持单臂 model 与多臂 arms 两种写法，统一归一化。 */
  const armList = normalizeArms(cfg);
  cfg.__arms = armList;                       // 引擎的 buildChain 读它
  if (armList.length) {
    MODEL = armList[0];                       // 第 0 条臂 = MODEL（老语义）
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
  if (typeof renderAngles === 'function') renderAngles();
  if (typeof renderVehicle === 'function') renderVehicle();
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

/* ---------------- 多臂归一化 ----------------
   配置里两种写法都收，内部统一成"一个臂数组"：

     单臂（老写法，推荐继续用）：
         model: { name, base:{xyz,yaw}, links:[...] }

     多臂：
         arms: [ {name, base:{xyz,yaw}, links:[...]}, {...} ]
         （写了 arms 就不用写 model；两个都写时以 arms 为准）

   归一化之后：
     · ARMS[a]  = 第 a 条臂的模型（含 name/base/links）
     · MODEL    = ARMS[0] 的别名 ⇒ 所有老代码/老自检照旧
     · 各臂的 base.yaw 若没写，回落到 rules.frame.yaw（与单臂一致的行为）

   为什么要支持多臂：双臂机器人两条臂的安装位置与朝向都不同，
   但关节角是同一时刻的一组向量，"协同"就是同一组 q 下两条臂同时到某处。 */
function normalizeArms(cfg) {
  let list = [];
  if (Array.isArray(cfg.arms) && cfg.arms.length) {
    list = cfg.arms.map(a => JSON.parse(JSON.stringify(a)));
  } else if (cfg.model) {
    list = [JSON.parse(JSON.stringify(cfg.model))];
  }
  const fy = (cfg.rules && cfg.rules.frame && typeof cfg.rules.frame.yaw === 'number')
    ? cfg.rules.frame.yaw : 0;
  list.forEach((m, i) => {
    if (!m.base) m.base = { xyz: [0, 0, 0], yaw: fy };
    if (!Array.isArray(m.base.xyz)) m.base.xyz = [0, 0, 0];
    /* 只有【单臂】才把安装朝向统一成 frame.yaw（老行为，不能变）；
       多臂时每条臂的 yaw 是自己的，帧里的 yaw 只描述"任务系怎么摆"。 */
    if (list.length === 1 && typeof cfg.rules !== 'undefined' && cfg.rules &&
        cfg.rules.frame && typeof cfg.rules.frame.yaw === 'number') {
      m.base.yaw = cfg.rules.frame.yaw;
    }
    if (!m.name) m.name = '臂' + i;
    if (!Array.isArray(m.links)) m.links = [];
  });
  return list;
}

/* 当前生效的所有臂（引擎的 buildChain 会调它） */
function activeArms() {
  return (CONFIG && CONFIG.__arms) ? CONFIG.__arms : null;
}

/* ---------------- 每条臂自己的装配位 ----------------
   多臂时两条臂装在车上的位置不同（一左一右），所以"它相对世界坐标系原点在哪"
   是【每条臂各一份】。任务域校核按臂分别做 —— 这是"单臂三轴检验 × 2"的前提：
   否则两条臂共用一个装配位，基座重合、可达集完全相同，
   "分别校核两次"就退化成把同一个答案抄两遍。

   取值优先级：arm.asm → 模型级 vehicle.asm → RULES.init（单臂老配置兜底）。 */
function asmOfArm(a) {
  a = a || 0;
  const A = ARMS[a];
  if (A && A.model && A.model.asm) {
    const s = A.model.asm;
    return { x: +s.x || 0, y: +s.y || 0, z: +s.z || 0 };
  }
  if (MODEL && MODEL.vehicle && MODEL.vehicle.asm) {
    const s = MODEL.vehicle.asm;
    return { x: +s.x || 0, y: +s.y || 0, z: +s.z || 0 };
  }
  const ini = (RULES && RULES.init) ? RULES.init : { x: 0, y: 0, z: 0 };
  return { x: +ini.x || 0, y: +ini.y || 0, z: +ini.z || 0 };
}
function asmAllArms() {
  const n = (typeof armCount === 'function') ? armCount() : 1;
  const out = [];
  for (let a = 0; a < n; a++) out.push(asmOfArm(a));
  return out;
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
