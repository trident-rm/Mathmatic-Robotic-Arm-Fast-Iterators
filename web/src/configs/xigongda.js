/* ==================================================================
   配置：西工大机械臂
   ------------------------------------------------------------------
   这份是 configs/tongji.js【原样复制】过来的：杆长、转动轴、限位、
   规则（表5-10）、车体尺寸，一个数都没动，只改了 id 与显示名。
   ⇒ 现在这里的数字【还是同济那条臂的】，不是西工大的实机尺寸。

   自己开始改的时候，只需要动这个文件（引擎不用碰）：
     · 臂本身      model.links 的 pos（杆长/偏置）、axis（转动轴）、limits（限位）
     · 安装朝向    model.base.yaw
     · 竞赛规则    rules（换一套规则手册，或整块删掉/置 null）
     · 车体尺寸    chassis（改完记得把 v +1，否则老存档会把新默认值盖回去）

   改完在界面上：顶栏「配置」下拉框选「西工大机械臂」。

   ★ 三级（装配）判据用的参数在下面的 rules.act 里，也全是从示例抄过来的：
       lift / rotRange / rotStep  上推高度、旋转角范围、连续校核的档距
       steps                      旋转序列（一步一步写轴与角）：
                                   'p' = 把能量单元往上翻的那根水平轴（e⃗ × Z）；
                                   'q' = 插入轴 e⃗ 本身。两步各绕什么轴直接决定
                                   "三级到底难在哪"，规则手册怎么写就怎么填
       insToolAxis                插入方向对应工具系的哪根轴（实测是 x，别改成 z）
     这几项改臂时要一起核对；界面上「可达域·任务域数据」面板里的
     「可装配 / 可多姿态」两行就是它们算出来的。
   ==================================================================================== */
defineConfig('xigongda', {
  label: '西工大机械臂',

  /* ---------------- 臂本身 ----------------
     ★ 这已经是【真机数据】，不是当初照抄示例的那份占位值 ——
       由用户 2026-10-12 从界面上导出（`保存 JSON` → `西工大机械臂.json`）后填进来的。
       与示例的差别：整条链的杆长与限位全不一样（见下）。
     安装 yaw = -90（把本体 +Y 转到整车 +X），base 仍在原点。 */
  model: {
    name: '西工大机械臂',
    base: { xyz: [0, 0, 0], yaw: -90 },
    /* 装配位：臂相对【整车系 OXYZ 原点】装在哪。
       ★ 只有 x / y 参与任务点换算（见 04-config.js 的 mapObj().toBase）；
         高度用 rules.frame.baseH，所以这里 z 写 0 就行，
         界面上「装配参数」那个 z=800 是历史默认值，不参与换算。 */
    asm: { x: -500, y: -200, z: 0 },
    links: [
      { name: 'base_link', joint: 'base_joint', type: 'revolute', posMode: 'origin',
        pos: [0, 0, 0], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null },

      { name: 'link1', joint: 'joint1', type: 'revolute', posMode: 'parent',
        pos: [0, 0, 95], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+X', limits: [0, 150] },

      { name: 'link2', joint: 'joint2', type: 'revolute', posMode: 'parent',
        pos: [0, 478.84, 33.39], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '-X', limits: [-81, 270] },

      { name: 'link3', joint: 'joint3', type: 'revolute', posMode: 'parent',
        pos: [21.71, 36.96, 164], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null },

      { name: 'link4', joint: 'joint4', type: 'revolute', posMode: 'parent',
        pos: [0, 0, 268], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+X', limits: [-70, 250] },

      { name: 'link5', joint: 'joint5', type: 'revolute', posMode: 'parent',
        pos: [0, 0, 0], axes: { x: '+X', y: '-Z', z: '+Y' }, axis: '+Z', limits: null },

      { name: 'gripper', joint: 'gripper_fix', type: 'fixed', posMode: 'parent',
        pos: [0, 0, 89.14], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null }
    ]
  },

  /* ---------------- 竞赛规则（规则手册 表 5-10）----------------
     frame 描述【整车系 OXYZ】与【臂 base 系】的关系。三件事分开写，别混：

       yaw    : 臂本体装在车上的朝向（= model.base.yaw，buildChain 用的那个）
       baseH  : base 原点离任务系地面的高度（mm）
       taskMap: 任务系 OXYZ 相对整车系的三轴对应关系
                'swap'    : base.X = -OXYZ.X, base.Y = -OXYZ.Y  （绕 Z 180°）
                'noSwap'  : base.X = -OXYZ.Y, base.Y = +OXYZ.X  （绕 Z +90°）

     ★ 这三件事以前是【手写死】在 02 模块里的，而 MATLAB 的 t7 又自己写了一
       另一套，结果两边差 90°，t7 报出假的"0/200 可解"。
       现在引擎从这三个字段算出换算，MATLAB 侧照抄同一公式即可。
       见 docs/notes/base_yaw决策待定.md

     ⚠ 注意 yaw 与 taskMap 是【两个独立的东西】，别以为改一个另一个会跟着变：
       当前 yaw=-90 而 taskMap='swap'（等效绕 Z 180°），两者并不相等。
       这不是笔误 —— 是历史上就这么定的，保持原样以免改动已有结论。 */
  rules: {
    src: '规则手册 表5-10（x,y,z 单位 mm，角度 °）',
    frame: { yaw: -90, baseH: 170, taskMap: 'swap' },
    init: { x: -500, y: -200, z: 800, th: 0, ph: 90, al: 0 },   // 装配臂位置（OXYZ 下）
    levels: {
      1: { name: '一级', x: [-100, 0], y: [100, 300], z: [500, 700], th: [0, 0],    ph: [90, 90], al: [0, 0],
           note: '无平移、无旋转要求' },
      2: { name: '二级', x: [-100, 0], y: [100, 300], z: [500, 700], th: [-90, 90], ph: [0, 90],  al: [-45, 45],
           note: '放入后沿 z 平移 100mm，无旋转要求' },
      3: { name: '三级', x: [-100, 0], y: [100, 300], z: [500, 700], th: [-90, 90], ph: [0, 90],  al: [-45, 45],
           note: '放入后沿 z 平移 100mm；再以 P 轴为旋转轴旋转' }
    },
    /* 装配动作约定（与示例一致；缺了这几项判据会走内置默认值，
       所以显式写出来，免得"看着有配置、其实是默认"） */
    act: {
      gripAxis: 'x',
      insertAxis: 'y',
      lift: 100,
      rotRange: [0, 90],
      rotStep: 15,
      steps: [
        { axis: 'p', ang: 90 },      // 绕 P 轴 +90°：单元竖到与 Z 轴平行
        { axis: 'q', ang: 90 }       // 绕插入轴 ±90°：多姿态
      ],
      insToolAxis: 'x',
      Qaxis: 'X′ ( = e⃗ )',
      Paxis: 'Y′',
      eeZalign: '-e⃗'
    }
  },

  /* ---------------- 车体 / 伸展尺寸规则 ----------------
     ★ 底盘位置与示例不同：示例是 [0,140,0]（底盘在 base 系里往前偏 140），
       这条按用户 2026-10-12 从界面上调的值改为 [0,0,0]（底盘中心就在原点下方）。
       尺寸本身（580×580×105）还是占位值，真机尺寸确定后一起改。
     CHASSIS_V 改动默认值时 +1，否则老存档会把新默认值盖回去 */
  chassis: {
    v: 3,
    chassis: { posMode: 'base', pos: [0, 0, 0], size: [580, 580, 105], zDir: -1 },
    sizer:   { lwh: [1200, 1200, 1100], ballD: 1500, ballH: 480 }
  }
});
