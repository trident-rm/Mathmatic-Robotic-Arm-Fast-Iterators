/* ==================================================================
   配置：同济工程机械臂（示例）
   ------------------------------------------------------------------
   这个文件里的每一个数字都来自原来的 01/02/03 三个模块，是【原样搬过来的】，
   没有改过任何值 —— 搬完之后引擎里就不该再有这些数了。
   ==================================================================================== */
defineConfig('tongji', {
  label: '同济工程机械臂（示例）',

  /* ---------------- 臂本身 ----------------
     与 demoModel() 逐字一致：7 段，安装 yaw = -90（把本体 +Y 转到整车 +X） */
  model: {
    name: '同济工程机械臂（示例）',
    base: { xyz: [0, 0, 0], yaw: -90 },
    links: [
      { name: 'base_link', joint: 'base_joint', type: 'revolute', posMode: 'origin',
        pos: [0, 0, 0], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null },

      { name: 'link1', joint: 'joint1', type: 'revolute', posMode: 'parent',
        pos: [0, 0, 75], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+X', limits: [0, 122] },

      { name: 'link2', joint: 'joint2', type: 'revolute', posMode: 'parent',
        pos: [0, 290, 0], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '-X', limits: [-90, 200] },

      { name: 'link3', joint: 'joint3', type: 'revolute', posMode: 'parent',
        pos: [85.5, 0, 0], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null },

      { name: 'link4', joint: 'joint4', type: 'revolute', posMode: 'parent',
        pos: [0, -30, 401.47], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+X', limits: [-16, 220] },

      { name: 'link5', joint: 'joint5', type: 'revolute', posMode: 'parent',
        pos: [0, 0, 0], axes: { x: '+X', y: '-Z', z: '+Y' }, axis: '+Z', limits: null },

      { name: 'gripper', joint: 'gripper_fix', type: 'fixed', posMode: 'parent',
        pos: [0, 0, 211], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null }
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
    /* 装配动作约定 */
    act: {
      gripAxis: 'x',
      insertAxis: 'y',
      Qaxis: 'X′ ( = e⃗ )',
      Paxis: 'Y′',
      eeZalign: '-e⃗'
    }
  },

  /* ---------------- 车体 / 伸展尺寸规则 ----------------
     CHASSIS_V 改动默认值时 +1，否则老存档会把新默认值盖回去 */
  chassis: {
    v: 2,
    chassis: { posMode: 'base', pos: [0, 140, 0], size: [580, 580, 105], zDir: -1 },
    sizer:   { lwh: [1200, 1200, 1100], ballD: 1500, ballH: 480 }
  }
});
