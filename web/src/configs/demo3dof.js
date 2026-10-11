/* ==================================================================
   配置：最小测试臂（3 自由度，无竞赛规则、无车体尺寸）
   ------------------------------------------------------------------
   这条配置的用处不是"给人用"，而是【证明引擎是通用的】：
     · 没有 rules  → 任务域那块应该自己让开，不能崩
     · 没有 chassis→ 尺寸那块应该自己让开，不能崩
     · 3 自由度    → 引擎不能假设 6 关节
   web/checks/t12_arm_swap.js 会切到这条配置来跑全链路。
   ==================================================================================== */
defineConfig('demo3dof', {
  label: '最小测试臂（3 自由度 · 无规则）',

  model: {
    name: '最小测试臂 3DOF',
    base: { xyz: [0, 0, 0], yaw: 0 },
    links: [
      { name: 'base_link', joint: 'base_joint', type: 'revolute', posMode: 'origin',
        pos: [0, 0, 0], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null },

      { name: 'l1', joint: 'j1', type: 'revolute', posMode: 'parent',
        pos: [0, 0, 100], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+X', limits: [-90, 90] },

      { name: 'l2', joint: 'j2', type: 'revolute', posMode: 'parent',
        pos: [0, 200, 0], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+X', limits: [-90, 90] },

      { name: 'tip', joint: 'tip_fix', type: 'fixed', posMode: 'parent',
        pos: [0, 0, 90], axes: { x: '+X', y: '+Y', z: '+Z' }, axis: '+Z', limits: null }
    ]
  },

  rules: null,      // 没有竞赛规则
  chassis: null     // 没有车体尺寸规则
});
