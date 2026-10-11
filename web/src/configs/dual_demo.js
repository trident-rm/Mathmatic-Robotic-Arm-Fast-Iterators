/* ==================================================================
   配置：双臂示例（左臂 + 右臂，各自独立基座与安装朝向）
   ------------------------------------------------------------------
   用途：验证"引擎能跑多臂"，也是 web/checks/t13_multi_arm.js 的被测对象。

   两条臂的区别：
     · 基座位置不同（车头左前 / 右前，x 差 360mm）
     · 安装朝向不同（一边 yaw=-90、一边 yaw=+90，面对面朝前伸）
     · 关节数与限位也不同（左臂 4 自由度、右臂 3 自由度）
       ⇒ 顺便再压一次"引擎不许假设关节数"

   没有 rules / chassis：这是纯构型验证，不参加比赛。
   ==================================================================================== */
(function () {
  const ID = { x: '+X', y: '+Y', z: '+Z' };
  const rev = (name, joint, pos, axis, limits, axes) =>
    ({ name, joint, type: 'revolute', posMode: 'parent', pos,
       axes: axes || ID, axis, limits });
  const fix = (name, pos, axes) =>
    ({ name, joint: name + '_fix', type: 'fixed', posMode: 'parent', pos,
       axes: axes || ID, axis: '+Z', limits: null });

  defineConfig('dual_demo', {
    label: '双臂示例（左 4 轴 / 右 3 轴）',

    /* 多臂写法：arms 数组。每条臂各自 base 与 links。 */
    arms: [
      { /* ---- 左臂：装在车左前，朝前伸（yaw = -90）---- */
        name: '左臂',
        base: { xyz: [60, 180, 250], yaw: -90 },
        /* 相对世界坐标系(OXYZ)原点的装配位 —— 每条臂各一份。
           两条臂装的位置不同，"分别校核两次"才有意义。 */
        asm: { x: 60, y: 180, z: 0 },
        links: [
          { name: 'base_link', joint: 'base_joint', type: 'revolute',
            posMode: 'origin', pos: [60, 180, 250], axes: ID, axis: '+Z', limits: null },
          rev('L1', 'L_j1', [0, 0, 80],   '+X', [-120, 120]),
          rev('L2', 'L_j2', [0, 180, 0],  '+X', [-100, 100]),
          rev('L3', 'L_j3', [0, 0, 160],  '+Z', null),
          fix('L_tip', [0, 0, 100])
        ]
      },
      { /* ---- 右臂：装在车右前，与左臂镜像（yaw = +90）---- */
        name: '右臂',
        base: { xyz: [-60, 180, 250], yaw: 90 },
        asm: { x: -60, y: 180, z: 0 },
        links: [
          { name: 'base_link', joint: 'base_joint', type: 'revolute',
            posMode: 'origin', pos: [-60, 180, 250], axes: ID, axis: '+Z', limits: null },
          rev('R1', 'R_j1', [0, 0, 80],  '+X', [-120, 120]),
          rev('R2', 'R_j2', [0, 180, 0], '+X', [-100, 100]),
          fix('R_tip', [0, 0, 120])
        ]
      }
    ],

    rules: null,      // 双臂示例不参加比赛
    chassis: null
  });
})();
