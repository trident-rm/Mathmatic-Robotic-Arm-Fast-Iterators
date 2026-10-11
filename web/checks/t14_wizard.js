/*  t14_wizard.js —— 新建向导的自检（无头，Node 沙箱）

    怎么验的：
      向导本身是"表单 -> 数据"的转换器，核心逻辑（草稿 -> 臂模型）是纯的，
      所以直接在沙箱里驱动 WIZ 草稿、调 wizBuildArm() 检查产物。
      另外验一遍用户规范里的步骤顺序与按钮接线。

    跑法： node web/checks/t14_wizard.js
*/
'use strict';
const { loadEnv } = require('./t9_load.js');

let pass = 0, fail = 0;
const ok = (c, msg, extra) => {
  if (c) { pass++; console.log('  ✓ ' + msg); }
  else { fail++; console.log('  ✗ ' + msg + (extra !== undefined ? '   ← ' + JSON.stringify(extra) : '')); }
};
const near = (a, b, tol) => Math.abs(a - b) <= (tol === undefined ? 1e-9 : tol);

const { A } = loadEnv({ variants: false });
/* 向导的东西是 let/const，不挂 globalThis —— 它们由 __api 的 getter 导出，
   所以要从 A 上取（沙箱对象上没有）。 */
const W = k => A[k];

console.log('\n=== ① 步骤顺序符合规范 ===');
{
  ok(Array.isArray(W('WIZ_STEPS')), 'WIZ_STEPS 存在');
  const s = W('WIZ_STEPS');
  ok(s.length === 5, '一共 5 步：' + s.join(' → '), s);
  ok(s[0].indexOf('臂') >= 0, '第 1 步问臂数（单臂/双臂）', s[0]);
  ok(s[1].indexOf('安装') >= 0 || s[1].indexOf('基准') >= 0, '第 2 步是安装基准(base_yaw)', s[1]);
  ok(s[2].indexOf('关节') >= 0 && s[2].indexOf('连杆') >= 0, '第 3 步搭 joint 与 link', s[2]);
  ok(s[3].indexOf('装配') >= 0, '第 4 步是装配臂位置', s[3]);
  ok(s[4].indexOf('夹爪') >= 0, '第 5 步是夹爪插入轴', s[4]);
}

console.log('\n=== ② 草稿默认值与臂数切换 ===');
{
  W('wizReset')();
  const w = W('WIZ');
  ok(w.nArm === 1 && w.arms.length === 1, '默认单臂', [w.nArm, w.arms.length]);
  ok(w.arms[0].links.length === 3, '默认给 3 段占位', w.arms[0].links.length);
  ok(near(w.arms[0].yaw, -90), '默认 base_yaw = -90', w.arms[0].yaw);

  w.nArm = 2; W('wizEnsureArms')();
  ok(w.arms.length === 2, '切到双臂 ⇒ 两条臂草稿', w.arms.length);
  ok(w.arms[0].yaw === -90 && w.arms[1].yaw === -90, '两条臂初始 yaw 各自独立可改');
  w.arms[1].yaw = 90;
  w.nArm = 1; W('wizEnsureArms')();
  ok(w.arms.length === 1, '切回单臂 ⇒ 草稿裁到 1 条', w.arms.length);
}

console.log('\n=== ③ 草稿 -> 臂模型（纯转换） ===');
{
  W('wizReset')();
  const w = W('WIZ');
  w.arms[0].yaw = -90;
  w.arms[0].links = [
    { name: 'L1', joint: 'j1', pos: [0, 0, 75],  axis: '+X', limits: [0, 122],  movable: true },
    { name: 'L2', joint: 'j2', pos: [0, 290, 0], axis: '-X', limits: [-90, 200], movable: true },
    { name: '工具', joint: '',   pos: [0, 0, 211], axis: '+Z', limits: null,     movable: false }
  ];
  const m = W('wizBuildArm')(w.arms[0]);
  ok(m.links.length === 5, '产出 1(base_link) + 3 + 1(tip) = 5 段', m.links.length);
  ok(m.links[0].name === 'base_link' && m.links[0].posMode === 'origin',
     '第 0 段固定是 base_link（原点系）', m.links[0].name);
  ok(m.links[1].name === 'L1' && near(m.links[1].pos[2], 75), 'L1 offset 与 limit 正确',
     [m.links[1].name, m.links[1].pos, m.links[1].limits]);
  ok(m.links[2].axis === '-X', 'L2 轴 = -X', m.links[2].axis);
  ok(m.links[3].type === 'fixed' && m.links[3].limits === null,
     '第 3 段标成 fixed ⇒ 限位应为 null（fixed 没有限位概念）',
     [m.links[3].type, m.links[3].limits]);
  ok(m.links[4].name === 'tip' && m.links[4].type === 'fixed', '末端自动补一个 tip', m.links[4].name);
  ok(near(m.base.yaw, -90), 'base.yaw 跟着草稿', m.base.yaw);

  /* 关键：产出的模型必须能被引擎直接接受（不是"看起来像"而已） */
  A.buildChain(m);
  ok(A.NJ === 3, '引擎建链成功，NJ = 3（两段可动 + base_joint）', A.NJ);
  ok(A.CHAIN.length === 5, '链长 = 5 段', A.CHAIN.length);
  const lim = A.qLimits();
  ok(lim.length === 3 && near(lim[1][1], 122), 'joint1 上限 122° 传下去了', lim[1]);
}

console.log('\n=== ④ 双臂草稿转出两条臂 ===');
{
  W('wizReset')();
  const w = W('WIZ');
  w.nArm = 2; W('wizEnsureArms')();
  w.arms[0].yaw = -90;  w.arms[0].links = [{ name: 'A1', joint: 'a1', pos: [0, 0, 100], axis: '+X', limits: [-90, 90], movable: true }];
  w.arms[1].yaw = 90;   w.arms[1].links = [{ name: 'B1', joint: 'b1', pos: [0, 0, 120], axis: '+X', limits: [-90, 90], movable: true },
                                           { name: 'B2', joint: 'b2', pos: [0, 150, 0], axis: '+X', limits: [-90, 90], movable: true }];
  const built = w.arms.map(W('wizBuildArm'));
  ok(built.length === 2, '转出 2 条臂');
  ok(built[0].base.yaw === -90 && built[1].base.yaw === 90, '两条臂 yaw 各自独立',
     [built[0].base.yaw, built[1].base.yaw]);
  A.buildChain(built);
  ok(A.armCount() === 2, '引擎认下 2 条臂', A.armCount());
  ok(A.ARMS[0].nJ === 2 && A.ARMS[1].nJ === 3,
     '各臂关节数不同（2 / 3）也对', [A.ARMS[0].nJ, A.ARMS[1].nJ]);
  ok(A.NJ === 5, '扁平 NJ = 5', A.NJ);
}

console.log('\n=== ⑤ 校验：空名字 / 空链要拦住 ===');
{
  W('wizReset')();
  const w = W('WIZ');
  ok(W('wizValidate')(2) === '', '默认草稿能过校验', W('wizValidate')(2));
  w.arms[0].links = [];
  ok(W('wizValidate')(2) !== '', '空链被拦下', W('wizValidate')(2));
  w.arms[0].links = [{ name: '', joint: '', pos: [0, 0, 1], axis: '+X', limits: null, movable: true }];
  ok(W('wizValidate')(2) !== '', '段没名字被拦下', W('wizValidate')(2));
  w.arms[0].links[0].name = 'ok';
  ok(W('wizValidate')(2) === '', '补上名字就放行');
}

console.log('\n=== ⑥ 装配位置 / 夹爪轴是独立项，不混进 links ===');
{
  W('wizReset')();
  const w = W('WIZ');
  w.asm = { x: -500, y: -200, z: 800 };
  w.insertAxis = 'y'; w.gripAxis = 'x';
  const m = W('wizBuildArm')(w.arms[0]);
  const j = JSON.stringify(m);
  ok(j.indexOf('insertAxis') < 0 && j.indexOf('-500') < 0,
     '臂模型里不含装配位置与夹爪轴（它们不属于几何）');
  ok(w.insertAxis === 'y' && w.gripAxis === 'x', '草稿里保留着这两项',
     [w.insertAxis, w.gripAxis]);
}

console.log('\n=== ⑧ 坐标系朝向可改（新坐标系对齐父系哪个轴）===');
{
  W('wizReset')();
  const w = W('WIZ');
  /* 默认是 ID（与父系同向） */
  ok(JSON.stringify(w.arms[0].links[0].axes) === JSON.stringify({ x: '+X', y: '+Y', z: '+Z' }),
     '默认坐标系 = ID（三轴与父系同向）', w.arms[0].links[0].axes);

  /* 强制成非 ID 的右手系：x→+X, y→-Z, z→+Y（这就是同济 link5 的那种） */
  w.arms[0].links[0].axes = { x: '+X', y: '-Z', z: '+Y' };
  const m = W('wizBuildArm')(w.arms[0]);
  ok(JSON.stringify(m.links[1].axes) === JSON.stringify({ x: '+X', y: '-Z', z: '+Y' }),
     '自定义坐标系传到了模型里', m.links[1].axes);
  A.buildChain(m);
  ok(A.NJ >= 1, '非 ID 坐标系也能建链', A.NJ);
  /* 该段相对父系的姿态不该再是单位阵 */
  const F = A.fkAll(new Array(A.NJ).fill(0));
  const R1 = F[1].R;
  const isI = Math.abs(R1[0][0] - 1) < 1e-12 && Math.abs(R1[1][1] - 1) < 1e-12 &&
              Math.abs(R1[2][2] - 1) < 1e-12;
  ok(!isI, '带坐标系旋转的段，正解出来的姿态不是单位阵（朝向真的生效了）', R1);

  /* 校验要拦住左手系与重复轴 */
  w.arms[0].links[0].axes = { x: '+X', y: '+X', z: '+Z' };
  ok(W('wizValidate')(2) !== '', '三轴重复被拦下', W('wizValidate')(2));
  w.arms[0].links[0].axes = { x: '+X', y: '-Y', z: '+Z' };   // det = -1，左手系
  ok(W('wizValidate')(2) !== '', '左手系被拦下（det=-1）', W('wizValidate')(2));
  w.arms[0].links[0].axes = { x: '+X', y: '-Z', z: '+Y' };   // det = +1，合法
  ok(W('wizValidate')(2) === '', '合法的右手系放行', W('wizValidate')(2));
}

console.log('\n=== ⑦ 接线存在 ===');
{
  /* 沙箱里 document.getElementById 返回桩元素，所以只能验"函数在、能被调用" */
  ok(typeof W('wizOpen') === 'function', 'wizOpen 存在');
  ok(typeof W('wizNext') === 'function', 'wizNext 存在');
  ok(typeof W('wizPrev') === 'function', 'wizPrev 存在');
  ok(typeof W('wizFinish') === 'function', 'wizFinish 存在');
  ok(typeof W('initWizard') === 'function', 'initWizard 存在');
  let err = '';
  try { W('wizOpen')(); W('wizNext')(); W('wizPrev')(); W('wizClose')(); }
  catch (e) { err = e.message; }
  ok(!err, '打开/下一步/上一步/关闭 不抛异常', err);
}

console.log('\n=== ⑨ 旋转角度：每个关节一条滑条，能手动驱动 ===');
{
  A.applyConfig('tongji');
  A.Q = new Array(A.NJ).fill(0);
  const nJ = A.NJ;
  ok(typeof W('onChangeJoint') === 'function', 'onChangeJoint 存在');
  ok(typeof W('resetAllJoints') === 'function', 'resetAllJoints 存在');
  ok(typeof W('syncQReadout') === 'function', 'syncQReadout 存在');

  /* 改一个关节 => Q 跟着变；同步函数不该抛 */
  W('onChangeJoint')(1, 100);
  ok(Math.abs(A.Q[1] - 100) < 1e-9, 'onChangeJoint 改到了 Q[1]', A.Q[1]);
  let err = '';
  try { W('syncQReadout')(); W('resetAllJoints')(); } catch (e) { err = e.message; }
  ok(!err, 'syncQReadout / resetAllJoints 不抛异常（沙箱里 DOM 是桩）', err);

  /* 拖动关节必须真的改变末端位置 —— 这条是"滑条有用"的实质判据 */
  const p0 = A.fkAll(A.Q).slice(-1)[0].t.slice();
  A.Q = new Array(nJ).fill(0);
  A.Q[1] = 100;
  const p1 = A.fkAll(A.Q).slice(-1)[0].t.slice();
  const d = Math.hypot(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
  ok(d > 1, 'joint1 转到 100° 后末端位置明显变化（移动 ' + d.toFixed(1) + ' mm）', d);

  /* 限位要兜住：超出范围的指令值在 qLimits 内被夹 */
  const lim = A.qLimits();
  ok(lim.length === nJ, 'qLimits 覆盖全部 ' + nJ + ' 个关节', lim.length);
  ok(lim.every(([lo, hi]) => hi > lo), '每个关节的限位区间有效');
}

console.log('\n=== ⑩ 装配位置与夹爪轴进模型 JSON（往返） ===');
{
  A.applyConfig('tongji');
  const v0 = W('vehicleOf')();
  ok(v0 && v0.asm && typeof v0.asm.x === 'number', 'vehicleOf() 能给出装配位置', v0.asm);
  ok(typeof v0.insertAxis === 'string' && typeof v0.gripAxis === 'string',
     'vehicleOf() 带插入轴/夹紧轴', [v0.insertAxis, v0.gripAxis]);

  W('setVehicle')({ asm: { x: -321, y: 45, z: 900 }, insertAxis: 'z', gripAxis: 'y' });
  ok(A.MODEL.vehicle && A.MODEL.vehicle.asm.x === -321,
     'setVehicle 写进了 MODEL.vehicle', A.MODEL.vehicle && A.MODEL.vehicle.asm);

  /* ★ 关键：序列化必须带上它（用户要的就是这个） */
  const json = JSON.stringify(A.MODEL);
  ok(json.indexOf('"vehicle"') >= 0, 'JSON 里有 vehicle 段');
  ok(json.indexOf('-321') >= 0, 'JSON 里有装配位置数值');
  ok(json.indexOf('"insertAxis"') >= 0 && json.indexOf('"gripAxis"') >= 0,
     'JSON 里有插入轴/夹紧轴');

  /* 读回来 */
  const back = JSON.parse(json);
  ok(back.vehicle && back.vehicle.asm.x === -321 && back.vehicle.insertAxis === 'z',
     '反序列化后 vehicle 完整', back.vehicle);

  /* 没有 vehicle 的老 JSON 也要能用（向后兼容） */
  const oldModel = JSON.parse(JSON.stringify(A.demoModel()));
  delete oldModel.vehicle;
  const vOld = W('vehicleOf')(oldModel);
  ok(vOld && vOld.asm && typeof vOld.asm.x === 'number',
     '老 JSON（没有 vehicle）也能兜出一个默认值', vOld.asm);
}

console.log('\n' + '='.repeat(46));
if (fail) { console.log('FAIL=' + fail + '  PASS=' + pass); process.exitCode = 1; }
else console.log('全部通过：' + pass + ' 项');
