/*  t12_arm_swap.js —— 换构型回归自检：引擎是不是真的与"具体是哪条臂"无关

    为什么要这个：
      这个仓库长期只有一条臂（同济示例臂）。一旦有人把某个值写死进引擎，
      只有"真的换一条臂跑一遍"才发现得了。本自检就是那条安全网。

    检什么：
      ① 3 自由度臂：建链 / 正解 / 采样 / IK 全链路能跑，数值合理
      ② 7 自由度臂：同上（验证 NJ 不是写死的）
      ③ 连 base 安装朝向不同（yaw=0 / -90）也照样能跑
      ④ 换回同济示例臂，结果与它自己的定义一致（确认换臂不影响原模型）
      ⑤ 反向断言：引擎里不该出现"同济"专属常量

    跑法： node web/checks/t12_arm_swap.js      （在仓库根目录）
*/
'use strict';
const fs = require('fs'), path = require('path');
const { loadEnv } = require('./t9_load.js');
const P = require('../lib/paths');

let pass = 0, fail = 0;
const ok = (c, msg, extra) => {
  if (c) { pass++; console.log('  ✓ ' + msg); }
  else { fail++; console.log('  ✗ ' + msg + (extra !== undefined ? '   ← ' + JSON.stringify(extra) : '')); }
};
const near = (a, b, tol) => Math.abs(a - b) <= (tol === undefined ? 1e-6 : tol);

const { A } = loadEnv({ variants: false });

/* ---------- 造测试臂的工具：只有 ID 三轴，不碰同济那套 ---- */
const ID = { x: '+X', y: '+Y', z: '+Z' };
const rev = (name, joint, pos, axis, limits) =>
  ({ name, joint, type: 'revolute', posMode: 'parent', pos, axes: ID, axis, limits });
const fix = (name, pos) =>
  ({ name, joint: name + '_fix', type: 'fixed', posMode: 'parent', pos, axes: ID, axis: '+Z', limits: null });

function mkArm(name, yaw, mids) {
  return {
    name,
    base: { xyz: [0, 0, 0], yaw },
    links: [
      { name: 'base_link', joint: 'base_joint', type: 'revolute', posMode: 'origin',
        pos: [0, 0, 0], axes: ID, axis: '+Z', limits: null },
      ...mids,
      fix('tip', [0, 0, 90])
    ]
  };
}

/* 确定性随机数（mulberry32）—— 两次采样的 q 必须逐点相同，
   否则"最大伸展"这种估计量会被抽样噪声淹没，比不出朝向的差别 */
function mulberry32(a) {
  return function () {
    a |= 0; a = a + 0x6D2B79F5 | 0;
    let t = Math.imul(a ^ a >>> 15, 1 | a);
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}

/* 在给定模型上跑一遍全链路，返回统计量 */
function exercise(model, label, seed) {
  const R = { label };
  /* ★ 显式把模型交给 buildChain（多臂改造后 buildChain 支持传参）。
     直接写 A.MODEL = model 再空参调 buildChain 是不行的 ——
     空参会回落到"当前配置的臂列表"，手上这份就被忽略了。 */
  A.buildChain(model);
  R.nJ = A.NJ;
  R.limits = A.qLimits();
  R.nLim = R.limits.length;

  // 建链结果的规模必须对得上
  R.nLinks = model.links.length;
  R.chainLen = A.CHAIN.length;
  R.framesOK = A.CHAIN.length === model.links.length;

  // 零位正解
  A.Q = new Array(A.NJ).fill(0);
  const F = A.fkAll(A.Q);
  R.nFrames = F.length;
  R.home = F[F.length - 1].t.slice();

  // 杆长和（纯几何量，应该与限位/朝向无关）
  R.armLen = A.CHAIN.reduce((s, c) => s + Math.hypot(c.offset[0], c.offset[1], c.offset[2]), 0);

  // 随机采样：用固定种子，保证不同构型跑的是同一批 q
  const rng = mulberry32(seed === undefined ? 20261011 : seed);
  const N = 4000;
  let rmax = 0, rmin = Infinity;
  R.pts = new Array(N);
  for (let s = 0; s < N; s++) {
    const q = R.limits.map(([lo, hi]) => lo + rng() * (hi - lo));
    const p = A.fkAll(q).slice(-1)[0].t;
    R.pts[s] = p;
    const r = Math.hypot(p[0], p[1], p[2]);
    if (r > rmax) rmax = r;
    if (r < rmin) rmin = r;
  }
  R.rmax = rmax; R.rmin = rmin; R.N = N;

  // IK：拿零位末端位置当目标，应该能解回去
  A.Q = new Array(A.NJ).fill(0);
  const F2 = A.fkAll(A.Q);
  const tgt = F2[F2.length - 1].t.slice();
  const r = A.ikSolve6(tgt, [0, 0, 1], new Array(A.NJ).fill(0), 200, 2);
  R.ikOk = !!r.ok;
  R.ikE = r.e;
  return R;
}

/* ================= ① / ② / ③ 各种构型 ================= */
console.log('\n=== ① 3 自由度臂（yaw=0）===');
{
  const m = mkArm('3DOF-test', 0, [
    rev('l1', 'j1', [0, 0, 100], '+X', [-90, 90]),
    rev('l2', 'j2', [0, 200, 0], '+X', [-90, 90])
  ]);
  let R;
  try { R = exercise(m, '3dof'); } catch (e) { R = { label: '3dof', err: e.message }; }
  ok(!R.err, '3 自由度臂全链路不抛异常', R.err);
  if (!R.err) {
    ok(R.nJ === 3, 'NJ = 3（关节数是按模型算的，不是写死 6）', R.nJ);
    ok(R.nLim === 3, 'qLimits 返回 3 组', R.nLim);
    ok(R.nFrames === R.nLinks + 1,
       '正解返回 links+1 个坐标系（1 个整机原点 O + 每段一个）= ' + R.nFrames, R.nFrames);
    ok(R.framesOK, '链长与 links 数一致（' + R.chainLen + '）');
    ok(R.rmax > 0 && Number.isFinite(R.rmax), '最大伸展是有限正数', R.rmax);
    ok(near(R.rmin, 0, 200), '最小伸展合理（能收到自己附近）', R.rmin);
    ok(R.ikOk, 'ikSolve6 能解回零位末端位置', { ok: R.ikOk, e: R.ikE });
  }
}

console.log('\n=== ② 7 自由度臂（验证 NJ 不写死）===');
{
  const m = mkArm('7DOF-test', 0, [
    rev('l1', 'j1', [0, 0, 100], '+X', [-90, 90]),
    rev('l2', 'j2', [0, 200, 0], '+X', [-90, 90]),
    rev('l3', 'j3', [0, 0, 180], '+Z', null),
    rev('l4', 'j4', [0, 150, 0], '+X', [-90, 90]),
    rev('l5', 'j5', [0, 0, 120], '+Z', null),
    rev('l6', 'j6', [0, 80, 0], '+X', [-90, 90])
  ]);
  let R;
  try { R = exercise(m, '7dof'); } catch (e) { R = { label: '7dof', err: e.message }; }
  ok(!R.err, '7 自由度臂全链路不抛异常', R.err);
  if (!R.err) {
    ok(R.nJ === 7, 'NJ = 7', R.nJ);
    ok(R.nFrames === R.nLinks + 1, '正解返回 links+1 个坐标系 = ' + R.nFrames, R.nFrames);
    ok(R.rmax > 0 && Number.isFinite(R.rmax), '最大伸展是有限正数', R.rmax);
    ok(R.ikOk, 'ikSolve6 在 7 自由度下也能解', { ok: R.ikOk, e: R.ikE });
  }
}

console.log('\n=== ③ base 安装朝向 yaw = -90（与同济同款）也照样跑 ===');
{
  const m = mkArm('3DOF-yaw90', -90, [
    rev('l1', 'j1', [0, 0, 100], '+X', [-90, 90]),
    rev('l2', 'j2', [0, 200, 0], '+X', [-90, 90])
  ]);
  let Y0, Y90;
  try {
    const mids = m.links.slice(1, -1);          // 去掉 base_link 和 tip 之外的都在这里
    Y0  = exercise(mkArm('3DOF-yaw0',   0, mids), 'yaw0',   20261011);
    Y90 = exercise(mkArm('3DOF-yaw-90', -90, mids), 'yaw-90', 20261011);
  } catch (e) { Y0 = { err: e.message }; Y90 = { err: e.message }; }
  ok(!Y0.err && !Y90.err, 'yaw=0 与 yaw=-90 都能跑', [Y0.err, Y90.err]);
  if (!Y0.err && !Y90.err) {
    ok(near(Y0.armLen, Y90.armLen, 1e-9), '杆长和与安装朝向无关（纯几何量）',
       [Y0.armLen, Y90.armLen]);
    ok(near(Y0.rmax, Y90.rmax, 1e-6), '最大伸展与安装朝向无关（同一批 q，只差一个刚体旋转）',
       [Y0.rmax, Y90.rmax]);
    // 零位末端位置应该正好差一个绕 Z 的 yaw 旋转
    const d2r = Math.PI / 180, c = Math.cos(-90 * d2r), s = Math.sin(-90 * d2r);
    const pred = [c * Y0.home[0] - s * Y0.home[1], s * Y0.home[0] + c * Y0.home[1], Y0.home[2]];
    const err = Math.hypot(pred[0] - Y90.home[0], pred[1] - Y90.home[1], pred[2] - Y90.home[2]);
    ok(err < 1e-9, 'yaw=-90 的末端位置 = Rz(-90°) · yaw=0 的位置（朝向只是换个参考系）',
       { err, yaw0: Y0.home, yaw90: Y90.home });

    // 整条采样点云也应该逐点只差那个旋转 —— 这条最硬：
    // 它证明"改朝向"除了换参考系之外，什么都没动
    let worst = 0;
    for (let i = 0; i < Y0.pts.length; i++) {
      const a = Y0.pts[i], b = Y90.pts[i];
      const pr = [c * a[0] - s * a[1], s * a[0] + c * a[1], a[2]];
      worst = Math.max(worst, Math.hypot(pr[0] - b[0], pr[1] - b[1], pr[2] - b[2]));
    }
    ok(worst < 1e-9, '整条采样点云逐点只差 Rz(yaw)（' + Y0.pts.length + ' 个点，最大偏差 ' +
       worst.toExponential(2) + ' mm）', worst);
  }
}

console.log('\n=== ④ 换回同济示例臂，自身一致 ===');
{
  /* ★ 用 configModel('tongji') 取【配置里原始的那条臂】，
     不能用 A.demoModel() —— 后者是"当前模型"（前面几节已经换过臂了）。
     这个区别本身也值得记住：配置 = 原始定义，MODEL = 当前在算的东西。 */
  const demo = A.configModel('tongji');
  ok(!!demo && demo.links.length === 7, '配置 tongji 里有 7 段', demo && demo.links.length);
  A.MODEL = demo; A.buildChain();
  ok(A.NJ === 6, '同济示例臂 NJ = 6', A.NJ);
  A.Q = new Array(A.NJ).fill(0);
  const F = A.fkAll(A.Q), ee = F[F.length - 1].t;
  ok(Math.abs(ee[0] - 471) < 0.5 && Math.abs(ee[1] + 85.5) < 0.5 && Math.abs(ee[2] - 476.47) < 0.5,
     '零位末端位置 = [471, -85.5, 476.47] mm（与记录一致）', ee.map(v => +v.toFixed(3)));
  const lim = A.qLimits();
  ok(lim.length === 6, '6 组限位', lim.length);
  ok(near(lim[1][1], 122, 1e-9), 'joint1 上限 122°', lim[1][1]);
  ok(near(lim[2][0], -90, 1e-9) && near(lim[2][1], 200, 1e-9), 'joint2 限位 [-90, 200]', lim[2]);
  ok(near(demo.base.yaw, -90, 1e-9), '安装 yaw = -90°', demo.base.yaw);
}

console.log('\n=== ④b 配置里的规则 / 车体尺寸还在（搬出去之后没丢）===');
{
  const cfg = A.listConfigs();
  ok(cfg.includes('tongji'), '配置清单里有 tongji', cfg);
  ok(cfg.includes('demo3dof'), '配置清单里有 demo3dof（最小测试臂）', cfg);
  A.applyConfig('tongji');
  ok(!!A.RULES && !!A.RULES.levels && Object.keys(A.RULES.levels).length === 3,
     '同济配置带 3 级任务域规则');
  ok(A.RULES.baseH === 170, 'baseH = 170', A.RULES.baseH);
  ok(A.CHASSIS.size[0] === 580 && A.CHASSIS.size[2] === 105, '底盘尺寸 580×580×105',
     A.CHASSIS.size);
  ok(A.SIZER.lwh[0] === 1200 && A.SIZER.ballD === 1500, '尺寸规则 1200/1500', A.SIZER);
  ok(A.CHASSIS_V === 2, 'CHASSIS_V = 2', A.CHASSIS_V);

  /* 切到没有规则的配置：不能崩，RULES/MAP 应为空 */
  A.applyConfig('demo3dof');
  ok(A.RULES === null, 'demo3dof 没有规则 ⇒ RULES = null', A.RULES);
  ok(A.MAP === null, 'demo3dof 没有规则 ⇒ MAP = null', A.MAP);
  ok(A.NJ === 3, 'demo3dof 是 3 自由度', A.NJ);
  /* 切回来 */
  A.applyConfig('tongji');
  ok(A.NJ === 6 && !!A.RULES, '切回 tongji 正常', { NJ: A.NJ, hasRules: !!A.RULES });
}

console.log('\n=== ⑤ 反向断言：引擎源码里不该有同济专属常量 ===');
{
  /* 引擎 = 01/02/03 三个模块里【属于算法】的部分。
     现在还做不到完全分离（本自检就是用来暴露这个的），所以先只盯最刺眼的几个：
     示例臂的杆长不该出现在引擎里 —— 它应该来自配置。 */
  const mods = ['01-kinematics.js', '02-task-rules.js', '03-chassis-size.js'];
  const ENGINE_PAT = /401\.47|211|85\.5|290/;
  for (const f of mods) {
    const src = fs.readFileSync(path.join(P.MODULES, f), 'utf8');
    const hit = ENGINE_PAT.test(src);
    ok(!hit, '引擎模块 ' + f + ' 里没有示例臂的杆长常量',
       hit ? (src.match(ENGINE_PAT) || [])[0] : undefined);
  }
}

console.log('\n' + '='.repeat(46));
if (fail) { console.log('FAIL=' + fail + '  PASS=' + pass); process.exitCode = 1; }
else console.log('全部通过：' + pass + ' 项');
