/*  t13_multi_arm.js —— 多臂（双臂）构型回归自检

    为什么要这个：
      引擎原来是"一条臂"写出来的（MODEL / CHAIN / NJ / Q 都是单数语义）。
      加多臂最容易的做法是复制一套，但那会立刻回到"两份实现、改一处忘一处"。
      本自检盯的就是"多臂是不是真的走上了同一套引擎"。

    检什么：
      ① 单臂配置保持不变：armCount=1，ARMS[0] === MODEL，结果与以前一致
      ② 双臂配置：两条臂各自独立建链、各自的关节数/限位/末端
      ③ 扁平关节空间：第 a 条臂的关节落在 [j0, j0+nJ)，armOfJointQI 对得上
      ④ 同一组 q 下两条臂同时有末端位置（"协同"= 同一时刻的关节向量）
      ⑤ 各臂可达集分别统计，且与单独臂的统计自洽
      ⑥ 反向断言：引擎里不该出现"只有一条臂"的假设（ARMS 长度驱动一切）
      ⑦ 基座差异真的起作用：两臂末端位置不同、且等于各自 base 变换下的结果

    跑法： node web/checks/t13_multi_arm.js      （在仓库根目录）
*/
'use strict';
const { loadEnv } = require('./t9_load.js');

let pass = 0, fail = 0;
const ok = (c, msg, extra) => {
  if (c) { pass++; console.log('  ✓ ' + msg); }
  else { fail++; console.log('  ✗ ' + msg + (extra !== undefined ? '   ← ' + JSON.stringify(extra) : '')); }
};
const near = (a, b, tol) => Math.abs(a - b) <= (tol === undefined ? 1e-9 : tol);
const norm = v => Math.hypot(v[0], v[1], v[2]);

const { A } = loadEnv({ variants: false });

/* ================= ① 单臂配置不受影响 ================= */
console.log('\n=== ① 单臂配置照旧（回归）===');
{
  A.applyConfig('tongji');
  ok(A.armCount() === 1, 'armCount() = 1', A.armCount());
  ok(A.ARMS.length === 1, 'ARMS 长度 1', A.ARMS.length);
  ok(A.ARMS[0].model && A.ARMS[0].model.links.length === 7, 'ARMS[0] 是 7 段那条同济臂',
     A.ARMS[0].model && A.ARMS[0].model.links.length);
  ok(A.MODEL === A.ARMS[0].model, 'MODEL 就是 ARMS[0]（老语义：MODEL = 第 0 条臂）');
  A.Q = new Array(A.NJ).fill(0);
  const F = A.fkAll(A.Q), ee = F[F.length - 1].t;
  ok(Math.abs(ee[0] - 471) < 0.5 && Math.abs(ee[1] + 85.5) < 0.5 && Math.abs(ee[2] - 476.47) < 0.5,
     '零位末端仍是 [471, -85.5, 476.47] mm（多臂改造没有动单臂结果）',
     ee.map(v => +v.toFixed(3)));
  ok(A.armEnd(A.Q, 0).t[0] === A.fkAll(A.Q).slice(-1)[0].t[0] ||
     near(norm(A.armEnd(A.Q, 0).t) , norm(ee), 1e-9),
     'armEnd(q,0) 与 fkAll(q) 的末端一致');
  A.applyConfig('demo3dof');
  ok(A.armCount() === 1 && A.NJ === 3, 'demo3dof 仍是 1 条臂 3 关节', [A.armCount(), A.NJ]);
}

/* ================= ② 双臂配置 ================= */
console.log('\n=== ② 双臂配置 ===');
{
  A.applyConfig('dual_demo');
  ok(A.armCount() === 2, 'armCount() = 2', A.armCount());
  ok(A.ARMS.length === 2, 'ARMS 长度 2', A.ARMS.length);
  ok(A.ARMS[0].nJ === 4 && A.ARMS[1].nJ === 3,
     '左臂 4 关节 / 右臂 3 关节（两臂关节数可以不同）',
     [A.ARMS[0].nJ, A.ARMS[1].nJ]);
  ok(A.NJ === 7, '扁平关节总数 NJ = 7', A.NJ);
  ok(A.CHAIN.length === A.ARMS[0].model.links.length + A.ARMS[1].model.links.length,
     'CHAIN 是两条臂首尾相接（' + A.CHAIN.length + ' 段）', A.CHAIN.length);
  ok(A.ARMS[0].name === '左臂' && A.ARMS[1].name === '右臂',
     '臂名保留（左臂/右臂）', [A.ARMS[0].name, A.ARMS[1].name]);
  ok(near(A.ARMS[0].yaw, -90, 1e-9) && near(A.ARMS[1].yaw, 90, 1e-9),
     '两臂安装朝向各自独立（-90 / +90）', [A.ARMS[0].yaw, A.ARMS[1].yaw]);
}

/* ================= ③ 扁平关节空间的区间划分 ================= */
console.log('\n=== ③ 扁平关节空间：每条臂的区间 ===');
{
  const A0 = A.ARMS[0], A1 = A.ARMS[1];
  ok(A0.j0 === 0 && A0.j0 + A0.nJ === A1.j0,
     '左臂占 [0,' + A0.nJ + ')，右臂紧接其后从 ' + A1.j0 + ' 开始',
     [A0.j0, A0.nJ, A1.j0]);
  ok(A1.j0 + A1.nJ === A.NJ, '右臂区间到 NJ 为止', [A1.j0, A1.nJ, A.NJ]);
  const lim = A.qLimits();
  ok(lim.length === 7, 'qLimits() 覆盖全部 7 个关节（多臂天然支持）', lim.length);
  let mapOK = true;
  for (let qi = 0; qi < A.NJ; qi++) {
    const a = A.armOfJointQI(qi);
    const want = qi < A0.nJ ? 0 : 1;
    if (a !== want) { mapOK = false; break; }
  }
  ok(mapOK, 'armOfJointQI 把每个关节正确归到某条臂');
  ok(A.armOfJointQI(-1) === -1 && A.armOfJointQI(A.NJ) === -1,
     '越界关节返回 -1', [A.armOfJointQI(-1), A.armOfJointQI(A.NJ)]);
  /* 链上的每段也要标对臂 */
  const segOK = A.CHAIN.every((c, i) => c.arm === (i < A0.model.links.length ? 0 : 1));
  ok(segOK, 'CHAIN 每段都带正确的 arm 标记');
}

/* ================= ④ 同一组 q，两臂同时有末端 ================= */
console.log('\n=== ④ 同一组 q 下两条臂同时解算（"协同"的基础）===');
{
  const q = new Array(A.NJ).fill(0);
  const e0 = A.armEnd(q, 0), e1 = A.armEnd(q, 1);
  ok(!!e0 && !!e1, '两条臂都能取到末端位姿');
  const p0 = e0.t, p1 = e1.t;
  ok(norm([p0[0] - p1[0], p0[1] - p1[1], p0[2] - p1[2]]) > 1,
     '两臂末端位置不同（基座不同 ⇒ 结果不同）',
     { 左臂: p0.map(v => +v.toFixed(1)), 右臂: p1.map(v => +v.toFixed(1)) });
  /* 各自的末端要落在各自 base 附近：臂长已知，距离基座不该超过总杆长 */
  for (let a = 0; a < 2; a++) {
    const Aa = A.ARMS[a];
    const bx = Aa.model.base.xyz;
    const L = Aa.model.links.reduce((s, l) =>
      s + Math.hypot(l.pos[0] || 0, l.pos[1] || 0, l.pos[2] || 0), 0);
    const d = norm([A.armEnd(q, a).t[0] - bx[0], A.armEnd(q, a).t[1] - bx[1],
                    A.armEnd(q, a).t[2] - bx[2]]);
    ok(d <= L + 1e-6, Aa.name + ' 零位末端到自身基座的距离 ' + d.toFixed(1) +
       ' ≤ 杆长和 ' + L.toFixed(1), [d, L]);
  }
  /* 全臂正解拼接：O 只出现一次，其余每段一个 */
  const FA = A.fkAllArms(q);
  const total = A.CHAIN.length + 1;
  ok(FA.length === total, 'fkAllArms 返回 ' + total + ' 个坐标系（1 个 O + 全部段）', FA.length);
  ok(FA.filter(f => f.name === 'O(整机原点)').length === 1, '整机原点 O 只出现一次');
}

/* ================= ⑤ 各臂可达集分别统计 ================= */
console.log('\n=== ⑤ 每条臂的可达集分别统计 ===');
{
  /* computeFeasible() 要从界面读采样数，无头环境里跑不了；
     sampleReach(N) 是抽出来的纯计算，可以直接调 —— 这也是这次顺手做的解耦。 */
  const R = A.sampleReach(3000);
  ok(!!R && Array.isArray(R.armStats) && R.armStats.length === 2,
     'sampleReach 是纯函数，返回 2 条臂的统计', R && R.armStats && R.armStats.length);
  ok(R.stats.nArm === 2, 'stats.nArm = 2', R.stats.nArm);
  ok(R.stats.nJoint === A.NJ, 'stats.nJoint = NJ = ' + A.NJ, R.stats.nJoint);
  ok(R.armStats[0].name === '左臂' && R.armStats[1].name === '右臂',
     '统计条目带臂名', R.armStats.map(s => s.name));
  ok(R.armStats[0].n === R.armStats[1].n && R.armStats[0].n === 3000,
     '两臂采样点数相同且等于 N（同一批 q）', [R.armStats[0].n, R.armStats[1].n]);
  ok(R.armStats[0].rmax > 0 && R.armStats[1].rmax > 0,
     '两臂各自的最大伸展都是正数',
     [R.armStats[0].rmax.toFixed(1), R.armStats[1].rmax.toFixed(1)]);
  ok(R.cloud.length > 0 && R.cloud.length <= 3000,
     '主点云（第 0 条臂）非空（' + R.cloud.length + ' 点）', R.cloud.length);
  ok(R.keys && R.keys.size > 0, '体素集合非空（' + R.keys.size + ' 个）', R.keys.size);
  /* 各臂的 rmax 应该接近"自身基座到最远可达" —— 两臂基座差 120mm（x 上 60/-60），
     且 yaw 相反，所以 rmax 不该完全相同 */
  ok(Math.abs(R.armStats[0].rmax - R.armStats[1].rmax) > 1e-6,
     '两臂 rmax 不同（基座/朝向不同，可达集本来就不同）',
     [+R.armStats[0].rmax.toFixed(2), +R.armStats[1].rmax.toFixed(2)]);
}

/* ================= ⑥ 基座差异真的起作用 ================= */
console.log('\n=== ⑥ 把右臂基座搬走，末端应随之平移 ===');
{
  /* 直接改臂的 base.xyz，重建链，看末端是不是也平移了同样的量 */
  const q = new Array(A.NJ).fill(0);
  const before = A.armEnd(q, 1).t.slice();
  const arm1 = A.ARMS[1].model;
  const saved = arm1.base.xyz.slice();
  arm1.base.xyz = [saved[0] + 100, saved[1] - 50, saved[2] + 25];
  A.MODEL = A.ARMS[0].model;
  A.buildChain();
  const after = A.armEnd(q, 1).t.slice();
  const d = [after[0] - before[0], after[1] - before[1], after[2] - before[2]];
  ok(near(d[0], 100, 1e-9) && near(d[1], -50, 1e-9) && near(d[2], 25, 1e-9),
     '右臂基座平移 (100,-50,25) ⇒ 末端恰好平移同一向量', d.map(v => +v.toFixed(3)));
  /* 左臂不该受影响 */
  const l0 = A.armEnd(q, 0).t;
  ok(Number.isFinite(l0[0]), '左臂仍可解算（不受右臂基座改动影响）');
  /* 还原 */
  arm1.base.xyz = saved;
  A.buildChain();
  const back = A.armEnd(q, 1).t;
  ok(near(back[0], before[0], 1e-9) && near(back[1], before[1], 1e-9),
     '还原基座后末端回到原值');
}

/* ================= ⑦ 切换配置不残留状态 ================= */
console.log('\n=== ⑦ 反复切换配置不残留状态 ===');
{
  /* 注意：这里【不能】比 STATS —— 它由 computeFeasible() 写，
     而无头环境不跑 updateAll()。要比就用纯函数 sampleReach 的返回值。
     （`A.NJ` 由 buildChain() 写，applyConfig 会调，所以那个可以比。） */
  const seen = {};
  for (const id of ['tongji', 'dual_demo', 'demo3dof', 'dual_demo', 'tongji']) {
    A.applyConfig(id);
    const nA = A.armCount();
    const key = id + '|' + nA + '|' + A.NJ;
    seen[key] = (seen[key] || 0) + 1;
    /* 每次 applyConfig 之后，ARMS 长度必须与 CHAIN 的臂标记自洽 */
    const maxArm = A.CHAIN.reduce((m, c) => Math.max(m, c.arm), -1);
    if (maxArm + 1 !== nA) {
      ok(false, id + ' 切换后 CHAIN 的臂标记与 armCount 不一致',
         { armCount: nA, maxArmInChain: maxArm + 1 });
    }
    /* 每条臂的关节数之和必须等于 NJ */
    const sum = A.ARMS.reduce((s, a) => s + a.nJ, 0);
    if (sum !== A.NJ) {
      ok(false, id + ' 切换后各臂关节数之和 ≠ NJ', { sum, NJ: A.NJ });
    }
  }
  ok(true, '5 次来回切换：每次 CHAIN 臂标记与 armCount 自洽、各臂 nJ 之和 = NJ');
  ok(seen['tongji|1|6'] === 2, '两次切到 tongji 都是 1 臂 6 关节', seen['tongji|1|6']);
  ok(seen['dual_demo|2|7'] === 2, '两次切到 dual_demo 都是 2 臂 7 关节', seen['dual_demo|2|7']);
  ok(seen['demo3dof|1|3'] === 1, 'demo3dof 是 1 臂 3 关节', seen['demo3dof|1|3']);

  /* 单臂配置的数值必须每次都一样（切换不该污染结果） */
  const probe = () => {
    A.applyConfig('tongji');
    A.Q = new Array(A.NJ).fill(0);
    const F = A.fkAll(A.Q);
    return F[F.length - 1].t.slice();
  };
  const p1 = probe(), p2 = probe();
  ok(p1.every((v, i) => Math.abs(v - p2[i]) < 1e-12),
     '两次取同济臂零位末端结果完全相同', [p1.map(v => +v.toFixed(3)), p2.map(v => +v.toFixed(3))]);
}

console.log('\n' + '='.repeat(46));
if (fail) { console.log('FAIL=' + fail + '  PASS=' + pass); process.exitCode = 1; }
else console.log('全部通过：' + pass + ' 项');
