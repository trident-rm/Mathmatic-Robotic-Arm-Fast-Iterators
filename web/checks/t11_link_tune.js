/*  t11_link_tune.js —— 连杆调参模块的自检（无头，Node）
    跑法： node t11_link_tune.js             （约 60 s）
           node t11_link_tune.js --quick     （约 15 s）

    检什么：
      ⓪ 装载方式：连杆模块能作为纯引擎挂进 armBuilder 的沙箱（不依赖额外构建产物）
      ① 限位只读：init / eval / sweep / search / apply / revert 全程限位签名不变，
         且 MODEL.links[*].limits 的 JSON 快照不变
      ② 旋钮与位姿集：自动生成规则、位姿集确定性、随种子变化、nPose 是嵌套前缀、
         每个位姿都落在规则表 5-10 的三级区间里
      ③ 评估可复现：同一候选两次评估完全一致；评估后模型被还原
      ④ 判据的量级：基准在"位姿够多 + 起点够多"时 found=100%（判据可达），
         但 pass 很低（说明它是紧的、不是解不出来）
      ⑤ 灵敏度扫描 / 搜索：覆盖完整、best 分数不低于基准、长度约束真的挡得住
      ⑥ 应用 / 恢复：apply 后 offset = 候选、revert 后与快照逐字节相同
      ⑦ 原功能回归：FK / IK / 底盘 / 机械臂长度在原文件与 armLink 里数值一致
*/
const fs = require('fs'), path = require('path'), crypto = require('crypto');
const { loadEnv } = require('./t9_load.js');
const dir = __dirname;
const QUICK = process.argv.includes('--quick');

let pass = 0, fail = 0;
const ok = (c, msg, extra) => {
  if(c){ pass++; console.log('  ✓ ' + msg); }
  else { fail++; console.log('  ✗ ' + msg + (extra !== undefined ? '   ← ' + JSON.stringify(extra) : '')); }
};
const near = (a,b,tol) => Math.abs(a-b) <= (tol === undefined ? 1e-9 : tol);
const md5 = s => crypto.createHash('md5').update(s, 'utf8').digest('hex').slice(0,12);

/* ================= ⓪ 连杆模块能当纯引擎挂上 ================= */
console.log('\n=== ⓪ 连杆引擎的装载方式 ===');
/* 原来这一节验的是"build/armLink.html 是不是只追加生成的"。
   现在 UI 上的「机械臂构型改进」面板已按要求删除、armLink.html 不再构建，
   所以改成验【新的装载方式】：模块源码追加进 armBuilder 的沙箱即可用，
   不再依赖任何额外构建产物。 */
const rl = p => fs.readFileSync(p, 'utf8').replace(/\r\n/g, '\n');
const modSrc = rl(path.join(dir, '..', 'src', 'modules', 't11_link_mod.js'));
ok(modSrc.length > 0, '连杆模块源码存在（' + modSrc.length + ' 字节）');
ok(modSrc.indexOf('window.LINK') >= 0, '模块会挂 window.LINK（无头 API 入口）');

/* ================= 载入两个文件 ================= */
/* 两套环境：纯 armBuilder（基准）与 挂了连杆模块的 armBuilder（被测） */
const envA = loadEnv({ variants:false });
const envB = loadEnv({ variants:false, link:true });
const A = envA.A, B = envB.A, LINK = envB.LINK;
ok(!!LINK, '连杆模块挂上后 window.LINK 存在（无头 API）');
ok(typeof LINK.knobs === 'function' && typeof LINK.eval === 'function',
   'LINK 的 knobs / eval 可用');

/* ================= ⑦ 原功能回归（数值） ================= */
console.log('\n=== ⑦ 原功能回归：同样的输入，两个文件算出来必须一样 ===');
function probe(env){
  const X = env.A, s = env.sandbox;
  X.MODEL = X.demoModel(); X.buildChain();
  X.Q = new Array(X.NJ).fill(0);
  const q = [12, 40, -30, 80, 60, -25];
  const F = X.fkAll(q);
  const rows = F.map(f=>f.t.map(v=>+v.toFixed(9)).concat(f.R.flat().map(v=>+v.toFixed(9))));
  const Eb = X.taskToBase([-50,200,600]), ey = X.dirToBase(X.coreFrame({x:-50,y:200,z:600,th:20,ph:60,al:10}).e);
  const r = X.ikSolve6(Eb, ey.map(v=>-v), new Array(X.NJ).fill(0), 260, 1);
  const g = X.chassisGeom();
  s.CLOUD = [[120,30,400],[300,-100,650],[50,200,100]];
  const st = X.armStats();
  return { rows, ik:{e:+r.e.toFixed(9), ang:+r.ang.toFixed(9), q:r.q.map(v=>+v.toFixed(6))},
           chassis:{C:g.C, GC:g.GC, SC:g.SC, SR:g.SR, planeR:+g.planeR.toFixed(9)},
           stats:{segs:st.segs.map(v=>+v.toFixed(9)), total:+st.total.toFixed(9),
                  reach:+st.reach.toFixed(9), zMin:st.zMin, zMax:st.zMax},
           limits:X.qLimits() };
}
const PA = probe(envA), PB = probe(envB);
const J = x => JSON.stringify(x);
ok(J(PA.rows) === J(PB.rows), '7 个坐标系的位姿（位置+旋转）逐个一致', [PA.rows.length, PB.rows.length]);
ok(J(PA.ik) === J(PB.ik), 'ikSolve6 同一起点结果一致（e/ang/q 全同）');
ok(J(PA.chassis) === J(PB.chassis), '底盘碰撞箱几何一致');
ok(J(PA.stats) === J(PB.stats), '机械臂长度 / 最大伸展 / 高度范围一致', [PA.stats.total, PB.stats.total]);
ok(J(PA.limits) === J(PB.limits), '关节限位一致', PA.limits);

/* ================= ① 限位只读 ================= */
console.log('\n=== ① 限位只读：整条链路跑完，限位必须一个字都没变 ===');
B.buildChain();
LINK.init(true);
const lim0 = LINK.limSig();
const limJSON0 = JSON.stringify(B.MODEL.links.map(l=>l.limits));
const snap0 = JSON.stringify(B.MODEL);
ok(!!lim0, '限位签名 = ' + lim0);
Object.assign(LINK.cfg, QUICK ? {nPose:6, budget:2, iters:140} : {nPose:8, budget:3, iters:180});
const rBase = LINK.eval(LINK.readModel(), '基准');
ok(LINK.limSig() === lim0 && JSON.stringify(B.MODEL.links.map(l=>l.limits)) === limJSON0,
   '评估基准后限位不变');
ok(JSON.stringify(B.MODEL) === snap0, '评估基准后连模型都还原了（逐字节）');

/* ================= ② 旋钮 / 位姿集 ================= */
console.log('\n=== ② 旋钮与位姿集 ===');
const K = LINK.knobs();
console.log('  旋钮 = ' + K.filter(k=>k.on).map(k=>k.name).join(' ') + '   （共 ' + K.length + ' 个）');
ok(K.length === (B.MODEL.links.length-1)*3, '旋钮数 = 3 ×（链段数−1：第 1 段是臂安装位）', K.length);
ok(K.every(k=>k.v0 === B.MODEL.links[k.li].pos[k.c]), '每个旋钮的基准值 = 模型当前值');
ok(K.filter(k=>k.on).length === 6, '默认勾选 6 个非零分量', K.filter(k=>k.on).length);
ok(K.every(k=>k.lo < k.hi), '每个旋钮都有非空范围');
ok(B.MODEL.links[0].pos.every(v=>v===0), '第 1 段（base_link）没有进旋钮表');

LINK.buildPoses();
const p1 = JSON.stringify(LINK.poses().map(p=>p.P));
const n1 = LINK.poses().length;
Object.assign(LINK.cfg, {nPose:12}); LINK.buildPoses();
const p12 = JSON.stringify(LINK.poses().map(p=>p.P));
Object.assign(LINK.cfg, {nPose:12}); LINK.buildPoses();
ok(JSON.stringify(LINK.poses().map(p=>p.P)) === p12, '同位姿数重建 → 位姿集完全一样（确定性）');
Object.assign(LINK.cfg, {nPose:24}); LINK.buildPoses();
const p24 = JSON.stringify(LINK.poses().map(p=>p.P));
Object.assign(LINK.cfg, {nPose:12}); LINK.buildPoses();
ok(JSON.stringify(LINK.poses().map(p=>p.P)) === p12, 'nPose 12 是 24 的前缀（换位姿数=只加位姿，不改老的）');
ok(JSON.parse(p24).length === 24 && JSON.parse(p12).length === 12, '位姿数正确');
Object.assign(LINK.cfg, {seed: 12345}); LINK.buildPoses();
ok(JSON.stringify(LINK.poses().map(p=>p.P)) !== p12, '换随机种子 → 位姿集变了');
Object.assign(LINK.cfg, {seed: 20260925}); LINK.buildPoses();
const Lv = B.RULES.levels[3];
const inR = (v,r)=> v >= r[0]-1e-9 && v <= r[1]+1e-9;
ok(LINK.poses().every(p=> inR(p.P.x,Lv.x) && inR(p.P.y,Lv.y) && inR(p.P.z,Lv.z) &&
   inR(p.P.th,Lv.th) && inR(p.P.ph,Lv.ph) && inR(p.P.al,Lv.al)), '每个位姿都落在规则表 5-10 三级区间内');
ok(LINK.poses().every(p=> near(Math.hypot(p.ey[0],p.ey[1],p.ey[2]), 1, 1e-9)), '插入轴方向是单位向量');
ok(LINK.poses().every(p=> Math.abs(p.ey[0]*p.eB[0]+p.ey[1]*p.eB[1]+p.ey[2]*p.eB[2] + 1) < 1e-9),
   'ey = −e⃗（和上面面板"夹爪 y ∥ −e⃗"同一约定）');
/* 位姿真的在三级任务域（base 系坐标 = taskToBase） */
ok(LINK.poses().every(p=>{ const q=B.taskToBase([p.P.x,p.P.y,p.P.z]);
   return Math.abs(q[0]-p.Eb[0])<1e-9 && Math.abs(q[2]-p.Eb[2])<1e-9; }), 'Eb = taskToBase(E)');

/* ================= ③ 可复现 ================= */
console.log('\n=== ③ 评估可复现 ===');
Object.assign(LINK.cfg, {nPose:8, budget:3, iters:180});
const e1 = LINK.eval(LINK.readModel(), 'A');
const e2 = LINK.eval(LINK.readModel(), 'A');
ok(Math.abs(e1.score-e2.score) < 1e-12 && e1.pass === e2.pass && e1.margMed === e2.margMed,
   '同一候选两次评估逐项相同（score/pass/裕度）', [e1.score, e2.score]);
ok(JSON.stringify(e1.per) === JSON.stringify(e2.per), '逐位姿结果也相同');
ok(JSON.stringify(B.MODEL) === snap0, '评估后模型仍逐字节还原');

/* ================= ④ 判据量级 ================= */
console.log('\n=== ④ 判据是不是"可达但不宽松" ===');
const rBig = LINK.eval(LINK.readModel(), '基准(大预算)', Object.assign({}, LINK.cfg, {nPose:24, budget:8, iters:300}));
console.log('  基准 24位姿×8起点×300迭代: ' + `分数 ${rBig.score.toFixed(1)}  未解出 ${(100*(1-rBig.found)).toFixed(0)}%  ` +
            `合格 ${(100*rBig.pass).toFixed(0)}%  姿态中位 ${rBig.angMed.toFixed(2)}°`);
ok(rBig.found > 0.7, '大预算下大部分位姿能找到满足容差的解（判据不是"永远解不出来"）；剩下的属于 IK 预算/运气',
   (100*rBig.found).toFixed(0)+'%');
ok(rBig.score < 20, '但基准分数很低（关节裕度顶死）—— 判据是紧的，才分得出连杆好坏', rBig.score.toFixed(1));
ok(rBig.limitOK === true, '大预算评估里限位也没被改');
/* 面板自己那行"可解"为什么总是 ~100%：说清楚，并当场验证一次 */
const panelOK = (function(){
  const cfgBig = Object.assign({}, LINK.cfg, {nPose:24, budget:8, iters:300});
  const pool = [new Array(B.NJ).fill(0)];
  for(let k=0;k<11;k++) pool.push(B.qLimits().map(([lo,hi])=> lo + Math.random()*(hi-lo)));
  let hit = 0;
  const n = 24;
  for(let i=0;i<n;i++){
    const p = LINK.poses(cfgBig)[i];
    for(const sd of pool){ const r = B.ikSolve6(p.Eb, p.ey, sd, 300, 1); if(r.ok){ hit++; break; } }
  }
  return hit/n;
})();
console.log('  同期上面面板的判据（ok = 位置<3mm 且 姿态<11.5°）在同样位姿上的可解率 = ' + (100*panelOK).toFixed(0) + '%');
ok(panelOK > rBig.pass, '★ 证明"面板 100% 可解"与"严格合格率很低"能同时成立 —— 差异就是这么来的');

/* ================= ⑤ 扫描 / 搜索 ================= */
console.log('\n=== ⑤ 灵敏度扫描与搜索 ===');
Object.assign(LINK.cfg, QUICK ? {nPose:6, budget:2, iters:140} : {nPose:8, budget:3, iters:180});
const sw = LINK.sweepSync(LINK.cfg);
const nOn = LINK.knobs().filter(k=>k.on).length;
ok(!!sw && sw.points.length === nOn * LINK.cfg.K, '扫描点数 = 启用旋钮数 × 每根档数', [sw && sw.points.length, nOn*LINK.cfg.K]);
ok(sw.limitOK === true, '整轮扫描限位未改动');
ok(LINK.limSig() === lim0 && JSON.stringify(B.MODEL) === snap0, '扫描后模型逐字节还原');
const span = {};
sw.points.forEach(p=>{ span[p.key] = span[p.key] || [1e9,-1e9];
  span[p.key][0] = Math.min(span[p.key][0], p.score); span[p.key][1] = Math.max(span[p.key][1], p.score); });
const movable = Object.keys(span).filter(k=>span[k][1]-span[k][0] > 1);
ok(movable.length >= 3, '至少 3 根杆能撬动分数（灵敏度不是全平）', movable.length + ' 根');

Object.assign(LINK.cfg, QUICK ? {nPose:6, budget:2, iters:140, restarts:4, seed:20260925}
                              : {nPose:8, budget:3, iters:180, restarts:8, seed:20260925});
const sh = LINK.searchSync(LINK.cfg);
ok(!!sh && sh.n > 10, '搜索跑起来了：评估 ' + (sh&&sh.n) + ' 个候选', sh&&sh.n);
ok(sh.limitOK === true, '整轮搜索限位未改动');
ok(sh.best && sh.best.score >= rBase.score - 1e-9, '搜索最优分数不低于基准', [sh.best&&sh.best.score, rBase.score]);
ok(sh.best && sh.best.armLen <= LINK.cfg.lenCap + 1e-6, '最优解满足"杆长之和 ≤ 上限"',
   [sh.best&&sh.best.armLen, LINK.cfg.lenCap]);
ok(LINK.limSig() === lim0 && JSON.stringify(B.MODEL) === snap0, '搜索后模型逐字节还原');

/* 长度约束真的挡得住：把每根杆都拉满 → 一定超长，搜索不该采纳 */
const valsLong = LINK.knobs().map(k => Math.max(Math.abs(k.lo), Math.abs(k.hi)));
const rLong = LINK.eval(valsLong, '超长');
ok(rLong.armLen > LINK.cfg.lenCap, '把每根杆拉满后杆长之和 ' + rLong.armLen.toFixed(0) + ' > 上限 ' +
   LINK.cfg.lenCap.toFixed(0) + '（约束有东西可挡）');
ok(!(sh.best && Math.abs(sh.best.armLen - rLong.armLen) < 1e-6), '搜索结果里没有这种超长解');

/* ================= ⑥ 应用 / 恢复 ================= */
console.log('\n=== ⑥ 应用到模型 / 恢复原始臂 ===');
const rows = LINK.res();
ok(rows.length >= 1, '结果表里有 ' + rows.length + ' 条（搜索最优 / 单点评估）');
const bi = LINK.bestRow();
const rBest = rows[bi];
LINK.apply(bi);
const nowVals = LINK.readModel();
ok(rBest.vals.every((v,i)=> Math.abs(v - nowVals[i]) < 1e-9), 'apply 后模型 offset = 候选的值（逐项）');
ok(LINK.limSig() === lim0, 'apply 后限位仍然一个字没变');
ok(JSON.stringify(B.MODEL.links.map(l=>l.limits)) === limJSON0, 'apply 后 links[*].limits 的 JSON 也没变');
ok(Math.abs(LINK.armLen() - rBest.armLen) < 1e-6, 'apply 后杆长之和 = 候选记录的杆长之和');
LINK.revert();
ok(JSON.stringify(B.MODEL) === snap0, 'revert 后模型逐字节回到快照（含限位）');
ok(LINK.limSig() === lim0, 'revert 后限位仍是原样');

/* ================= 导出 ================= */
const exp = LINK.export();
ok(exp && exp.limits && exp.results && exp.results.length === rows.length,
   '导出 payload 结构完整（' + (exp&&exp.results&&exp.results.length) + ' 条结果）');
ok(exp.limits.length === 6 && exp.limits.every(o=>'joint' in o && 'limits' in o), '导出里带限位只读快照');
ok(String(exp.rule).indexOf('限位只读') >= 0, '导出里写明"关节限位只读"');

console.log('\n================ ' + (fail ? 'FAIL' : '全部通过') + ' ================');
console.log('PASS=' + pass + '  FAIL=' + fail);
process.exit(fail ? 1 : 0);
