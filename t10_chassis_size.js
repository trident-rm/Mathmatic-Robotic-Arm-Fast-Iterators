/*  t10_chassis_size.js —— 底盘碰撞箱 / 伸展尺寸 / 机械臂长度 的自检（无头）
    跑法： node t10_chassis_size.js
    默认值（2026-09-21 按用户截图改）：
      底盘中心点 = base (0, 140, 0)；箱体 580 × 580 × 105，z 往 −Z 长
    检什么：
      ① 底盘中心点 / 箱体范围 / 接地中心 / 球心 / 半径 / 地面交线半径
      ② 箱体"整机系轴向范围"必须由 8 个角算（跟车转的箱子，不能拿 中心±半长 直接加减）
      ③ 机械臂长度 = 各段杆长之和；最大伸展 ≤ 杆长之和
      ④ 球外采样点计数、draw3D()/sizePanelHTML() 不抛异常
      ⑤ 面板里【不该】再出现"当前位形/可达集 包围盒"那种行
*/
const { loadEnv } = require('./t9_load.js');
const { A, sandbox } = loadEnv({ variants:false });

let fail = 0, pass = 0;
const ok = (cond, msg, extra) => {
  if(cond){ pass++; console.log('  ✓ ' + msg); }
  else { fail++; console.log('  ✗ ' + msg + (extra !== undefined ? '   ← ' + extra : '')); }
};
const near = (a,b,tol) => Math.abs(a-b) <= (tol === undefined ? 1e-6 : tol);
const arrNear = (a,b,tol) => a.length === b.length && a.every((v,i)=>near(v,b[i],tol));
const S = v => '[' + v.map(x=>(+x).toFixed(3)).join(', ') + ']';

A.MODEL = A.demoModel();          // base yaw = -90，base.xyz = [0,0,0]
A.buildChain(); A.Q = new Array(A.NJ).fill(0);
sandbox.computeFeasible && sandbox.computeFeasible();

console.log('\n=== ① 默认参数下的几何（示例臂 · base yaw −90） ===');
const g = A.chassisGeom();
console.log('  底盘中心点 C  = ' + S(g.C) + '   （base 系 ' + S(A.CHASSIS.pos) + '）');
console.log('  箱体中心/半长 = ' + S(g.bx) + ' / ' + S(g.bh));
console.log('  接地中心 GC   = ' + S(g.GC));
console.log('  球心 SC / R   = ' + S(g.SC) + ' / ' + g.SR);
console.log('  地面交线半径  = ' + g.planeR.toFixed(3) + ' mm');
ok(arrNear(A.CHASSIS.pos, [0,140,0], 1e-9), '默认底盘中心点 = base (0, 140, 0)');
ok(arrNear(A.CHASSIS.size, [580,580,105], 1e-9), '默认箱体 = 580 × 580 × 105');
ok(arrNear(g.C,  [140,0,0], 1e-9), '底盘中心点（整机系）：yaw −90 把 base 的 +y 转到 +x ⇒ (140, 0, 0)');
ok(arrNear(g.bh, [290,290,52.5], 1e-9), '三个半长 = 290 / 290 / 52.5');
ok(near(g.zTop, 0, 1e-9) && near(g.zBot, -105, 1e-9),
   '箱体 z ∈ [−105, 0]（中心点在 base 原点高度，z 往下长 105）', [g.zTop, g.zBot]);
ok(arrNear(g.GC, [140,0,-105], 1e-9), '接地中心 = 箱体底面中心 = (140, 0, −105)');
ok(arrNear(g.SC, [140,0,375], 1e-9), '球心 = 接地中心 + (0,0,480) = (140, 0, 375)');
ok(near(g.SR, 750, 1e-9), '半径 = 1500/2 = 750');
ok(near(g.planeR, 576.2812, 5e-4), '地面交线半径 = √(750²−480²) ≈ 576.281 mm');

console.log('\n=== ② 箱体的"整机系轴向范围"必须由 8 个角算 ===');
const R = A.boxRange(g);
console.log('  轴向范围 x ' + R.mn[0].toFixed(1) + '~' + R.mx[0].toFixed(1) +
            ' · y ' + R.mn[1].toFixed(1) + '~' + R.mx[1].toFixed(1) +
            ' · z ' + R.mn[2].toFixed(1) + '~' + R.mx[2].toFixed(1));
ok(near(R.mx[2], 0, 1e-9) && near(R.mn[2], -105, 1e-9), 'z 范围 −105 ~ 0（不随 yaw 变）');
/* yaw −90 时 580 的方形箱子仍占 x/y 各 ±290（正方形看不出差别）；
   换成 300×580 的长方形+yaw 30° 才能验出"中心±半长"是错的 */
A.CHASSIS = Object.assign({}, A.CHASSIS, { pos:[0,0,0], size:[300,580,105] });
A.MODEL.base.yaw = 30;
const g2 = A.chassisGeom(), R2 = A.boxRange(g2);
const naive = [g2.bh[0], g2.bh[1]];                       // 错误做法：直接拿半长当范围
const real  = [(R2.mx[0]-R2.mn[0])/2, (R2.mx[1]-R2.mn[1])/2];
console.log('  300×580 箱体转 30°：范围半宽 = ' + real.map(v=>v.toFixed(1)).join(' / ') +
            '   而"半长"只有 ' + naive.map(v=>v.toFixed(1)).join(' / '));
ok(real[0] > naive[0] + 50 || real[1] > naive[1] + 50,
   '转过的长方形箱子的轴向范围 > 半长（证明逐角算法不是白写的）', [real, naive]);
A.CHASSIS = Object.assign({}, A.CHASSIS, { pos:[0,140,0], size:[580,580,105] });
A.MODEL.base.yaw = -90;

console.log('\n=== ③ 机械臂长度 ===');
const a = A.armStats();
console.log('  各段杆长 = ' + a.segs.map(v=>v.toFixed(1)).join(' / '));
console.log('  杆长之和 = ' + a.total.toFixed(1) + ' mm    最大伸展 = ' + a.reach.toFixed(0) + ' mm');
ok(a.segs.length === A.CHAIN.length, '段数 = 链长 ' + A.CHAIN.length +
   '（base_link 与 link5 的 offset 是 0，所以比 README 记的 6 段多一个 0）');
ok(arrNear(a.segs.map(v=>+v.toFixed(2)), [0, 75, 290, 85.5, 402.59, 0, 211], 0.01),
   '各段杆长 = 0 / 75 / 290 / 85.5 / 402.59 / 0 / 211（非零段与 README 记录一致）',
   a.segs.map(v=>+v.toFixed(2)));
ok(near(a.total, 1064.09, 0.01), '杆长之和 = 1064.1 mm', a.total);
ok(a.reach > 0 && a.reach <= a.total + 1e-6, '最大伸展 ≤ 杆长之和（限位只会让它更短）',
   [a.reach, a.total]);
ok(a.zMax > a.zMin, '末端高度范围非空：z ' + a.zMin.toFixed(0) + ' ~ ' + a.zMax.toFixed(0));

console.log('\n=== ④ 地面以下的采样点必须被剔除（否则"最大伸展"是假的）===');
const G4 = A.chassisGeom(), SC = G4.SC, zG = G4.GC[2];        // 地面 = 过接地中心的平面
const B0 = A.MODEL.base.xyz;
const pUp  = [1000, 0, zG + 10];     // 地面以上、较远
const pUp2 = [ 200, 0, zG + 50];     // 地面以上、较近
const pDn  = [1200, 0, zG - 300];    // 地面以下，而且比任何地上点都远（旧版就是被它撑大的）
const pDn2 = [  50, 0, zG - 5];      // 刚穿地一点点
A.CLOUD = [pUp, pUp2, pDn, pDn2];
const s4 = A.armStats();
const dOf = p => Math.hypot(p[0]-B0[0], p[1]-B0[1], p[2]-B0[2]);
console.log('  地面 z = ' + zG.toFixed(1) + '；地上 2 个 / 地下 2 个');
console.log('  最大伸展 = ' + s4.reach.toFixed(1) + '（地上最远 ' + Math.max(dOf(pUp),dOf(pUp2)).toFixed(1) +
            '；不剔除的话会是 ' + Math.max(...[pUp,pUp2,pDn,pDn2].map(dOf)).toFixed(1) + '）');
ok(s4.used === 2 && s4.under === 2, '统计到地面以上 2 个、地面以下 2 个', [s4.used, s4.under]);
ok(near(s4.reach, Math.max(dOf(pUp), dOf(pUp2)), 1e-6),
   '最大伸展只用地上点算（地下那个 1200 远的没进来）', [s4.reach, dOf(pDn)]);
ok(s4.reach < dOf(pDn) - 100, '★ 剔除前后差得很多，旧版的数就是被地下点撑大的');
ok(near(s4.zMin, zG + 10, 1e-6), '末端高度下限 = 地上点里最低的 ' + (zG+10).toFixed(0), s4.zMin);
ok(s4.zMin >= zG - 1e-9, '★ 末端高度不会低于地面');
ok(Math.hypot(G4.GC[0]-SC[0], G4.GC[1]-SC[1], G4.GC[2]-SC[2]) === 480,
   '球心仍在接地中心 z 轴往上 480');
/* 一个地下点都没有时，数字不该变 */
A.CLOUD = [pUp, pUp2];
const s5 = A.armStats();
ok(s5.under === 0 && near(s5.reach, s4.reach, 1e-9) && near(s5.zMin, s4.zMin, 1e-9),
   '全是地上点时，剔除逻辑不影响结果');

console.log('\n=== ⑤ 面板：要机械臂长度、不要包围盒、也不要红点 ===');
A.CLOUD = []; sandbox.computeFeasible && sandbox.computeFeasible();
let err = null, html = '';
try { html = A.sizePanelHTML(); } catch(e){ err = e; }
ok(!err, 'sizePanelHTML() 正常返回', err && err.message);
ok(html.indexOf('机械臂长度') >= 0, '面板有「机械臂长度」段');
ok(html.indexOf('杆长之和') >= 0 && html.indexOf('最大伸展') >= 0, '面板有「杆长之和」「最大伸展」两行');
ok(html.indexOf('接地中心') >= 0 && html.indexOf('地面上限圈') >= 0, '面板仍有接地中心 / 地面上限圈');
ok(html.indexOf('只统计地面以上') >= 0, '★ 面板写明了"只统计地面以上"的采样点');
ok(html.indexOf('当前位形') < 0, '★ 面板里【没有】"当前位形（骨架近似…）"了');
ok(html.indexOf('骨架近似：各坐标系连线') < 0, '★ 面板里【没有】"骨架近似：各坐标系连线"了');
ok(html.indexOf('可达集（末端采样') < 0, '★ 面板里【没有】"可达集（末端采样…）"了');
ok(html.indexOf('长(X)') < 0 && html.indexOf('宽(Y)') < 0 && html.indexOf('高(Z)') < 0,
   '★ 面板里【没有】长(X)/宽(Y)/高(Z) 那三行包围盒了');
ok(html.indexOf('离球心最远') < 0 && html.indexOf('采样点在球外') < 0,
   '★ 面板里不再出现"离球心最远"/"采样点在球外"');
ok(typeof sandbox.drawBallViolations === 'undefined', '★ 画红点的 drawBallViolations() 已经删掉了');

console.log('\n=== ⑥ 出图 / 开关 / 自检钩子 ===');
err = null; try { A.draw3D(); } catch(e){ err = e; }
ok(!err, 'draw3D() 正常返回', err && err.message);
A.SHOWBOX = { chassis:false, lwh:false, ball:false };
err = null; try { A.draw3D(); } catch(e){ err = e; }
ok(!err, '三个框全关后 draw3D() 不抛异常', err && err.message);
A.SHOWBOX = { chassis:true, lwh:true, ball:true };
const info = sandbox.SIZEINFO();
ok(info && arrNear(info.GC, [140,0,-105], 1e-9) && near(info.SR, 750), 'SIZEINFO() 自检钩子可用');
ok(info && near(info.armLen, 1064.09, 0.01), 'SIZEINFO() 里带着机械臂长度', info && info.armLen);
ok(sandbox.CHASSIS_V === undefined && A.CHASSIS_V === 2,
   'CHASSIS_V = 2（let/const 不进 globalThis，只能从 __api 读）', A.CHASSIS_V);

console.log('\n---------------------------------------------');
console.log(fail === 0 ? ('全部通过：' + pass + ' 项') : ('失败 ' + fail + ' 项 / 共 ' + (pass+fail)));
process.exit(fail === 0 ? 0 : 1);
