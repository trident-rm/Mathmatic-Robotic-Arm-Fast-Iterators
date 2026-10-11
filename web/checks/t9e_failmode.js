/*  t9e_failmode.js —— 失败到底是"停在局部极小"还是"根本没收敛"？
    对每个 (位姿, 随机起点)：跑页面同款 ikSolve6，分类：
      ok                成功
      stationarity小     最终点满足 Jᵀr ≈ 0（真的收敛了，只是收敛到非零局部极小）
      stationarity大     最终点还没到驻点（迭代被截断/在震荡）
      顶到限位           有关节停在限位上
    用法： node t9e_failmode.js [位姿数=120] [每姿势起点数=8]                      */
const { loadEnv, makePoses, makeSeeds } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const LIM = A.qLimits(), NJ = A.NJ;
const NP = parseInt(process.argv[2] || '120', 10);
const NS = parseInt(process.argv[3] || '8', 10);
const poses = makePoses(A, NP);
const seeds = makeSeeds(A, NP, NS);
const T = poses.map(P => ({ Eb: A.taskToBase([P.x, P.y, P.z]),
                            dir: A.dirToBase(A.coreFrame(P).e).map(v => -v) }));

/* 用与页面相同的残差约定：r = [位置误差 ; 40·(轴角误差向量)]，梯度 = Jᵀr */
function gradNorm(q, targetP, tdir, w) {
  const F = A.fkAll(q), ee = F[F.length-1];
  const ep = V.subv(targetP, ee.t);
  const tz = V.unit(tdir);
  const zc = [ee.R[0][1], ee.R[1][1], ee.R[2][1]];
  const ax = V.cross(zc, tz), s = Math.hypot(...ax), c = V.dot3(zc, tz);
  let eo, ang;
  if (s < 1e-8) { if (c > 0) { eo = [0,0,0]; ang = 0; }
    else { const p = Math.abs(zc[0]) < 0.9 ? V.cross(zc,[1,0,0]) : V.cross(zc,[0,1,0]);
           eo = V.scl(V.unit(p), Math.PI*w); ang = Math.PI; } }
  else { ang = Math.atan2(s,c); eo = V.scl(V.scl(ax,1/s), ang*w); }
  const J = [];
  for (let i = 1; i < F.length; i++) { const f = F[i]; if (!f.movable) continue;
    const nn = Math.hypot(f.axis[0], f.axis[1], f.axis[2]);
    const a = V.mulv(f.R, [f.axis[0]/nn, f.axis[1]/nn, f.axis[2]/nn]);
    J.push(V.cross(a, V.subv(ee.t, f.t)).concat(V.scl(a, w))); }
  const r = ep.concat(eo);
  let g = 0;
  for (let k = 0; k < J.length; k++) { let s2 = 0;
    for (let i = 0; i < 6; i++) s2 += J[k][i]*r[i]; g += s2*s2; }
  const dp = Math.hypot(...ep);
  return { g: Math.sqrt(g), dp, ang, rnorm: Math.hypot(...r) };
}

let ok = 0, stSmall = 0, stBig = 0, limHit = 0, n = 0;
const failAng = [], failDp = [], failG = [];
for (let i = 0; i < NP; i++) {
  for (let s = 0; s < NS; s++) {
    n++;
    const r = V.solve6(A, T[i].Eb, T[i].dir, seeds[i][s], LIM, {});
    if (r.ok) { ok++; continue; }
    const gg = gradNorm(r.q, T[i].Eb, T[i].dir, 40);
    const g0 = gradNorm(seeds[i][s], T[i].Eb, T[i].dir, 40).g;   // 起点梯度，用作尺度参考
    failAng.push(gg.ang*180/Math.PI); failDp.push(gg.dp); failG.push(gg.g/Math.max(g0,1e-9));
    if (gg.g < 0.02*g0) stSmall++;
    else if (gg.g > 0.2*g0) stBig++;
    let onLim = false;
    for (let k = 0; k < NJ; k++)
      if (Math.abs(r.q[k]-LIM[k][0]) < 0.5 || Math.abs(r.q[k]-LIM[k][1]) < 0.5) onLim = true;
    if (onLim) limHit++;
  }
}
const med = a => { const b = a.slice().sort((x,y)=>x-y); return b[Math.floor(b.length/2)]; };
console.log('=== 失败模式分解（%d 个位姿 × %d 个起点 = %d 次求解）===', NP, NS, n);
console.log('  成功                        %d  (%.1f%%)', ok, 100*ok/n);
console.log('  失败                        %d  (%.1f%%)', n-ok, 100*(n-ok)/n);
console.log('    ├ 已到驻点（‖Jᵀr‖<2%‖Jᵀr₀‖，真·局部极小） %d  (%.1f%%)', stSmall, 100*stSmall/n);
console.log('    ├ 远未驻定（‖Jᵀr‖>20%‖Jᵀr₀‖，没收敛/震荡） %d  (%.1f%%)', stBig, 100*stBig/n);
console.log('    └ 其余（中间地带）                        %d', (n-ok-stSmall-stBig));
console.log('  失败解里有关节顶在限位上的     %d  (%.1f%%)', limHit, 100*limHit/n);
console.log('  失败解的中位：位置误差 %.1f mm  姿态偏差 %.1f°  相对梯度 %.3f',
            med(failDp), med(failAng), med(failG));
