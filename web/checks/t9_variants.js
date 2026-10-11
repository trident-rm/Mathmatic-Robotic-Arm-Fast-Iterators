/*  t9_variants.js —— 在 armBuilder.html 的沙箱里定义 IK 变体（T9 实验用）
    注意：本文件被拼进 armBuilder.html 的两个 <script> 之后一起执行，
    所以可以直接用 A.fkAll / A.qLimits 等；向量小工具在本文件内自带一份，
    公式与页面里的完全一致（cross / subv / mulv / unit）。                          */
globalThis.__V = (function () {
  const D2R = Math.PI / 180;
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const subv  = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
  const mulv  = (M, v) => [M[0][0]*v[0]+M[0][1]*v[1]+M[0][2]*v[2],
                           M[1][0]*v[0]+M[1][1]*v[1]+M[1][2]*v[2],
                           M[2][0]*v[0]+M[2][1]*v[1]+M[2][2]*v[2]];
  const dot3  = (a, b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  const unit  = a => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0]/n, a[1]/n, a[2]/n]; };
  const scl   = (a, s) => [a[0]*s, a[1]*s, a[2]*s];

  /* n×n 线性方程组（部分主元高斯消元；页面里的 solveN 只做无主元消元） */
  function solveLin(A, b) {
    const n = b.length, M = A.map((r, i) => r.concat([b[i]]));
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c+1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-14) return new Array(n).fill(0);
      const t = M[c]; M[c] = M[p]; M[p] = t;
      for (let r = 0; r < n; r++) { if (r === c) continue;
        const f = M[r][c] / M[c][c];
        for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
    }
    return M.map((r, i) => r[i] ? r[n]/r[i] : 0);
  }

  /* 位置雅可比列：a × (pe - p_joint)（与页面同） */
  function jacCols(F, pe) {
    const J = [];
    for (let i = 1; i < F.length; i++) {
      const f = F[i]; if (!f.movable) continue;
      const nn = Math.hypot(f.axis[0], f.axis[1], f.axis[2]);
      const a = mulv(f.R, [f.axis[0]/nn, f.axis[1]/nn, f.axis[2]/nn]);
      J.push({ a, jc: cross(a, subv(pe, f.t)) });
    }
    return J;
  }
  function clampTo(q, LIM) {
    for (let k = 0; k < q.length; k++) q[k] = Math.min(Math.max(q[k], LIM[k][0]), LIM[k][1]);
    return q;
  }

  /* 只解位置（页面 ikPos 的可配置版） */
  function posSolve(A, target, q0, LIM, iters, lam0, maxStep) {
    let q = q0.slice(), lam = lam0 === undefined ? 40 : lam0;
    const ms = maxStep === undefined ? 0.3 : maxStep;
    for (let it = 0; it < iters; it++) {
      const F = A.fkAll(q), pe = F[F.length-1].t;
      const ep = subv(target, pe);
      if (Math.hypot(ep[0], ep[1], ep[2]) < 0.5) break;
      const J = jacCols(F, pe).map(o => o.jc);
      const M = [[0,0,0],[0,0,0],[0,0,0]];
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
        let s = 0; for (const jc of J) s += jc[i]*jc[j];
        M[i][j] = s + (i === j ? lam*lam : 0);
      }
      const y = solveLin(M, ep);
      for (let k = 0; k < J.length; k++) {
        let d = 0; for (let i = 0; i < 3; i++) d += J[k][i]*y[i];
        d = Math.max(-ms, Math.min(ms, d)) / D2R;      // ★ 弧度 → 角度
        q[k] = Math.min(Math.max(q[k]+d, LIM[k][0]), LIM[k][1]);
      }
      lam *= 0.98;
    }
    const F = A.fkAll(q), e = Math.hypot(...subv(target, F[F.length-1].t));
    return { q, ok: e < 1.5, e };
  }

  /* 主求解器：o 控制各开关
       iters    迭代上限（页面 200）
       lam0     初始阻尼（页面 40）  decay 每步衰减（页面 0.985）
       lamAdapt 真 LM：步长没改善就加倍、改善就缩小
       wOgate   页面那套 wO 门控（远 0.02 / 中 0.25 / 近 1）
       lineSearch 步长不改善就折半（最多 8 次）
       keepBest 全程保留最优（按 score = dp + 20*ang）
       posPhase 先用纯位置解做种子（页面有），posIters 其次数（页面 90）
       axIdx    对齐轴（页面插调用 1 = 夹爪 y）                            */
  function solve6(A, targetP, tdir, q0, LIM, o) {
    o = Object.assign({ iters:200, lam0:40, decay:0.985, lamAdapt:false, wOgate:true,
                        lineSearch:false, keepBest:false, posPhase:true, posIters:90,
                        oriW:40, axIdx:1, maxStep:0.2, maxStepPos:0.3, clampMode:'each',
                        activeSet:false,
                        lamMin:0, lamMax:400, lamUp:2.0, lamDown:0.7, lamAdaptMin:1.5 }, o || {});
    let q = q0.slice();
    if (o.posPhase) q = posSolve(A, targetP, q, LIM, o.posIters, o.lam0, o.maxStepPos).q;
    const tz = unit(tdir);
    let lam = o.lam0;
    let bestQ = q.slice(), bestScore = Infinity, bestE = Infinity, bestAng = Infinity;
    const evalAt = (qq) => {
      const F = A.fkAll(qq), ee = F[F.length-1];
      const ep = subv(targetP, ee.t);
      const zc = [ee.R[0][o.axIdx], ee.R[1][o.axIdx], ee.R[2][o.axIdx]];
      const ang = Math.acos(Math.max(-1, Math.min(1, dot3(zc, tz))));
      return { dp: Math.hypot(ep[0],ep[1],ep[2]), ang, ep, zc };
    };
    let itersUsed = o.iters;
    for (let it = 0; it < o.iters; it++) {
      const F = A.fkAll(q), ee = F[F.length-1];
      const ep = subv(targetP, ee.t);
      const zc = [ee.R[0][o.axIdx], ee.R[1][o.axIdx], ee.R[2][o.axIdx]];
      const ax = cross(zc, tz), s = Math.hypot(ax[0], ax[1], ax[2]), c = dot3(zc, tz);
      let eo, ang;
      if (s < 1e-8) {
        if (c > 0) { eo = [0,0,0]; ang = 0; }
        else { const p = Math.abs(zc[0]) < 0.9 ? cross(zc, [1,0,0]) : cross(zc, [0,1,0]);
               eo = scl(unit(p), Math.PI * o.oriW); ang = Math.PI; }
      } else { ang = Math.atan2(s, c); eo = scl(scl(ax, 1/s), ang * o.oriW); }
      const dp = Math.hypot(ep[0], ep[1], ep[2]);
      if (dp < 2 && ang < 0.15) { itersUsed = it; return { q, ok:true, e:dp, ang, iters:it }; }
      const score = Math.hypot(dp, o.oriW * ang);        // = ||[ep; 40·eo]||，与 DLS 最小化的量一致
      if (score < bestScore) { bestScore = score; bestQ = q.slice(); bestE = dp; bestAng = ang; }
      const wO = o.wOgate ? (dp > 25 ? 0.02 : (dp > 8 ? 0.25 : 1)) : 1;
      const eoW = scl(eo, wO);
      const cols = jacCols(F, ee.t);
      const J = cols.map(o2 => o2.jc.concat(scl(o2.a, o.oriW * wO)));
      const nJ = J.length, M = [];
      for (let i = 0; i < 6; i++) { const row = new Array(6).fill(0); M.push(row); }
      for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
        let s2 = 0; for (const jc of J) s2 += jc[i]*jc[j];
        M[i][j] = s2 + (i === j ? lam*lam : 0);
      }
      const y = solveLin(M, ep.concat(eoW));
      const dq = new Array(nJ);
      for (let k = 0; k < nJ; k++) {
        let d = 0; for (let i = 0; i < 6; i++) d += J[k][i]*y[i];
        dq[k] = d;                                     // 弧度，先不裁
      }
      /* 活动集：把"顶在限位上、且这一步还要往外推"的关节冻结，
         用剩下的自由度重新解一次 —— 否则被裁掉的那部分运动量就白丢了 */
      if (o.activeSet) {
        const frozen = new Array(nJ).fill(false);
        let nf = 0;
        for (let k = 0; k < nJ; k++) {
          const atLo = q[k] <= LIM[k][0] + 1e-9, atHi = q[k] >= LIM[k][1] - 1e-9;
          if ((atLo && dq[k] < 0) || (atHi && dq[k] > 0)) { frozen[k] = true; nf++; }
        }
        if (nf > 0 && nf < nJ) {
          const idx = [], Jr = [];
          for (let k = 0; k < nJ; k++) if (!frozen[k]) { idx.push(k); Jr.push(J[k]); }
          const m2 = idx.length, M2 = [];
          for (let i = 0; i < 6; i++) M2.push(new Array(m2).fill(0));
          for (let i = 0; i < 6; i++) for (let j = 0; j < m2; j++) {
            let s2 = 0; for (const jc of Jr) s2 += jc[i]*jc[j];
            M2[i][j] = s2 + (i === j ? lam*lam : 0);
          }
          /* 最小二乘：解 (JrᵀJr + λ²I) z = Jrᵀ r，再 dq = z */
          const G = [], b = new Array(m2).fill(0);
          for (let a2 = 0; a2 < m2; a2++) { G.push(new Array(m2).fill(0)); }
          const rvec = ep.concat(eoW);
          for (let a2 = 0; a2 < m2; a2++) {
            for (let b2 = 0; b2 < m2; b2++) { let s2 = 0;
              for (const jc of Jr) s2 += jc[a2]*jc[b2];
              G[a2][b2] = s2 + (a2 === b2 ? lam*lam : 0); }
            let s3 = 0; for (let i = 0; i < 6; i++) s3 += Jr[a2][i]*rvec[i];
            b[a2] = s3;
          }
          const z = solveLin(G, b);
          for (let k = 0; k < nJ; k++) dq[k] = 0;
          for (let a2 = 0; a2 < m2; a2++) dq[idx[a2]] = z[a2];
        }
      }
      /* 步长处理：each = 页面原样（每个关节各自裁 ±0.2rad，会破坏线性约束）
                   scale = 整条方向等比缩小（保持方向）
                   none  = 不裁，交给 λ 控制                                    */
      const cm = o.clampMode || 'each';
      if (cm === 'each') {
        for (let k = 0; k < nJ; k++) dq[k] = Math.max(-o.maxStep, Math.min(o.maxStep, dq[k]));
      } else if (cm === 'scale') {
        let m = 0; for (let k = 0; k < nJ; k++) m = Math.max(m, Math.abs(dq[k]));
        if (m > o.maxStep) { const f = o.maxStep / m; for (let k = 0; k < nJ; k++) dq[k] *= f; }
      }
      for (let k = 0; k < nJ; k++) dq[k] /= D2R;        // 弧度 → 角度
      const qTry = clampTo(q.map((v, k) => v + dq[k]), LIM);
      if (o.lineSearch || o.lamAdapt) {
        const f = (r) => Math.hypot(r.dp, o.oriW * r.ang);
        const cur = evalAt(q), nxt = evalAt(qTry);
        if (f(nxt) < f(cur) - 1e-12) {
          q = qTry; if (o.lamAdapt) lam = Math.max(o.lamAdaptMin, lam * o.lamDown);
        } else {
          if (o.lamAdapt) lam = Math.min(o.lamMax, lam * o.lamUp);
          if (o.lineSearch) {                       // 折半直到改善（最多 8 次）
            let t = 0.5, moved = false;
            for (let h = 0; h < 8; h++) {
              const qh = clampTo(q.map((v, k) => v + dq[k]*t), LIM);
              if (f(evalAt(qh)) < f(cur) - 1e-12) { q = qh; moved = true; break; }
              t *= 0.5;
            }
            if (!moved) { lam = Math.min(o.lamMax, lam * 1.6); }
          }
        }
      } else { q = qTry; }
      if (!o.lamAdapt) lam = Math.max(o.lamMin, lam * o.decay);
    }
    const qq = o.keepBest ? bestQ : q;
    const r = evalAt(qq);
    return { q: qq, ok: (r.dp < 3 && r.ang < 0.2), e: r.dp, ang: r.ang, iters: o.iters };
  }

  return { solve6, posSolve, solveLin, cross, subv, mulv, dot3, unit, scl, D2R };
})();
