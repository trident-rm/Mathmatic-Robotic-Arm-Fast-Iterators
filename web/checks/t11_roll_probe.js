/*  t11_roll_probe.js —— 绕插入轴（Q 轴）的"自转能力"探针（回答三级难度的 R4：绕 Q 轴转 ±90°）
    跑法： node t11_roll_probe.js            （约 1~2 min）
           node t11_roll_probe.js --link      （改成量连杆模块里的当前模型）

    为什么单独一个探针、没做进 armLink 面板：
      它要解的是【位置3 + 两轴姿态2 + 自转角1 = 6 个约束 / 6 个关节】的位姿 IK，
      比面板那套 5 自由度问题难得多、也更依赖数值运气，不适合放进"要用来做优化指标"的地方。
      这里只当诊断用：告诉你"这个臂能不能绕插入轴连续转"。

    判据：roll ∈ [−90°, +90°) 每 15° 打一个点（12 点，正好覆盖 R4 要的 90° 行程），
          每个点用"上一个成功解"热启动；能连续命中的最长弧 = 该位姿的自转跨度。
*/
const { loadEnv, mulberry32 } = require('./t9_load.js');
/* --link：把连杆模块挂进来量「当前模型」；默认量基准臂。
   （原来靠 build/armLink.html，那个产物已随该 UI 面板一起去掉） */
const USE_AL = process.argv.includes('--link') || process.argv.includes('--armLink');
const { A, sandbox, LINK } = loadEnv({ variants:false, link: USE_AL });
A.buildChain();
const L = USE_AL ? LINK : null;
if(L){ L.init(true); }

const V = {
  sub:(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]],
  crs:(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
  dot:(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2],
  len:a=>Math.hypot(a[0],a[1],a[2]),
  nrm:a=>{const n=V.len(a)||1; return [a[0]/n,a[1]/n,a[2]/n];},
  scl:(a,s)=>[a[0]*s,a[1]*s,a[2]*s],
  mulv:(R,v)=>[R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2], R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2], R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]],
};
function solveN(M0,b){
  const n=b.length, M=M0.map((r,i)=>r.concat([b[i]]));
  for(let c=0;c<n;c++){
    let p=c; for(let r=c+1;r<n;r++) if(Math.abs(M[r][c])>Math.abs(M[p][c])) p=r;
    const t=M[c]; M[c]=M[p]; M[p]=t;
    if(Math.abs(M[c][c])<1e-14) continue;
    for(let r=0;r<n;r++){ if(r===c) continue; const f=M[r][c]/M[c][c];
      for(let k=c;k<=n;k++) M[r][k]-=f*M[c][k]; }
  }
  return M.map((r,i)=> Math.abs(M[i][i])<1e-14 ? 0 : r[n]/M[i][i]);
}
const W = 40;
function axErr(cur,tgt,w){
  const a=V.crs(cur,tgt), s=V.len(a), c=V.dot(cur,tgt);
  if(s<1e-9) return c>0 ? [0,0,0] :
    V.scl(V.nrm(Math.abs(cur[0])<0.9?V.crs(cur,[1,0,0]):V.crs(cur,[0,1,0])), Math.PI*w);
  return V.scl(V.scl(a,1/s), Math.atan2(s,c)*w);
}
/* 位姿 IK：位置 + 夹爪 y∥eY + 夹爪 x∥eX（6 约束 / 6 关节） */
function ikPose(tgtP, eY, eX, q0, iters){
  let q = A.ikPos(tgtP, q0, 90).q.slice(), lam = 40;
  const lim = A.qLimits(), tY = V.nrm(eY), tX = V.nrm(eX);
  let bestQ = q.slice(), bestF = Infinity;
  for(let it=0; it<(iters||260); it++){
    const F = A.fkAll(q), ee = F[F.length-1];
    const ep = V.sub(tgtP, ee.t);
    const yc = [ee.R[0][1],ee.R[1][1],ee.R[2][1]], xc = [ee.R[0][0],ee.R[1][0],ee.R[2][0]];
    const ay = Math.acos(Math.max(-1,Math.min(1,V.dot(yc,tY))));
    const ax2 = Math.acos(Math.max(-1,Math.min(1,V.dot(xc,tX))));
    const dp = V.len(ep), rr = Math.hypot(dp, W*Math.max(ay,ax2));
    if(rr < bestF){ bestF = rr; bestQ = q.slice(); }
    if(dp < 2 && ay < 0.06 && ax2 < 0.06) return {q, ok:true, e:dp, ang:Math.max(ay,ax2)};
    const J = [];
    for(let i=1;i<F.length;i++){ const f=F[i]; if(!f.movable) continue;
      const a = V.mulv(f.R, V.nrm(f.axis));
      J.push(V.crs(a, V.sub(ee.t,f.t)).concat(V.scl(a,W))); }
    const A6 = []; for(let i=0;i<6;i++) A6.push([0,0,0,0,0,0]);
    for(let i=0;i<6;i++)for(let j=0;j<6;j++){ let s=0; for(const jc of J) s+=jc[i]*jc[j];
      A6[i][j] = s + (i===j ? lam*lam : 0); }
    const eoY = axErr(yc,tY,W), eoX = axErr(xc,tX,W);
    const y = solveN(A6, ep.concat([eoY[0]+eoX[0], eoY[1]+eoX[1], eoY[2]+eoX[2]]));
    for(let k=0;k<J.length;k++){ let d=0; for(let i=0;i<6;i++) d+=J[k][i]*y[i];
      d = Math.max(-0.2, Math.min(0.2, d))/A.D2R;
      q[k] = Math.min(Math.max(q[k]+d, lim[k][0]), lim[k][1]); }
    lam *= 0.985;
  }
  const F = A.fkAll(bestQ), ee = F[F.length-1];
  const yc=[ee.R[0][1],ee.R[1][1],ee.R[2][1]], xc=[ee.R[0][0],ee.R[1][0],ee.R[2][0]];
  const ang = Math.max(Math.acos(Math.max(-1,Math.min(1,V.dot(yc,tY)))),
                       Math.acos(Math.max(-1,Math.min(1,V.dot(xc,tX)))));
  return {q:bestQ, ok: V.len(V.sub(tgtP,ee.t))<3 && ang<0.2, e:V.len(V.sub(tgtP,ee.t)), ang};
}
const wrap = a => { let x = ((a%180)+180)%180; if(x >= 90) x -= 180; return x; };
function frameForRoll(eB, uB, rollDeg){
  const y = V.nrm(V.scl(eB,-1));
  const x0 = V.nrm(V.sub(uB, V.scl(y, V.dot(uB,y))));
  const z0 = V.crs(x0, y);
  const a = rollDeg*A.D2R, c = Math.cos(a), s = Math.sin(a);
  return { x: V.nrm([x0[0]*c+z0[0]*s, x0[1]*c+z0[1]*s, x0[2]*c+z0[2]*s]), y };
}
/* 分层位姿集（与 armLink 模块同一套做法，互相对得上） */
function halton(i,b){ let f=1,r=0; while(i>0){ f/=b; r+=f*(i%b); i=Math.floor(i/b);} return r; }
const LV = A.RULES.levels[3], BS = [2,3,5,7,11,13];
const rng = mulberry32(20260925), OFF = BS.map(()=>rng());
const NPOSE = 24;
const poses = [];
for(let i=1;i<=NPOSE;i++){
  const u = BS.map((b,k)=>(halton(i,b)+OFF[k])%1);
  const P = {x:LV.x[0]+u[0]*(LV.x[1]-LV.x[0]), y:LV.y[0]+u[1]*(LV.y[1]-LV.y[0]),
             z:LV.z[0]+u[2]*(LV.z[1]-LV.z[0]), th:LV.th[0]+u[3]*(LV.th[1]-LV.th[0]),
             ph:LV.ph[0]+u[4]*(LV.ph[1]-LV.ph[0]), al:LV.al[0]+u[5]*(LV.al[1]-LV.al[0])};
  const F0 = A.coreFrame(P);
  poses.push({P, Eb:A.taskToBase([P.x,P.y,P.z]), eB:A.dirToBase(F0.e), uB:A.dirToBase(F0.u)});
}
const ROLLS = []; for(let r=-90;r<90;r+=15) ROLLS.push(r);
const LIM = A.qLimits();
console.log((USE_AL?'当前模型（连杆模块）':'基准臂') + '：' + NPOSE + ' 个三级任务域位姿 × ' +
            ROLLS.length + ' 个自转角（−90°~+90°，15° 步长）');
console.log('判据：位置 <3mm 且 夹爪 y∥−e⃗、夹爪 x∥x(roll) 两轴都 <11.5°，命中即用该解热启动下一个角度\n');
const spans = [], hits = [];
for(let pi=0; pi<poses.length; pi++){
  const p = poses[pi];
  /* 先用面板那套 5 自由度解当种子（命中率高得多） */
  let seed = null;
  for(let k=0;k<6 && !seed;k++){
    const s0 = k===0 ? new Array(A.NJ).fill(0) : LIM.map(([lo,hi])=>lo+Math.random()*(hi-lo));
    const r = A.ikSolve6(p.Eb, V.scl(V.nrm(p.eB),-1), s0, 300, 1);
    if(r.ok) seed = r.q;
  }
  const bits = [];
  let warm = seed || new Array(A.NJ).fill(0);
  for(const rd of ROLLS){
    const fr = frameForRoll(p.eB, p.uB, rd);
    const t = ikPose(p.Eb, fr.y, fr.x, warm, 240);
    bits.push(t.ok ? 1 : 0);
    if(t.ok) warm = t.q;
  }
  let best = 0, cur = 0;
  for(const b of bits){ if(b){ cur++; if(cur>best) best=cur; } else cur=0; }
  spans.push(best*15); hits.push(bits.reduce((a,v)=>a+v,0));
  console.log(`  位姿 ${String(pi+1).padStart(2)}  z=${String(Math.round(p.P.z)).padStart(4)} φ=${String(Math.round(p.P.ph)).padStart(3)}  ` +
    `命中 ${bits.join('')}  命中数 ${hits[pi]}/12  最大连续跨度 ${spans[pi]}°`);
}
const med = a=>{ const s=a.slice().sort((x,y)=>x-y); return s[Math.floor(s.length/2)]; };
console.log('\n=== 结论 ===');
console.log('  位姿内最大连续自转跨度：中位 ' + med(spans) + '°，最大 ' + Math.max(...spans) + '°');
console.log('  跨度 ≥ 90°（够做 R4 的 ±90° 行程）的位姿占比 = ' +
            (100*spans.filter(v=>v>=90).length/spans.length).toFixed(0) + '%');
console.log('  一个角度都解不出来的位姿 = ' + spans.filter((v,i)=>hits[i]===0).length + '/' + spans.length);
console.log('  12 个角度全解出来的位姿 = ' + hits.filter(v=>v===12).length + '/' + spans.length);
console.log('  命中数分布 = ' + JSON.stringify(hits.reduce((m,v)=>(m[v]=(m[v]||0)+1,m),{})));
