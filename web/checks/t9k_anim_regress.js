/*  t9k_anim_regress.js —— 插入动画路径：打补丁前 vs 打补丁后
    关注两件事：① 每帧是否 ok ② 帧间跳变（关节 / 末端位置）——关节跳不等于臂在跳，
    因为插入姿态下解集是一条 1 维流形，只要末端位置连续就没问题。
    用法： node t9k_anim_regress.js                                                  */
const { loadEnv, mulberry32 } = require('./t9_load.js');
const { A, V } = loadEnv({ variants: true });
const NJ = A.NJ, D2R = A.D2R;

const I = A.RULES.init;
const CORE = {x:I.x, y:I.y, z:I.z, th:I.th, ph:I.ph, al:I.al};
const rotY = th => [[Math.cos(th),0,Math.sin(th)],[0,1,0],[-Math.sin(th),0,Math.cos(th)]];
const rotX = th => [[1,0,0],[0,Math.cos(th),-Math.sin(th)],[0,Math.sin(th),Math.cos(th)]];
const mv = (M,v) => [M[0][0]*v[0]+M[0][1]*v[1]+M[0][2]*v[2],
                     M[1][0]*v[0]+M[1][1]*v[1]+M[1][2]*v[2],
                     M[2][0]*v[0]+M[2][1]*v[1]+M[2][2]*v[2]];
function geom(A2){
  const F0 = A.coreFrame(CORE);
  let e = A.dirToBase(F0.e); const e0b = e.slice();
  e = mv(rotY((A2.lift||0)*D2R), e); e = mv(rotX((A2.qfix||0)*D2R), e);
  return { e, e0b };
}
const seat = A.taskToBase([CORE.x, CORE.y, CORE.z]);
const insLen = 220, standoff = 40;

/* solver: 'old' = 变体里复刻的旧页面版本；'new' = 现在页面里的 ikSolve6 */
function runPath(solver, nFrame){
  const rng = mulberry32(4242);                    // 让补种子的随机序列两边一致
  const rand = () => rng();
  let seed = new Array(NJ).fill(0), prevQ = seed.slice(), prevP = null;
  let okN = 0, maxJoint = 0, maxEE = 0, worstFrame = -1, restartFrames = 0;
  const log = [];
  for(let f=0; f<=nFrame; f++){
    const ins = f/nFrame;
    const G = geom({ins});
    const tgt = [0,1,2].map(k => seat[k] + G.e0b[k]*(standoff + (1-ins)*insLen));
    const dir = G.e0b.map(v => -v);
    const solve = (qq) => solver === 'old'
      ? V.solve6(A, tgt, dir, qq, A.qLimits(), {})
      : A.ikSolve6(tgt, dir, qq, 320, 1);
    let r = solve(seed), usedRestart = false;
    for(let t=0; t<2 && !r.ok && r.e > 8; t++){
      usedRestart = true; restartFrames++;
      const lim = A.qLimits();
      const seed2 = lim.map(([lo,hi]) => lo + rand()*(hi-lo));
      const r2 = solve(seed2);
      if(r2.e < r.e) r = r2;
    }
    const ee = A.fkAll(r.q)[A.CHAIN.length].t;
    if(f > 0){
      let j = 0; for(let k=0;k<NJ;k++) j = Math.max(j, Math.abs(r.q[k]-prevQ[k]));
      const dp = Math.hypot(ee[0]-prevP[0], ee[1]-prevP[1], ee[2]-prevP[2]);
      if(j > maxJoint){ maxJoint = j; worstFrame = f; }
      maxEE = Math.max(maxEE, dp);
      log.push({f, j, dp, ok:r.ok});
    }
    prevQ = r.q.slice(); prevP = ee; seed = r.q.slice();
    if(r.ok) okN++;
  }
  return { okN, maxJoint, maxEE, worstFrame, restartFrames, log };
}
const N = 120;
for(const s of ['old','new']){
  const r = runPath(s, N);
  console.log('=== %s 求解器 ===', s === 'old' ? '打补丁前（变体复刻）' : '打补丁后（页面本体）');
  console.log('  ok 帧: %d/%d   帧间最大关节跳变: %s°  帧间最大末端跳变: %s mm   触发了补种子的帧: %d',
    r.okN, N+1, r.maxJoint.toFixed(1), r.maxEE.toFixed(2), r.restartFrames);
  const big = r.log.filter(x => x.j > 20).sort((a,b)=>b.j-a.j).slice(0,4);
  if(big.length) console.log('  关节跳 >20° 的帧: ' +
    big.map(x => 'f'+x.f+'(关节'+x.j.toFixed(0)+'°/末端'+x.dp.toFixed(1)+'mm/'+(x.ok?'ok':'fail')+')').join('  '));
}
