#!/usr/bin/env node
/*  t16_level3.js —— 三级（装配全过程）判据 + 装配动画的无头自检

    为什么单独一个文件：
      三级逐段判据与装配动画都属【UI 触发链 + 真几何】，Node 沙箱里跑不出
      （沙箱没有真 DOM、也没有正确的关节树渲染路径）。所以照 t10_ui_probe 的做法：
      生成一个带自检脚本的 armBuilder 副本，用 Edge 无头 --dump-dom 读结果。

    运动链（用户口径 = 规则手册三级）：
      ① 就位/插入 → ② 上推 LIFT → ③ 绕 P 轴 +90°（单元竖到与 Z 平行）
      → ④ 绕 Q 轴 ±SPIN（多姿态）
      装配盘（六边形工装）【全程不动】，动的是能量单元与夹着它的手。

    两个结论分开报：
      可装配 = ①②③；可多姿态 = 再加 ④

    跑法： node web/checks/t16_level3.js          （只生成 html）
           node web/checks/t16_level3.js --run    （生成并跑 Edge，打印结果）
*/
'use strict';
const fs = require('fs'), path = require('path'), cp = require('child_process');
const dir = __dirname, src = path.join(dir, '..', '..', 'build', 'armBuilder.html');
const out = path.join(dir, 't16_level3.html');
const NL = String.fromCharCode(10);

function wrap(CODE, tag) {
  /* ★ 函数名要带 tag：两段探针在【同一个页面作用域】里，
     都叫 go() 的话后者会把前者覆盖掉 —— 表现是"前一段结果凭空消失"（踩过）。 */
  return [
    '<script>',
    'window.__E=window.__E||[]; window.addEventListener("error",function(e){ window.__E.push(String(e.message)); });',
    'function go' + tag + '(){',
    '  var R=[], ok=function(c,m,x){ R.push((c?"✓ ":"✗ ")+m+(c?"":"   ← "+JSON.stringify(x))); };',
    '  try{',
    CODE,
    '  }catch(e){ R.push("✗ 探针异常 "+(e&&e.stack?e.stack:e)); }',
    '  var o=document.createElement("pre"); o.id="PROBE_' + tag + '";',
    '  o.textContent=R.join(String.fromCharCode(10))+String.fromCharCode(10)+"---END---"+String.fromCharCode(10)+',
    '    "FAIL="+R.filter(function(s){return s[0]==="✗";}).length+" PASS="+R.filter(function(s){return s[0]==="✓";}).length;',
    '  document.body.appendChild(o);',
    '}',
    'if(document.readyState==="complete") setTimeout(go' + tag + ',600); else window.addEventListener("load",function(){ setTimeout(go' + tag + ',600); });',
    '<' + '/script>'
  ].join(NL);
}

/* ---- ① 运动链几何 + 判据 + 面板 ---- */
const P1 = [
  '  document.getElementById("modal").classList.add("hidden");',
  '  applyConfig("tongji"); updateAll();',
  '  var CF=l3Conf();',
  '  ok(CF.steps.length===2, "旋转分成两步（绕P、绕Q）", CF.steps.length);',
  '  ok(CF.steps[0].axis==="p" && CF.steps[1].axis==="q", "第一步绕P轴、第二步绕Q轴",',
  '     [CF.steps[0].axis, CF.steps[1].axis]);',
  '  ok(CF.lift===100, "上推高度读配置（100mm）", CF.lift);',
  '  ok(insAxisIdx()===0, "插入方向对应工具 x 轴（实测定，不是想当然的 z）", insAxisIdx());',
  '  var Pt={x:-50,y:200,z:600,th:0,ph:90,al:0};',
  '  var off=asmOfArm(0);',
  '  ok(off.x===-500 && off.y===-200, "装配位取自模型的 asm，与 rules.init 一致", off);',
  '  var prev=G_LIFT; G_LIFT=0;',
  '  var T0=taskPoseBase(Pt,[0,0],off,{});',
  '  var PL=taskPoseBase(Pt,[0,0],off,{role:"plate"});',
  '  G_LIFT=100;',
  '  var T0u=taskPoseBase(Pt,[0,0],off,{});',
  '  var T1=taskPoseBase(Pt,[90,0],off,{});',
  '  var T2=taskPoseBase(Pt,[90,90],off,{});',
  '  var PL2=taskPoseBase(Pt,[90,90],off,{role:"plate"});',
  '  G_LIFT=prev;',
  '  ok(Math.abs(T0u.E[2]-(PL.E[2]+100))<1e-6, "★ 上推 100mm 体现在【能量单元】的高度上", [T0u.E[2],PL.E[2]]);',
  '  var angZ=function(v){ return Math.acos(Math.max(-1,Math.min(1,Math.abs(v[2]))))*180/Math.PI; };',
  '  ok(angZ(T0.er)>89, "★ 插入姿态下单元轴水平（与 Z 夹角 "+angZ(T0.er).toFixed(1)+" 度）");',
  '  ok(angZ(T1.er)<1, "★ 绕 P 转 90 度后单元轴【与 Z 轴平行】（夹角 "+angZ(T1.er).toFixed(1)+" 度）");',
  '  /* ★ 最后一步绕的是【插入轴 e】（固定、水平），不是"单元自身的轴"。',
  '     绕自身轴是原地自转 ⇒ 位置一点都不变，画面上就是不动（踩过）。',
  '     绕固定的 e 转，单元会被带开：绕 90 度后它与 Z 的夹角应回到 90 度。 */',
  '  ok(angZ(T2.er)>80, "★ 绕插入轴 e 转 90 度后单元轴被带到水平（与 Z 夹角 "+angZ(T2.er).toFixed(1)+" 度）");',
  '  var dQ=[T2.E[0]-T1.E[0],T2.E[1]-T1.E[1],T2.E[2]-T1.E[2]];',
  '  var erDiff=Math.abs(T2.er[0]-T1.er[0])+Math.abs(T2.er[1]-T1.er[1])+Math.abs(T2.er[2]-T1.er[2]);',
  '  ok(erDiff>1e-6, "★ 绕 e 转时单元的【朝向】确实变了（不是原地自转）",',
  '     {绕P后:T1.er.map(function(x){return +x.toFixed(2);}), 绕e后:T2.er.map(function(x){return +x.toFixed(2);})});',
  '  ok(Math.abs(T1.er[2])>0.99, "绕P那一步是把单元翻起来（轴向变为竖直）", T1.er[2].toFixed(4));',
  '  var dP=[T1.E[0]-PL2.E[0],T1.E[1]-PL2.E[1],T1.E[2]-PL2.E[2]];',
  '  ok(Math.hypot(dP[0],dP[1])<1e-9, "★ 装配盘位置全程不动（水平分量差 "+Math.hypot(dP[0],dP[1]).toFixed(6)+"）");',
  '  var dv=0; for(var k=0;k<3;k++) dv+=PL2.e[k]*PL.e[k];',
  '  ok(dv>0.999999, "★ 装配盘朝向也全程不动", dv.toFixed(6));',
  '  /* ★ 盘面基必须是"盘内"两根轴：拿插入轴 e 当盘内轴会让盘子绕 e 转 90°',
  '     （用户一眼看出"盘子横过来了"，而上面那条"盘不动"照样是绿的 —— 所以补这一条）。 */',
  '  var d0=function(a,b){ var s=0; for(var i=0;i<3;i++) s+=a[i]*b[i]; return s; };',
  '  ok(Math.abs(d0(PL.e,PL.u))<1e-9, "★ 盘面基 Fu 垂直于插入轴（在盘内，不是法向）", d0(PL.e,PL.u).toExponential(2));',
  '  ok(Math.abs(d0(PL.u,PL.v))<1e-9, "★ 盘面基两根轴互相垂直", d0(PL.u,PL.v).toExponential(2));',
  '  var F0=coreFrame(Pt), uW=dirToBase(F0.u), vW=dirToBase(F0.v);',
  '  var near=function(a,b){ var s=0; for(var i=0;i<3;i++) s+=Math.abs(a[i]-b[i]); return s<1e-9; };',
  '  ok(near(PL.u,uW) && near(PL.v,vW), "★ 盘面基与 coreFrame 的 u/v 一致（没被换成 e）");',
  '  var g=asmFeasibleA(Pt,0,{q0:Q.slice(),seed0:12});',
  '  ok(g.reach&&g.lift&&g.rotP, "★ 中心点：插入+上推+绕P竖起 = 可装配",',
  '     {reach:g.reach,lift:g.lift,rotP:g.rotP});',
  '  ok(g.multi===true, "★ 中心点：再绕Q ±90 度 = 可多姿态", {rotQ:g.rotQ});',
  '  ok(g.asm===true, "两个结论是分开的字段（asm / multi）", {asm:g.asm,multi:g.multi});',
  '  var dp=document.getElementById("dataBody").innerHTML;',
  '  ok(dp.indexOf("可装配（插入 + 上推 + 绕P竖起）")>=0, "面板有「可装配」这一行");',
  '  ok(dp.indexOf("可多姿态")>=0, "面板有「可多姿态」这一行");',
  '  ok(dp.indexOf("绕 P 轴转 +90")>=0 && dp.indexOf("绕 Q 轴转 ±90")>=0, "面板写明两步旋转各自的名字");',
  '  ok(dp.indexOf("真正的任务域")>=0, "面板有真正的任务域（现算，不是手册死盒子）");',
  '  ok(dp.indexOf("卡在哪一段")>=0, "面板有卡在哪一段的计数");',
  '  ok(window.__E.length===0, "这一轮无 JS 错误", window.__E);'
].join(NL);

/* ---- ② 装配动画：直接驱动 animTick（给合成时刻），不依赖 rAF ---- */
const P2 = [
  '  document.getElementById("modal").classList.add("hidden");',
  '  applyConfig("tongji"); updateAll();',
  '  var rows=[];',
  '  /* ★ 不用 rAF 等它自己跑：无头环境里虚拟时钟会把时间抢走，',
  '     动画永远跑不完，会得出"卡住"的假结论。直接给合成时刻驱动。 */',
  '  startAnim();',
  '  var base=performance.now();',
  '  for(var k=0;k<=400;k++){',
  '    if(!ANIM) break;',
  '    animTick(base + k*40);',
  '    var F=fkAll(Q), ee=F[F.length-1], A2=eeAxes(ee);',
  '    rows.push({i:(ANIM?ANIM.i:5), dz:(QASM?QASM.dz:0),',
  '               degs:(QASM&&QASM.degs?QASM.degs:[0,0]).slice(), p:[ee.t[0],ee.t[1],ee.t[2]],',
  '               x:A2.x.slice()});',
  '  }',
  '  ok(!ANIM, "动画走完了（没卡住）", !!ANIM);',
  '  var byI={}; rows.forEach(function(r){ (byI[r.i]=byI[r.i]||[]).push(r); });',
  '  var idx=Object.keys(byI).map(Number).sort(function(a,b){return a-b;});',
  '  ok(idx.length>=5, "走过 5 个以上阶段（就位/插入/上推/绕P/绕Q/完成）", idx);',
  '  var span=function(fs,f){ var lo=1e9,hi=-1e9; fs.forEach(function(r){ var v=f(r); if(v<lo)lo=v; if(v>hi)hi=v; }); return hi-lo; };',
  '  var dist=function(fs){ var lo=[1e9,1e9,1e9],hi=[-1e9,-1e9,-1e9];',
  '    fs.forEach(function(r){ for(var i=0;i<3;i++){ if(r.p[i]<lo[i])lo[i]=r.p[i]; if(r.p[i]>hi[i])hi[i]=r.p[i]; } });',
  '    return Math.hypot(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]); };',
  '  var up=byI[2]||[], rp=byI[3]||[], rq=byI[4]||[];',
  '  ok(span(up,function(r){return r.dz;})>90, "★ 上推段：dz 从 0 走到 100（"+span(up,function(r){return r.dz;}).toFixed(0)+"mm）");',
  '  ok(dist(up)>50, "★ 上推段：手真的跟着往上走（"+dist(up).toFixed(0)+"mm）");',
  '  ok(span(rp,function(r){return r.degs[0];})>80, "★ 绕P段：转角从 0 走到 90（"+span(rp,function(r){return r.degs[0];}).toFixed(0)+"度）");',
  '  /* 绕 P 那一步：夹爪抓的是【端面】，端面就在 P 轴上 ⇒ 手只改姿态、几乎不平移。',
  '     所以这里【不能】断言"手要位移几十毫米"（那样写是错的）；',
  '     要验的是：零件确实被翻起来了（末端姿态跟着变）。 */',
  '  var poseSpan=function(fs){ var mx=0; fs.forEach(function(a){ fs.forEach(function(b){',
  '    var c=Math.max(-1,Math.min(1,a.x[0]*b.x[0]+a.x[1]*b.x[1]+a.x[2]*b.x[2]));',
  '    var d=Math.acos(c)*180/Math.PI; if(d>mx)mx=d; }); }); return mx; };',
  '  ok(rp.length>3 && (poseSpan(rp)>10 || dist(rp)>10), "★ 绕P段：末端姿态（或位置）确实变了（姿态 "+(rp.length?poseSpan(rp).toFixed(0):0)+"度 / 位移 "+(rp.length?dist(rp).toFixed(0):0)+"mm）");',
  '  ok(span(rq,function(r){return r.degs[1];})>80, "★ 绕插入轴段：转角从 0 走到 90（"+span(rq,function(r){return r.degs[1];}).toFixed(0)+"度）");',
  '  ok(dist(rq)>10, "★ 绕插入轴段：手跟着绕（"+dist(rq).toFixed(0)+"mm）");',
  '  ok(window.__E.length===0, "动画全程无 JS 错误", window.__E);'
].join(NL);

const html = fs.readFileSync(src, 'utf8')
  .replace('</body>', wrap(P1, 'A') + wrap(P2, 'B') + '</body>');
fs.writeFileSync(out, html, 'utf8');
console.log('已生成 ' + out + '（' + html.length + ' 字节）');

if (process.argv.includes('--run')) {
  const edge = 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe';
  const url = 'file:///' + out.replace(/\\/g, '/').split('/').map((s, i) => i === 0 ? s :
    encodeURIComponent(s)).join('/');
  const tmp = path.join(process.env.TEMP, 'dsh-edge-t16' + Date.now());
  const r = cp.spawnSync(edge, ['--headless=new', '--disable-gpu', '--hide-scrollbars',
    '--virtual-time-budget=150000', '--window-size=1700,1000', '--user-data-dir=' + tmp,
    '--dump-dom', url], { encoding: 'utf8', maxBuffer: 64 * 1024 * 1024 });
  const all = String(r.stdout || '');
  let txt = '', nF = 0, nP = 0;
  for (const tag of ['A', 'B']) {
    /* --dump-dom 会把属性引号去掉（id=PROBE_A），所以引号必须可选 */
    const m = new RegExp('<pre id="?PROBE_' + tag + '"?>([\\s\\S]*?)<\\/pre>').exec(all);
    if (!m) { console.log('没抓到 PROBE_' + tag + '；stderr:\n' + (r.stderr || '').slice(0, 1500)); process.exit(2); }
    const t = m[1].replace(/&amp;/g, '&').replace(/&lt;/g, '<').replace(/&gt;/g, '>');
    txt += '\n===== ' + (tag === 'A' ? '① 运动链与判据' : '② 装配动画') + ' =====\n' + t + '\n';
    nF += +( /FAIL=(\d+)/.exec(t) || [0, 1] )[1];
    nP += +( /PASS=(\d+)/.exec(t) || [0, 0] )[1];
  }
  console.log(txt);
  console.log('合计 PASS=' + nP + ' FAIL=' + nF);
  process.exit(nF === 0 ? 0 : 1);
}
