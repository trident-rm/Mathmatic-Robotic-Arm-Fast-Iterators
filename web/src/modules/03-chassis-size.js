"use strict";
/* ==================================================================
   底盘碰撞箱 + 伸展尺寸校核（只做【显示】与【尺寸校核】，不参与任何运动学）
   ------------------------------------------------------------------
   ① 底盘中心点：由 base 系决定（可切"相对整机原点"），默认 base 下 (0, 140, 0)
   ② 碰撞箱：x/y 以中心点为中心、z 从中心点沿【−Z】延伸 size_z
      ⇒ 中心点 = 箱体【上表面中心】，箱体 580(x) × 580(y) × 105(z)
      ⇒ 接地中心 = 箱体【底面中心】= 中心点 + (0,0,−105) = base (0,140,−105)
   ③ 伸展尺寸（规则）：
      · 长宽高 ≤ 1200 × 1200 × 1100
      · 直径 1500 的球：机器人放在一个与地面平行、距球心 480 的平面上，
        在该平面上的任意变形/移动不得超出此球
      ⇒ 球心 = 接地中心沿地面法向(+Z)抬 480，半径 750
      ⇒ 球与放置平面交线半径 = √(750²−480²) ≈ 576.3（地面上真正卡人的那个圈）
   ④ 另外只报一个【机械臂长度】（各段杆长之和 + 采样估的最大伸展）。
      ★ 刻意【不】报"当前位形/可达集的长宽高包围盒"：模型里没有杆件的碰撞体，
        而且臂的 yaw 限位还没加（模型上能转到车背面），拿骨架去量长宽高只会得出假数。
   ================================================================== */
/* ★ 车体尺寸规则来自配置（configs/*.js 的 chassis 段），引擎不再自带同济的数。
   CHASSIS_V 由配置给（改配置里的默认值时要 +1，否则老存档会把新默认值盖回去）。
   "恢复默认"取的是【当前配置】的默认值。 */
let CHASSIS_V = 2;
const chassisDefault = () => (typeof CONFIG !== 'undefined' && CONFIG && CONFIG.chassis)
  ? CONFIG.chassis.chassis : { posMode:'base', pos:[0,0,0], size:[1,1,1], zDir:-1 };
const sizerDefault   = () => (typeof CONFIG !== 'undefined' && CONFIG && CONFIG.chassis)
  ? CONFIG.chassis.sizer   : { lwh:[0,0,0], ballD:0, ballH:0 };
const C_CHASSIS = '#ffd400', C_LWH = '#00e5c0', C_BALL = '#a78bfa';
const F_SM = "11px system-ui,'Microsoft YaHei'";
const F_BD = "bold 11px system-ui,'Microsoft YaHei'";

/* ---------------- 几何：全部换算到【整机原点系】 ---------------- */
function chassisGeom(){
  if(!MODEL) return null;
  const Rb = rotZdeg(MODEL.base.yaw), tb = MODEL.base.xyz;
  /* 底盘中心点：相对 base 系 ⇒ 先按 base 的 yaw 转，再平移到整机系 */
  const C = (CHASSIS.posMode === 'base')
          ? addv(mulv(Rb, CHASSIS.pos.slice()), tb)
          : [+CHASSIS.pos[0]||0, +CHASSIS.pos[1]||0, +CHASSIS.pos[2]||0];
  const nz = v => { const a = Math.abs(+v); return a > 1e-6 ? a : 1e-6; };
  const sx = nz(CHASSIS.size[0]), sy = nz(CHASSIS.size[1]), sz = nz(CHASSIS.size[2]);
  const down = (CHASSIS.zDir < 0);
  const zTop = down ? C[2] : C[2] + sz;
  const zBot = down ? C[2] - sz : C[2];
  const bx = [C[0], C[1], (zTop + zBot)/2];          // 箱体几何中心
  const bh = [sx/2, sy/2, (zTop - zBot)/2];          // 三个半长
  const GC = [C[0], C[1], zBot];                     // 接地中心 = 箱体底面中心
  const SR = nz(SIZER.ballD)/2;
  const ballH = +SIZER.ballH || 0;
  const SC = [GC[0], GC[1], GC[2] + ballH];          // 球心：接地中心沿 +Z 抬 ballH
  const lh = SIZER.lwh.map(v => nz(v)/2);
  const LC = [GC[0], GC[1], GC[2] + lh[2]];          // 长宽高框：xy 以接地中心为中心，z 从地面往上
  const planeR = (SR > Math.abs(ballH)) ? Math.sqrt(SR*SR - ballH*ballH) : 0;
  return {C, bx, bh, GC, SC, SR, ballH, LC, lh, Rb, planeR, zTop, zBot};
}
/* 盒子的 8 个角（i 的 bit2/1/0 = x/y/z 的正负）与 6 个面（顺时针环） */
function boxCorners(c, h, R){
  const out = [];
  for(let i=0;i<8;i++){
    const s = [(i&4)?1:-1, (i&2)?1:-1, (i&1)?1:-1];
    out.push(addv(c, mulv(R, [s[0]*h[0], s[1]*h[1], s[2]*h[2]])));
  }
  return out;
}
const BOX_FACES = [[0,2,6,4],[1,5,7,3],[0,4,5,1],[2,3,7,6],[0,1,3,2],[4,6,7,5]];
const BOX_EDGES = (()=>{ const E=[]; for(let i=0;i<8;i++) for(let j=i+1;j<8;j++){
  const d=i^j; if(d===1||d===2||d===4) E.push([i,j]); } return E; })();

/* ---------------- 画：线框盒 ---------------- */
function drawBoxFrame(g, S, c, h, R, color, lw, dash, fillA){
  const P = boxCorners(c, h, R).map(S);
  g.save();
  if(fillA){
    g.globalAlpha = fillA; g.fillStyle = color;
    for(const f of BOX_FACES){
      g.beginPath();
      f.forEach((k,i)=> i ? g.lineTo(P[k][0],P[k][1]) : g.moveTo(P[k][0],P[k][1]));
      g.closePath(); g.fill();
    }
    g.globalAlpha = 1;
  }
  g.strokeStyle = color; g.lineWidth = lw || 1.6;
  if(dash) g.setLineDash(dash);
  g.beginPath();
  for(const [i,j] of BOX_EDGES){ g.moveTo(P[i][0],P[i][1]); g.lineTo(P[j][0],P[j][1]); }
  g.stroke();
  g.restore();
  return P;
}

/* ---------------- 画：底盘箱 / 长宽高框 / 限制球（在点云下层） ---------------- */
function drawSizeOverlay(g, S, label){
  const G = chassisGeom(); if(!G) return;
  const zc = G.GC[2];

  /* ---- ① 放置平面（过接地中心、平行地面）+ 球与它的交线 ---- */
  const sq = Math.max(G.SR*1.5, G.lh[0]*1.1);
  g.save();
  g.strokeStyle = 'rgba(167,139,250,.28)'; g.lineWidth = 1; g.setLineDash([4,6]);
  const c1=S([G.GC[0]-sq,G.GC[1]-sq,zc]), c2=S([G.GC[0]+sq,G.GC[1]-sq,zc]),
        c3=S([G.GC[0]+sq,G.GC[1]+sq,zc]), c4=S([G.GC[0]-sq,G.GC[1]+sq,zc]);
  g.beginPath(); g.moveTo(c1[0],c1[1]); g.lineTo(c2[0],c2[1]);
  g.lineTo(c3[0],c3[1]); g.lineTo(c4[0],c4[1]); g.closePath(); g.stroke();
  g.restore();
  if(SHOWBOX.ball && G.planeR > 1){
    const pts=[];
    for(let a=0;a<=72;a++){ const t = a*2*Math.PI/72;
      pts.push(S([G.GC[0]+G.planeR*Math.cos(t), G.GC[1]+G.planeR*Math.sin(t), zc])); }
    g.save();
    g.strokeStyle = 'rgba(167,139,250,.9)'; g.lineWidth = 1.6; g.setLineDash([7,5]);
    g.beginPath(); pts.forEach((p,i)=> i ? g.lineTo(p[0],p[1]) : g.moveTo(p[0],p[1])); g.stroke();
    g.restore();
    label('地面上限圈 R='+G.planeR.toFixed(0), pts[0][0]+6, pts[0][1]+15,
          'rgba(190,170,255,.95)', F_SM);
  }

  /* ---- ② 限制球 ⌀1500：球心 = 接地中心 + 480 ---- */
  if(SHOWBOX.ball){
    g.save();
    g.globalAlpha = .3; g.strokeStyle = C_BALL; g.lineWidth = 1;
    for(let k=1;k<=5;k++){                                   // 纬线
      const ph = Math.PI*k/6, r = G.SR*Math.sin(ph), dz = G.SR*Math.cos(ph), pts=[];
      for(let a=0;a<=48;a++){ const t = a*Math.PI/24;
        pts.push(S([G.SC[0]+r*Math.cos(t), G.SC[1]+r*Math.sin(t), G.SC[2]+dz])); }
      g.beginPath(); pts.forEach((p,i)=> i ? g.lineTo(p[0],p[1]) : g.moveTo(p[0],p[1])); g.stroke();
    }
    for(let k=0;k<6;k++){                                    // 经线
      const th = Math.PI*k/6, ct = Math.cos(th), st = Math.sin(th), pts=[];
      for(let a=0;a<=48;a++){ const t = a*Math.PI/24, sr = G.SR*Math.sin(t);
        pts.push(S([G.SC[0]+sr*ct, G.SC[1]+sr*st, G.SC[2]+G.SR*Math.cos(t)])); }
      g.beginPath(); pts.forEach((p,i)=> i ? g.lineTo(p[0],p[1]) : g.moveTo(p[0],p[1])); g.stroke();
    }
    /* 正交投影下球的轮廓就是屏幕上的圆（proj 是纯旋转，保长），半径 = 屏幕距离 */
    const p0 = S(G.SC), p1 = S([G.SC[0]+G.SR, G.SC[1], G.SC[2]]);
    const rp = Math.hypot(p1[0]-p0[0], p1[1]-p0[1]);
    g.globalAlpha = .85; g.lineWidth = 1.8;
    g.beginPath(); g.arc(p0[0], p0[1], rp, 0, 6.2832); g.stroke();
    g.restore();
    label('限制球 ⌀'+(2*G.SR).toFixed(0)+'  R='+G.SR.toFixed(0),
          p0[0]+rp*0.70, p0[1]-rp*0.70, C_BALL, F_BD);
    /* 球心 → 接地中心 的 480 */
    const sg0 = S(G.GC);
    g.save();
    g.strokeStyle = 'rgba(167,139,250,.9)'; g.lineWidth = 1.4; g.setLineDash([4,3]);
    g.beginPath(); g.moveTo(p0[0],p0[1]); g.lineTo(sg0[0],sg0[1]); g.stroke();
    g.restore();
    label('球心距接地中心 '+G.ballH.toFixed(0), (p0[0]+sg0[0])/2+7, (p0[1]+sg0[1])/2,
          'rgba(190,170,255,.95)', F_SM);
    g.fillStyle = C_BALL; g.beginPath(); g.arc(p0[0],p0[1],3.4,0,6.2832); g.fill();
    label('球心', p0[0]+6, p0[1]-6, C_BALL, F_SM);
  }

  /* ---- ③ 长宽高框 1200 × 1200 × 1100 ---- */
  if(SHOWBOX.lwh){
    drawBoxFrame(g, S, G.LC, G.lh, G.Rb, C_LWH, 1.8, [9,5], 0.035);
    const t = S([G.LC[0], G.LC[1], G.LC[2]+G.lh[2]-2]);
    label('长宽高 '+SIZER.lwh.join('×')+'（地面起算）', t[0]+8, t[1]-8, C_LWH, F_BD);
  }

  /* ---- ④ 底盘碰撞箱（黄框）---- */
  if(SHOWBOX.chassis){
    drawBoxFrame(g, S, G.bx, G.bh, G.Rb, C_CHASSIS, 2.4, null, 0.075);
    /* 底面四边（= 接地中心所在的面）单独描粗 */
    const P = boxCorners(G.bx, G.bh, G.Rb).map(S);
    g.save(); g.strokeStyle = C_CHASSIS; g.lineWidth = 3.4; g.beginPath();
    const f = BOX_FACES[4];
    for(let k=0;k<4;k++){ const a=P[f[k]], b=P[f[(k+1)%4]];
      g.moveTo(a[0],a[1]); g.lineTo(b[0],b[1]); }
    g.stroke(); g.restore();
    /* 标签挂在【底面右下角】——底盘上表面中心正好是臂的 base，那里已经挤满了坐标系名 */
    let bi = f[0], bs = -Infinity;
    for(const k of f){ const sc = P[k][0] + P[k][1]*0.6; if(sc > bs){ bs = sc; bi = k; } }
    label('底盘碰撞箱 '+CHASSIS.size.join('×'), P[bi][0]+6, P[bi][1]+18, C_CHASSIS, F_BD);
  }

  /* ---- ⑤ 两个关键点：底盘中心点 / 接地中心 ---- */
  const pc = S(G.C), pg = S(G.GC);
  g.save();
  if(SHOWBOX.chassis){
    g.fillStyle = C_CHASSIS; g.beginPath(); g.arc(pc[0],pc[1],4,0,6.2832); g.fill();
    g.strokeStyle = C_CHASSIS; g.lineWidth = 2;
    g.beginPath(); g.arc(pg[0],pg[1],7,0,6.2832); g.stroke();
    g.beginPath();
    g.moveTo(pg[0]-12,pg[1]); g.lineTo(pg[0]-4,pg[1]);
    g.moveTo(pg[0]+4,pg[1]);  g.lineTo(pg[0]+12,pg[1]);
    g.moveTo(pg[0],pg[1]-12); g.lineTo(pg[0],pg[1]-4);
    g.moveTo(pg[0],pg[1]+4);  g.lineTo(pg[0],pg[1]+12);
    g.stroke();
  }
  g.restore();
  if(SHOWBOX.chassis){
    /* 底盘中心点的标签【右对齐】放在点的左边：那一点周围全是臂的坐标系名，往右放必然打架 */
    const txt = '底盘中心 base(' + CHASSIS.pos.join(', ') + ')';
    g.font = F_BD;
    const tw = g.measureText ? g.measureText(txt).width : txt.length*7;
    label(txt, pc[0]-tw-8, pc[1]-7, C_CHASSIS, F_BD);
    label('接地中心', pg[0]+10, pg[1]+17, C_CHASSIS, F_BD);
  }
}

/* ---- 超出限制球的采样点【不画】 ----
   曾经画过红点，删掉了。原因：采出来的"超球"点里有**一大片在接地中心 z 以下**，
   也就是机械臂**穿透到地面以下**去了——限位没管地面，那种点根本不是真的超出，
   画出来只会让人以为臂真的伸到球外面了（见 README 第 5 节第 8 条）。 */

/* ---------------- 校核：底盘箱的范围 + 机械臂长度 ---------------- */
/* ★ 底盘箱是【跟着车转】的（= 整机系绕 Z 转 base.yaw），所以整机系下的
     轴向范围必须由 8 个角算，不能拿"中心±半长"直接加减（方箱子看不出来，长方形就会错） */
function boxRange(G){
  const P = boxCorners(G.bx, G.bh, G.Rb);
  const mn=[Infinity,Infinity,Infinity], mx=[-Infinity,-Infinity,-Infinity];
  for(const p of P) for(let i=0;i<3;i++){ if(p[i]<mn[i]) mn[i]=p[i]; if(p[i]>mx[i]) mx[i]=p[i]; }
  return {mn, mx};
}
/* 机械臂长度：
     · segs = 各段杆长 = 相邻坐标系原点之间的距离（就是各段 offset 的模）
     · total = 各段之和 —— 这就是"把臂摊直了有多长"，与限位无关
     · reach = 当前【限位】下末端离 base 原点的最远距离（从可达采样里量，是估计值）
   ★ 不给"包围盒长宽高"：模型里没有杆件的碰撞体，臂的 yaw 限位也还没加，
     拿骨架量长宽高只会得出假数（见文件头注释） */
function armStats(){
  const G = chassisGeom();
  const segs = CHAIN.map(c => Math.hypot(c.offset[0], c.offset[1], c.offset[2]));
  const total = segs.reduce((a,b)=>a+b, 0);
  const b0 = MODEL.base.xyz;
  const zG = G ? G.GC[2] : -Infinity;          // 地面 = 过接地中心、平行地面的那张平面
  let reach = 0, zMax = -Infinity, zMin = Infinity, under = 0, used = 0;
  for(const p of CLOUD){
    /* ★ 地面以下的采样点直接丢掉：限位里没有地面，采出来的"地下点"是假的
       （臂不可能穿透地面），算进"最大伸展"会把数字撑大 */
    if(p[2] < zG){ under++; continue; }
    used++;
    const d = Math.hypot(p[0]-b0[0], p[1]-b0[1], p[2]-b0[2]);
    if(d > reach) reach = d;
    if(p[2] > zMax) zMax = p[2];
    if(p[2] < zMin) zMin = p[2];
  }
  if(!used){ zMin = zMax = zG; }
  return {segs, total, reach, zMin, zMax, n:CLOUD.length, used, under, zG};
}

function sizePanelHTML(){
  const G = chassisGeom();
  if(!G) return '<div id="sizeSect"></div>';
  const A = armStats();
  const R = boxRange(G);
  const P = p => '(' + p.map(v=>v.toFixed(0)).join(', ') + ')';
  const R2 = (mn,mx,i) => mn[i].toFixed(0)+' ~ '+mx[i].toFixed(0);
  return `<div id="sizeSect">
    <div class="sect">底盘碰撞箱 / 伸展尺寸（只显示，不参与运动学）</div>
    <div class="grid">
      <span>底盘中心点（base 系）</span><b>${CHASSIS.posMode==='base'?P(CHASSIS.pos):'—'}</b>
      <span>底盘中心点（整机系）</span><b>${P(G.C)}</b>
      <span>底盘箱范围（整机系）</span><b>x ${R2(R.mn,R.mx,0)} · y ${R2(R.mn,R.mx,1)} · z ${R2(R.mn,R.mx,2)}</b>
      <span>接地中心（= 箱体底面中心）</span><b>${P(G.GC)}</b>
      <span>球心 / 半径</span><b>${P(G.SC)} · R ${G.SR.toFixed(0)}</b>
      <span>地面上限圈半径 √(R²−${G.ballH.toFixed(0)}²)</span><b>${G.planeR.toFixed(1)} mm</b>
    </div>

    <div class="sect">机械臂长度</div>
    <div class="grid">
      <span>各段杆长（相邻坐标系间距）</span><b>${A.segs.map(v=>v.toFixed(1)).join(' / ')} mm</b>
      <span>杆长之和 ⇒ 机械臂长度</span><b>${A.total.toFixed(1)} mm</b>
      <span>最大伸展（末端离 base 原点）</span><b>${A.reach.toFixed(0)} mm</b>
      <span>末端高度范围 z</span><b>${A.zMin.toFixed(0)} ~ ${A.zMax.toFixed(0)} mm</b>
    </div>
    <div class="one" style="font-size:10.5px">
      「杆长之和」是把臂摊直了的总长，与限位无关；「最大伸展」是<b>当前限位下</b>末端能到的最远处，
      由 ${A.n} 个可达采样点量出（<b>估计值</b>）。<br>
      ★ <b>只统计地面以上</b>的采样点：地面 = 过接地中心、平行地面的平面（z = ${A.zG.toFixed(0)}）。
      本次 ${A.used}/${A.n} 个在地面以上，另有 <b>${A.under}</b> 个落到地面以下
      —— 那是<b>限位没管地面</b>造成的假点（臂不可能穿透地面），全部剔除，否则"最大伸展"会被撑大。<br>
      规则：长宽高 ≤ ${SIZER.lwh.join('×')}；⌀${(2*G.SR).toFixed(0)} 球、球心 = 接地中心 z 往上 ${G.ballH.toFixed(0)}。
      <b>不报包围盒</b>：模型里没有杆件碰撞体、臂的 yaw 限位也还没加，量出来是假数。
    </div>
  </div>`;
}
/* 改底盘参数只重画尺寸段（analyzeTask 里的 IK 复核很贵，不要跟着重跑） */
function refreshSize(){
  const el = document.getElementById('dataBody');
  if(!el) return;
  const old = document.getElementById('sizeSect');
  if(old) old.outerHTML = sizePanelHTML();
  else if(PANEL_REST) el.innerHTML = sizePanelHTML() + PANEL_REST;
}

/* ==================================================================
   右侧：底盘 / 伸展尺寸 设置面板
   ================================================================== */
function openChassisForm(force){
  const f = document.getElementById('cform');
  if(f.className === 'on' && !force){ f.className = ''; return; }   // 再点一次收起
  f.className = 'on';
  const num = (id,v,st) => `<input type="number" id="${id}" step="${st}" value="${v}">`;
  f.innerHTML = `
    <div class="fld" style="justify-content:space-between">
      <b>底盘碰撞箱 / 伸展尺寸</b>
      <span style="display:flex;gap:6px">
        <button id="cReset">恢复默认</button>
        <button id="cClose">收起</button>
      </span>
    </div>
    <div class="fld"><label>中心点参照</label>
      <label class="rad"><input type="radio" name="cPos" value="base"
        ${CHASSIS.posMode==='base'?'checked':''}>相对 base 系</label>
      <label class="rad"><input type="radio" name="cPos" value="origin"
        ${CHASSIS.posMode!=='base'?'checked':''}>相对整机原点 O</label></div>
    <div class="fld"><label>中心点坐标</label>
      X ${num('cPx',CHASSIS.pos[0],1)} Y ${num('cPy',CHASSIS.pos[1],1)}
      Z ${num('cPz',CHASSIS.pos[2],1)} mm</div>
    <div class="sub">默认 base (0, 140, 0) ⇒ base 正 y 方向 140mm、与 base 同高</div>
    <div class="fld"><label>箱体尺寸</label>
      X ${num('cSx',CHASSIS.size[0],10)} Y ${num('cSy',CHASSIS.size[1],10)}
      Z ${num('cSz',CHASSIS.size[2],10)} mm</div>
    <div class="fld"><label>Z 延伸方向</label>
      <select id="cZdir">
        <option value="-1" ${CHASSIS.zDir<0?'selected':''}>−Z 向下（中心点 = 上表面中心）</option>
        <option value="1"  ${CHASSIS.zDir>0?'selected':''}>+Z 向上（中心点 = 下表面中心）</option>
      </select></div>
    <div class="sub">x/y 以中心点为中心；接地中心 = 箱体底面中心（默认 = 中心点 − 105mm）</div>
    <div class="fld"><label>长宽高限</label>
      ${num('cLw',SIZER.lwh[0],10)} × ${num('cLh',SIZER.lwh[1],10)} × ${num('cLz',SIZER.lwh[2],10)} mm</div>
    <div class="fld"><label>限制球</label>
      直径 ${num('cBd',SIZER.ballD,50)} mm</div>
    <div class="fld"><label></label>
      球心距接地中心 ${num('cBh',SIZER.ballH,10)} mm（沿地面法向）</div>
    <div class="sub">★ 改完离开输入框（或按回车）就生效，三维视图与数据面板同步刷新</div>`;
  f.onchange = applyChassisForm;               // 事件委托：任一控件 change 即应用（不必滚到底找按钮）
  document.getElementById('cClose').onclick = () => { f.className = ''; };
  document.getElementById('cReset').onclick = () => {
    Object.assign(CHASSIS, JSON.parse(JSON.stringify(chassisDefault())));
    Object.assign(SIZER,   JSON.parse(JSON.stringify(sizerDefault())));
    openChassisForm(true);                       // 重画表单 ⇒ 输入框回到默认值
    applyChassisForm();
  };
}
function applyChassisForm(){
  const g = id => document.getElementById(id);
  if(!g('cPx')) return;
  CHASSIS.posMode = document.querySelector('input[name=cPos]:checked').value;
  CHASSIS.pos  = [+g('cPx').value||0, +g('cPy').value||0, +g('cPz').value||0];
  CHASSIS.size = [+g('cSx').value||0, +g('cSy').value||0, +g('cSz').value||0];
  CHASSIS.zDir = (+g('cZdir').value < 0) ? -1 : 1;
  SIZER.lwh    = [+g('cLw').value||0, +g('cLh').value||0, +g('cLz').value||0];
  SIZER.ballD  = +g('cBd').value||0;
  SIZER.ballH  = +g('cBh').value||0;
  refreshSize(); draw3D(); saveLocal();
}


/* ---------------- 控件接线 ---------------- */
document.getElementById('btnChassis').onclick = () => openChassisForm();
const CB_MAP = {cbChassis:'chassis', cbLWH:'lwh', cbBall:'ball'};
function syncBoxChecks(){                    // 存档恢复后再回填一次（loadCfg 在 DOMContentLoaded 里跑）
  for(const id in CB_MAP){
    const el = document.getElementById(id);
    if(el) el.checked = SHOWBOX[CB_MAP[id]] !== false;
  }
  if(typeof syncVisMenu === 'function') syncVisMenu();
}
/* 四个显示开关（坐标架 / 底盘框 / 长宽高框 / 限制球）现在收在顶栏一个下拉里，
   回填交给 06-ui.js 的 syncVisMenu()。一键按钮 btnFrames 已随改版去掉。 */
for(const id in CB_MAP){
  document.getElementById(id).onchange = e => {
    SHOWBOX[CB_MAP[id]] = e.target.checked; draw3D(); saveLocal();
    if(typeof syncVisMenu === 'function') syncVisMenu();   // 下拉里的勾选状态跟着走
  };
}
/* 自检钩子：#size 直接打开底盘/尺寸面板；SIZEINFO() 供无头自检读取全部推导值 */
window.addEventListener('DOMContentLoaded', () => {
  syncBoxChecks();
  if(location.hash.indexOf('size') >= 0) openChassisForm(true);
});
window.SIZEINFO = function(){
  const G = chassisGeom(); if(!G) return null;
  const A = armStats(), R = boxRange(G);
  return {baseYaw: MODEL.base.yaw, C:G.C, box:{c:G.bx, h:G.bh}, range:R, GC:G.GC,
          SC:G.SC, SR:G.SR, planeR:G.planeR, lwh:SIZER.lwh,
          segs:A.segs, armLen:A.total, reach:A.reach, zMin:A.zMin, zMax:A.zMax,
          cloudN:A.n, showbox:{chassis:SHOWBOX.chassis, lwh:SHOWBOX.lwh, ball:SHOWBOX.ball}};
};