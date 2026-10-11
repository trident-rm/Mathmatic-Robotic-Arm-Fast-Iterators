/*  t10_ui_probe.js —— 生成一个带自检脚本的 armBuilder 副本，用 Edge 无头 --dump-dom 读结果
    跑法： node t10_ui_probe.js            （只生成 t10_ui_probe.html）
           node t10_ui_probe.js --run      （生成并直接跑 Edge，打印结果）
    为什么不用 jsdom：这里要验的是【canvas 上真的画/不画】，
    所以直接数像素——黄框(#ffd400) / 长宽高框(#00e5c0) / 球(#a78bfa) 的数量。 */
const fs = require('fs'), path = require('path'), cp = require('child_process');
const dir = __dirname, src = path.join(dir, '..', '..', 'build', 'armBuilder.html');
/* --shot：不跑断言，只把"两个面板同时打开"的副本生成出来，供截图看布局 */
const SHOT = process.argv.includes('--shot');
const out = path.join(dir, SHOT ? 't10_both_probe.html' : 't10_ui_probe.html');

const shotProbe = `
<script>
window.addEventListener('DOMContentLoaded', () => {
  setTimeout(() => { openChassisForm(true); openForm(0); }, 60);   // 底盘设置 + 编辑 link
});
<\/script>
`;

const probe = `
<script>
window.addEventListener('DOMContentLoaded', () => {           // 注册在最后 ⇒ 跑在 loadCfg/setModel 之后
  const R = [], ok = (c,m,x) => R.push((c?'✓ ':'✗ ')+m+(c?'':'   ← '+JSON.stringify(x)));
  const $ = id => document.getElementById(id);
  const cv = $('c3d'), ctx = cv.getContext('2d');
  function count(spec, tol){                                   // 数画布上某颜色的像素
    const im = ctx.getImageData(0,0,cv.width,cv.height).data;
    const test = (typeof spec === 'function') ? spec
      : (r,g,b) => Math.abs(r-spec[0])<=tol[0] && Math.abs(g-spec[1])<=tol[1] &&
                   Math.abs(b-spec[2])<=(tol[2]===undefined?tol[0]:tol[2]);
    let n = 0;
    for(let i=0;i<im.length;i+=4)
      if(im[i+3]>200 && test(im[i],im[i+1],im[i+2])) n++;
    return n;
  }
  const draw = () => { draw3D(); };
  const YEL=[255,212,0], TEA=[0,229,192], RED=[255,59,59];
  /* 紫 = 限制球；★ 不能只按颜色距离卡——同屏还有两个蓝紫：
       关节轴洋红 #e879f9（b−r = 17）与可达点云蓝 #3ea6ff（r 只有 62）。
     球 #a78bfa 的特征是【红也高(167) + 蓝比红高 83 + 蓝比绿高 111】 */
  const isPur = (r,g,b) => r > 110 && (b - r) > 45 && (b - g) > 60 && b > 120;
  const tY = [10,10,30], tT = [12,12,12], tR = [14,14,14];

  ok(!!MODEL && MODEL.links.length === 7, 'DOMContentLoaded 后模型已载入（示例 7 段）', MODEL && MODEL.links.length);

  /* ---- ① 数据面板里有没有尺寸校核段 ---- */
  const dp = $('dataBody').innerHTML;
  ok(dp.indexOf('底盘碰撞箱 / 伸展尺寸') >= 0, '数据面板出现"底盘碰撞箱 / 伸展尺寸"段');
  ok(dp.indexOf('接地中心') >= 0, '面板里有接地中心');
  ok(dp.indexOf('机械臂长度') >= 0 && dp.indexOf('杆长之和') >= 0, '面板里有「机械臂长度」（杆长之和）');
  ok(dp.indexOf('当前位形') < 0 && dp.indexOf('可达集（末端采样') < 0,
     '★ 面板里已经【没有】"当前位形"/"可达集（末端采样…）"了');
  const info = SIZEINFO();
  ok(Math.abs(info.C[0] - 140) < 1e-6 && Math.abs(info.C[1]) < 1e-6,
     '底盘中心点 = base (0, 140, 0) ⇒ 整机系 (140, 0, 0)', info.C);
  ok(Math.abs(info.GC[2] + 105) < 1e-6, '接地中心 z = −105（箱高 105）', info.GC);
  ok(Math.abs(info.SC[2] - 375) < 1e-6, '球心 z = 375（−105+480）', info.SC);
  ok(Math.abs(info.SR - 750) < 1e-6, '球半径 750', info.SR);
  ok(Math.abs(info.box.h[0] - 290) < 1e-6 && Math.abs(info.box.h[2] - 52.5) < 1e-6,
     '箱体半长 290 / 290 / 52.5', info.box.h);
  ok(Math.abs(info.armLen - 1064.09) < 0.01, '机械臂长度 = 1064.1 mm', info.armLen);
  ok(info.reach > 0 && info.reach < info.armLen, '最大伸展 ' + info.reach.toFixed(0) +
     ' mm（受限位约束，小于摊直总长）');

  /* ---- ② 三根线的显隐开关：数像素 ---- */
  SHOWBOX.chassis = true; SHOWBOX.lwh = true; SHOWBOX.ball = true; draw();
  const y1 = count(YEL,tY), t1 = count(TEA,tT), p1 = count(isPur), r1 = count(RED,tR);
  ok(y1 > 300, '底盘黄框画出来了（黄像素 '+y1+'）');
  ok(t1 > 300, '长宽高框画出来了（青像素 '+t1+'）');
  ok(p1 > 300, '限制球画出来了（紫像素 '+p1+'）');

  $('cbChassis').checked = false; $('cbChassis').onchange({target:{checked:false}}); draw();
  ok(count(YEL,tY) < y1*0.1, '关掉"底盘框"后黄像素几乎归零（'+y1+' → '+count(YEL,tY)+'）');
  $('cbChassis').checked = true;  $('cbChassis').onchange({target:{checked:true}});  draw();
  ok(count(YEL,tY) > y1*0.9, '再打开"底盘框"黄像素回来了');
  ok(SHOWBOX.chassis === true, 'SHOWBOX.chassis 同步为 true');

  $('cbLWH').checked = false; $('cbLWH').onchange({target:{checked:false}}); draw();
  ok(count(TEA,tT) < t1*0.1, '关掉"长宽高框"后青像素归零（'+t1+' → '+count(TEA,tT)+'）');
  $('cbLWH').checked = true;  $('cbLWH').onchange({target:{checked:true}});  draw();

  $('cbBall').checked = false; $('cbBall').onchange({target:{checked:false}}); draw();
  ok(count(isPur) < p1*0.1, '关掉"球"后紫像素归零（'+p1+' → '+count(isPur)+'）');
  $('cbBall').checked = true;  $('cbBall').onchange({target:{checked:true}});  draw();
  ok(count(isPur) > p1*0.9, '打开"球"后紫像素回来了（'+count(isPur)+'）');
  ok(count(RED,tR) === 0, '★ 画布上【一个红点都没有】（球外采样点已删掉）', count(RED,tR));
  ok($('legend').innerHTML.indexOf('球外采样点') < 0, '★ 图例里的"球外采样点"也删了');

  /* ---- ②b 一键按钮：同时隐藏/显示【两个框】（限制球不受影响） ---- */
  SHOWBOX.chassis = SHOWBOX.lwh = true; syncBoxChecks(); draw();
  const yOn = count(YEL,tY), tOn = count(TEA,tT), pOn = count(isPur);
  /* 改版：四个显示开关收进一个下拉菜单（#btnVis + #visMenu），
     原来那个"一键隐藏框"按钮已经删掉。这里验的是"菜单能开关、勾选状态跟着走"。 */
  ok(!!$('btnVis') && !!$('visMenu'), '顶栏有「👁 显示」下拉与它的菜单');
  ok($('visMenu').querySelectorAll('input[type=checkbox]').length === 4,
     '菜单里有 4 个勾选项（坐标架/底盘框/长宽高框/限制球）');
  ok($('cbChassis').checked === true && $('cbLWH').checked === true,
     '两个框默认是勾上的');
  SHOWBOX.chassis = SHOWBOX.lwh = false; syncVisMenu();
  ok($('cbChassis').checked === false && $('cbLWH').checked === false,
     '改 SHOWBOX 后 syncVisMenu() 会把勾选取消');
  SHOWBOX.chassis = SHOWBOX.lwh = true; syncVisMenu();
  ok($('cbChassis').checked === true && $('cbLWH').checked === true, '再同步一次 ⇒ 勾回来');
  $('cbLWH').checked = true; $('cbLWH').onchange({target:{checked:true}}); draw();

  /* ---- ③ 设置面板：改值 → 即时生效 ---- */
  openChassisForm(true);
  ok(!!$('cPx') && !!$('cSx') && !!$('cZdir'), '底盘/尺寸设置面板能打开并渲染出输入框');
  ok($('cPy').value === '140' && $('cSz').value === '105' &&
     $('cPx').value === '0' && $('cSx').value === '580',
     '表单默认值 = base(0,140,0) / 580×580×105',
     [$('cPx').value,$('cPy').value,$('cPz').value,$('cSx').value,$('cSz').value]);
  $('cSz').value = '200'; $('cform').onchange({target:$('cSz')});        // 事件委托：change 即应用
  const i2 = SIZEINFO();
  ok(Math.abs(i2.box.h[2] - 100) < 1e-6, '把箱高改成 200 后半长变 100', i2.box.h);
  ok(Math.abs(i2.GC[2] + 200) < 1e-6, '接地中心跟着到 −200（0−200）', i2.GC);
  ok(Math.abs(i2.SC[2] - 280) < 1e-6, '球心跟着到 280（−200+480）', i2.SC);
  ok($('dataBody').innerHTML.indexOf('机械臂长度') >= 0, '改完参数数据面板还在（没被清掉）');

  $('cReset').onclick();                                                 // 恢复默认
  const i3 = SIZEINFO();
  ok(Math.abs(i3.C[0] - 140) < 1e-6 && Math.abs(i3.box.h[2] - 52.5) < 1e-6,
     '恢复默认后回到 base y=140 / 半长 52.5', [i3.C, i3.box.h]);
  ok($('cSz') && $('cSz').value === '105', '恢复默认后输入框也回到 105', $('cSz') && $('cSz').value);

  /* ---- ④ 参数化成"整机原点"参照 ---- */
  const rb = document.querySelector('input[name=cPos][value=origin]');
  rb.checked = true; $('cPx').value = '0'; $('cPy').value = '0'; $('cPz').value = '0';
  $('cform').onchange({target:rb});
  ok(SIZEINFO().C.length === 3, '切"相对整机原点"后不报错');

  /* ---- ④ 右栏上下分隔条：拖动改「模型链 / 可达域」高度 ---- */
  const sp = $('split'), listEl = $('bdList'), panEl = $('bdData');
  ok(!!sp && !!listEl && !!panEl, '分隔条 #split 存在');
  const m1 = listEl.getBoundingClientRect().height, m2 = panEl.getBoundingClientRect().height;
  /* 改版后右栏是【四个】分区（模型链 / 旋转角度 / 装配参数 / 可达域），
     后两块高度固定且不参与分配，所以留给"模型链 ↔ 可达域"的总空间变小了。
     默认 130px 是当前布局下的合理下限（两栏时曾是 ~195px）。 */
  ok(m1 > 125, '默认比例下模型链有 ' + m1.toFixed(0) + 'px（可滚动，够看清几段）');
  const dragTo = dy => {
    sp.dispatchEvent(new MouseEvent('mousedown', {clientY:300, bubbles:true}));
    window.dispatchEvent(new MouseEvent('mousemove', {clientY:300+dy, bubbles:true}));
    const h = listEl.getBoundingClientRect().height;
    window.dispatchEvent(new MouseEvent('mouseup', {bubbles:true}));
    return h;
  };
  const h1 = dragTo(120);
  /* 四分区下被分配的总空间小，同样的拖动距离换来的像素增长也小；
     不再要求严格 1:1，只要求：方向对、确实动了、量级合理、有上下界。 */
  ok(h1 - m1 > 20 && (h1 - m1) <= 120 + 15,
     '往下拖 120px ⇒ 模型链长高 ' + (h1-m1).toFixed(0) + 'px（方向与量级合理）');
  const h2 = dragTo(-60);
  ok(h2 < h1, '往上拖 60px ⇒ 缩回 ' + (h1-h2).toFixed(0) + 'px');
  const h3 = dragTo(9999);
  ok(h3 < m1 + m2, '往下猛拖被夹住（' + h3.toFixed(0) + ' < 两栏之和 ' + (m1+m2).toFixed(0) + '）');
  ok(panEl.getBoundingClientRect().height > 100, '被夹住时可达域仍有 ' +
     panEl.getBoundingClientRect().height.toFixed(0) + 'px，没被拖没');
  sp.dispatchEvent(new MouseEvent('dblclick', {bubbles:true}));
  const h4 = listEl.getBoundingClientRect().height;
  ok(Math.abs(h4 - m1) < 8, '双击复原回默认比例（' + m1.toFixed(0) + ' → ' + h4.toFixed(0) + '）');
  ok(!!SIZEINFO(), '拖动分隔条不影响三维/面板数据');

  /* ---- ⑥ 模型链改完 → 旋转角度面板必须跟着重画 ----
     这一节是补的历史欠账：模型链的编辑/删除确实调了 updateAll()，
     但 updateAll() 只更新【引擎】（CHAIN / NJ / qLimits），
     旋转角度面板是渲染那一刻的 DOM 快照，没人重画它。
     症状（用户报的）：改限位后滑条范围还是老的、加一段面板里不出现这条关节、
     删一段面板里还留着。t10/t12/t13/t14 全绿也照样漏 —— 因为它们都不碰这条链。 */
  {
    const sliders = () => [...document.querySelectorAll('#qbody input[type=range]')];
    const nSl = () => sliders().length;
    ok(nSl() === NJ, '⑥0 开口：滑条条数 = 可动关节数（' + NJ + '）', [nSl(), NJ]);

    /* ⑥1 改限位：joint1 上限 122 → 99 */
    const eds = [...document.querySelectorAll('#list button.ed')];
    eds[1].click();
    $('fHi').value = '99'; $('fOK').click();
    ok(Math.abs(qLimits()[1][1] - 99) < 1e-9, '⑥1 引擎里 joint1 上限 = 99', qLimits()[1]);
    ok(sliders()[1] && sliders()[1].max === '99',
       '⑥2★ 旋转角度滑条范围跟着改成 99（原来还是 122）',
       sliders()[1] ? [sliders()[1].min, sliders()[1].max] : 'no slider');
    ok($('qnum1') && $('qnum1').max === '99', '⑥3 数字框的上限也跟着改',
       $('qnum1') && $('qnum1').max);

    /* ⑥4 加一段（带限位 ±30） */
    $('btnAdd').click();
    $('fName').value = 'linkX'; $('fJoint').value = 'jointX';
    $('fLo').value = '-30'; $('fHi').value = '30';
    $('fOK').click();
    ok(NJ === 7 && nSl() === 7, '⑥4★ 加一段后面板多出一条滑条（7 条）', [NJ, nSl()]);
    ok($('qbody').innerText.indexOf('jointX') >= 0, '⑥5 新关节出现在面板里');
    ok(sliders()[6] && sliders()[6].max === '30', '⑥6 新滑条范围 = 新限位（0~30）',
       sliders()[6] ? [sliders()[6].min, sliders()[6].max] : 'no slider');

    /* ⑥7 删掉它 */
    const oldConfirm = window.confirm; window.confirm = () => true;
    const dels = [...document.querySelectorAll('#list button.del')];
    dels[dels.length - 1].click();
    window.confirm = oldConfirm;
    ok(NJ === 6 && nSl() === 6, '⑥7★ 删一段后面板少一条（回到 6 条）', [NJ, nSl()]);

    /* ⑥8 引擎值与面板值必须是同一个数。
       原来滑条 step = (hi−lo)/200（joint1 = 0.61°，是个"歪"网格）：
       onChangeJoint 吸附一次、浏览器渲染时又吸附一次，两边差一格
       ⇒ 「引擎里 77、滑条上 76.9」。现在两处共用同一个吸附函数。 */
    onChangeJoint(1, 77);
    const s1 = sliders()[1];
    ok(Math.abs(Q[1] - (+s1.value)) < 1e-9,
       '⑥8★ 引擎里的角度与滑条上的值一致（都是 ' + Q[1] + '）', [Q[1], +s1.value]);
    ok(Math.abs(+$('qnum1').value - Q[1]) < 1e-9, '⑥9 数字框也是同一个数',
       [+$('qnum1').value, Q[1]]);
    ok(Math.abs(+s1.step - 1) < 1e-9, '⑥10 滑条步长 = 1°（不再用 (hi−lo)/200 这种歪网格）', +s1.step);
    onChangeJoint(1, 0);
  }

  /* ---- ⑦ 存档：按构型分开存 + 模型/关节角/当前配置都要跟着走 ----
     原来存档只写"模型 + 规则侧设置"，而且是【一份、不带构型】：
       · 切构型后 configId 不更新 ⇒ 重载时配置回到默认、模型却是换过来那份，两边对不上
       · 关节角根本不存 ⇒ 拖完刷新就归零
       · 只有一份模型 ⇒ 在两套构型之间来回切，切走那一下就把另一套的改动冲掉
     现在：模型与设置都按 configId 分键，视图偏好（分隔条/显示开关）全局一份。 */
  {
    const lsOK = (() => { try { localStorage.setItem('__t', '1'); localStorage.removeItem('__t'); return true; }
                          catch (e) { return false; } })();
    if (!lsOK) {
      ok(true, '⑦ 浏览器禁用了 localStorage，跳过存档断言（界面上会有橙色提示）');
    } else {
      const startCfg = activeConfigId();
      onChangeJoint(2, 33);
      saveLocal();                                   // 直接落盘，不等 rAF
      const c = JSON.parse(localStorage.getItem(lsCfgKey()) || 'null');
      const m = JSON.parse(localStorage.getItem(lsModelKey()) || 'null');
      ok(lsCfgKey() === 'armBuilderCfg:' + startCfg, '⑦1★ 设置按键分开存：' + lsCfgKey(), lsCfgKey());
      ok(lsModelKey() === 'armBuilderModel:' + startCfg, '⑦2★ 模型按键分开存：' + lsModelKey(), lsModelKey());
      ok(!!c && Array.isArray(c.q) && Math.abs(c.q[2] - 33) < 1e-9,
         '⑦3★ 关节角进了存档（原来只存模型，角度刷新就没了）', c && c.q);
      ok(!!c && c.configId === startCfg, '⑦4 存档里的 configId = 当前配置', [c && c.configId, startCfg]);
      ok(!!m && canonModel(m) === canonModel(MODEL),
         '⑦5 存档里的模型 = 当前模型（口径与读取端一致）');
      ok(!!c && !!c.modelCfg, '⑦6 存档带着"配置里原始那条臂"的快照（旧版存档迁移时认领用）');
      ok(!!JSON.parse(localStorage.getItem('armBuilderView') || 'null'),
         '⑦7★ 视图偏好单独一份（与构型无关，切构型不该重置）');
      ok((JSON.parse(localStorage.getItem('armBuilderView') || '{}').configId) === startCfg,
         '⑦8★ "上次用的是哪条构型"记在全局那份里（不能靠扫键取第一个：那会取到最早那条）');

      /* ⑦9 换构型：切走再切回，两边改动都要在 —— 这是"每条构型各存一份"的全部意义。
         这一节必须走真的下拉框 onchange（那里有 holdSave 闸门），
         不然中间态的 saveLocal 会把新构型的存档写坏（踩过：切回来角度全零）。 */
      const sel = $('selConfig');
      if (sel && CONFIGS['demo3dof'] && startCfg !== 'demo3dof') {
        const oldConfirm = window.confirm; window.confirm = () => true;
        sel.value = 'demo3dof'; sel.onchange();
        ok(activeConfigId() === 'demo3dof', '⑦9 切到 demo3dof', activeConfigId());
        ok(!!localStorage.getItem('armBuilderModel:demo3dof'), '⑦10 新构型也落了一份存档');
        const back = JSON.parse(localStorage.getItem(lsCfgKey(startCfg)) || 'null');
        ok(!!back && Array.isArray(back.q) && Math.abs(back.q[2] - 33) < 1e-9,
           '⑦11★ 切走之后原构型的关节角存档没被动（' + JSON.stringify(back && back.q) + '）');
        sel.value = startCfg; sel.onchange();
        window.confirm = oldConfirm;
        ok(activeConfigId() === startCfg, '⑦12 切回来', activeConfigId());
        ok(Math.abs(Q[2] - 33) < 1e-9, '⑦13★ 切回来关节角还是 33（各存各的）', Q[2]);
        ok(MODEL.name === ((CONFIGS[startCfg].model || {}).name || MODEL.name),
           '⑦14 切回来模型也对', [MODEL.name, (CONFIGS[startCfg].model || {}).name]);
      }

      /* ⑦15 反向：结构性不符必须判否（配置源码改了、存档还是老的 ⇒ 别硬套） */
      const bad = JSON.parse(JSON.stringify(MODEL));
      bad.links.pop();
      ok(!modelFitsConfig(bad), '⑦15 反向：段数对不上的存档不能套到当前构型上');
      ok(modelFitsConfig(MODEL), '⑦16 当前模型本身是"对得上"的（判定不是恒假）');

      onChangeJoint(2, 0); saveLocal();
    }
  }

  /* ---- ⑧ 顶栏的「已保存」提示 ----
     用户问过"到底有没有实时保存" —— 看不见的机制等于不存在。 */
  {
    const chip = $('savedChip');
    ok(!!chip, '⑧1★ 顶栏有「已保存」提示', chip && chip.textContent);
    /* ★ 反斜杠要写两次：这段代码是塞进 probe 模板字符串里的，
       单写 \s \d 会被模板字符串吃成 s d（踩过：断言恒假，看着像功能没做）。 */
    ok(!!chip && /已保存\\s*\\d\\d:\\d\\d/.test(chip.textContent), '⑧2 提示里带时间',
       chip && chip.textContent);
    if (chip) {
      saveLocal();
      ok(chip.classList.contains('flash'), '⑧3★ 落盘时闪一下（改动能被看见）');
      ok((chip.title || '').indexOf('JSON') >= 0, '⑧4 悬停说明里讲清了存档范围', chip.title);
    }
  }

  /* ---- ⑨ 收尾：把结果铺到页面上（交给 --dump-dom） ---- */
  document.body.innerHTML = '<pre id="PROBE">' + R.join('\\n') +
    '\\n---END---\\nFAIL=' + R.filter(s=>s[0]==='✗').length + ' PASS=' + R.filter(s=>s[0]==='✓').length +
    '</pre>';
});
<\/script>
`;
const html = fs.readFileSync(src, 'utf8').replace('</body>', (SHOT ? shotProbe : probe) + '</body>');
fs.writeFileSync(out, html, 'utf8');
console.log('已生成 ' + out + '（' + html.length + ' 字节）');

if(SHOT){
  const edge = 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe';
  const url = 'file:///' + out.replace(/\\/g,'/').split('/').map((s,i)=> i===0 ? s :
     encodeURIComponent(s)).join('/') + '#demosave';
  const tmp = path.join(process.env.TEMP, 'dsh-edge-shot' + Date.now());
  cp.spawnSync(edge, ['--headless=new','--disable-gpu','--hide-scrollbars',
    '--virtual-time-budget=9000','--window-size=1700,1000','--user-data-dir='+tmp,
    '--screenshot=' + path.join(dir, '..', '..', 'docs', 'screenshots', 't10_both.png'), url], { encoding:'utf8' });
  console.log('已截图 t10_both.png');
  process.exit(0);
}

if(process.argv.includes('--run')){
  const edge = 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe';
  const url = 'file:///' + out.replace(/\\/g,'/').split('/').map((s,i)=> i===0 ? s :
     encodeURIComponent(s)).join('/') + '#demosave';
  const tmp = path.join(process.env.TEMP, 'dsh-edge-probe' + Date.now());
  const r = cp.spawnSync(edge, ['--headless=new','--disable-gpu','--hide-scrollbars',
    '--virtual-time-budget=20000','--window-size=1700,1000','--user-data-dir='+tmp,
    '--dump-dom', url], { encoding:'utf8', maxBuffer: 64*1024*1024 });
  const m = /<pre id="PROBE">([\s\S]*?)<\/pre>/.exec(r.stdout || '');
  if(!m){ console.log('没抓到 PROBE 输出；stderr:\n' + (r.stderr||'').slice(0,2000)); process.exit(2); }
  console.log(m[1].replace(/&amp;/g,'&').replace(/&lt;/g,'<').replace(/&gt;/g,'>'));
  process.exit(/FAIL=0/.test(m[1]) ? 0 : 1);
}
