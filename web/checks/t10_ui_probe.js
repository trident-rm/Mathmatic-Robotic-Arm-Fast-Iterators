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

  /* ---- ⑤ 收尾：把结果铺到页面上（交给 --dump-dom） ---- */
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
