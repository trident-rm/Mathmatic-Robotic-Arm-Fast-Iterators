/*  t11_ui_probe.js —— 生成一个带自检脚本的 armLink 副本，用 Edge 无头 --dump-dom 读结果
    跑法： node t11_ui_probe.js             （只生成 t11_ui_probe.html）
           node t11_ui_probe.js --run       （生成并直接跑 Edge，打印断言）
           node t11_ui_probe.js --shot      （只截图：面板打开 + 结果表 + 对比小图）

    为什么不用 jsdom：这里要验的是【真的点得动、真的画出来了】——
      面板开合、扫描进度、结果表、点行应用、canvas 像素、恢复原始臂。 */
const fs = require('fs'), path = require('path'), cp = require('child_process');
const dir = __dirname, src = path.join(dir, 'armLink.html');
const SHOT = process.argv.includes('--shot');
const out = path.join(dir, SHOT ? 't11_panel_shot.html' : 't11_ui_probe.html');
const EDGE = 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe';

/* --shot 模式：把面板打开、跑一个 5 档扫描、选中一行，供截图 */
const shotProbe = `
<script>
window.addEventListener('DOMContentLoaded', async () => {
  const sleep = ms => new Promise(r=>setTimeout(r,ms));
  for(let i=0;i<40 && !MODEL;i++) await sleep(50);   // ★ let 声明不会挂到 window 上，只能用裸标识符
  LINK.init(true);
  Object.assign(LINK.cfg, {nPose:12, budget:4, iters:220, K:5});
  LINK.open();
  await sleep(50);
  document.getElementById('lSweep').onclick();
  while(LINK.busy()) await sleep(100);
  document.querySelectorAll('#lres tr.rowc')[0].onclick();
  await sleep(200);
  document.getElementById('lform').style.maxHeight = 'none';
});
<\/script>`;

const probe = `
<script>
window.addEventListener('DOMContentLoaded', async () => {      // 注册在最后 ⇒ 跑在 setModel 之后
  const R = [], ok = (c,m,x) => R.push((c?'✓ ':'✗ ')+m+(c?'':'   ← '+JSON.stringify(x)));
  const $ = id => document.getElementById(id);
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const txt = id => { const e = $(id); return e ? (e.textContent||'') + '|' + (e.innerHTML||'') : ''; };
  const cvPix = id => { const c = $(id); const g = c.getContext('2d');
    const im = g.getImageData(0,0,c.width,c.height).data; let n = 0;
    for(let i=3;i<im.length;i+=4) if(im[i] > 40) n++; return n; };
  try {
  for(let i=0;i<40 && !MODEL;i++) await sleep(50);   // ★ let 声明不会挂到 window 上，只能用裸标识符
  ok(!!MODEL && MODEL.links.length === 7, 'DOMContentLoaded 后示例模型已在（7 段）', MODEL && MODEL.links.length);

  /* ---- ① 原功能还在（顶栏按钮 / 数据面板 / 3D） ---- */
  const ids = ['btnChassis','btnFrames','btnSave','btnLoad','btnDemo','cbBall','c3d','dataBody','list','split'];
  ok(ids.every(i=>!!$(i)), '原界面控件与画布都还在', ids.filter(i=>!$(i)));
  ok($('legend').innerHTML.indexOf('底盘碰撞箱') >= 0, '图例没被破坏');
  ok(cvPix('c3d') > 5000, '三维视图照常画出来了（非透明像素 ' + cvPix('c3d') + '）');
  ok($('dataBody').innerHTML.indexOf('底盘碰撞箱 / 伸展尺寸') >= 0, '原数据面板段还在');
  ok($('dataBody').innerHTML.indexOf('连杆（当前 / 基准）') >= 0, '★ 数据面板里追加了「连杆（当前 / 基准）」段');

  /* ---- ② 按钮 + 面板开合 ---- */
  ok(!!$('btnLink'), '顶栏多了「⛓ 连杆调参」按钮');
  ok(!$('lform').classList.contains('on'), '一开始 #lform 是收起的');
  $('btnLink').onclick();
  await sleep(30);
  ok($('lform').classList.contains('on'), '点按钮后面板打开');
  const L0 = LINK.limSig(), LIM = JSON.stringify(MODEL.links.map(l=>l.limits));
  ok($('lform').innerHTML.indexOf('连杆调参') >= 0, '面板标题出现');
  ok($('lform').innerHTML.indexOf('不动限位') >= 0, '面板写明「只改杆，不动限位」');
  ok($('lform').innerHTML.indexOf('base_joint[不限]') >= 0, '面板把只读限位签名显示出来了');
  ok(document.querySelectorAll('#lform .kOn').length === 18, '旋钮表 18 行（3 分量 × 6 段）',
     document.querySelectorAll('#lform .kOn').length);
  ok(document.querySelectorAll('#lform .kOn:checked').length === 6, '默认勾 6 个非零分量',
     document.querySelectorAll('#lform .kOn:checked').length);
  ok($('lcmp') && cvPix('lcmp') > 200, '对比小图（YZ 投影）画出来了（像素 ' + ($('lcmp')?cvPix('lcmp'):'-') + '）');

  /* ---- ③ 小预算：评估基准 ---- */
  Object.assign(LINK.cfg, {nPose:6, budget:2, iters:140, K:5, restarts:1});
  $('lEvalBase').onclick();
  while(LINK.busy()) await sleep(50);
  await sleep(30);
  ok(document.querySelectorAll('#lres tr.rowc').length >= 1, '「评估基准」后结果表出现行',
     document.querySelectorAll('#lres tr.rowc').length);
  ok($('lres').innerHTML.indexOf('基准') >= 0, '结果表里有「基准」这一行');
  ok(LINK.limSig() === L0, '评估基准后限位签名不变');
  ok(JSON.stringify(MODEL.links.map(l=>l.limits)) === LIM, '评估基准后 links[*].limits 逐个不变');
  const nRow1 = document.querySelectorAll('#lres tr.rowc').length;

  /* ---- ④ 灵敏度扫描 ---- */
  $('lSweep').onclick();
  while(LINK.busy()) await sleep(50);
  await sleep(30);
  ok($('lsw').innerHTML.indexOf('灵敏度扫描') >= 0, '扫描完出现了「灵敏度扫描」曲线区');
  ok(document.querySelectorAll('#lsw tr').length >= 25, '曲线区里每根杆若干档都在（' +
     document.querySelectorAll('#lsw tr').length + ' 行）');
  ok(document.querySelectorAll('#lsw tr td i').length >= 25, '★ 每档都画了分数条（i 元素）');
  ok($('lnote').textContent.indexOf('完成') >= 0, '底部小结说本轮完成：' + $('lnote').textContent);
  ok(LINK.limSig() === L0, '扫描完限位签名仍不变');

  /* ---- ⑤ 搜索（极小预算） ---- */
  $('lSearch').onclick();
  while(LINK.busy()) await sleep(50);
  await sleep(30);
  ok(document.querySelectorAll('#lres tr.rowc').length > nRow1, '搜索后结果表又多了行（搜索最优）',
     document.querySelectorAll('#lres tr.rowc').length - nRow1);
  ok(LINK.limSig() === L0, '搜索完限位签名仍不变');

  /* ---- ⑥ 点结果行 = 应用；再恢复原始臂 ---- */
  const snap = JSON.stringify(MODEL);
  ok(Array.isArray(LINK.demoDiff()) && LINK.demoDiff().length === 0,
     '★ 原版模型：LINK.demoDiff() 是空数组', LINK.demoDiff());
  ok($('lform').innerHTML.indexOf('当前模型 = 同济示例模型') >= 0,
     '面板顶部写着「✓ 当前模型 = 同济示例模型（原版，一字未改）」');
  ok($('dataBody').innerHTML.indexOf('当前模型 = 同济示例模型') >= 0,
     '数据面板的连杆段也有这一行（就在模型链下面，不容易看错）');
  const bi = LINK.bestRow();
  const row = document.querySelectorAll('#lres tr.rowc')[bi];
  row.onclick();
  await sleep(50);
  const vals = LINK.readModel();
  const r = LINK.res()[bi];
  ok(r.vals.every((v,i)=>Math.abs(v-vals[i])<1e-9), '点行 = 把该方案写进模型（offset 逐项相同）');
  const changed = LINK.knobs().filter((k,i)=>Math.abs(r.vals[i]-k.v0)>1e-6).map(k=>k.name);
  ok(changed.length > 0 && LINK.demoDiff().join(' ').indexOf(changed[0]) >= 0,
     '★ 应用后 demoDiff 立刻标出改过的杆（本例：' + changed.join(', ') + '）', LINK.demoDiff());
  ok($('lform').innerHTML.indexOf('≠ 同济示例模型') >= 0, '面板顶部改成「⚠ 当前模型 ≠ 同济示例模型」');
  ok($('lform').innerHTML.indexOf('点一行 = 应用 + 存进浏览器本地存档') >= 0,
     '面板写明"点一行会存进浏览器本地存档"（防止刷新后以为还是原臂）');
  ok(LINK.limSig() === L0 && JSON.stringify(MODEL.links.map(l=>l.limits)) === LIM,
     '★ 应用候选后限位一个字没改');
  ok(LINK.armLen() < 1064.09 + 1e-6, '应用的候选确实把臂改短了（' + LINK.armLen().toFixed(0) + ' mm）');
  ok($('lsegs').innerHTML.indexOf('杆长之和') >= 0, '对比栏更新出了当前/基准杆长');

  $('lRevert').onclick();
  await sleep(80);
  ok(JSON.stringify(MODEL) === snap, '★ 「恢复原始臂」后模型逐字节回到原样（含限位）');
  ok(LINK.limSig() === L0, '恢复后限位签名仍不变');
  ok($('lform').classList.contains('on'), '恢复原始臂不会把面板关掉');

  /* ---- ⑦ 收尾：检查原面板的其它钩子没被搞坏 ---- */
  ok(typeof SIZEINFO === 'function' && !!SIZEINFO(), 'SIZEINFO() 还能用（底盘自检钩子）');
  ok(typeof analyzeTask === 'function', 'analyzeTask 仍是函数（我们只是包了一层）');
  analyzeTask();
  ok($('dataBody').innerHTML.indexOf('连杆（当前 / 基准）') >= 0, '再次 analyzeTask 后我们的段还在（不会丢）');

  /* ---- ⑧ 旋钮表的控件：勾选 / 范围 / 一键按钮 / 单根扫描 / 手改一格 ---- */
  const cbs = () => document.querySelectorAll('#lform .kOn:checked').length;
  $('lOnNone').onclick(); await sleep(20);
  ok(cbs() === 0, '「全不勾」把 18 个旋钮全取消');
  $('lOnAll').onclick(); await sleep(20);
  ok(cbs() === 18, '「全部」勾上 18 个');
  $('lOnLen').onclick(); await sleep(20);
  ok(cbs() === 5, '「只勾杆长方向」= 每段绝对值最大的那个分量 → 5 个（link5 的 offset 是 0，不进；link4 只留 z）', cbs());
  $('lOnNonzero').onclick(); await sleep(20);
  ok(cbs() === 6, '「只勾非零分量」也回到 6 个');
  const lo0 = document.querySelector('#lform .kLo').value, hi0 = document.querySelector('#lform .kHi').value;
  $('lSetRange').onclick(); await sleep(20);
  const lo1 = document.querySelector('#lform .kLo').value, hi1 = document.querySelector('#lform .kHi').value;
  ok(lo1 !== lo0 || hi1 !== hi0, '「按比例设范围」真的改了上下界（' + lo0 + '~' + hi0 + ' → ' + lo1 + '~' + hi1 + '）');
  const nBefore = document.querySelectorAll('#lres tr.rowc').length;
  const hadSw = document.querySelectorAll('#lsw table').length;
  document.querySelector('#lform .kScan').onclick();
  while(LINK.busy()) await sleep(50);
  await sleep(30);
  ok(document.querySelectorAll('#lres tr.rowc').length > nBefore, '单根 ⟲ 扫描把 5 档都进了结果表');
  ok(document.querySelectorAll('#lsw table').length === hadSw, '单根扫描不覆盖原来的灵敏度曲线区');
  ok(LINK.limSig() === L0, '单根扫描后限位仍不变');
  const pay = LINK.export();
  ok(pay && pay.results.length === LINK.res().length && pay.knobs.length === 18,
     '导出 payload 里结果条数与旋钮数都对', [pay&&pay.results.length, pay&&pay.knobs.length]);
  const v0 = LINK.readModel()[2];
  const inp = document.querySelectorAll('#lform .kCur')[2];
  inp.value = String(v0 + 12); inp.onchange();
  await sleep(30);
  ok(Math.abs(LINK.readModel()[2] - (v0 + 12)) < 1e-9, '改「当前」输入框 = 直接改模型该段 offset');
  ok(LINK.limSig() === L0 && JSON.stringify(MODEL.links.map(l=>l.limits)) === LIM, '手改 offset 也不动限位');
  $('lRevert').onclick(); await sleep(60);
  ok(JSON.stringify(MODEL) === snap, '再恢复一次仍逐字节回到快照');

  } catch(e){ R.push('✗ 探针自己抛异常：' + (e && e.stack || e)); }

  document.body.innerHTML = '<pre id="PROBE">' + R.join('\\n') +
    '\\n---END---\\nFAIL=' + R.filter(s=>s[0]==='✗').length + ' PASS=' + R.filter(s=>s[0]==='✓').length +
    '</pre>';
});
<\/script>`;

const html = fs.readFileSync(src, 'utf8').replace('</body>', (SHOT ? shotProbe : probe) + '</body>');
fs.writeFileSync(out, html, 'utf8');
console.log('已生成 ' + out + '（' + html.length + ' 字节）');

const url = 'file:///' + out.replace(/\\/g,'/').split('/').map((s,i)=> i===0 ? s :
   encodeURIComponent(s)).join('/') + '#demosave';
const tmp = () => path.join(process.env.TEMP, 'dsh-t11-edge-' + Date.now());

if(SHOT){
  cp.spawnSync(EDGE, ['--headless=new','--disable-gpu','--hide-scrollbars',
    '--virtual-time-budget=90000','--window-size=1700,1150','--user-data-dir='+tmp(),
    '--screenshot=' + path.join(dir,'t11_panel.png'), url], { encoding:'utf8' });
  console.log('已截图 t11_panel.png');
  process.exit(0);
}
if(process.argv.includes('--run')){
  const r = cp.spawnSync(EDGE, ['--headless=new','--disable-gpu','--hide-scrollbars',
    '--virtual-time-budget=120000','--window-size=1700,1150','--user-data-dir='+tmp(),
    '--dump-dom', url], { encoding:'utf8', maxBuffer: 128*1024*1024 });
  const m = /<pre id="PROBE">([\s\S]*?)<\/pre>/.exec(r.stdout || '');
  if(!m){ console.log('没抓到 PROBE；stderr:\n' + (r.stderr||'').slice(0,3000)); process.exit(2); }
  console.log(m[1].replace(/&amp;/g,'&').replace(/&lt;/g,'<').replace(/&gt;/g,'>').replace(/&quot;/g,'"'));
  process.exit(/FAIL=0/.test(m[1]) ? 0 : 1);
}
