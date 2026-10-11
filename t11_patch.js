/*  t11_patch.js —— 由 armBuilder.html + 连杆模块 生成 armLink.html（可重复跑，幂等）
    跑法： node t11_patch.js
    源：   t11_link_css.css（追加到 <style> 末尾）
           t11_link_mod.js（追加成第 3 个 <script>）
    这样 armBuilder.html 保持一个字节都没动，两份文件的差异就是本脚本改动的那 5 处。 */
const fs = require('fs'), path = require('path'), crypto = require('crypto');
const dir = __dirname;
const SRC = path.join(dir, 'armBuilder.html');
const OUT = path.join(dir, 'armLink.html');
const CSS = path.join(dir, 't11_link_css.css');
const MOD = path.join(dir, 't11_link_mod.js');

const css = fs.readFileSync(CSS, 'utf8');
const mod = fs.readFileSync(MOD, 'utf8');
const md5 = s => crypto.createHash('md5').update(s, 'utf8').digest('hex').slice(0, 12);

let html = fs.readFileSync(SRC, 'utf8');
const srcMD5 = md5(html);
const before = html.length;

function sub(what, from, to){
  const n = html.split(from).length - 1;
  if(n !== 1) throw new Error(`锚点「${what}」命中 ${n} 次（应为 1）：${JSON.stringify(from.slice(0,60))}`);
  html = html.replace(from, to);
  console.log(`  ✓ ${what}`);
}

console.log('armLink.html ← armBuilder.html(' + srcMD5 + ') + 连杆模块');
sub('页面标题', '<title>机械臂可行性集 · 建模器</title>',
                '<title>机械臂连杆 · 任务适应性评估器</title>');
sub('CSS', '</style>', css + '</style>');
sub('顶栏按钮', '<button id="btnChassis">▦ 底盘 / 尺寸</button>',
    '<button id="btnLink" title="只改连杆 offset，不改关节限位">⛓ 连杆调参</button>\n' +
    '      <button id="btnChassis">▦ 底盘 / 尺寸</button>');
sub('面板容器', '<div id="form"></div>', '<div id="form"></div>\n      <div id="lform"></div>');
sub('模块脚本', '</script>\n</body>',
    '</script>\n' +
    '<!-- ===== 以下为 armLink 追加的连杆调参模块（源：t11_link_mod.js · md5 ' + md5(mod) + '） ===== -->\n' +
    '<script>\n' + mod + '\n</script>\n</body>');

fs.writeFileSync(OUT, html, 'utf8');
const h = md5(html);
console.log('  → ' + OUT + '  ' + html.length + ' 字节（源 ' + before + '，+ ' +
            (html.length - before) + '）  md5=' + h);
/* 模块源码 md5 写进文件头注释，便于自检发现"改源没重生成" */
fs.writeFileSync(OUT, html.replace('<html lang="zh-CN">',
  '<html lang="zh-CN">\n<!-- armLink：由 t11_patch.js 生成 · armBuilder(' + srcMD5 + ') + 连杆模块(' + md5(mod) +
  ') · css(' + md5(css) + ') -->'), 'utf8');
console.log('  ✓ 版本注释已写入（armBuilder ' + srcMD5 + ' / 模块 ' + md5(mod) + ' / css ' + md5(css) + '）');
