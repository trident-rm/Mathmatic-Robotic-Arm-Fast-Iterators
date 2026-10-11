#!/usr/bin/env node
/*  t11_patch.js —— armBuilder(+连杆模块) → armLink.html    【纯合成，只做 5 处追加】

    两种跑法：
        1) 正常情况不用单独跑它： node web/build.js
           （build.js 会先合成 armBuilder，再调本模块的 buildLink()）
        2) 只想重建 armLink（要求 build/armBuilder.html 已是最新）：
               node web/checks/t11_patch.js

    追加的 5 处：页面标题 / CSS / 顶栏按钮 / 面板容器 / 第 4 个 <script>
    原 3 个 <script> 与整段 <style> 一个字节都不动 —— 自检 t11_link_tune.js 会逐字节比对。
*/
'use strict';
const fs = require('fs'), path = require('path'), crypto = require('crypto');
const P = require('../lib/paths');

/* 读文本统一成 LF —— 交付物是逐字节合成 + 断言 md5 的，行尾必须与平台无关 */
const read = p => fs.readFileSync(p, 'utf8').replace(/\r\n/g, '\n');
const md5  = s => crypto.createHash('md5')
                        .update(String(s).replace(/\r\n/g, '\n'), 'utf8')
                        .digest('hex').slice(0, 12);

/* 由 armBuilder 的完整文本生成 armLink 的完整文本（纯函数，不碰磁盘） */
function buildLink(armBuilderHtml, opts) {
  opts = opts || {};
  const css = opts.css !== undefined ? opts.css : read(P.LINK_CSS);
  const mod = opts.mod !== undefined ? opts.mod : read(P.LINK_MOD);

  let html = String(armBuilderHtml).replace(/\r\n/g, '\n');   // 输出保证 LF
  const srcMD5 = md5(html);

  const sub = (what, from, to) => {
    const n = html.split(from).length - 1;
    if (n !== 1) {
      throw new Error('锚点「' + what + '」命中 ' + n + ' 次（应为 1）：' +
                      JSON.stringify(from.slice(0, 60)));
    }
    html = html.replace(from, to);
  };

  sub('页面标题', '<title>机械臂可行性集 · 建模器</title>',
                  '<title>机械臂连杆 · 任务适应性评估器</title>');
  sub('CSS', '</style>', css + '</style>');
  sub('顶栏按钮', '<button id="btnChassis">▦ 底盘 / 尺寸</button>',
      '<button id="btnLink" title="只改连杆 offset，不改关节限位">⛓ 连杆调参</button>\n' +
      '      <button id="btnChassis">▦ 底盘 / 尺寸</button>');
  sub('面板容器', '<div id="form"></div>', '<div id="form"></div>\n      <div id="lform"></div>');
  sub('模块脚本', '</script>\n</body>',
      '</script>\n' +
      '<!-- ===== 以下为 armLink 追加的连杆调参模块（源：t11_link_mod.js · md5 ' +
      md5(mod) + '） ===== -->\n' +
      '<script>\n' + mod + '\n</script>\n</body>');

  /* 版本注释里带上三个源文件的 md5，自检据此发现"改了源没重新构建" */
  html = html.replace('<html lang="zh-CN">',
    '<html lang="zh-CN">\n<!-- armLink：由 t11_patch.js 生成 · armBuilder(' + srcMD5 +
    ') + 连杆模块(' + md5(mod) + ') · css(' + md5(css) + ') -->');

  return { html, srcMD5, modMD5: md5(mod), cssMD5: md5(css) };
}

module.exports = { buildLink };

/* ---------------- CLI ---------------- */
if (require.main === module) {
  if (!fs.existsSync(P.ARM_BUILDER)) {
    console.error('找不到 ' + P.ARM_BUILDER);
    console.error('先跑  node web/build.js');
    process.exit(1);
  }
  const src = read(P.ARM_BUILDER);
  const r = buildLink(src);
  fs.writeFileSync(P.ARM_LINK, r.html, 'utf8');
  console.log('armLink.html ← armBuilder(' + r.srcMD5 + ') + 连杆模块(' + r.modMD5 +
              ') + css(' + r.cssMD5 + ')');
  console.log('  → ' + P.ARM_LINK + '  ' + r.html.length + ' 字节');
}
