#!/usr/bin/env node
/*  web/build.js —— 构建交付物到 build/

    用法：  node web/build.js            （在仓库根目录跑）

    产物（单文件、零依赖、双击就能开）：
        build/armBuilder.html    建模器

    日志会打印每个模块的 md5，改了哪个模块一眼能看出来。
    构建是幂等的：同样的源码一定得到同样的字节。
*/
'use strict';
const fs = require('fs'), path = require('path');
const A = require('./lib/assemble');

/* armBuilder.html 的基准 md5。
   ── 2026-10 配置层改造：从 5077ab35c6fd 变成 5dff80d537f1。
      这一变是【有意】的：示例臂的杆长、比赛规则、车体尺寸从引擎里搬到了
      web/src/configs/tongji.js，交付物里多了一段配置 + 一个启动块。
      搬的时候逐值对照过，同济示例的数值结果没有变（t12 会核对）。
   ★ 以后凡是"不该变却变了"的情况，这里会拦住 —— 改这个常量前先想清楚。 */
const EXPECTED_ARM_BUILDER_MD5 = '52651eb1f40aba01d505694b98072cf9';

const BUILD = path.join(A.ROOT, 'build');

function log(...a) { console.log(...a); }
function line() { log('-'.repeat(64)); }

line();
log('web/build.js —— 合成单文件交付物');
line();

/* ---- 1. 检查骨架与模块 ---- */
const tpl = A.readTemplate();
log('[1] 骨架模板');
log('    src/apps/armBuilder.html   ' + tpl.length + ' 字节');
for (const name of A.MODULES) {
  const p = path.join(A.SRC, 'modules', name + '.js');
  const t = A.read(p);
  log('    modules/' + (name + '.js').padEnd(20) + A.md5(t).slice(0, 12) +
      '  ' + String(t.split('\n').length).padStart(4) + ' 行');
}

/* ---- 2. 合成 armBuilder ---- */
log('\n[2] 合成 build/armBuilder.html');
const armBuilder = A.buildArmBuilder();
const gotMd5 = A.md5(armBuilder);

/* 先落盘，再做基准比对 —— 这样"基准要更新"时不会把构建卡住 */
A.writeLF(path.join(BUILD, 'armBuilder.html'), armBuilder);

if (gotMd5 !== EXPECTED_ARM_BUILDER_MD5) {
  if (process.argv.includes('--update-md5')) {
    const p = __filename;
    const src = fs.readFileSync(p, 'utf8');
    const re = /const EXPECTED_ARM_BUILDER_MD5 = '[0-9a-f]+';/;
    if (!re.test(src)) { log('    ✗ 找不到 EXPECTED_ARM_BUILDER_MD5，无法自动更新'); process.exit(1); }
    fs.writeFileSync(p, src.replace(re, "const EXPECTED_ARM_BUILDER_MD5 = '" + gotMd5 + "';"), 'utf8');
    log('    ⚠ md5 变了，已用 --update-md5 写回新基准：' + gotMd5);
    log('      旧基准：' + EXPECTED_ARM_BUILDER_MD5);
    log('      ★ 确认这次变化是有意的、且自检全过之后，再把上面这行提交。');
  } else {
    log('    ✗ md5 与基准不一致（产物已写出，自检可以照跑）');
    log('      基准 ' + EXPECTED_ARM_BUILDER_MD5);
    log('      实际 ' + gotMd5);
    log('      ⇒ 若不是有意改动，说明代码被改坏了。若是有意改动（比如改配置层），');
    log('        先跑自检确认没问题，再用  node web/build.js --update-md5  更新基准。');
    process.exitCode = 2;
  }
} else {
  log('    ✓ md5 ' + gotMd5.slice(0, 12) + ' —— 与基准一致');
}

/* ---- 3. 汇总 ----
   ※ 这里原来还会合成 build/armLink.html（armBuilder + 连杆调参模块）。
     2026-10 按要求把「机械臂构型改进」那块 UI 删掉了，交付物只剩 armBuilder。
     连杆搜索/复核引擎（web/src/modules/t11_link_mod.js）仍然保留，
     但改成由自检按需挂载（loadEnv({link:true})），不再产出第二个 HTML。 */
line();
log('产物：');
for (const f of ['armBuilder.html']) {
  const p = path.join(BUILD, f);
  log('  build/' + f.padEnd(18) + String(fs.statSync(p).size).padStart(8) + ' 字节' +
      '   md5 ' + A.md5(fs.readFileSync(p, 'utf8')).slice(0, 12));
}
log('用法： 双击 build/armBuilder.html');
line();
