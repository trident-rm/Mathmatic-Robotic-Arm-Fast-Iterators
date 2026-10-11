#!/usr/bin/env node
/*  web/build.js —— 构建交付物到 build/

    用法：  node web/build.js            （在仓库根目录跑）

    产物（单文件、零依赖、双击就能开）：
        build/armBuilder.html    建模器
        build/armLink.html       建模器 + 连杆调参

    日志会打印每个模块的 md5，改了哪个模块一眼能看出来。
    构建是幂等的：同样的源码一定得到同样的字节。
*/
'use strict';
const fs = require('fs'), path = require('path');
const A = require('./lib/assemble');

/* 重构前的 armBuilder.html 的 md5。合成结果必须与它逐字节相同，
   否则说明"抽模块"这一步改动了行为。 */
const EXPECTED_ARM_BUILDER_MD5 = '5077ab35c6fdcb9fb3be9fbcee573348';

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
if (gotMd5 !== EXPECTED_ARM_BUILDER_MD5) {
  log('    ✗ md5 不一致！');
  log('      期望 ' + EXPECTED_ARM_BUILDER_MD5);
  log('      实际 ' + gotMd5);
  log('      ⇒ 模块合成结果与重构前不同，抽模块这步改变了代码，必须查。');
  process.exit(1);
}
log('    ✓ md5 ' + gotMd5.slice(0, 12) + ' —— 与重构前逐字节相同');

A.writeLF(path.join(BUILD, 'armBuilder.html'), armBuilder);

/* ---- 3. 合成 armLink（= armBuilder + 连杆模块）---- */
log('\n[3] 合成 build/armLink.html');
const patch = require(path.join(A.WEB, 'checks', 't11_patch.js'));
const r = patch.buildLink(armBuilder);
A.writeLF(path.join(BUILD, 'armLink.html'), r.html);
log('    armBuilder(' + r.srcMD5 + ') + 连杆模块(' + r.modMD5 + ') + css(' + r.cssMD5 + ')');
log('    ✓ md5 ' + A.md5(r.html).slice(0, 12));

/* ---- 4. 汇总 ---- */
line();
log('产物：');
for (const f of ['armBuilder.html', 'armLink.html']) {
  const p = path.join(BUILD, f);
  log('  build/' + f.padEnd(18) + String(fs.statSync(p).size).padStart(8) + ' 字节' +
      '   md5 ' + A.md5(fs.readFileSync(p, 'utf8')).slice(0, 12));
}
log('用法： 双击 build/armBuilder.html 或 build/armLink.html');
line();
