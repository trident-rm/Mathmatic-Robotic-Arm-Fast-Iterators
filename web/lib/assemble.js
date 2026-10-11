/*  web/lib/assemble.js —— 模块合成器（唯一负责"源码 -> 单文件 HTML"的地方）

    背景：交付物必须是【单文件、零依赖、双击就能开】，
          但源码要能分模块维护。所以源码是模块，交付物是构建产物。

    armBuilder.html 的组成：
        web/src/apps/armBuilder.html        骨架模板（含 3 个 __MODULE 标记）
      + web/src/modules/01-kinematics.js    运动学 / 模型 / 3D 渲染
      + web/src/modules/02-task-rules.js    任务域 / 体素 / IK / 动画
      + web/src/modules/03-chassis-size.js  底盘碰撞箱 / 伸展尺寸 / 面板

    ★ 保证：合成结果与重构前的 armBuilder.html 逐字节相同（md5 5077ab35c6fd...）
*/
'use strict';
const fs = require('fs'), path = require('path'), crypto = require('crypto');

const WEB = path.join(__dirname, '..');           // web/
const SRC = path.join(WEB, 'src');
const ROOT = path.join(WEB, '..');                // 仓库根

const TEMPLATE = path.join(SRC, 'apps', 'armBuilder.html');
/* 引擎模块：顺序有意义，别乱调（变量声明与启动顺序都靠它）
   01~03 = 算（运动学 / 任务域 / 底盘尺寸）
   04    = 配置层
   06    = 界面骨架（折叠面板 / 勾选下拉 / 旋转角度表 / 分隔条）
   ※ 没有 05：05 是 bootstrap，单独放在最后（在所有配置之后）
   ※ 06 必须排在 01~03 之后：它在加载时就会读 SHOWBOX / SHOW_FRAMES / CHAIN */
const MODULES = ['01-kinematics', '02-task-rules', '03-chassis-size', '04-config', '06-ui', '07-wizard'];
const BOOTSTRAP = '05-bootstrap';
const CONFIG_DIR = path.join(SRC, 'configs');
const CONFIG_INDEX = path.join(CONFIG_DIR, 'index.txt');

const read = p => fs.readFileSync(p, 'utf8');

/* 读文本并统一成 LF。
   ★ 必须做这个：Windows 上 core.autocrlf=true 会把源文件检出成 CRLF，
     而交付物是逐字节合成 + 断言 md5 的，行尾一变就对不上。
     .gitattributes 已经把行尾钉死成 LF，这里再加一道保险，
     让构建在任何 clone / 任何 git 配置下都得到同样的字节。            */
const readLF = p => read(p).replace(/\r\n/g, '\n');

const md5 = s => crypto.createHash('md5').update(s, 'utf8').digest('hex');

/* 读配置清单（一行一个 id，# 开头为注释） */
function readConfigIds() {
  return readLF(CONFIG_INDEX)
    .split('\n')
    .map(s => s.trim())
    .filter(s => s && !s.startsWith('#'));
}

/* 把骨架里的 <!--__MODULE:name__--> 替换成模块源码 */
function embedModules(template, modDir) {
  let out = String(template).replace(/\r\n/g, '\n');
  for (const name of MODULES) {
    const marker = '<!--__MODULE:' + name + '__-->';
    if (!out.includes(marker)) throw new Error('模板里没有标记 ' + marker);
    out = out.replace(marker, () => readLF(path.join(modDir, name + '.js')));
  }
  /* 配置文件与应用顺序表打进同一个 script 块：
     先 defineConfig(...)，最后 05-bootstrap 调 initDefaultConfig() */
  const marker = '<!--__MODULES__-->';
  if (!out.includes(marker)) throw new Error('模板里没有标记 ' + marker);
  const parts = [];
  const ids = readConfigIds();
  if (!ids.length) throw new Error('配置清单是空的：' + CONFIG_INDEX);
  for (const id of ids) {
    const p = path.join(CONFIG_DIR, id + '.js');
    if (!fs.existsSync(p)) throw new Error('配置清单里有 ' + id + '，但找不到 ' + p);
    parts.push(readLF(p));
  }
  parts.push(readLF(path.join(modDir, BOOTSTRAP + '.js')));
  out = out.replace(marker, () => parts.join('\n'));
  return out;
}

/* 读骨架模板（统一 LF） */
function readTemplate() { return readLF(TEMPLATE); }

/* 合成 armBuilder.html 的完整文本 */
function buildArmBuilder() {
  return embedModules(readTemplate(), path.join(SRC, 'modules'));
}

/* 写文件（统一 LF，不加 BOM），并返回 md5 */
function writeLF(file, text) {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  const lf = String(text).replace(/\r\n/g, '\n');
  fs.writeFileSync(file, lf, 'utf8');
  return md5(lf);
}

/* 给生成物加一行"这是构建产物"的头注释 */
function banner(kind, extra) {
  return '<!-- ' + kind + '：构建产物，请勿直接编辑 · ' + extra +
         ' · 改源码后跑 node web/build.js -->\n';
}

module.exports = { WEB, SRC, ROOT, TEMPLATE, MODULES,
                   read, readLF, md5, embedModules, readTemplate,
                   buildArmBuilder, writeLF, banner };
