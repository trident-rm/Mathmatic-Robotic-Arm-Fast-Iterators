/*  web/lib/paths.js —— 各处共用的路径解析（模块化后位置变了，只在这里定义一次）

    仓库布局：
        build/                    构建产物（双击即用；armLink/armFeas 不入库）
        web/src/apps/             骨架模板
        web/src/modules/          可编辑的源码模块
        web/checks/               无头自检
        web/lib/                  共用库
        web/pack/                 打包
        docs/screenshots|data     截图与探针产物

    所有 js 都用 require('./paths.js') 拿路径，不要再各自拼 path.join(__dirname, ...)
*/
'use strict';
const path = require('path');

const ROOT  = path.resolve(__dirname, '..', '..');   // 仓库根
const BUILD = path.join(ROOT, 'build');
const WEB   = path.join(ROOT, 'web');
const DOCS  = path.join(ROOT, 'docs');

module.exports = {
  ROOT, BUILD, WEB, DOCS,
  SCREENSHOTS: path.join(DOCS, 'screenshots'),
  DATA:        path.join(DOCS, 'data'),

  /* 源码 */
  TEMPLATE:  path.join(WEB, 'src', 'apps', 'armBuilder.html'),
  MODULES:   path.join(WEB, 'src', 'modules'),
  LINK_MOD:  path.join(WEB, 'src', 'modules', 't11_link_mod.js'),
  LINK_CSS:  path.join(WEB, 'src', 'modules', 't11_link_css.css'),

  /* 构建产物 */
  ARM_BUILDER: path.join(BUILD, 'armBuilder.html'),
  ARM_LINK:    path.join(BUILD, 'armLink.html'),
  ARM_FEAS:    path.join(BUILD, 'armFeas.html'),
  ARM_VIEW:    path.join(ROOT, 'matlab', 'core', 'armview.html'),
};
