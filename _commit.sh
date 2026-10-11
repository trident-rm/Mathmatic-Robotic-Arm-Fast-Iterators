#!/usr/bin/env bash
set -e
cd "/e/robomaster/26-27年/同济工程研究/Matlab测试" || exit 1
rm -f _stage.sh _stage_out.txt
git add -A

cat > /tmp/cmsg.txt <<'MSG'
模块化重整：按功能分目录 + 拆分内联脚本 + 引入构建

背景：项目在快速迭代中被写成一堆平铺文件，armBuilder.html 里塞了 1633 行
内联脚本，模块边界完全靠约定。本次重整按功能分目录、把大文件拆成模块。

一、按模块功能分目录（全部用 git mv，104 个文件保留历史）
  build/    交付物（单文件 HTML，双击即用）
  web/      源码 src/ · 自检 checks/ · 共用库 lib/ · 打包 pack/
  matlab/   setup.m · model/（臂型+建树+正解）· core/（计算+界面+导出）· checks/
  docs/     notes/（报告）· screenshots/（28 张截图）· data/（模型 JSON + 探针产物）

二、拆分与整合代码接口
  armBuilder.html 的 3 个内联 <script> 抽成 3 个可编辑模块：
      01-kinematics.js   551 行  运动学 / 模型 / 3D 渲染
      02-task-rules.js   649 行  任务域 / 体素 / IK / 动画
      03-chassis-size.js 433 行  底盘碰撞箱 / 伸展尺寸 / 面板
  原文件变成 218 行骨架模板，含 3 个 <!--__MODULE:xxx__--> 标记。
  新增构建：node web/build.js  →  build/armBuilder.html + build/armLink.html
  新增共用库：web/lib/assemble.js（合成器）、web/lib/paths.js（路径只定义一次）
  t11_patch.js 的追加逻辑抽成纯函数 buildLink()，构建与自检共用同一份实现。
  matlab/setup.m 统一加搜索路径，替代各脚本里的 addpath(here)。
  MATLAB 产物改到 docs/ 与 build/，不再散在源码目录里。

三、"没改坏"是可验证的（这次重整的安全绳）
  · 抽出模块前先记下三个交付物的 md5；web/build.js 内置断言
    armBuilder.html 必须等于 5077ab35c6fd...，不等就报错退出。
  · 实测三个产物与重整前【逐字节相同】：
      armBuilder.html 5077ab35c6fd   armLink.html bd7bea45bba5
      armFeas.html    927c86bf3b85
  · 203 项无头断言全部通过：
      t10_chassis_size 39 · t11_link_tune 60 · t10_ui_probe 50 · t11_ui_probe 54
    含 t11_link_tune 里"原 3 个 script 与 armLink 逐字节相同"的比对。

四、已知未覆盖
  · 界面脚本之间仍靠共享全局（MODEL/CHAIN/Q）耦合 —— 本次只拆文件不改运行时，
    彻底解耦会破坏"逐字节相同"这条安全绳，留作后续。
  · MATLAB 侧未实跑（开发机没装 MATLAB），改动是机械性的（加路径、改产物目录），
    首次在装有 MATLAB 的机器上跑请留意 matlab/checks/t2_app_smoke.m 输出。

五、文档
  README.md 重写：新增目录结构、构建流程、"怎么保证没改坏"、已知边界。
  HISTORY.md 顶部加新旧路径对照表，正文保持原样（当时的工作记录，不事后美化）。
MSG

git commit -F /tmp/cmsg.txt 2>&1 | tail -5

echo
echo "=== 提交结果 ==="
git log --oneline -2
echo
git show --stat HEAD | head -4
echo
echo "=== 工作区（应干净）==="
git status --short | head
echo "(空 = 干净)"
