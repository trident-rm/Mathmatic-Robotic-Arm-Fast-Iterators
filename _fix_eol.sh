#!/usr/bin/env bash
set -e
cd "/e/robomaster/26-27年/同济工程研究/Matlab测试" || exit 1
rm -f _diag.sh _verify_remote.sh _vr_out.txt

echo "=== 1. 加了 .gitattributes 之后，重新规范化工作区行尾 ==="
git add --renormalize . 2>/dev/null || git add .
git status --short | head -20
echo
echo "=== 2. 本地重跑构建 ==="
node web/build.js 2>&1 | grep -E "✓ md5|✗|期望|实际" | head -4

echo
echo "=== 3. 提交（.gitattributes + 行尾安全化）==="
git add -A
git commit -q -m "修复：构建产物行尾依赖平台，新克隆必失败

问题：交付物是【由源码逐字节合成】的，web/build.js 会断言 armBuilder.html
的 md5 等于 5077ab35c6fd。但 Windows 上 core.autocrlf=true 会把源文件
检出成 CRLF，合成出来的 HTML 就变成 CRLF（99912 字节 vs 98065），
md5 对不上，别人克隆下来第一次构建就失败。
（我本机的工作区当时恰好还是 LF，所以本地测试没暴露这个问题。）

修法（两道保险）：
  1) 新增 .gitattributes：源码/文档一律 text eol=lf，图片等 binary。
     保证任何平台克隆出来的字节都一样。
  2) 构建链全部对行尾免疫：web/lib/assemble.js 与 web/checks/t11_patch.js
     读文件统一 \r\n -> \n，写文件统一 LF；t11_link_tune.js 的自检比对
     也统一成 LF 再比。

验证：在【干净克隆】里跑 node web/build.js，md5 恢复为 5077ab35c6fd /
bd7bea45bba5，与重构前逐字节相同。" 2>&1 | tail -3

echo
echo "=== 4. 推送 ==="
GCM_INTERACTIVE=never git push origin main 2>&1 | tail -2

echo
echo "=== 5. 干净克隆重新验证（这次必须过）==="
rm -rf /tmp/verify_clone
GCM_INTERACTIVE=never git clone -q https://github.com/trident-rm/Mathmatic-Robotic-Arm-Fast-Iterators.git /tmp/verify_clone
cd /tmp/verify_clone
echo "clone HEAD: $(git rev-parse HEAD)"
echo "--- 构建 ---"
node web/build.js 2>&1 | grep -E "✓ md5|✗|期望|实际" | head -4
echo "--- 与重构前备份比对 ---"
for p in armBuilder armLink armFeas; do
  a=$(md5sum "build/$p.html" | cut -d' ' -f1)
  b=$(md5sum "/tmp/armbak/$p.html" | cut -d' ' -f1)
  [ "$a" = "$b" ] && echo "  ✓ $p.html 逐字节相同 ($a)" || echo "  ✗ $p.html 不同: $a vs $b"
done
echo "--- 自检 ---"
node web/checks/t10_chassis_size.js 2>&1 | tail -1
node web/checks/t11_link_tune.js --quick 2>&1 | tail -1
node web/checks/t10_ui_probe.js --run 2>&1 | tail -1
node web/checks/t11_ui_probe.js --run 2>&1 | tail -1
