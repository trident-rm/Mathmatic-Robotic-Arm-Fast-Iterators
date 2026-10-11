#!/bin/bash
# 临时：清掉误提交的 tmp_msg.txt，再 push。用完即删。
cd "$(dirname "$0")"
set -e
git add -A
git commit -q -m "清理：删掉误提交的临时文件 tmp_msg.txt"
echo "=== push ==="
git push origin main
echo
echo "=== 确认仓库根目录干净 ==="
git status -sb | head -2
git ls-files | grep -i '^tmp' || echo "  ✓ 仓库里没有 tmp* 文件"
ls tmp_* 2>/dev/null || echo "  ✓ 工作区也没有 tmp* 文件"
echo
git log --oneline -3
