#!/bin/bash
# 临时：清掉误提交的消息文件 + 最终确认。用完即删。
cd "$(dirname "$0")"
git add -A
git commit -q -m "清理：删掉误提交的临时文件 tmp_msg2.txt"
git push origin main
echo
echo "=== 最终确认 ==="
echo "HEAD        : $(git rev-parse --short HEAD)"
echo "origin/main : $(git rev-parse --short origin/main)"
[ "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)" ] && echo "=> 完全同步" || echo "=> 不一致"
echo "工作区改动: $(git status --porcelain | wc -l)"
echo
echo "仓库里误入的 tmp* 文件: $(git ls-files | grep -c '^tmp' || true)"
git ls-files | grep '^tmp' || echo "  ✓ 无"
echo
git log --oneline -4
rm -f tmp_*.sh
