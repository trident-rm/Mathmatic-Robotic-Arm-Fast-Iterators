#!/bin/bash
# 临时：归档导出文件 + 更新基准 + 提交推送。用完即删。
cd "$(dirname "$0")"
set -e

# 1. 导出的两份 JSON 归档到 docs/data/（快照留档）
mkdir -p docs/data
mv "西工大机械臂.json"        "docs/data/导出_西工大机械臂.json"
mv "同济工程机械臂（示例）.json" "docs/data/导出_同济工程机械臂示例.json"

# 2. 更新基准并复核
node web/build.js --update-md5 2>&1 | grep -E 'md5|旧基准'
node web/build.js 2>&1 | grep -E '✓|✗'

# 3. 清掉临时脚本（注意：消息文件别删，提交要用）
rm -f tmp_v2.sh tmp_cmp2.js tmp_json.sh tmp_clean.sh tmp_exp.sh tmp_exp2.js tmp_ls*.js tmp_d*.sh tmp_*.ps1 tmp_*.html

# 4. 提交
git add -A
git commit -q -F tmp_msg2.txt
rm -f tmp_msg2.txt

echo "=== push ==="
git push origin main

echo
echo "=== 结果 ==="
echo "HEAD        : $(git rev-parse --short HEAD)"
echo "origin/main : $(git rev-parse --short origin/main)"
git log --oneline -3
echo
echo "工作区剩余改动: $(git status --porcelain | wc -l)"
git status --porcelain
