#!/usr/bin/env bash
set -u
cd "/e/robomaster/26-27年/同济工程研究/Matlab测试" || exit 1

echo "=== README 残留检查 ==="
grep -c "armLink" README.md || echo "  0 (已清)"

echo
echo "=== 提交文档修正 ==="
git add -A
git commit -q -m "文档：清掉 README 里残留的 armLink 引用（连杆界面已移除）"
GCM_INTERACTIVE=never git push -q origin main
echo "pushed  HEAD=$(git rev-parse --short HEAD)"

echo
echo "=== 干净 clone 复核 ==="
rm -rf /tmp/vc3
GCM_INTERACTIVE=never git clone -q https://github.com/trident-rm/Mathmatic-Robotic-Arm-Fast-Iterators.git /tmp/vc3
cd /tmp/vc3 || exit 1
echo "clone HEAD: $(git rev-parse --short HEAD)"
node web/build.js 2>&1 | grep -E "md5|✗"
for t in t10_chassis_size t12_arm_swap t13_multi_arm t14_wizard; do
  printf "%-18s " "$t"; node "web/checks/$t.js" 2>&1 | tail -1
done
printf "%-18s " t10_ui_probe;  node web/checks/t10_ui_probe.js --run 2>&1 | tail -1
printf "%-18s " t11_link_tune; node web/checks/t11_link_tune.js --quick 2>&1 | tail -1
echo
echo "=== 交付物只有 armBuilder 一个 ==="
ls -1 build/
