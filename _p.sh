#!/usr/bin/env bash
cd "/e/robomaster/26-27年/同济工程研究/Matlab测试" || exit 1
git add -A
git commit -q -m "文档：README 补记行尾必须锁死 LF 这条坑（干净 clone 才测得出）"
GCM_INTERACTIVE=never git push -q origin main
echo "=== PUSHED ==="
git log --oneline
echo
git status --short
echo "(空 = 干净)"
