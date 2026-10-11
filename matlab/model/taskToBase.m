function p = taskToBase(p)
%TASKTORBASE 任务系(OXYZ，地面系) → 臂 base 系（整车系）的坐标换算
%
%   p = taskToBase(p)      % p 为 1x3 或 3x1，单位 mm
%
%   ★ 与网页 04-config.js 的 mapObj().toBase 【同一套公式】，不许各写一份：
%         base = Rz(φ)·(p − [0,0,baseH])，  φ = taskMapPhi(taskMap)
%
%   为什么强调"不许各写一份"：t7_ik_matlab.m 原来自己手写了一个 [-p1,-p2,p3-170]
%   （那是 Rz(180°) 那一类），和网页实际用的换算差 90°，于是 200 个位姿的
%   目标点全部摆错，报出"RST 可解 0/200"这种假结论。见
%   docs/notes/base_yaw决策待定.md。
%
%   配套：baseToTask / dirToBase（方向向量不受平移影响，要单独一个）

b = armBase();
p = p(:).';
phi = taskMapPhi(b.taskMap);
c = cosd(phi); s = sind(phi);
q = [p(1), p(2), p(3) - b.baseH];          % 先减掉安装高度
p = [c*q(1) - s*q(2), s*q(1) + c*q(2), q(3)];
end
