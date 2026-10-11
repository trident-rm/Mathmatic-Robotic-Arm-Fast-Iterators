function d = dirToBase(d)
%DIRTOBASE 方向向量：任务系(OXYZ) → 臂 base 系（整车系）
%
%   d = dirToBase(d)
%
%   方向向量不受平移影响，所以只转不抬。与网页 mapObj().dirToBase 同一套公式。
%
%   典型用途：把科技核心的装配轴 e⃗（定义在 OXYZ 下）转到 base 系，
%   再拿去当 IK 的姿态目标。姿态算错会表现为 IK 大量失败
%   （位置对了但姿态差 90°）—— 见 docs/notes/base_yaw决策待定.md。

b = armBase();
d = d(:).';
phi = taskMapPhi(b.taskMap);
c = cosd(phi); s = sind(phi);
d = [c*d(1) - s*d(2), s*d(1) + c*d(2), d(3)];
end
