function p = baseToTask(p)
%BASETOTASK 臂 base 系（整车系） → 任务系(OXYZ) 的坐标换算
%
%   p = baseToTask(p)
%
%   ★ 与网页 04-config.js 的 mapObj().toTask 同一套公式（taskToBase 的逆）：
%         OXYZ = Rz(−φ)·base + [0,0,baseH]
%
%   见 taskToBase.m 的说明（为什么必须共用一份实现）。

b = armBase();
p = p(:).';
phi = taskMapPhi(b.taskMap);
c = cosd(phi); s = sind(phi);
q = [c*p(1) + s*p(2), -s*p(1) + c*p(2), p(3)];   % Rz(−φ)
p = [q(1), q(2), q(3) + b.baseH];
end
