%% t0b_debug.m — 定位第 3 节维度报错 + 验证 RTB islimit 的批量输入问题
clc
here = fileparts(mfilename('fullpath'));
run(fullfile(fileparts(mfilename('fullpath')), '..', 'setup.m'));

fprintf('--- RTB islimit 单条 vs 批量 ---\n');
mdl_puma560
q1 = deg2rad([0 0 0 0 0 300]);
fprintf('单条 q(6)=300deg -> %s (期望 [0;0;0;0;0;1])\n', mat2str(p560.islimit(q1)));
q2 = [0 0 0 0 0 0; q1];
r = p560.islimit(q2);
fprintf('批量 2x6       -> size %s, 值 %s  <-- 静默错误\n', mat2str(size(r)), mat2str(r));
fprintf('直接比 qlim    -> %s (正解)\n', mat2str((q2 > p560.qlim(:,2).') - (q2 < p560.qlim(:,1).')));

fprintf('\n--- 自建 3 关节臂的尺寸 ---\n');
a = makeArm([0 0.290 0.402 0.176], [0 0 1; 0 1 0; 0 1 0], [-150 150; -5 145; -150 150], 'A');
ss = manipulatorStateSpace(a);
fprintf('StateBounds size = %s\n', mat2str(size(ss.StateBounds)));
[W, C] = generateRobotWorkspace(a, {}, 'MaxNumSamples', 100, 'IgnoreSelfCollision', 'off');
fprintf('W size = %s, C size = %s, class(C) = %s\n', mat2str(size(W)), mat2str(size(C)), class(C));
if ~isstruct(C)
    fprintf('C 前 3 行:\n'); disp(C(1:3,:));
end
fprintf('机器人 body 名: %s\n', strjoin(a.BodyNames, ', '));
fprintf('homeConfiguration size = %s\n', mat2str(size(a.homeConfiguration)));
