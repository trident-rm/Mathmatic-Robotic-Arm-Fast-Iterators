%% t3_debug_conv.m — 查清 RST 里 setFixedTransform 与关节转动的作用顺序
% 疑问：子体位姿到底是
%   A) T = T_prev * Trans(offset) * Rot(axis,q)   （URDF 语义：关节轴过 offset 点）
%   B) T = T_prev * Rot(axis,q) * Trans(offset)   （offset 跟着子体转，轴过父原点）
clc
fprintf('--- 单关节：offset=[0 0 75]mm, axis=x ---\n');
r = rigidBodyTree('DataFormat','row');
j = rigidBodyJoint('j1','revolute'); j.JointAxis=[1 0 0];
setFixedTransform(j, trvec2tform([0 0 0.075]));
b = rigidBody('link1'); b.Joint=j; addBody(r,b,r.BaseName);
for q = [0 pi/6 pi/2]
    T = getTransform(r, q, 'link1');
    fprintf('  q=%5.1f deg -> t = %s\n', rad2deg(q), mat2str(round(T(1:3,4).'*1000, 4)));
end
fprintf('  A 预期 q=90: [0 0 75]\n  B 预期 q=90: [0 -75 0]\n');

fprintf('\n--- 两关节：j1 offset=0 绕z; j2 offset=[0 0 75] 绕x ---\n');
r2 = rigidBodyTree('DataFormat','row');
par = r2.BaseName;
for k = 1:2
    jk = rigidBodyJoint(sprintf('j%d',k),'revolute');
    if k==1, jk.JointAxis=[0 0 1]; off=[0 0 0];
    else,    jk.JointAxis=[1 0 0]; off=[0 0 75]; end
    setFixedTransform(jk, trvec2tform(off/1000));
    bk = rigidBody(sprintf('link%d',k)); bk.Joint=jk;
    addBody(r2, bk, par); par = bk.Name;
end
for q = [0 0; 30 0; 0 90; 30 90].'
    T = getTransform(r2, deg2rad(q.'), 'link2');
    fprintf('  q=[%3.0f %3.0f] -> t = %s\n', q, mat2str(round(T(1:3,4).'*1000, 4)));
end

fprintf('\n--- 关节点位置（各 body 原点）随 q 变化 ---\n');
for q = [0 0; 0 90].'
    q = q.';                       % 迭代取到的是列，转成 1x2
    T1 = getTransform(r2, deg2rad(q), 'link1');
    T2 = getTransform(r2, deg2rad(q), 'link2');
    fprintf('  q=[%3.0f %3.0f] -> link1 %s | link2 %s\n', q, ...
        mat2str(round(T1(1:3,4).'*1000,3)), mat2str(round(T2(1:3,4).'*1000,3)));
end
