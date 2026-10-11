%% t5_debug_rot.m — RST 的 revolute 关节能否带【旋转】的 setFixedTransform
% 需求（第六步）：link5 原点与 link4 相同，但
%     link5.z = link4.y ,  link5.y = -link4.z  ⇒  固定变换 = 绕 x 转 -90°
%     joint5 绕 link5 自身 z 轴自由转动
% 要确认：子体位姿到底是
%   A) T = T_prev · T_fixed(R,o) · Rot(axis, q)   （URDF 语义，轴在子体系）
%   B) T = T_prev · Rot(axis, q) · T_fixed        （顺序反了）
%   C) RST 直接忽略旋转部分 / 报错
clc
a  = -90*pi/180;
Rx = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
fprintf('Rx(-90) 各轴映射： x->%s  y->%s  z->%s\n', ...
    mat2str(round((Rx*[1;0;0]).',4)), mat2str(round((Rx*[0;1;0]).',4)), ...
    mat2str(round((Rx*[0;0;1]).',4)));

fprintf('\n--- 测试 A：revolute 关节 + 带旋转的 setFixedTransform ---\n');
r = rigidBodyTree('DataFormat','row');
j = rigidBodyJoint('j1','revolute'); j.JointAxis = [0 0 1];
Tf = eye(4); Tf(1:3,1:3) = Rx;
try
    setFixedTransform(j, Tf);
    b = rigidBody('link5'); b.Joint = j; addBody(r, b, r.BaseName);
    for q = [0 pi/2]
        T = getTransform(r, q, 'link5');
        R = T(1:3,1:3);
        fprintf('  q=%5.1f°: 子体 x=%s y=%s z=%s | t=%s\n', rad2deg(q), ...
            mat2str(round((R*[1;0;0]).',4)), mat2str(round((R*[0;1;0]).',4)), ...
            mat2str(round((R*[0;0;1]).',4)), mat2str(round(T(1:3,4).',4)));
    end
    fprintf('  A 预期 q=0: y=[0 0 -1] z=[0 1 0]；q=90: z 不变=[0 1 0]\n');
    fprintf('  B 预期 q=0: y=[0 0 -1] z=[0 1 0]；q=90: z 会变（被 T_fixed 再转一次）\n');
catch ME
    fprintf('  失败: %s\n', ME.message);
end

fprintf('\n--- 测试 D：直接用两体法（固定体带旋转 + 零偏置 revolute） ---\n');
r2 = rigidBodyTree('DataFormat','row');
jf = rigidBodyJoint('fix_rot','fixed');
Tf2 = eye(4); Tf2(1:3,1:3) = Rx;
setFixedTransform(jf, Tf2);
bf = rigidBody('link5_mount'); bf.Joint = jf; addBody(r2, bf, r2.BaseName);
jr = rigidBodyJoint('j5','revolute'); jr.JointAxis = [0 0 1];
setFixedTransform(jr, trvec2tform([0 0 0]));
br = rigidBody('link5'); br.Joint = jr; addBody(r2, br, 'link5_mount');
for q = [0 pi/2]
    T = getTransform(r2, q, 'link5');
    R = T(1:3,1:3);
    fprintf('  q=%5.1f°: 子体 y=%s z=%s\n', rad2deg(q), ...
        mat2str(round((R*[0;1;0]).',4)), mat2str(round((R*[0;0;1]).',4)));
end
