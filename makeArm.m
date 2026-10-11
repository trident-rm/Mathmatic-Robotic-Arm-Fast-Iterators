function robot = makeArm(offs, axes_, limDeg)
%MAKEARM 纯代码建 rigidBodyTree（不写 URDF、不碰 SolidWorks）
%
%   robot = makeArm(offs, axes_, limDeg)
%
% 输入
%   offs   : 1x(n+1) 各关节相对父坐标系的 x 方向偏移 [m]，最后一个是末端工具长度
%   axes_  : n x 3   每个关节的转轴（在本体坐标系下表达）
%   limDeg : n x 2   每个关节的限位 [deg]，[min max]
%
% 输出
%   robot  : rigidBodyTree（DataFormat = 'row'），末端 body 名为 'tool'
%            注意 rigidBodyTree 没有 Name 属性，设计名请在外面用结构体带
%
% 用法：改一个数字就是改一版设计，存成结构体扫参数即可迭代臂型
%   r = makeArm([0 0.290 0.402 0.176], [0 0 1;0 1 0;0 1 0], [-150 150;-5 145;-150 150]);
%   [W, C] = generateRobotWorkspace(r, {}, 'MaxNumSamples', 5e4, 'IgnoreSelfCollision', 'off');
%
% 说明
%   - 关节数 n = size(axes_,1)，第 i 个关节的轴向 axes_(i,:)，限位 limDeg(i,:)
%   - 偏移 offs(i) 是"父坐标系原点 -> 本关节原点"沿 x 的距离
%   - offs(end) 是最后一个关节到末端 tool 的沿 x 距离（等效工具长度）
%   - 总伸展长度 = offs(2) + offs(3) + ... + offs(end)（对全伸姿态）

if nargin < 3
    error('makeArm: 至少需要 offs / axes_ / limDeg 三个输入');
end
n = size(axes_, 1);
assert(numel(offs) == n + 1, 'offs 长度必须为 n+1（含末端工具偏移）');
assert(size(limDeg, 1) == n && size(limDeg, 2) == 2, 'limDeg 必须为 n x 2');

robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', n + 2);
parent = robot.BaseName;
for i = 1:n
    jnt = rigidBodyJoint(sprintf('j%d', i), 'revolute');
    jnt.JointAxis = axes_(i,:);
    jnt.PositionLimits = deg2rad(limDeg(i,:));
    setFixedTransform(jnt, trvec2tform([offs(i) 0 0]));
    b = rigidBody(sprintf('link%d', i));
    b.Joint = jnt;
    addBody(robot, b, parent);
    parent = b.Name;
end
tool = rigidBody('tool');
setFixedTransform(tool.Joint, trvec2tform([offs(end) 0 0]));
addBody(robot, tool, parent);
end
