function [robot, nJ] = buildChain(spec, base)
%BUILDCHAIN 由 armSpec 的链定义建 rigidBodyTree（纯杆+关节，无碰撞几何）
%
%   [robot, nJ] = buildChain(spec)
%   [robot, nJ] = buildChain(spec, base)     % base 省略时取 armBase()
%
% spec 见 armSpec.m：struct 数组，字段 name/type/axis/offset/limits
% base 见 armBase.m：struct，字段 yaw/baseH/taskMap
% nJ = 可动关节数（type 非 fixed 的个数）
%
% ★ 安装朝向（base.yaw）会作为一个【根部固定变换】加进树里，
%   于是整棵树的参考系 = 整车系，与网页 buildChain() 第 71 行
%   `let R = rotZdeg(MODEL.base.yaw)` 完全一致。
%
%   为什么必须加：不加的话 MATLAB 算的是"臂底座系"，网页算的是"整车系"，
%   同一组 q 两边末端位置差一个 Rz(yaw)（实测 yaw=-90 时差 90°）。
%   后果不是"差一点"，而是拿任务域目标点去解 IK 时目标点整体摆错 0.5m，
%   报出"0/200 可解"这种假结论 —— 详见 docs/notes/base_yaw决策待定.md。
%
% 说明：
%   - offset 单位 mm，内部转 m
%   - limits = [] 表示不设限位（rigidBodyJoint 保持默认 ±pi）
%   - 不产生任何 collision geometry，本工具只做运动学可行性

if nargin < 2 || isempty(base), base = armBase(); end

robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', numel(spec) + 3);
parent = robot.BaseName;

% ---- 根部固定变换：安装朝向 ----
% 加一个 fixed body 把整棵树转过去。这样后面每个 link 的 offset 语义
% 与网页 buildChain() 的 "父系 = 整机原点系" 完全等价。
if abs(base.yaw) > 1e-12
    jbase = rigidBodyJoint('base_mount', 'fixed');
    Tb = eye(4);
    Tb(1:3,1:3) = rotDeg('z', base.yaw);
    setFixedTransform(jbase, Tb);
    bb = rigidBody('base_mount');
    bb.Joint = jbase;
    addBody(robot, bb, parent);
    parent = bb.Name;
end

nJ = 0;
for i = 1:numel(spec)
    s = spec(i);
    assert(~strcmp(s.name, 'base_mount'), ...
           'buildChain: body 名 base_mount 是保留的（根部安装变换占用），请改名');
    jname = s.name;
    if isfield(s, 'joint') && ~isempty(s.joint), jname = s.joint; end
    switch lower(s.type)
        case 'fixed'
            jnt = rigidBodyJoint(jname, 'fixed');
        case {'revolute', 'prismatic'}
            jnt = rigidBodyJoint(jname, lower(s.type));
            ax = s.axis(:).';
            assert(norm(ax) > 0, 'buildChain: %s 的 axis 不能是零向量', s.name);
            jnt.JointAxis = ax / norm(ax);
            if ~isempty(s.limits)
                lim = deg2rad(sort(s.limits));
                % 先设 HomePosition 再设 PositionLimits：
                % 顺序反了 RST 会警告 "home 落在新限位外，已重置为区间中点"
                % （joint2 限位 [90,200] 而默认 home=0 就会触发）
                jnt.HomePosition = min(max(0, lim(1)), lim(2));
                jnt.PositionLimits = lim;
            elseif strcmpi(s.type, 'revolute')
                % limits = [] ⇒ 不限位。回转关节的物理全周就是 ±180°，
                % 显式写成 ±pi，避免依赖 RST 的默认值；采样也按 ±180° 走。
                jnt.PositionLimits = [-pi, pi];
            else
                error('buildChain: prismatic 关节 %s 必须给 limits（移动副没有天然全周）', s.name);
            end
            nJ = nJ + 1;
        otherwise
            error('buildChain: 未知关节类型 "%s"', s.type);
    end
    % 固定变换 = 旋转 rot（可选）+ 平移 offset；对 revolute 关节 RST 也会吃旋转部分
    Tf = eye(4);
    if isfield(s, 'rot') && ~isempty(s.rot)
        assert(isequal(size(s.rot), [3 3]), 'buildChain: %s 的 rot 必须是 3x3', s.name);
        Tf(1:3,1:3) = s.rot;
    end
    Tf(1:3,4) = s.offset(:).' / 1000;
    setFixedTransform(jnt, Tf);
    b = rigidBody(s.name);
    b.Joint = jnt;
    addBody(robot, b, parent);
    parent = b.Name;
end
end
