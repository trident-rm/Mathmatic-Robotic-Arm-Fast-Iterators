function [robot, nJ] = buildChain(spec)
%BUILDCHAIN 由 armSpec 的链定义建 rigidBodyTree（纯杆+关节，无碰撞几何）
%
%   [robot, nJ] = buildChain(spec)
%
% spec 见 armSpec.m：struct 数组，字段 name/type/axis/offset/limits
% nJ = 可动关节数（type 非 fixed 的个数）
%
% 说明：
%   - offset 单位 mm，内部转 m
%   - limits = [] 表示不设限位（rigidBodyJoint 保持默认 ±pi）
%   - 不产生任何 collision geometry，本工具只做运动学可行性

robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', numel(spec) + 2);
parent = robot.BaseName;
nJ = 0;
for i = 1:numel(spec)
    s = spec(i);
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
