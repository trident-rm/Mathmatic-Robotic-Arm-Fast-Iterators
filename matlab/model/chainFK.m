function [poses, T] = chainFK(spec, base, qDeg)
%CHAINFK 纯 MATLAB 正解（不依赖 RST）——armview.html 里 JS 用的是同一套递推
%
%   [poses, T] = chainFK(spec, base, qDeg)
%
% 递推式（第 i 个 body）：
%     T_i = T_{i-1} * Trans(offset_i) * Rot(axis_i, q_i)
% 和 RST 的约定一致：setFixedTransform 给"父->关节"的平移，
% 关节转动发生在子体坐标系下、绕 JointAxis。
%
% 输入
%   spec  : 链定义（name/joint/type/axis/offset/limits）
%   base  : struct('xyz',[x y z] mm,'yaw',deg)
%   qDeg  : 1 x nJ 可动关节角（deg），按 spec 中可动关节出现的顺序
% 输出
%   poses : 1 x (n+1) cell，poses{1}=base 位姿，poses{i+1}=第 i 个 body 位姿
%           每个元素 struct('R',3x3,'t',3x1(mm))
%   T     : 末端位姿
%
% 用途：给 smoke test 拿它跟 RST 的 getTransform 交叉验证，
%       验证过了，HTML 里那套同构的 JS 递推才有依据。

a  = deg2rad(base.yaw);
T0 = struct('R', [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1], 't', base.xyz(:));
poses = cell(1, numel(spec) + 1);
poses{1} = T0;
T = T0;
k = 0;
for i = 1:numel(spec)
    s = spec(i);
    Rf = eye(3);
    if isfield(s, 'rot') && ~isempty(s.rot), Rf = s.rot; end
    Rj = eye(3);
    if ~strcmpi(s.type, 'fixed')
        k = k + 1;
        u = s.axis(:) / norm(s.axis);
        Rj = rotAxis(u, deg2rad(qDeg(k)));
    end
    % T_i = T_{i-1} · T_fixed(Rf, offset) · Rot(axis, q)
    %   ⇒ R = R_prev·Rf·Rj ，t = R_prev·offset + t_prev
    T = struct('R', T.R * Rf * Rj, 't', T.R * s.offset(:) + T.t);
    poses{i+1} = T;
end
end

function R = rotAxis(u, th)
%ROTAXIS Rodrigues 公式，绕任意单位轴 u 转 th（rad）
K = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
R = eye(3) + sin(th)*K + (1-cos(th))*(K*K);
end
