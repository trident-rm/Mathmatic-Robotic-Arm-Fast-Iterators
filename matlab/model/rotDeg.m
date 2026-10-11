function R = rotDeg(ax, ang)
%ROTDEG 绕 x/y/z 轴转 ang【度】的旋转矩阵（右手正方向）
%
%   R = rotDeg('x', -90)
%
% 用途：armSpec 里给某个 body 指定"相对父坐标系的固定旋转"，
%       例如第六步 link5：z 轴 = link4 的 y 正方向，y 轴 = -link4 的 z 方向
%       ⇒ 等价于绕 x 转 -90°，写成 spec(6).rot = rotDeg('x', -90)
%
% 注意：RST 的 setFixedTransform 对 revolute 关节同样吃旋转部分，
%       且作用顺序是 T = T_prev · T_fixed · Rot(axis,q)（URDF 语义），已实测确认。

c = cosd(ang);  s = sind(ang);
switch lower(ax)
    case 'x', R = [1 0 0; 0 c -s; 0 s c];
    case 'y', R = [c 0 s; 0 1 0; -s 0 c];
    case 'z', R = [c -s 0; s c 0; 0 0 1];
    otherwise, error('rotDeg: 轴只能是 x / y / z');
end
end
