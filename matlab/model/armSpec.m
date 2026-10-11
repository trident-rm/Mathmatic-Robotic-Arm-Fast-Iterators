function spec = armSpec()
%ARMSPEC 当前机械臂的运动学链定义（按步骤逐步补充，只改这一个文件）
%
%   每个元素描述一个 body 及其与【父坐标系】的连接：
%     name   : body 名字
%     joint  : 关节名字（可选；不写就用 body 名）
%     type   : 'fixed' | 'revolute' | 'prismatic'
%     axis   : 1x3 转轴/移动轴（在本体坐标系下表达；fixed 时忽略）
%     offset : 1x3 本 body 坐标系原点相对【父坐标系】原点的偏移（mm）
%     limits : 1x2 关节限位（deg）；[] 表示"不限制"
%     rot    : 3x3 本 body 坐标系相对父坐标系的【固定旋转】（可选，[]=无旋转）
%              用 rotDeg('x',-90) 这样写；作用顺序 T = T_prev · T_fixed(rot,offset) · Rot(axis,q)
%
%   注：没有单独的"杆"字段 —— 杆就是【相邻两个坐标系之间的距离】，
%       即 offset 本身（例如 link1 的杆 = base_link→link1 的 75mm）。
%
%   约定：
%     - 整机原点坐标系 O 就是 rigidBodyTree 的 base，永远是单位位姿
%     - 第一个 body 的父坐标系 = O
%     - 单位：长度 mm，角度 deg（内部转 rad）
%
%   当前进度：
%     第一步 base_link —— 坐标在原点的不限位回转关节（全周 ±180°）
%     第二步 link1     —— 沿 base 的 z 正方向 75mm；零位坐标系与 base 完全一致；
%                        joint1 绕其 x 轴旋转，零位时 link1 与 y 轴正方向平行，
%                        角度 0°~122°
%     第三步 link2     —— 沿 link1 的 y 正方向 290mm；零位坐标系与 link1 一致；
%                        joint2 轴取 -x（正向 = "+z 转向 +y"）；零位指 +z；
%                        限位 -90° ~ +200°（-90° 时杆指 -y）
%     第四步 link3     —— 沿 link2 的 x 正方向（= joint2 正方向）85.5mm；
%                        零位坐标系与 link2 一致；joint3 绕其 z 轴，
%                        与 base_joint 一样是【不限位】的自由回转关节
%     第五步 link4     —— 在 link3 的 z+ 401.47mm、y- 30mm 处（offset=[0 -30 401.47]）；
%                        零位坐标系与 link3 一致（两系 y 方向平齐）；
%                        joint4 绕其 x 轴（+x，即"y 转向 -z 为负角"）；限位 -16°~220°
%     第六步 link5     —— 原点与 link4 重合；坐标系 = link4 绕 x 转 -90°
%                        （即 link5.z = link4.y、link5.y = -link4.z）；
%                        joint5 绕自身 z 轴，自由转动（不限位）
%     第七步 gripper   —— 沿 link5 的 z 方向 211mm；无旋转；刚性固定在 link5 上（fixed）

spec = struct('name', {}, 'joint', {}, 'type', {}, 'axis', {}, 'offset', {}, ...
              'limits', {}, 'rot', {});

% ---- 第一步：base link（回转关节，不限位，坐标系在原点） ----
spec(1).name   = 'base_link';
spec(1).joint  = 'base_joint';
spec(1).type   = 'revolute';
spec(1).axis   = [0 0 1];
spec(1).offset = [0 0 0];
spec(1).limits = [];        % [] = 不限位

% ---- 第二步：link1（沿 base 的 z+ 方向 75mm，绕 x 轴，0~122°） ----
spec(2).name   = 'link1';
spec(2).joint  = 'joint1';
spec(2).type   = 'revolute';
spec(2).axis   = [1 0 0];   % 绕 x 轴
spec(2).offset = [0 0 75];  % base 的 z 正 75mm；无旋转 ⇒ 零位 xyz 与 base 一致
spec(2).limits = [0 122];   % 零位 = link1 与 y 轴正方向平行；上限 122°

% ---- 第三步：link2（沿 link1 的 y+ 方向 290mm，-90~200°） ----
spec(3).name   = 'link2';
spec(3).joint  = 'joint2';
spec(3).type   = 'revolute';
spec(3).axis   = [-1 0 0];  % 轴取 -x（正向 = "+z 转向 +y"），使 -90° 时杆指 -y
spec(3).offset = [0 290 0]; % link1 的 y 正 290mm；无旋转 ⇒ 零位 xyz 与 link1 一致
spec(3).limits = [-90 200]; % 零位 0° 在范围内，指向 +z；-90° 指 -y；上限 200°

% ---- 第四步：link3（沿 link2 的 x+ 方向 85.5mm，绕 z 轴，不限位） ----
spec(4).name   = 'link3';
spec(4).joint  = 'joint3';
spec(4).type   = 'revolute';
spec(4).axis   = [0 0 1];   % 绕 z 轴
spec(4).offset = [85.5 0 0];% joint2 正方向（= link2 的 +x）85.5mm
spec(4).limits = [];        % [] = 不限位，与 base_joint 一样全周回转

% ---- 第五步：link4（link3 的 z+401.47、y-30 处；绕 x 轴，-16~220°） ----
spec(5).name   = 'link4';
spec(5).joint  = 'joint4';
spec(5).type   = 'revolute';
spec(5).axis   = [1 0 0];   % 绕 +x（"y 往 -z 转"为负角 ⇒ 正是 +x 右手定则）
spec(5).offset = [0 -30 401.47]; % 相对 link3：z 正 401.47，y 负 30
spec(5).limits = [-16 220]; % 下限 -16°，上限 220°

% ---- 第六步：link5（原点与 link4 重合；坐标系绕 x 转 -90°；绕自身 z 轴自由转） ----
%   要求：link5 的 z 轴 = link4 的 y 正方向；link5 的 y 轴 = -link4 的 z 方向
%   Rx(-90) 恰好做到：y->-z、z->+y（已用 t5_debug_rot 实测确认）
spec(6).name   = 'link5';
spec(6).joint  = 'joint5';
spec(6).type   = 'revolute';
spec(6).axis   = [0 0 1];        % 绕自身 z 轴（转动后该轴 = link4 的 y 方向）
spec(6).offset = [0 0 0];        % 与 link4 坐标原点重合
spec(6).limits = [];             % 自由转动，不限位
spec(6).rot    = rotDeg('x', -90);

% ---- 第七步：gripper（沿 link5 的 z 方向 211mm；无旋转；刚性固定） ----
%   joint5 的轴就是 link5 的 z 方向，所以夹爪原点落在 joint5 的轴上
%   ⇒ joint5 转动只改变夹爪姿态、不移动其原点
spec(7).name   = 'gripper';
spec(7).joint  = 'gripper_fix';
spec(7).type   = 'fixed';
spec(7).axis   = [0 0 1];       % fixed 关节，axis 不参与运算（占位）
spec(7).offset = [0 0 211];     % 沿 link5 的 z 方向 211mm
spec(7).limits = [];            % fixed 关节无限位概念
spec(7).rot    = [];            % 与 link5 无相对旋转
end
