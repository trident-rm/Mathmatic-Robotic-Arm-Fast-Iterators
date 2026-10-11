function P = armFeasData(cfg)
%ARMFEASDATA 由链定义算出一套"可直接喂给 HTML 显示"的可行性集数据（JSON 友好）
%
%   P = armFeasData(cfg)
%
% cfg 字段（长度 mm，角度 deg）
%   spec     : 链定义，见 armSpec.m（struct 数组：name/type/axis/offset/limits）
%   base     : struct('xyz',[x y z],'yaw',d)  臂在整机原点系 O 里的安装位姿
%   crescent : struct('on',bool,'pair',[i j],'R_OUT',.,'R_IN',.,'CY2',.)
%              pair 里的 i/j 是"可动关节序号"（不是 spec 下标）；可为空/关闭
%   boxes    : struct array，字段 name / box(3x2, 整机系 mm)  任务域，可空
%   name     : 标题
%   nSample / nCloud / nGrid : 采样点数 / 点云下采样 / 关节空间网格分辨率
%
% 输出 P：全数值/字符串，可直接 jsonencode，或塞进 uihtml 的 Data
%
% 本工具只做运动学可行性：无碰撞几何、无动力学。
% 关节数可以为 0（例如只建了 base link 的阶段），此时点云为空、统计为 0。

% ---------------- 参数默认值 ----------------
if ~isfield(cfg,'nSample') || isempty(cfg.nSample), cfg.nSample = 5e4;  end
if ~isfield(cfg,'nCloud')  || isempty(cfg.nCloud),  cfg.nCloud  = 2500; end
if ~isfield(cfg,'nGrid')   || isempty(cfg.nGrid),   cfg.nGrid   = 81;   end
if ~isfield(cfg,'name')    || isempty(cfg.name),    cfg.name    = 'arm'; end
if ~isfield(cfg,'base')    || isempty(cfg.base), ...
        cfg.base = struct('xyz',[0 0 0],'yaw',0); end
if ~isfield(cfg,'boxes'),   cfg.boxes = struct('name',{},'box',{}); end
if ~isfield(cfg,'crescent') || isempty(cfg.crescent), cfg.crescent = struct('on',false); end
if ~isfield(cfg.crescent,'on'), cfg.crescent.on = false; end

spec = cfg.spec;
rng(0);

% ---------------- 1 建链（纯杆+关节） ----------------
[arm, nJ] = buildChain(spec);
nb = arm.NumBodies;

% 可动关节的采样范围（未设限位的按 ±180deg 处理）
limDeg = zeros(nJ, 2);
jNames = cell(1, nJ); jTypes = cell(1, nJ); jAxes = zeros(nJ, 3);
k = 0;
for i = 1:numel(spec)
    if ~strcmpi(spec(i).type, 'fixed')
        k = k + 1;
        jNames{k} = spec(i).name;
        jTypes{k} = lower(spec(i).type);
        jAxes(k,:) = spec(i).axis(:).' / norm(spec(i).axis);
        if isempty(spec(i).limits), limDeg(k,:) = [-180 180];
        else,                       limDeg(k,:) = sort(spec(i).limits); end
    end
end

% 显示位形：优先取"零位"（用户描述坐标系时都以零位为参照）。
% 若 0 落在某关节限位之外，就把 0 夹到最近的那一侧（而不是取中点），
% 这样画面仍然最贴近"零位描述"。（例如 joint2 限位 [90,200] ⇒ 显示在 90°）
qHome = zeros(1, nJ);
k = 0;
for i = 1:numel(spec)
    if ~strcmpi(spec(i).type, 'fixed')
        k = k + 1;
        if ~isempty(spec(i).limits)
            qHome(k) = min(max(0, min(spec(i).limits)), max(spec(i).limits));
        end
    end
end

% ---------------- 2 安装位姿：臂基座系 -> 整机原点系 O ----------------
% 不要用 rotz()：RTB(Peter Corke) 同名函数角度单位约定不同，手写避免歧义
% 点云内部统一按"米"变换、最后 ×1000 回 mm（曾因 mm 与 m 相加导致 base 高度只加了 0.2mm）
a  = deg2rad(cfg.base.yaw);
Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
Tb = [Rz, cfg.base.xyz(:)/1000; 0 0 0 1];
toWorldMM = @(pMeters) ((Tb(1:3,1:3)*pMeters.' + Tb(1:3,4)).')*1000;

% ---------------- 3 采样：区间盒 vs 月牙过滤 ----------------
cres = cfg.crescent;
useCres = isfield(cres,'on') && cres.on && nJ >= 2 && isfield(cres,'pair') && ~isempty(cres.pair);
if nJ == 0
    qdeg = zeros(0, 0); ok = false(0,1);
    Wbox = zeros(0,3); Wcrs = zeros(0,3); qOK = zeros(0,nJ);
else
    qdeg = rand(cfg.nSample, nJ).*(limDeg(:,2)-limDeg(:,1)).' + limDeg(:,1).';
    if useCres
        g  = crescentMargin(qdeg(:,cres.pair(1)), qdeg(:,cres.pair(2)), ...
                            cres.R_OUT, cres.R_IN, cres.CY2);
        ok = g <= 0;
    else
        ok = true(cfg.nSample, 1);
    end
    [Pm, ~] = batchFK(arm, deg2rad(qdeg), arm.BodyNames{end});
    Wbox = toWorldMM(Pm);
    Wcrs = Wbox(ok, :);
    qOK  = qdeg(ok, :);
end

% ---------------- 4 显示用：各 body 坐标系 + 骨架 ----------------
nodes = struct('name', {}, 'joint', {}, 'type', {}, 'o', {}, 'R', {}, 'axis', {}, ...
               'axLabel', {}, 'lim', {}, 'unlimited', {});
% 原点系 O 本身
nodes(1) = mkNode('O(整机原点)', '', 'fixed', [0 0 0], eye(3), [0 0 0], '', [], false);
% 臂基座系
nodes(2) = mkNode('base(臂安装)', '', 'fixed', cfg.base.xyz(:).', Rz, [0 0 0], '', [], false);
pose = zeros(nb + 1, 3);
pose(1,:) = cfg.base.xyz(:).';
for i = 1:nb
    nm = arm.BodyNames{i};
    T = getTransform(arm, deg2rad(qHome), nm);     % 米，臂基座系；注意 qHome 是【度】，必须转弧度
    o = toWorldMM(T(1:3,4).');                     % 整机系 mm
    Rw = Rz * T(1:3,1:3);
    s = spec(i);
    jn = s.name;
    if isfield(s,'joint') && ~isempty(s.joint), jn = s.joint; end
    if strcmpi(s.type, 'fixed')
        axW = [0 0 0]; axL = ''; lim = []; unl = false;
    else
        axW = (Rw * s.axis(:)).';                  % 关节轴在整机系下的方向（零位）
        axL = axisLabel(s.axis);
        if isempty(s.limits), lim = [-180 180]; unl = true;
        else,                 lim = sort(s.limits); unl = false; end
    end
    nodes(end+1) = mkNode(nm, jn, lower(s.type), o, Rw, axW, axL, lim, unl);  %#ok<AGROW>
    pose(i+1,:) = o;
end

% ---------------- 5 关节空间月牙图 ----------------
if useCres
    gi = linspace(limDeg(cres.pair(1),1), limDeg(cres.pair(1),2), cfg.nGrid);
    gj = linspace(limDeg(cres.pair(2),1), limDeg(cres.pair(2),2), cfg.nGrid);
    [GI, GJ] = meshgrid(gi, gj);
    Gm = crescentMargin(GI, GJ, cres.R_OUT, cres.R_IN, cres.CY2);
    areaFrac = 100*mean(Gm(:) <= 0);
    mSub = min(1500, size(qOK,1));
    dots = qOK(randperm(size(qOK,1), max(mSub,0)), [cres.pair(1) cres.pair(2)]);
    cresOut = struct('on', true, 'qi', cres.pair(1), 'qj', cres.pair(2), ...
                     'R_OUT', cres.R_OUT, 'R_IN', cres.R_IN, 'CY2', cres.CY2, ...
                     'gi', gi, 'gj', gj, 'grid', Gm, 'dots', dots, ...
                     'box', limDeg(cres.pair,:), 'labels', {{jNames{cres.pair(1)}, jNames{cres.pair(2)}}});
else
    Gm = []; areaFrac = 0; dots = zeros(0,2);
    cresOut = struct('on', false, 'qi', 1, 'qj', min(2,max(nJ,1)), 'R_OUT', 0, ...
                     'R_IN', 0, 'CY2', 0, 'gi', 1:5, 'gj', 1:5, 'grid', zeros(5), ...
                     'dots', dots, 'box', [0 1; 0 1], 'labels', {{'-','-'}});
end

% ---------------- 6 统计 ----------------
vox = @(W) size(unique(floor(W/50), 'rows'), 1);   % W 单位 mm，体素 50mm
if nJ == 0 || isempty(Wcrs)
    R = []; voxBox = 0; voxCrs = 0; rMin = 0; rMax = 0; inAny = 0;
else
    R = vecnorm(Wcrs - cfg.base.xyz, 2, 2);
    voxBox = vox(Wbox); voxCrs = vox(Wcrs);
    rMin = min(R); rMax = max(R);
    inAny = 0;
end
boxCov = zeros(1, numel(cfg.boxes));
if ~isempty(Wcrs)
    inAnyMask = false(size(Wcrs,1),1);
    for kk = 1:numel(cfg.boxes)
        b = cfg.boxes(kk).box;
        inb = all(Wcrs >= b(:,1).' & Wcrs <= b(:,2).', 2);
        boxCov(kk) = 100*nnz(inb)/size(Wcrs,1);
        inAnyMask = inAnyMask | inb;
    end
    inAny = 100*nnz(inAnyMask)/numel(inAnyMask);
end

% ---------------- 7 打包 ----------------
idxBox = randperm(size(Wbox,1), min(cfg.nCloud, size(Wbox,1)));
idxCrs = randperm(size(Wcrs,1), min(cfg.nCloud, size(Wcrs,1)));

P = struct();
P.meta = struct('title', cfg.name, 'generated', datestr(now,'yyyy-mm-dd HH:MM:SS'), ...
                'model', 'rigidBodyTree 纯杆+关节，无碰撞几何', 'nJoint', nJ);
P.base = struct('xyz', cfg.base.xyz, 'yaw', cfg.base.yaw);

% ---- 给 HTML 做实时正解用的链数据（JS 用它自己递推，拖动滑条不用问 MATLAB） ----
kin = struct('name', {}, 'joint', {}, 'type', {}, 'axis', {}, 'axLabel', {}, ...
             'offset', {}, 'lim', {}, 'unlimited', {}, 'movable', {}, 'rot', {});
for i = 1:numel(spec)
    s = spec(i);
    jn = s.name;
    if isfield(s,'joint') && ~isempty(s.joint), jn = s.joint; end
    mv = ~strcmpi(s.type, 'fixed');
    if mv && isempty(s.limits), lim = [-180 180]; unl = true;
    elseif mv,                 lim = sort(s.limits); unl = false;
    else,                      lim = []; unl = false; end
    rot = [];
    if isfield(s,'rot') && ~isempty(s.rot), rot = s.rot; end
    kin(i) = struct('name', s.name, 'joint', jn, 'type', lower(s.type), ...
                    'axis', s.axis(:).', 'axLabel', ternary(mv, axisLabel(s.axis), ''), ...
                    'offset', s.offset(:).', 'lim', lim, 'unlimited', unl, 'movable', mv, ...
                    'rot', rot);
end
P.kin = kin;
P.q0  = qHome;                     % 初始位形（deg，按可动关节顺序）

P.chain = spec;
P.nodes = nodes;
P.pose = pose;
P.cloud = struct('box', Wbox(idxBox,:), 'cres', Wcrs(idxCrs,:));
P.crescent = cresOut;
P.stats = struct('feasRate', 100*nnz(ok)/max(cfg.nSample,1), 'areaFrac', areaFrac, ...
                 'voxBox', voxBox, 'voxCres', voxCrs, 'rMin', rMin, 'rMax', rMax, ...
                 'nSample', cfg.nSample, 'inAnyBox', inAny, 'nJoint', nJ);
P.boxes = cfg.boxes;
for kk = 1:numel(P.boxes), P.boxes(kk).cov = boxCov(kk); end
end

% ---------------- 局部函数 ----------------
function n = mkNode(name, joint, type, o, R, axis, axLabel, lim, unlimited)
n = struct('name', name, 'joint', joint, 'type', type, 'o', o, 'R', R, ...
           'axis', axis, 'axLabel', axLabel, 'lim', lim, 'unlimited', unlimited);
end

function s = axisLabel(v)
%AXISLABEL 把轴向写成 '+X'/'-Y' 这样的短标签
v = v(:).' / norm(v);
[~, i] = max(abs(v));
c = 'XYZ';
sg = '+';
if v(i) < 0, sg = '-'; end
s = [sg c(i)];
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
