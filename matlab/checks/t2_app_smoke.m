%% t2_app_smoke.m — 可行性集工具链无头自检
% 覆盖：
%   1 当前 armSpec：坐标系/偏移/关节轴/限位是否与设计描述一致（逐步累加时自动跟着变）
%   2 三关节合成链回归（月牙过滤等全链路）
%   3 界面构造 / 控件改动重算 / 导出固定的一份 HTML
% 渲染正确性由 Edge 无头截图另行核对（t2_html_render_check.png）

clc; close all;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
run(fullfile(here, '..', 'setup.m'));
pass = true;

fprintf('=========== 1 当前 armSpec（逐步累加） ===========\n');
spec = armSpec();
fprintf('链元素 %d 个：%s\n', numel(spec), strjoin({spec.name}, ' -> '));
for i = 1:numel(spec)
    jn = '';
    if isfield(spec(i), 'joint') && ~isempty(spec(i).joint), jn = spec(i).joint; end
    fprintf('  %-10s joint=%-11s type=%-8s axis=%-9s offset=%-11s limits=%s\n', ...
            spec(i).name, jn, spec(i).type, mat2str(spec(i).axis), ...
            mat2str(spec(i).offset), mat2str(spec(i).limits));
end

% ★ base 必须与 buildChain 用的那套一致（armBase.m）。
%   以前这里写死 yaw=0，而 buildChain 不看 yaw，两边"碰巧"一致；
%   现在安装朝向进了树，这里的 cfg.base 也得跟上，否则 chainFK 与 RST 会差一个 Rz(yaw)。
ab = armBase();
cfg = struct('name', '当前臂型', 'spec', spec, ...
             'base', struct('xyz', [0 0 0], 'yaw', ab.yaw), ...
             'crescent', struct('on', false), ...
             'boxes', struct('name', {}, 'box', {}), 'nSample', 1e4);
tic; P0 = armFeasData(cfg); t0 = toc;
fprintf('\narmFeasData: %.3f s | 关节数 %d | 结点数 %d | 点云 %d\n', ...
        t0, P0.meta.nJoint, numel(P0.nodes), size(P0.cloud.cres,1));

% --- 断言 A：原点系 O 是单位位姿 ---
nO = P0.nodes(1);
assert(norm(nO.o) < 1e-9 && max(abs(nO.R - eye(3)), [], 'all') < 1e-12, ...
       '原点系 O 应为单位位姿');
fprintf('  ✓ 原点坐标系 O 为单位位姿（位置/姿态误差 ~0）\n');

% --- 断言 B：零位（全零）时各 body 原点 = base.xyz + Rz(base.yaw)·累积 offset ---
% 注意：P0.nodes 是【显示位形】下的结果（joint2 限位 [90,200]，显示位形是 90° 而非 0°），
%       所以"零位"的核对必须用独立参考实现 chainFK 在全零处算，不能直接拿 nodes 比。
% ★ 安装朝向进来之后，累积 offset 必须先绕 Z 转 base.yaw 再加到 base.xyz 上
%   （以前 yaw=0，直接相加碰巧对；yaw≠0 就不能再那样了）。
nJ0 = P0.meta.nJoint;
ps0 = chainFK(spec, cfg.base, zeros(1, nJ0));
aa  = deg2rad(cfg.base.yaw);
RzB = [cos(aa) -sin(aa) 0; sin(aa) cos(aa) 0; 0 0 1];
cum = cfg.base.xyz(:).';
off = zeros(3, 1);                 % 累积 offset（在臂基座系里）
okOff = true;
rotSeen = false;      % 一旦祖先里出现过固定旋转，"原点=累积 offset"的简单关系就不再成立
for i = 1:numel(spec)
    if isfield(spec(i),'rot') && ~isempty(spec(i).rot), rotSeen = true; end
    off = off + spec(i).offset(:);
    cum = cfg.base.xyz(:).' + (RzB * off).';
    if rotSeen
        fprintf('  %-10s 自身或祖先含固定旋转，原点累加关系跳过（由 chainFK vs RST 覆盖）\n', ...
                spec(i).name);
        continue
    end
    got = ps0{i+1}.t(:).';
    d = norm(got - cum);
    fprintf('  %-10s 零位原点 期望 %-15s 实得 %-15s 误差 %.3g mm\n', ...
            spec(i).name, mat2str(cum), mat2str(round(got,6)), d);
    okOff = okOff && d < 1e-9;
end
assert(okOff, '零位下各 body 原点应等于累积 offset');
fprintf('  ✓ 零位下各 body 原点位置与坐标系描述一致\n');

% --- 断言 C：零位时"每个 body 相对父体的姿态 = 它自己的 rot"（与祖先旋转无关，可直接查） ---
okR = true;
for i = 1:numel(spec)
    Rexp = eye(3);
    if isfield(spec(i),'rot') && ~isempty(spec(i).rot), Rexp = spec(i).rot; end
    Rrel = ps0{i}.R.' * ps0{i+1}.R;        % 该 body 相对其父坐标系的姿态
    dRk = max(abs(Rrel - Rexp), [], 'all');
    if ~isequal(Rexp, eye(3))
        fprintf('  %-10s 相对父体姿态 = 指定 rot（误差 %.3g）\n', spec(i).name, dRk);
    end
    okR = okR && dRk < 1e-12;
end
assert(okR, '零位下各 body 相对父体的姿态应等于其 rot（未指定则为单位阵）');
fprintf('  ✓ 零位：各 body 相对父体的姿态 = 各自 rot（未指定的即无相对旋转）\n');

psH = chainFK(spec, cfg.base, P0.q0);      % P0.q0 = 实际显示位形
dPn = 0; dPr = 0;
for i = 1:numel(spec)
    dPn = max(dPn, norm(P0.nodes(2+i).o(:) - psH{i+1}.t(:)));
    dPr = max(dPr, max(abs(P0.nodes(2+i).R - psH{i+1}.R), [], 'all'));
end
fprintf('  显示位形 q0 = %s deg（0 落在限位外的关节夹到最近边界）\n', mat2str(P0.q0));
fprintf('  armFeasData vs chainFK：位置差 %.3g mm，姿态差 %.3g\n', dPn, dPr);
assert(dPn < 1e-9 && dPr < 1e-12, 'armFeasData 的坐标系与 chainFK 不一致');
fprintf('  ✓ 显示位形下的坐标系与独立参考实现完全一致\n');

% --- 断言 D：关节轴与限位 ---
% 注意：nodes 里的 axis 是【显示位形下的世界系方向】，而 spec.axis 是【本体方向】，
%       两者只有在"该 body 的姿态与 base 对齐"时才相等。所以这里用节点自身的 R 换算，
%       验证的是 armFeasData 有没有把世界轴向算对（而不是拿它跟本体轴硬比）。
for i = 1:numel(spec)
    nd = P0.nodes(2+i);
    if strcmpi(spec(i).type, 'fixed'), continue; end
    u = spec(i).axis(:) / norm(spec(i).axis);
    axExp = (nd.R * u).';
    fprintf('  %-11s 世界轴向 期望 %-22s 实得 %-22s | 限位 %-11s 不限位=%d\n', ...
            nd.joint, mat2str(round(axExp,4)), mat2str(round(nd.axis,4)), ...
            mat2str(nd.lim), nd.unlimited);
    assert(norm(nd.axis - axExp) < 1e-12, '%s 的世界轴向不一致', nd.joint);
    if isempty(spec(i).limits)
        assert(nd.unlimited, '%s 应标记为不限位', nd.joint);
    else
        assert(~nd.unlimited && isequal(nd.lim, sort(spec(i).limits)), ...
               '%s 限位不符', nd.joint);
    end
end
fprintf('  ✓ 关节世界轴向与限位全部吻合\n');

% --- 断言 E：末端可达集 ---
ee = P0.nodes(end);
rho = vecnorm(P0.cloud.cres(:,1:2), 2, 2);
zc  = P0.cloud.cres(:,3);
Lsum = 0;
for i = 1:numel(spec), Lsum = Lsum + norm(spec(i).offset); end   % 各段杆长之和
fprintf('  ✓ 末端（%s）%d 个采样点 | ρ %.2f ~ %.2f mm | z %.2f ~ %.2f mm | 体素 %d\n', ...
        ee.name, size(P0.cloud.cres,1), min(rho), max(rho), min(zc), max(zc), P0.stats.voxCres);
rMax = max(vecnorm(P0.cloud.cres, 2, 2));
assert(rMax <= Lsum + 1e-6, '末端到原点的距离不应超过各段长度之和 %.1f mm', Lsum);
assert(max(zc) - min(zc) > 400 && P0.stats.voxCres > 500, ...
       '第五步后末端应扫出真实的立体可达集');
fprintf('  ✓ 立体可达集（z 跨度 %.0f mm > 400，体素数 %d）；到原点最大 %.1f ≤ 上界 %.1f mm\n', ...
        max(zc)-min(zc), P0.stats.voxCres, rMax, Lsum);

% --- 断言 G：joint2 限位与杆方向；杆长 = 相邻坐标系间距 ---
% joint2 axis=-x、零位杆指 +z ⇒ rod(θ) = R(-x,θ)·(0,0,1) = (0, sinθ, cosθ)
% ★ 解码表描述的是【臂基座系】里的杆方向。chainFK 现在算的是整机系
%   （根部含 Rz(base.yaw)），所以比之前要左乘 Rz(-yaw) 把它"卸回"基座系。
%   这里把断言改成在基座系里做，这样解码表本身不用改。
RzInv = RzB.';
inBase = @(R, v) RzInv * (R * v);
lo2 = min(spec(3).limits); hi2 = max(spec(3).limits);
assert(lo2 == -90 && hi2 == 200, 'joint2 限位应为 -90 ~ +200');
fprintf('  joint2 杆方向(轴-x，臂基座系): ');
nJt = P0.meta.nJoint;
for th = [-90 0 90 180 200]
    qq = zeros(1, nJt); qq(3) = th;
    ps = chainFK(spec, cfg.base, qq);
    d = inBase(ps{4}.R, [0;0;1]);          % link2 是第 3 个 body ⇒ poses{4}
    fprintf('%+d° %s  ', th, mat2str(round(d.',3)));
end
fprintf('\n');
qq = zeros(1, nJt); qq(3) = -90;
ps = chainFK(spec, cfg.base, qq);
assert(norm(inBase(ps{4}.R,[0;0;1]) - [0 -1 0].') < 1e-12, '-90° 应指 -y');
ps = chainFK(spec, cfg.base, zeros(1, nJt));
assert(norm(inBase(ps{4}.R,[0;0;1]) - [0 0 1].')  < 1e-12, '零位应指 +z');
fprintf('  ✓ 零位 0° 指 +z（0° 在限位内）；-90° 指 -y；上限 200°\n');
% 杆长 = 相邻坐标系间距 = offset
seg = zeros(1, numel(spec));
ps0b = chainFK(spec, cfg.base, zeros(1, P0.meta.nJoint));
prev = ps0b{1}.t;
for i = 1:numel(spec)
    seg(i) = norm(ps0b{i+1}.t - prev);
    prev = ps0b{i+1}.t;
end
fprintf('  相邻坐标系间距(=杆长, mm): %s\n', mat2str(round(seg(2:end),2)));
segExp = zeros(1, numel(spec));
for i = 1:numel(spec), segExp(i) = norm(spec(i).offset); end
assert(max(abs(seg(2:end) - segExp(2:end))) < 1e-9, '杆长应等于相邻坐标系间距');
fprintf('  ✓ 杆长 = 相邻坐标系间距（直接来自各段 offset）\n');

% --- 断言 H：第六步 link5 的坐标系关系：z = 父的 y，y = -父的 z，x = 父的 x ---
i5 = find(strcmp({spec.name}, 'link5'), 1);
ps = chainFK(spec, cfg.base, zeros(1, nJt));
R5 = ps{i5+1}.R;  R4 = ps{i5}.R;
% 把 link5 的三轴表达在 link4 的坐标系下
e5 = R4.' * R5;                 % link5 相对 link4 的姿态
fprintf('  %s 相对 %s 的姿态 = \n', spec(i5).name, spec(i5-1).name); disp(round(e5,6));
assert(norm(e5*[0;1;0] - [0;0;-1]) < 1e-12, 'link5 的 y 轴应 = link4 的 -z');
assert(norm(e5*[0;0;1] - [0;1;0])  < 1e-12, 'link5 的 z 轴应 = link4 的 y');
assert(norm(e5*[1;0;0] - [1;0;0])  < 1e-12, 'link5 的 x 轴应 = link4 的 x');
fprintf('  ✓ link5.z = link4.y、link5.y = -link4.z、link5.x = link4.x（原点重合）\n');
assert(norm(ps{i5+1}.t - ps{i5}.t) < 1e-12, 'link5 原点应与 link4 重合');
% link5 的转动轴（自身 z）在整机系下 = link4 的 y 方向
ax5 = R5 * (spec(i5).axis(:)/norm(spec(i5).axis));
fprintf('  joint5 轴(自身z) 在整机系 = %s （应为 link4 的 y = %s）\n', ...
        mat2str(round(ax5.',4)), mat2str(round((R4*[0;1;0]).',4)));
assert(norm(ax5 - R4*[0;1;0]) < 1e-12, 'joint5 轴应等于 link4 的 y 方向');
fprintf('  ✓ joint5 绕自身 z（= link4 的 y）自由转动\n');

% --- 断言 I：第七步 gripper 沿 link5 的 z 方向 211mm、无相对旋转 ---
ig = find(strcmp({spec.name}, 'gripper'), 1);
ps = chainFK(spec, cfg.base, zeros(1, nJt));
offInParent = ps{ig}.R.' * (ps{ig+1}.t - ps{ig}.t);   % 位移在 link5 系下表达
fprintf('  gripper 相对 link5 的位移(在 link5 系下) = %s mm，应为 [0 0 211]\n', ...
        mat2str(round(offInParent.',3)));
assert(norm(offInParent - [0;0;211]) < 1e-9, 'gripper 应沿 link5 的 z 方向 211mm');
dg = max(abs(ps{ig}.R.' * ps{ig+1}.R - eye(3)), [], 'all');
assert(dg < 1e-12, 'gripper 相对 link5 应无旋转');
fprintf('  ✓ gripper 沿 link5.z 211mm、无相对旋转（fixed 刚接）\n');
% gripper 原点在 joint5 轴上 ⇒ 只转 joint5 不移动它
qg = zeros(1, nJt); qg(6) = 77;
psA = chainFK(spec, cfg.base, zeros(1, nJt));
psB = chainFK(spec, cfg.base, qg);
fprintf('  joint5 转 77° 后 gripper 原点位移 = %.3g mm（应在 joint5 轴上 ⇒ ~0）\n', ...
        norm(psB{ig+1}.t - psA{ig+1}.t));
assert(norm(psB{ig+1}.t - psA{ig+1}.t) < 1e-9, 'joint5 转动不应移动在轴上的点');
js = jsonencode(P0);
fprintf('  JSON %.0f KB，往返解析 %s\n', numel(js)/1024, ...
        string(~isempty(jsondecode(js))));
fprintf('  结点列表: %s\n', strjoin({P0.nodes.name}, ' | '));

% --- 断言 F：纯 MATLAB 正解 chainFK 与 RST 交叉验证 ---
% HTML 里的 JS 正解与 chainFK 是同一套递推；验证了 chainFK，JS 那套才有依据
limAll = zeros(P0.meta.nJoint, 2); k = 0;
for i = 1:numel(spec)
    if strcmpi(spec(i).type, 'fixed'), continue; end
    k = k + 1;
    if isempty(spec(i).limits), limAll(k,:) = [-180 180];
    else,                       limAll(k,:) = sort(spec(i).limits); end
end
armC = buildChain(spec);
% ★ RST 那边的正解现在【已经含安装朝向】（buildChain 把 base.yaw 当根部固定变换），
%   而 chainFK 的 base 参数是同一个来源 ⇒ 两边都已在整机系，这里【不再】补 Tw。
%   以前是 buildChain 不含 yaw、测试手动 Tw*Tr，现在反过来了；
%   两处都补就会转两次，是这次改造最容易犯的错。
%   （armC.BodyNames 里可能有根部安装体 base_mount，取链上真正的 body 名来比。）
lkC = find(~strcmp(armC.BodyNames, 'base_mount'));
assert(numel(lkC) == numel(spec), '链上 body 数应与 spec 一致');
rng(7); dP = 0; dR = 0;
for t = 1:20
    qd = limAll(:,1).' + rand(1, P0.meta.nJoint).*(limAll(:,2)-limAll(:,1)).';
    ps = chainFK(spec, cfg.base, qd);
    for i = 1:numel(spec)
        Tr = getTransform(armC, deg2rad(qd), armC.BodyNames{lkC(i)});
        % 坑 1：ps{i+1}.t 是 3x1、Tr(1:3,4) 也是 3x1；若一个写成 .' 就变 3x3，norm 成 Frobenius 范数
        % 坑 2：Tr 是米、ps 是毫米，必须 *1000，否则 z=75 那种点会报 ~75 的假误差
        pRst = Tr(1:3,4)*1000;
        if t == 1 && i == numel(spec)
            fprintf('  样例 q=[%.1f %.1f]deg  %s: chainFK %s | RST %s mm\n', qd, ...
                    spec(i).name, mat2str(round(ps{i+1}.t(:).',4)), mat2str(round(pRst.',4)));
        end
        dP = max(dP, norm(ps{i+1}.t(:) - pRst));
        dR = max(dR, max(abs(ps{i+1}.R - Tr(1:3,1:3)), [], 'all'));
    end
end
fprintf('  chainFK vs RST getTransform（20 组随机构型 × %d 个 body）：位置 %.3g mm，姿态 %.3g\n', ...
        numel(spec), dP, dR);
assert(dP < 1e-9 && dR < 1e-12, 'chainFK 与 RST 正解不一致');
fprintf('  ✓ JS/HTML 用的那套正解递推与 RST 完全一致\n');

fprintf('\n=========== 2 三关节合成链回归（月牙链路） ===========\n');
s3 = struct('name', {}, 'joint', {}, 'type', {}, 'axis', {}, 'offset', {}, 'limits', {});
s3(1) = mk('base_link', 'base_joint', 'revolute', [0 0 1],  [0 0 0],   []);
s3(2) = mk('link1',     'joint1',     'revolute', [1 0 0],  [0 0 75],  [0 122]);
s3(3) = mk('link2',     'joint2',     'revolute', [0 0 1],  [0 200 0],[0 150]);
s3(4) = mk('link3',     'joint3',     'revolute', [1 0 0],  [0 0 300],[-120 120]);
s3(5) = mk('tool',      'tool_fix',   'fixed',    [0 0 1],  [0 150 0],[]);
% 几何是随手编的，只为验证全链路
cfg3 = cfg; cfg3.spec = s3; cfg3.name = '回归用链（含月牙）';
cfg3.crescent = struct('on', true, 'pair', [2 3], 'R_OUT', 80, 'R_IN', 60, 'CY2', 45);
tic; P3 = armFeasData(cfg3); t3 = toc;
fprintf('armFeasData: %.3f s | 关节数 %d | 月牙内占比 %.2f%% | 体素 %d -> %d | 半径 %.0f~%.0f mm\n', ...
        t3, P3.meta.nJoint, P3.stats.feasRate, P3.stats.voxBox, P3.stats.voxCres, ...
        P3.stats.rMin, P3.stats.rMax);
fprintf('  结点: %s\n', strjoin({P3.nodes.name}, ' | '));
assert(P3.meta.nJoint == 4 && ~isempty(P3.cloud.cres), '合成链应产生点云');
assert(P3.stats.feasRate > 0 && P3.stats.feasRate < 100, '月牙应过滤掉一部分');
assert(P3.stats.rMax > P3.stats.rMin, '合成链末端的可达半径应有分布');
fprintf('  ✓ 全链路（采样→月牙过滤→正解→坐标系→统计）正常\n');

fprintf('\n=========== 3 界面（无头 uifigure + uihtml） ===========\n');
appOK = false; htmlFile = '';
try
    app = armFeasApp();
    appOK = true;
    fprintf('armFeasApp 构造 OK（关节数 %d，控件字段 %d）\n', ...
            app.P.meta.nJoint, numel(fieldnames(app.C)));
    for k = 1:5
        switch k
            case 1, app.C.bz.Value = 350;                 nm = 'Base 抬高 z=350';
            case 2, app.C.hi(2).Value = 100;              nm = 'joint1 上限改 100';
            case 3, app.C.ax(2).Value = '+Z';             nm = 'joint1 轴向改 +Z';
            case 4, app.C.ax(2).Value = '+X'; app.C.hi(2).Value = 122; nm = '复位';
            case 5, app.C.nSample.Value = '1e4 (快)';     nm = '采样 1e4';
        end
        app.refresh();
        fprintf('  [%-18s] -> 关节数 %d | 结点 %d | err="%s"\n', ...
                nm, app.P.meta.nJoint, numel(app.P.nodes), app.LastError);
    end
    fprintf('  uihtml 回读: Data 类型 %s | 字段 %d\n', class(app.H.Data), ...
            numel(fieldnames(app.H.Data)));
    pause(2.0);
    fprintf('  uihtml 双向通道: HTMLSaid = [%s] (无头下 CEF 不加载，空属正常)\n', ...
            strtrim(evalc('disp(app.HTMLSaid)')));
    appClean = armFeasApp();
    htmlFile = appClean.export();
    d = dir(htmlFile);
    fprintf('已导出（固定文件） %s (%.0f KB)\n', htmlFile, d.bytes/1024);
catch ME
    pass = false;
    fprintf('界面/导出失败: %s\n  at %s:%d\n', ME.message, ME.stack(1).name, ME.stack(1).line);
end

fprintf('\n=========== 摘要 ===========\n');
fid = fopen(fullfile(here,'..','..','docs','data','t2_app_summary.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, 't2_app_smoke 摘要  %s\n', datestr(now,'yyyy-mm-dd HH:MM:SS')); %#ok<DATST,TNOW1>
fprintf(fid, '1 当前链: %s | 关节数 %d | 末端 %s @ %s mm\n', ...
        strjoin({spec.name}, '->'), P0.meta.nJoint, ee.name, mat2str(ee.o));
fprintf(fid, '2 合成链回归: 关节 %d, 月牙内 %.2f%%, 体素 %d->%d, 半径 %.0f~%.0f mm, %.3f s\n', ...
        P3.meta.nJoint, P3.stats.feasRate, P3.stats.voxBox, P3.stats.voxCres, ...
        P3.stats.rMin, P3.stats.rMax, t3);
fprintf(fid, '3 界面构造 %s；导出 %s\n', string(appOK), htmlFile);
fprintf(fid, '4 渲染验证：Edge --headless --screenshot 核对 t2_html_render_check.png\n');
fclose(fid);
fprintf('已保存 t2_app_summary.txt | 总体 %s\n', string(pass));

% ---------------- 局部函数 ----------------
function s = mk(name, joint, type, axis, offset, limits)
s = struct('name', name, 'joint', joint, 'type', type, ...
           'axis', axis, 'offset', offset, 'limits', limits);
end
