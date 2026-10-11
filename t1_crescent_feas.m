%% t1_crescent_feas.m — 用 RST 算"带月牙耦合限位"的可行性集（全程不用碰撞检测）
%
% 回答的问题：
%   只给"杆 + 关节 + 坐标系 + 角度限位"（不给任何碰撞几何、不做碰撞检测），
%   RST 能不能算出带月牙限位的机械臂可行性集？
%
% 三种做法，全部只依赖 RST 的运动学与 IK：
%   A 区间盒          generateRobotWorkspace 原生（只认 PositionLimits 的矩形）
%   B 采样 + 关节角过滤  自采样后在【关节角空间】用月牙判据剔除（不碰几何）
%   C 重参数化        用 crescentBounds 把月牙变成"盒"，任何只认盒的工具都能用
% 外加：
%   D 目标可行性判定   generalizedInverseKinematics(位置约束+关节区间) 多起点
%                     → 再用月牙判据过滤，回答"这个点到底能不能到"
%
% 运行：cd 到本文件夹，输入  t1_crescent_feas
% 输出：t1_crescent_jointspace.png / t1_crescent_taskspace.png / t1_crescent_summary.txt

clc; close all;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(here);

%% ==================== 0  限位参数（全部是"角度"） ====================
R_OUT = 80;  R_IN = 60;  CY2 = 45;        % 度：月牙 = 大盘80 减 偏心小盘60@(0,45)
J1LIM = [-150 150];                       % 腰 yaw 普通区间限位
BJ2   = [-80 80];                         % pitch1 的区间外包
BJ3   = [-80 80];                         % pitch2 的区间外包
limDeg = [J1LIM; BJ2; BJ3];
ee = 'tool';
fprintf('==== 0 限位参数 ====\n');
fprintf('月牙: 大盘 R=%g - 偏心小盘 R=%g @ (0,%g)  [度]\n', R_OUT, R_IN, CY2);
fprintf('区间外包盒: j2=[%g,%g] j3=[%g,%g] (j1=[%g,%g])\n', BJ2, BJ3, J1LIM);

%% ==================== 1  纯杆 + 关节建臂，零碰撞几何 ====================
arm = makeArm([0 0.290 0.402 0.176], [0 0 1; 0 -1 0; 0 -1 0], limDeg);
fprintf('\n==== 1 模型 ====\n');
fprintf('body: %s  (共 %d 个, 全部是刚性杆+转动关节, 无 collision geometry)\n', ...
        strjoin(arm.BodyNames, ' -> '), arm.NumBodies);

%% ==================== 2  方式A：RST 原生区间盒 ====================
N = 2e5;
rng(0);
tic; [Wbox, ~] = generateRobotWorkspace(arm, {}, 'MaxNumSamples', N, ...
                                        'IgnoreSelfCollision', 'off');
tA = toc;
fprintf('\n==== 2 方式A 区间盒(generateRobotWorkspace) ====\n');
fprintf('%d 点 / %.3f s  → 但它对月牙一无所知（认为整个矩形都可行）\n', size(Wbox,1), tA);

%% ==================== 3  方式B：自采样 + 关节角月牙过滤 ====================
qdeg = rand(N,3).*(limDeg(:,2)-limDeg(:,1)).' + limDeg(:,1).';      % N x 3 [度]
tic; g = crescentMargin(qdeg(:,2), qdeg(:,3), R_OUT, R_IN, CY2); tB = toc;
ok = g <= 0;
fprintf('\n==== 3 方式B 采样+关节角过滤 ====\n');
fprintf('判据耗时 %.4f s / %d 点 = %.3f us/点（纯角度运算，与几何无关）\n', tB, N, 1e6*tB/N);
fprintf('可行 %d / %d = %.2f%%\n', nnz(ok), N, 100*nnz(ok)/N);
[Wc, ~] = batchFK(arm, deg2rad(qdeg(ok,:)),  ee);
[Wd, ~] = batchFK(arm, deg2rad(qdeg(~ok,:)), ee);

%% ==================== 4  方式C：重参数化，把月牙变成盒 ====================
x = -R_OUT + 2*R_OUT*rand(N,1);
[lo, hi] = crescentBounds(x, R_OUT, R_IN, CY2);
y = lo + rand(N,1).*(hi - lo);
qC = [J1LIM(1) + diff(J1LIM)*rand(N,1), x, y];
gC = crescentMargin(qC(:,2), qC(:,3), R_OUT, R_IN, CY2);
fprintf('\n==== 4 方式C 重参数化 ====\n');
fprintf('采 %d 点，最大裕度 max(g) = %.3g（<=0 即全部落在月牙内，0 浪费）\n', N, max(gC));
fprintf('可行率 %.2f%%  ← 对比方式B的 %.2f%%，多出来的全是白采的\n', ...
        100*nnz(gC<=0)/N, 100*nnz(ok)/N);
[Wr, ~] = batchFK(arm, deg2rad(qC), ee);

%% ==================== 5  月牙到底砍掉了多少 ====================
[G2, G3] = meshgrid(BJ2(1):0.25:BJ2(2), BJ3(1):0.25:BJ3(2));
Gg = crescentMargin(G2, G3, R_OUT, R_IN, CY2);
areaFrac = mean(Gg(:) <= 0);
vox = @(W) size(unique(floor(W/0.05), 'rows'), 1);
vBox = vox(Wbox); vCres = vox(Wc);
fprintf('\n==== 5 月牙的影响 ====\n');
fprintf('关节空间: 区间盒内只有 %.2f%% 的面积真的可行（月牙砍掉 %.2f%%）\n', ...
        100*areaFrac, 100*(1-areaFrac));
fprintf('任务空间: 同样 %d 次采样, 体素(50mm) 盒 %d 个 -> 月牙 %d 个 (剩 %.2f%%)\n', ...
        N, vBox, vCres, 100*vCres/vBox);

%% ==================== 6  目标可行性判定（GIK 多起点 + 月牙过滤） ====================
fprintf('\n==== 6 目标点能不能到（GIK 多起点 + 月牙过滤） ====\n');
gik = generalizedInverseKinematics('RigidBodyTree', arm, ...
                                   'ConstraintInputs', {'position','joint'});
pTgt = constraintPositionTarget(ee);
pTgt.PositionTolerance = 0.001;                     % 1 mm
jBnd = constraintJointBounds(arm);
jBnd.Bounds = deg2rad(limDeg);

nT = 30; ns = 3;
grp = {'真可行(月牙内位形产生)', Wc; '仅盒可行(月牙外位形产生)', Wd};
res = zeros(2,3);                                   % [达标, 盒内解存在, 月牙内解存在]
for gi = 1:2
    P = grp{gi,2};
    P = P(randperm(size(P,1), min(nT,size(P,1))), :);
    for k = 1:size(P,1)
        pTgt.TargetPosition = P(k,:);
        gotBox = false; gotCres = false;
        for s = 1:ns
            q0 = deg2rad(rand(1,3).*(limDeg(:,2)-limDeg(:,1)).' + limDeg(:,1).');
            try
                % 注意 GIK 的调用约定：初始猜测在最前，其后按 ConstraintInputs 顺序给约束
                qS = qrow(gik(q0, pTgt, jBnd));
            catch
                break
            end
            if any(~isfinite(qS)), continue; end
            if norm(tform2trvec(getTransform(arm, qS, ee)) - P(k,:)) > 1e-3, continue; end
            gotBox = true;
            if crescentMargin(rad2deg(qS(2)), rad2deg(qS(3)), R_OUT, R_IN, CY2) <= 0
                gotCres = true; break
            end
        end
        res(gi,1) = res(gi,1) + 1;
        res(gi,2) = res(gi,2) + gotBox;
        res(gi,3) = res(gi,3) + gotCres;
    end
    fprintf('%s: %d 个目标 | 关节盒内可解 %d | 且满足月牙 %d\n', ...
            grp{gi,1}, res(gi,1), res(gi,2), res(gi,3));
end

%% ==================== 7  图 ====================
try
    ns_sub = min(4000, size(qdeg,1));
    sub = qdeg(randperm(size(qdeg,1), ns_sub), :);
    sub = sub(crescentMargin(sub(:,2), sub(:,3), R_OUT, R_IN, CY2) <= 0, :);

    f1 = figure('Visible','off','Position',[80 80 620 560]);
    contourf(G2, G3, Gg, [-60 -20 -1 0 1 20 60], 'LineColor', 'none'); hold on
    contour(G2, G3, Gg, [0 0], 'k-', 'LineWidth', 1.6);
    plot(sub(:,2), sub(:,3), '.', 'MarkerSize', 3, 'Color', [1 1 1]);
    plot([BJ2(1) BJ2(2) BJ2(2) BJ2(1) BJ2(1)], [BJ3(1) BJ3(1) BJ3(2) BJ3(2) BJ3(1)], ...
         'r--', 'LineWidth', 1.4);
    axis equal; grid on; xlabel('q_2 (deg)'); ylabel('q_3 (deg)');
    title({'关节空间: 月牙(填色) vs 区间盒(红虚线)', '白点=月牙内样本'});
    colorbar;
    exportgraphics(f1, fullfile(here, 't1_crescent_jointspace.png'), 'Resolution', 120);

    f2 = figure('Visible','off','Position',[80 80 700 560]);
    plot(Wbox(1:2:end,1)*1000, Wbox(1:2:end,3)*1000, '.', 'MarkerSize', 2, ...
         'Color', [.78 .78 .78]); hold on
    plot(Wc(1:2:end,1)*1000, Wc(1:2:end,3)*1000, '.', 'MarkerSize', 2, 'Color', [0 .35 .8]);
    axis equal; grid on; xlabel('x (mm)'); ylabel('z (mm)');
    title('任务空间 x-z 剖面: 灰=区间盒, 蓝=月牙内');
    legend({'区间盒', '月牙内'}, 'Location', 'best');
    exportgraphics(f2, fullfile(here, 't1_crescent_taskspace.png'), 'Resolution', 120);
    fprintf('\n已保存 t1_crescent_jointspace.png / t1_crescent_taskspace.png\n');
catch ME
    fprintf('[绘图跳过] %s\n', ME.message);
end

%% ==================== 8  摘要 ====================
fid = fopen(fullfile(here, 't1_crescent_summary.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, 't1_crescent_feas 摘要  %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS')); %#ok<DATST,TNOW1>
fprintf(fid, 'MATLAB %s (%s)\n', version, version('-release'));
fprintf(fid, '模型: 纯杆+关节 rigidBodyTree, 无任何 collision geometry\n');
fprintf(fid, '月牙: 大盘%g - 偏心小盘%g@(0,%g) 度\n', R_OUT, R_IN, CY2);
fprintf(fid, '区间盒: j2=[%g,%g] j3=[%g,%g] j1=[%g,%g]\n', BJ2, BJ3, J1LIM);
fprintf(fid, 'A generateRobotWorkspace: %d 点 %.3f s (只认区间盒)\n', size(Wbox,1), tA);
fprintf(fid, 'B 采样+角度过滤: 判据 %.3f us/点, 可行率 %.2f%%\n', 1e6*tB/N, 100*nnz(ok)/N);
fprintf(fid, 'C 重参数化: max(g)=%.3g, 可行率 %.2f%% (0 浪费)\n', max(gC), 100*nnz(gC<=0)/N);
fprintf(fid, 'D 关节空间可行面积占比 %.2f%%\n', 100*areaFrac);
fprintf(fid, 'E 任务空间体素 盒 %d -> 月牙 %d (%.2f%%)\n', vBox, vCres, 100*vCres/vBox);
fprintf(fid, 'F GIK 多起点+月牙过滤:\n');
for gi = 1:2
    fprintf(fid, '   %s: n=%d 盒内可解=%d 且月牙满足=%d\n', ...
            grp{gi,1}, res(gi,1), res(gi,2), res(gi,3));
end
fclose(fid);
fprintf('已保存 t1_crescent_summary.txt\n');
fprintf('\n==================== 结束 ====================\n');

%% ---------------- 本地函数 ----------------
function q = qrow(q)
%QROW GIK 返回值统一成行向量（DataFormat 为 struct 时也能用）
if isstruct(q)
    c = struct2cell(q);
    q = cell2mat(c)';
else
    q = q(:)';
end
end
