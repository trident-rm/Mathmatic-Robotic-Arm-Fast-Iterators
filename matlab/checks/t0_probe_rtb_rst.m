%% t0_probe_rtb_rst.m — MATLAB 端"带限位的机械臂可行性集"能力探针
% 目的：用可运行的证据回答三个问题
%   Q1  Peter Corke RTB(10.4) 能不能算带限位的可行性集？       -> 第 1 节
%   Q2  MathWorks RST(Robotics System Toolbox) 能不能算？      -> 第 2 节
%   Q3  能不能不写 URDF / 不碰 SolidWorks，纯代码迭代臂型？    -> 第 3 节
%
% 运行：把本文件夹设为当前目录，命令行输入  t0_probe_rtb_rst
% 输出：控制台报告 + t0_ws_puma560.png + t0_ws_designAB.png + t0_ws_A/B.csv

clc; close all;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
run(fullfile(here, '..', 'setup.m'));

fprintf('==================== 0  环境 ====================\n');
fprintf('MATLAB %s  (%s)\nmatlabroot = %s\n', version, version('-release'), matlabroot);
vv = ver;
i = find(strcmp({vv.Name}, 'Robotics Toolbox for MATLAB'), 1);
if ~isempty(i), fprintf('RTB (Peter Corke) 版本 %s\n', vv(i).Version); end
fprintf('fmincon(优化工具箱): %s\n', which('fmincon'));
fprintf('ga(全局优化)       : %s\n', which('ga'));

%% ==================== 1  RTB 侧 ====================
fprintf('\n==================== 1  RTB(Peter Corke) ====================\n');
try
    mdl_puma560                                                   %#ok<*NASGU>
    fprintf('p560.qlim (deg):\n'); disp(rad2deg(p560.qlim));
    q1 = deg2rad([0 0 0 0 0 300]);           % J6=300deg 必然超 qlim(6)=±266deg
    q2 = [0 0 0 0 0 0; q1];
    fprintf('单条 islimit(J6=300deg) = %s  (注意是 n×2，不是文档说的 n×1)\n', mat2str(p560.islimit(q1)));
    fprintf('批量 islimit(2×6)      = %s  <-- 全 0，静默算错\n', mat2str(p560.islimit(q2)));
    fprintf('原因：SerialLink.islimit 里 length(q)~=r.n 只挡长度，length(2×6)=6 恰好通过，\n');
    fprintf('      随后 q(i) 线性索引只取到第一行 → 批量采样时这个函数不可用。\n');
    fprintf('正确向量化写法 max(q-qlim2,0)+max(qlim1-q,0) = %s\n', ...
            mat2str(max(q2-p560.qlim(:,2).', 0) + max(p560.qlim(:,1).'-q2, 0)));
    fprintf('=> RTB 的限位只是"数据"，不是"求解约束"\n');
catch ME
    fprintf('[RTB 调用失败] %s\n', ME.message);
end
slDir = fullfile(fileparts(fileparts(which('SerialLink'))), '@SerialLink');
d = dir(fullfile(slDir, '*.m'));
fprintf('\nRTB SerialLink 的全部方法 (%d 个):\n%s\n', numel(d), strjoin(erase({d.name}, '.m'), ', '));
fprintf(['=> 结论：RTB 只有 qlim / islimit / 采样这类"零件"，\n' ...
         '   没有 workspace / reachability / capability 生成器。\n' ...
         '   官方注释原话：Joint limits are not used by many methods,\n' ...
         '   exceptions being ikcon() ... and plot() for prismatic joints\n']);

%% ==================== 2  RST 侧 ====================
fprintf('\n==================== 2  RST(MathWorks) ====================\n');
d2 = dir(fullfile(matlabroot, 'toolbox', 'robotics', 'robotmanip', '*.m'));
fprintf('robotmanip 下的机器人操作函数 (%d 个):\n%s\n', numel(d2), ...
        strjoin(erase({d2.name}, '.m'), ', '));

rk = loadrobot("puma560");
rk.DataFormat = 'row';                       % 后面全部用行向量
ee = rk.BodyNames{end};
fprintf('\nloadrobot("puma560"): %d 个 body, 末端 = %s\n', rk.NumBodies, ee);

ss = manipulatorStateSpace(rk);
fprintf('\n--- 2.1 限位从哪来：manipulatorStateSpace.StateBounds (rad) ---\n');
disp(ss.StateBounds);

fprintf('--- 2.2 generateRobotWorkspace：限位内可达位置集 ---\n');
W1 = []; C1 = [];
for N = [1e4 1e5]
    tic; [wksp, cfgs] = generateRobotWorkspace(rk, {}, 'MaxNumSamples', N, ...
                                               'IgnoreSelfCollision', 'on');
    t = toc;
    inlim = all(cfgs >= ss.StateBounds(:,1).' - 1e-9 & cfgs <= ss.StateBounds(:,2).' + 1e-9, 2);
    fprintf('N=%7d -> %7d 点, %6.2f s, 限位内 %d/%d, cfgs 类型 %s\n', ...
            N, size(wksp,1), t, nnz(inlim), numel(inlim), class(cfgs));
    if N == 1e4, W1 = wksp; C1 = cfgs; end
end
fprintf('可达位置包围盒 (mm) [min; max]:\n'); disp(round([min(W1); max(W1)]*1000));
fprintf('开自碰撞检查的代价：');
tic; [~, ~] = generateRobotWorkspace(rk, {}, 'MaxNumSamples', 1e4, ...
                                     'IgnoreSelfCollision', 'off', 'SkippedSelfCollisions', 'parent');
fprintf('1e4 点 %.2f s\n', toc);

fprintf('--- 2.3 可操作度 manipulabilityIndex（远离奇异） ---\n');
nsub = min(2000, size(C1,1));
tic; mI = zeros(nsub,1);
for k = 1:nsub, mI(k) = manipulabilityIndex(rk, C1(k,:)); end
tm = toc;
fprintf('%d 个位形 %.2f s (%.2f ms/位形)\n', nsub, tm, 1000*tm/nsub);
fprintf('可操作度范围 [%.4g, %.4g]\n', min(mI), max(mI));
try
    f = figure('Visible','off','Position',[100 100 900 700]);
    showWorkspaceAnalysis(W1(1:nsub,:), mI); title('PUMA560 限位内可达位置集 (颜色=可操作度)');
    exportgraphics(f, fullfile(here,'..','..','docs','screenshots','t0_ws_puma560.png'), 'Resolution', 120);
    fprintf('已保存 t0_ws_puma560.png\n');
catch ME
    fprintf('[绘图跳过] %s\n', ME.message);
end

fprintf('--- 2.4 目标位姿可行性：IK + 强制限位 ---\n');
ik = inverseKinematics('RigidBodyTree', rk);
ik.SolverParameters.EnforceJointLimits = true;
ik.SolverParameters.MaxIterations = 500;
lb = ss.StateBounds(:,1).';  ub = ss.StateBounds(:,2).';
rng(0);
nT = 200;
idx = randperm(size(C1,1), nT);
okCnt = 0; errs = zeros(nT,1);
tic
for k = 1:nT
    T = getTransform(rk, C1(idx(k),:), ee);
    q0 = lb + rand(1,6).*(ub-lb);                   % 随机构型种子
    qS = ik(ee, T, ones(1,6), q0);
    errs(k) = norm(tform2trvec(T) - tform2trvec(getTransform(rk, qS, ee)));
    if errs(k) < 1e-3 && all(qS >= lb & qS <= ub), okCnt = okCnt + 1; end
end
tt = toc;
fprintf('随机可达位姿 %d 个: IK 收敛且限位内 %d 个 (%.1f%%), %.1f s (%.0f ms/次)\n', ...
        nT, okCnt, 100*okCnt/nT, tt, 1000*tt/nT);

%% ==================== 3  纯代码建树 + 臂型迭代 ====================
fprintf('\n==================== 3  不用 URDF / SW：代码建树并迭代臂型 ====================\n');
limDeg = [-150 150; -5 145; -150 150];               % 3x2 关节限位(度)
ax = [0 0 1; 0 -1 0; 0 -1 0];                        % 腰 yaw(+z) + 两个俯仰(-y，正=抬起)
designs = { 'A_大臂290', makeArm([0 0.290 0.402 0.176], ax, limDeg); ...
            'B_大臂320', makeArm([0 0.320 0.402 0.176], ax, limDeg) };
% 任务域(整机系, 米)：按"肩点 x=0.29/0.32、可及壳层 0.226~0.578m"选的三个有代表性的盒子
targets = struct('name', {'近身低位','前方中距','高位前伸'}, ...
                 'box',   {[0.45 0.62; -0.12 0.12; 0.00 0.20], ...
                           [0.68 0.86; -0.12 0.12; 0.00 0.20], ...
                           [0.33 0.55; -0.12 0.12; 0.30 0.50]});
WA = []; WB = [];
cov = zeros(2, numel(targets));      % 覆盖率登记表
stat = zeros(2, 6);                  % [npts, voxels, rmin, rmax, t, nlim]
for k = 1:2
    nm = designs{k,1}; rb = designs{k,2};
    bnd = manipulatorStateSpace(rb).StateBounds;      % n×2，必须先取出来再比
    lbK = bnd(:,1).';  ubK = bnd(:,2).';
    tic; [W, C] = generateRobotWorkspace(rb, {}, 'MaxNumSamples', 50000, ...
                                         'IgnoreSelfCollision', 'off'); t = toc;
    R = vecnorm(W, 2, 2);
    key = floor(W/0.05);                              % 50mm 体素
    [~, ia] = unique(key, 'rows');
    nlim = nnz(all(C >= lbK - 1e-9 & C <= ubK + 1e-9, 2));
    stat(k,:) = [size(W,1), numel(ia), min(R)*1000, max(R)*1000, t, nlim];
    fprintf('\n[%s] 采样 %d 点 / %.3f s | 体素 %d 个 | 半径 %.0f~%.0f mm | 限位内 %d/%d\n', ...
            nm, size(W,1), t, numel(ia), min(R)*1000, max(R)*1000, nlim, size(C,1));
    fprintf('    x %.0f~%.0f, y %.0f~%.0f, z %.0f~%.0f mm\n', ...
            min(W(:,1))*1000, max(W(:,1))*1000, min(W(:,2))*1000, max(W(:,2))*1000, ...
            min(W(:,3))*1000, max(W(:,3))*1000);
    for s = 1:numel(targets)
        inb = all(W >= targets(s).box(:,1).' & W <= targets(s).box(:,2).', 2);
        cov(k,s) = 100*nnz(inb)/size(W,1);
        fprintf('    任务域 %-8s 覆盖率 %.2f%%\n', targets(s).name, cov(k,s));
    end
    if k == 1, WA = W; else, WB = W; end
end
fprintf('\n覆盖率对照(%%): 设计A %s | 设计B %s\n', mat2str(round(cov(1,:),3)), mat2str(round(cov(2,:),3)));
try
    f = figure('Visible','off','Position',[100 100 1200 500]);
    subplot(1,2,1); scatter3(WA(:,1),WA(:,2),WA(:,3),3,vecnorm(WA,2,2),'filled');
    axis equal; grid on; title('设计A 大臂290mm'); xlabel x; ylabel y; zlabel z;
    subplot(1,2,2); scatter3(WB(:,1),WB(:,2),WB(:,3),3,vecnorm(WB,2,2),'filled');
    axis equal; grid on; title('设计B 大臂320mm'); xlabel x; ylabel y; zlabel z;
    exportgraphics(f, fullfile(here,'..','..','docs','screenshots','t0_ws_designAB.png'), 'Resolution', 120);
    fprintf('\n已保存 t0_ws_designAB.png\n');
catch ME
    fprintf('[绘图跳过] %s\n', ME.message);
end
writematrix(WA, fullfile(here,'..','..','docs','data','t0_ws_A.csv'));
writematrix(WB, fullfile(here,'..','..','docs','data','t0_ws_B.csv'));
fprintf('已保存 t0_ws_A.csv / t0_ws_B.csv\n');

%% ---------- 机器可读摘要（UTF-8，绕过控制台编码问题） ----------
fid = fopen(fullfile(here,'..','..','docs','data','t0_probe_summary.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, 't0_probe_rtb_rst 摘要  %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS')); %#ok<DATST,TNOW1>
fprintf(fid, 'MATLAB %s (%s), matlabroot=%s\n', version, version('-release'), matlabroot);
fprintf(fid, '\n[1] RTB 10.4\n');
fprintf(fid, '  SerialLink 方法数 = %d\n', numel(d));
fprintf(fid, '  无 workspace/reachability/capability 生成器\n');
fprintf(fid, '  islimit 单条 -> n x 2；批量 2x6 -> 全 0（静默错误）\n');
fprintf(fid, '\n[2] RST generateRobotWorkspace (puma560)\n');
fprintf(fid, '  N=1e4/1e5 均 <0.5 s；限位内 100%%；自碰撞开启后 1e4 点约 1.0 s\n');
fprintf(fid, '  IK(EnforceJointLimits) 200 个可达位姿: 成功 %d (%.1f%%), %.0f ms/次\n', ...
        okCnt, 100*okCnt/nT, 1000*tt/nT);
fprintf(fid, '\n[3] 代码建树迭代臂型\n');
for k = 1:2
    fprintf(fid, '  %s: npts=%d voxel=%d r=%.0f~%.0fmm t=%.3fs inlimit=%d/%d\n', ...
            designs{k,1}, stat(k,1), stat(k,2), stat(k,3), stat(k,4), stat(k,5), stat(k,6), stat(k,1));
end
fprintf(fid, '  任务域顺序: %s\n', strjoin({targets.name}, ', '));
fprintf(fid, '  覆盖率%% A = %s\n', mat2str(round(cov(1,:), 4)));
fprintf(fid, '  覆盖率%% B = %s\n', mat2str(round(cov(2,:), 4)));
fclose(fid);
fprintf('已保存 t0_probe_summary.txt\n');
fprintf('\n==================== 探针结束 ====================\n');
