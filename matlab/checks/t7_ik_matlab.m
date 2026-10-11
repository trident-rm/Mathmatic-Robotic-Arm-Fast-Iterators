function t7_ik_matlab()
% t7_ik_matlab —— 用 MATLAB Robotics System Toolbox 的 inverseKinematics
% 复核 armBuilder.html 里那份自写 IK 的结论（同一批位姿）
% 读 t7_ik_js.csv（由 t7_ik_node.js 生成），对每个位姿独立求解，比较双方结论。
d = fileparts(mfilename('fullpath'));
T = readtable(fullfile(d,'..','..','docs','data','t7_ik_js.csv'));
spec = armSpec();
arm  = buildChain(spec);
nJ   = sum(~strcmpi({spec.type},'fixed'));
eeName = spec(end).name;

ik = inverseKinematics('RigidBodyTree', arm);
try, ik.SolverParameters.AllowRandomRestart = true; catch, end
try, ik.SolverParameters.MaxIterations = 800;        catch, end
q0 = homeConfiguration(arm);
weights = [1 1 1 0.8 0.8 0.8];

rolls = 0:45:315;                 % 对齐轴自转是自由的，扫一圈模拟"只要 y 轴对上即可"
nOK = 0; mErr = nan(height(T),1); mOK = false(height(T),1);
for i = 1:height(T)
    P = struct('x',T.x(i),'y',T.y(i),'z',T.z(i),'th',T.th(i),'ph',T.ph(i),'al',T.al(i));
    Eb = taskToBase([P.x P.y P.z]);            % 与 JS 完全相同的映射
    eB = dirToBase(coreFrameJS(P));
    yt = -eB(:); yt = yt/norm(yt);
    best = inf; got = false;
    for roll = rolls
        tmp = [1;0;0]; if abs(dot(tmp,yt))>0.9, tmp=[0;1;0]; end
        x0 = cross(tmp,yt); x0 = x0/norm(x0); z0 = cross(x0,yt);
        Rr = axang2rotm([yt' roll*pi/180]) * [x0 yt z0];
        Tt = eye(4); Tt(1:3,1:3) = Rr; Tt(1:3,4) = Eb(:);
        [cfg, ~] = ik(eeName, Tt, weights, q0);
        Tnow = getTransform(arm, cfg, eeName);
        e = norm(Tnow(1:3,4) - Eb(:));
        ynow = Tnow(1:3,2)/norm(Tnow(1:3,2));
        ang = acos(max(-1,min(1,dot(ynow,yt))));
        if e < best, best = e; end
        if e < 3 && ang < 0.2, got = true; break; end
    end
    mErr(i) = best; mOK(i) = got; if got, nOK = nOK + 1; end
    if mod(i,50)==0, fprintf('  ... %d/%d\n', i, height(T)); end
end

jsOK = logical(T.ok);
fprintf('\n===== 交叉验证结果（%d 个三级任务域位姿）=====\n', height(T));
fprintf('  armBuilder.html 自写 IK : 可解 %d/%d = %.1f%%\n', sum(jsOK), numel(jsOK), 100*mean(jsOK));
fprintf('  MATLAB RST inverseKinematics : 可解 %d/%d = %.1f%%\n', nOK, height(T), 100*nOK/height(T));
fprintf('  两者结论一致 : %d/%d = %.1f%%\n', sum(jsOK==mOK), height(T), 100*mean(jsOK==mOK));
fprintf('  ── 只在 JS 里可解 : %d 个（RST 说不行）\n', sum(jsOK & ~mOK));
fprintf('  ── 只在 RST 里可解 : %d 个（★ 说明自写 IK 漏解）\n', sum(~jsOK & mOK));
fprintf('  RST 最优位置误差 中位 = %.2f mm\n', median(mErr,'omitnan'));
writetable(table((1:height(T))', jsOK, mOK, mErr, 'VariableNames',{'idx','js_ok','matlab_ok','matlab_err'}), ...
           fullfile(d,'..','..','docs','data','t7_ik_compare.csv'));
fprintf('  对照明细已存 t7_ik_compare.csv\n');
end

% ---- 与 JS 一致的坐标映射（MAP.swap = true, 底盘高 170）----
function p = taskToBase(p), p = [-p(1), -p(2), p(3)-170]; end
function d = dirToBase(d),  d = [-d(1), -d(2),  d(3)];     end
function e = coreFrameJS(P)
    th = P.th*pi/180; ph = P.ph*pi/180;
    e = [sin(ph)*cos(th), sin(ph)*sin(th), cos(ph)];
end
