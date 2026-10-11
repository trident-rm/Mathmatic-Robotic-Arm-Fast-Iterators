%% t1b_debug_gik.m — 定位 GIK 为什么全 0
clc
here = fileparts(mfilename('fullpath'));
addpath(here);

R_OUT = 80; R_IN = 60; CY2 = 45;
limDeg = [-150 150; -80 80; -80 80];
arm = makeArm([0 0.290 0.402 0.176], [0 0 1; 0 -1 0; 0 -1 0], limDeg);
ee = 'tool';
fprintf('DataFormat = %s\n', arm.DataFormat);

% 取一个已知可行的位形，算出它的末端位置当目标
qTrue = [10, -30, -40];                       % 度
p = tform2trvec(getTransform(arm, deg2rad(qTrue), ee));
fprintf('目标位置 (m): %s   (由 q=[%s] 生成)\n', mat2str(round(p,4)), num2str(qTrue));

gik = generalizedInverseKinematics('RigidBodyTree', arm, ...
                                   'ConstraintInputs', {'position','joint'});
pTgt = constraintPositionTarget(ee);
jBnd = constraintJointBounds(arm);
jBnd.Bounds = deg2rad(limDeg);
fprintf('SolverAlgorithm = %s\n', gik.SolverAlgorithm);

for tol = [0.001 0.01]
    pTgt.PositionTolerance = tol;
    q0 = deg2rad([0 0 0]);
    try
        [qS, info] = gik(q0, pTgt, jBnd);
        fprintf('tol=%g -> class(qS)=%s\n', tol, class(qS));
        if isstruct(qS)
            disp(qS)
        else
            fprintf('  qS = %s (deg %s)\n', mat2str(round(qS,4)), mat2str(round(rad2deg(qS),2)));
        end
        disp(info)
    catch ME
        fprintf('tol=%g -> 异常: %s\n', tol, ME.message);
        for k=1:numel(ME.stack), fprintf('    %s : %d\n', ME.stack(k).name, ME.stack(k).line); end
    end
end
