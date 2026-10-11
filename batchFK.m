function [eePos, tform] = batchFK(robot, cfg, eeName)
%BATCHFK 批量正运动学（rigidBodyTree），比 getTransform 循环快两个量级
%
%   [eePos, tform] = batchFK(robot, cfg)
%   [eePos, tform] = batchFK(robot, cfg, eeName)
%
% 输入
%   robot  : rigidBodyTree（DataFormat='row'）
%   cfg    : N x n 关节位形【弧度】
%   eeName : 末端 body 名，默认取最后一个 body
%
% 输出
%   eePos  : N x 3 末端位置（米）
%   tform  : 4 x 4 x (numBodies+1) x N 全部 body 的位姿
%
% 内部走 RST 自己的 fastrbt 批量内核（generateRobotWorkspace 用的就是它），
% 失败时自动退回 getTransform 循环。

if nargin < 3 || isempty(eeName)
    eeName = robot.BodyNames{end};
end

try
    kinModel = robotics.manip.internal.fastrbt.RigidBodyTreeKinematics(robot);
    tform = forwardKinematics(kinModel, cfg);
    idx = robotics.manip.internal.RigidBodyTreeUtils.findBodyIndexByName(robot, eeName);
    eePos = robotics.internal.tform2trvec(tform(:,:,idx+1,:));
    eePos = reshape(eePos, [], 3);
catch ME
    warning('batchFK:fallback', '快速内核不可用(%s)，退回循环', ME.message);
    N = size(cfg, 1);
    eePos = zeros(N, 3);
    tform = [];
    for i = 1:N
        Ti = getTransform(robot, cfg(i,:), eeName);
        eePos(i,:) = tform2trvec(Ti);
    end
end
end
