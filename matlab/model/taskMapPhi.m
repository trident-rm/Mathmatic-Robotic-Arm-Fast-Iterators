function phi = taskMapPhi(kind)
%TASKMAPPHI 任务系 → 整车系的绕 Z 角度（°），由 taskMap 约定决定
%
%   phi = taskMapPhi('swap')     -> 180
%   phi = taskMapPhi('noSwap')   ->  90
%
%   网页 04-config.js 里有一张同名表 TASKMAP_PHI，两边必须一致。
%   单独成文件是为了不在 taskToBase/baseToTask/dirToBase 里各写一份。

if nargin < 1 || isempty(kind), kind = 'swap'; end
switch lower(char(kind))
    case 'noswap', phi = 90;
    otherwise,     phi = 180;      % 'swap' 是默认
end
end
