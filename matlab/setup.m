function setup()
%SETUP 把本仓库的 MATLAB 代码加进搜索路径（新克隆下来先跑一次这个）
%
%   >> run('<仓库>/matlab/setup.m')     % 或者在 matlab/ 目录下直接  setup
%
% 模块化之后 .m 分在三个目录里，所以必须先加路径：
%     matlab/model   臂型定义 + 建树 + 正解（armSpec / buildChain / chainFK / rotDeg / makeArm）
%     matlab/core    可行性集计算与界面（armFeasApp / armFeasData / armFeasExport / batchFK / 月牙）
%     matlab/checks  无头自检与探针（t*.m，不需要建模时可以不看）
%
% 用法：
%     run('<仓库>/matlab/setup.m')
%     app = armFeasApp;
%
% 注意：本函数【不】切换当前目录，所以 t10_pack.js 之类的相对路径不受影响。

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end

addpath(fullfile(here, 'model'));
addpath(fullfile(here, 'core'));
addpath(fullfile(here, 'checks'));

fprintf('已加入搜索路径：\n');
fprintf('  %s\n', fullfile(here, 'model'));
fprintf('  %s\n', fullfile(here, 'core'));
fprintf('  %s\n', fullfile(here, 'checks'));
fprintf('\n可以开始：  app = armFeasApp;\n');
end
