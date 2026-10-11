%% t4_dump_check.m — 打印 MATLAB 参考实现的各坐标系轴向（用于和浏览器截图对表）
clc
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(here);

spec = armSpec();
nJ  = sum(~strcmpi({spec.type}, 'fixed'));
q0  = zeros(1, nJ);                % 与 armBuilder.html 的初始 Q 相同（全零位）
ps = chainFK(spec, struct('xyz',[0 0 0],'yaw',0), q0);

fprintf('q0(deg) = %s\n', strjoin(string(q0), ', '));
fprintf('\n--- MATLAB chainFK 参考值 ---\n');
fprintf('（poses{1} 是 base，第 i 个 body 在 poses{i+1}）\n');
T0 = ps{1};
fprintf('\nbase(臂安装)  (joint: -)\n  o = %s\n', mat2str(round(T0.t.',2)));
for i = 1:numel(spec)
    p = ps{i+1};
    jn = '-';
    if isfield(spec(i),'joint') && ~isempty(spec(i).joint), jn = spec(i).joint; end
    fprintf('\n%s  (joint: %s)\n', spec(i).name, jn);
    fprintf('  o = %s\n', mat2str(round(p.t.',2)));
    fprintf('  X = %s\n', mat2str(round(p.R(:,1).',5)));
    fprintf('  Y = %s\n', mat2str(round(p.R(:,2).',5)));
    fprintf('  Z = %s\n', mat2str(round(p.R(:,3).',5)));
end
