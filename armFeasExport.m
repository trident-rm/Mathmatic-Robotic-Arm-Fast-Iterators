function outFile = armFeasExport(P, outFile)
%ARMFEASEXPORT 把可行性集数据写成"单文件 HTML"，别人双击浏览器就能看（不需要 MATLAB）
%
%   outFile = armFeasExport(P)          % 固定写到 armFeas.html（反复覆盖同一份）
%   outFile = armFeasExport(P, outFile) % 写到指定文件
%
% 固定文件名是刻意的：浏览器里开着这一个页签，MATLAB 每次导出后
% 按 ⟳ 或勾上"自动刷新 3s"就能看到最新结果，不用每次换文件。
%
% P 为 armFeasData 的输出；模板 armview.html 与 uihtml 界面共用同一套渲染代码。

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
if nargin < 2 || isempty(outFile)
    outFile = fullfile(here, 'armFeas.html');      % ← 固定的一份 HTML
end

% 读模板（UTF-8）
fid = fopen(fullfile(here, 'armview.html'), 'r', 'n', 'UTF-8');
assert(fid > 0, 'armFeasExport: 找不到 armview.html');
tmpl = fread(fid, '*char').';
fclose(fid);

% 注入数据
inject = sprintf('<script>window.__ARM_DATA__ = %s;</script>', jsonencode(P));
html = strrep(tmpl, '<!--ARM_DATA-->', inject);

% 写文件（模板里已声明 <meta charset="utf-8">，浏览器会按 UTF-8 解析中文）
fid = fopen(outFile, 'w', 'n', 'UTF-8');
assert(fid > 0, 'armFeasExport: 无法写入 %s', outFile);
fwrite(fid, unicode2native(html, 'UTF-8'));      % 直接写字节，避免二次编码
fclose(fid);
end
