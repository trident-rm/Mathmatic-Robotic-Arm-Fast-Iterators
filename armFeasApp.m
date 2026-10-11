classdef armFeasApp < handle
%ARMFEASAPP 机械臂可行性集交互界面（左侧 MATLAB 控件 / 右侧 HTML 实时显示）
%
%   用法：
%     app = armFeasApp;            % 打开界面
%     app.refresh();               % 也可无界面调用
%     f = app.export();            % 导出单文件 HTML（别人不用 MATLAB 就能看）
%
%   模型来源：armSpec.m（逐步补充的链定义）
%     往 armSpec.m 里加一个 body/关节，界面里自动多出一行控件，无需改本文件。
%
%   依赖：Robotics System Toolbox；本文件夹的 armSpec/buildChain/batchFK/
%         crescentMargin/armFeasData/armview.html

    properties
        UIFigure
        Grid
        H                % uihtml 组件
        C = struct()     % 控件句柄
        P = []           % 最近一次算出的 payload
        LastError = ''
        HTMLSaid = []    % HTML 侧回调（验证 uihtml 双向通道）
        MovIdx = []      % 可动关节在 spec 中的下标
    end

    methods
        function app = armFeasApp()
            here = fileparts(mfilename('fullpath'));
            if isempty(here), here = pwd; end
            app.UIFigure = uifigure('Name', '机械臂可行性集 · RST + HTML', ...
                                    'Position', [60 60 1520 880]);
            app.Grid = uigridlayout(app.UIFigure, [1 2]);
            app.Grid.ColumnWidth = {390, '1x'};
            app.Grid.RowHeight = {'1x'};
            app.Grid.Padding = [8 8 8 8];
            app.Grid.ColumnSpacing = 10;

            pnl = uipanel(app.Grid, 'Title', '参数（改动即重算）');
            app.buildControls(pnl);

            app.H = uihtml(app.Grid);
            app.H.HTMLSource = fullfile(here, 'armview.html');
            app.H.DataChangedFcn = @(src, ev) app.onHTMLMsg(ev);

            app.attachCallbacks();
            app.refresh();
        end

        function refresh(app)
        %REFRESH 按当前控件算一遍可行性集并推到 HTML 面板
            cfg = app.readCfg();
            t0 = tic;
            try
                app.P = armFeasData(cfg);
                app.LastError = '';
            catch ME
                app.LastError = ME.message;
                warning('armFeasApp:compute', '计算失败: %s', ME.message);
                return
            end
            pushed = '未推送';
            try
                app.H.Data = app.P;
                pushed = '已推送 HTML';
            catch ME
                app.LastError = ME.message;
                pushed = '推送跳过(无头)';
                warning('armFeasApp:push', ...
                        '数据已算出但未推送到 HTML（无头模式下属正常）: %s', ME.message);
            end
            if isfield(app.C, 'lblStatus') && isvalid(app.C.lblStatus)
                if app.P.meta.nJoint == 0
                    app.C.lblStatus.Text = sprintf('%.2f s | 关节数 0（仅 base link）| %s', ...
                                                   toc(t0), pushed);
                else
                    app.C.lblStatus.Text = sprintf('%.2f s | 关节 %d | 可行率 %.2f%% | 体素 %d | %s', ...
                        toc(t0), app.P.meta.nJoint, app.P.stats.feasRate, ...
                        app.P.stats.voxCres, pushed);
                end
            end
        end

        function f = export(app)
        %EXPORT 导出到固定的一份 HTML（armFeas.html，反复覆盖，双击即可看）
            if isempty(app.P), app.refresh(); end
            here = fileparts(mfilename('fullpath'));
            f = fullfile(here, 'armFeas.html');      % ← 固定文件名，浏览器页签可以常开
            armFeasExport(app.P, f);
        end
    end

    methods (Access = private)
        function buildControls(app, pnl)
            % MATLAB 的 Layout 不能在构造函数里用 name-value 传，必须建完再赋
            spec = armSpec();
            app.MovIdx = find(~strcmpi({spec.type}, 'fixed'));
            nMov = numel(app.MovIdx);

            g = uigridlayout(pnl, [28 4]);
            g.ColumnWidth = {104, '1x', '1x', '1x'};
            g.RowHeight = repmat({26}, 1, 28);
            g.Padding = [8 8 8 8];
            g.RowSpacing = 6;
            app.C.g = g;
            r = 1;

            sec(g, r, '① 原点坐标系 O 与 Base 安装位姿'); r = r + 1;
            r = rowlabel(g, r, 'Base 预设');
            app.C.basePreset = uidropdown(g, 'Items', ...
                {'地面 z=0 (与原点重合)', '底盘顶面 z=200', '抬升座 z=350', '自定义'}, ...
                'Value', '地面 z=0 (与原点重合)');
            place(app.C.basePreset, r, [2 4]); r = r + 1;

            r = rowlabel(g, r, 'X / Y / Z (mm)');
            app.C.bx = uispinner(g, 'Limits', [-2000 2000], 'Value', 0, 'Step', 10);
            app.C.by = uispinner(g, 'Limits', [-2000 2000], 'Value', 0, 'Step', 10);
            app.C.bz = uispinner(g, 'Limits', [-1000 2000], 'Value', 0, 'Step', 10);
            place(app.C.bx, r, 2); place(app.C.by, r, 3); place(app.C.bz, r, 4); r = r + 1;

            r = rowlabel(g, r, 'Yaw (deg)');
            app.C.byaw = uispinner(g, 'Limits', [-180 180], 'Value', 0, 'Step', 5);
            place(app.C.byaw, r, 2); r = r + 1;

            sec(g, r, '② 关节轴向与限位（来自 armSpec.m）'); r = r + 1;
            axItems = {'+X', '-X', '+Y', '-Y', '+Z', '-Z'};
            for k = 1:nMov
                s = spec(app.MovIdx(k));
                if strcmpi(s.type, 'prismatic')
                    r = rowlabel(g, r, sprintf('%s (移动)', s.name));
                else
                    r = rowlabel(g, r, s.name);
                end
                axH(k) = uidropdown(g, 'Items', axItems, 'Value', axisName(s.axis)); %#ok<AGROW>
                if isempty(s.limits)
                    lo0 = -180; hi0 = 180;
                else
                    lo0 = s.limits(1); hi0 = s.limits(2);
                end
                loH(k) = uispinner(g, 'Limits', [-360 360], 'Value', lo0, 'Step', 5); %#ok<AGROW>
                hiH(k) = uispinner(g, 'Limits', [-360 360], 'Value', hi0, 'Step', 5); %#ok<AGROW>
                place(axH(k), r, 2); place(loH(k), r, 3); place(hiH(k), r, 4); r = r + 1;
            end
            if nMov == 0
                l = uilabel(g, 'Text', '（暂无关节 —— 在 armSpec.m 里补下一步）', ...
                            'FontColor', [.55 .62 .78]);
                place(l, r, [1 4]); r = r + 1;
                axH = gobjects(1,0); loH = gobjects(1,0); hiH = gobjects(1,0);
            end
            app.C.ax = axH; app.C.lo = loH; app.C.hi = hiH;
            app.C.hintLim = uilabel(g, 'Text', '限位填 ±180 即视为「不限位」', ...
                                    'FontColor', [.42 .48 .62]);
            place(app.C.hintLim, r, [1 4]); r = r + 1;

            sec(g, r, '③ 月牙耦合限位'); r = r + 1;
            r = rowlabel(g, r, '启用');
            app.C.cresOn = uicheckbox(g, 'Text', '月牙限位', 'Value', false);
            place(app.C.cresOn, r, [2 4]); r = r + 1;

            r = rowlabel(g, r, '耦合关节对');
            jn = {spec(app.MovIdx).name};
            if nMov < 2, jn = {'-'}; end
            app.C.qi = uidropdown(g, 'Items', jn, 'Value', jn{end});
            app.C.qj = uidropdown(g, 'Items', jn, 'Value', jn{end});
            place(app.C.qi, r, 2); place(app.C.qj, r, 3); r = r + 1;

            r = rowlabel(g, r, '大盘R / 小盘R');
            app.C.rOut = uispinner(g, 'Limits', [1 360], 'Value', 80, 'Step', 5);
            app.C.rIn  = uispinner(g, 'Limits', [1 360], 'Value', 60, 'Step', 5);
            place(app.C.rOut, r, 2); place(app.C.rIn, r, 3); r = r + 1;

            r = rowlabel(g, r, '小盘圆心 y (deg)');
            app.C.cy2 = uispinner(g, 'Limits', [-360 360], 'Value', 45, 'Step', 5);
            place(app.C.cy2, r, 2); r = r + 1;

            sec(g, r, '④ 计算规模 / 导出'); r = r + 1;
            r = rowlabel(g, r, '采样点');
            app.C.nSample = uidropdown(g, 'Items', {'1e4 (快)', '5e4', '2e5 (细)'}, 'Value', '5e4');
            place(app.C.nSample, r, [2 4]); r = r + 1;

            r = rowlabel(g, r, '导出');
            app.C.btnExport = uibutton(g, 'Text', '导出单文件 HTML（别人免 MATLAB 可看）', ...
                'ButtonPushedFcn', @(s, e) app.onExport());
            place(app.C.btnExport, r, [2 4]); r = r + 1;

            app.C.lblStatus = uilabel(g, 'Text', '就绪', 'FontColor', [.62 .72 .9]);
            place(app.C.lblStatus, r, [1 4]);

            app.C.basePreset.ValueChangedFcn = @(s, e) app.onPreset();
        end

        function attachCallbacks(app)
            cb = @(s, e) app.refresh();
            fns = {'bx', 'by', 'bz', 'byaw', 'rOut', 'rIn', 'cy2', 'cresOn', 'nSample', 'qi', 'qj'};
            for k = 1:numel(fns), app.C.(fns{k}).ValueChangedFcn = cb; end
            for k = 1:numel(app.C.ax)
                app.C.ax(k).ValueChangedFcn = cb;
                app.C.lo(k).ValueChangedFcn = cb;
                app.C.hi(k).ValueChangedFcn = cb;
            end
        end

        function onPreset(app)
            switch app.C.basePreset.Value
                case '地面 z=0 (与原点重合)', app.C.bz.Value = 0;
                case '底盘顶面 z=200',        app.C.bz.Value = 200;
                case '抬升座 z=350',          app.C.bz.Value = 350;
            end
            app.refresh();
        end

        function onExport(app)
            f = app.export();
            fprintf('已导出: %s\n', f);
            web(f, '-browser');
        end

        function onHTMLMsg(app, ev)
            try
                app.HTMLSaid = ev.Data;
            catch
                app.HTMLSaid = '收到了事件但没有 Data 字段';
            end
        end

        function cfg = readCfg(app)
            spec = armSpec();
            mv = find(~strcmpi({spec.type}, 'fixed'));
            for k = 1:numel(mv)
                spec(mv(k)).axis = axisVec(app.C.ax(k).Value);
                lo = app.C.lo(k).Value;  hi = app.C.hi(k).Value;
                % 约定：限位填 ±180 视为"不限位"（界面提示同此），这样 [] 语义不会被网格控件吃掉
                if lo == -180 && hi == 180
                    spec(mv(k)).limits = [];
                else
                    spec(mv(k)).limits = sort([lo, hi]);
                end
            end

            cfg.name = '机械臂可行性集（RST 计算 / HTML 显示）';
            cfg.spec = spec;
            cfg.base = struct('xyz', [app.C.bx.Value app.C.by.Value app.C.bz.Value], ...
                              'yaw', app.C.byaw.Value);

            % 月牙：pair 用"可动关节序号"
            on = app.C.cresOn.Value && numel(mv) >= 2;
            pair = [numel(mv)-1, numel(mv)];
            if on
                nm = {spec(mv).name};
                i1 = find(strcmp(nm, app.C.qi.Value), 1);
                i2 = find(strcmp(nm, app.C.qj.Value), 1);
                if ~isempty(i1) && ~isempty(i2) && i1 ~= i2, pair = [i1 i2]; end
            end
            cfg.crescent = struct('on', on, 'pair', pair, ...
                                  'R_OUT', app.C.rOut.Value, 'R_IN', app.C.rIn.Value, ...
                                  'CY2', app.C.cy2.Value);

            cfg.nSample = sscanf(app.C.nSample.Value, '%g');   % 必须 %g：%d 会把 '5e4' 读成 5
            if isnan(cfg.nSample), cfg.nSample = 5e4; end

            % 任务域先留空（本阶段只做机械臂自身可行性）。
            % 需要时按下面格式补：整机系 mm，3x2 的 [min;max]
            cfg.boxes = struct('name', {}, 'box', {});
            % cfg.boxes = struct('name', {'低位抓取'}, 'box', {[450 750; -250 250; 150 400]});
        end
    end
end

% ---------------- 局部辅助 ----------------
function sec(g, r, txt)
l = uilabel(g, 'Text', txt, 'FontWeight', 'bold', 'FontColor', [.65 .78 1]);
l.Layout.Row = r; l.Layout.Column = [1 4];
end

function r = rowlabel(g, r, txt)
l = uilabel(g, 'Text', txt, 'FontColor', [.62 .68 .8]);
l.Layout.Row = r; l.Layout.Column = 1;
end

function place(h, r, c)
h.Layout.Row = r; h.Layout.Column = c;
end

function v = axisVec(s)
switch s
    case '+X', v = [ 1  0  0];
    case '-X', v = [-1  0  0];
    case '+Y', v = [ 0  1  0];
    case '-Y', v = [ 0 -1  0];
    case '+Z', v = [ 0  0  1];
    case '-Z', v = [ 0  0 -1];
    otherwise, v = [ 0  0  1];
end
end

function nm = axisName(v)
v = v(:).' / norm(v);
[~, i] = max(abs(v));
c = 'XYZ'; s = '+';
if v(i) < 0, s = '-'; end
nm = [s c(i)];
end
