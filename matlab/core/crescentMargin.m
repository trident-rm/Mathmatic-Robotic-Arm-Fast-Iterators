function g = crescentMargin(x, y, R_OUT, R_IN, CY2)
%CRESCENTMARGIN 月牙限位的裕度函数（g <= 0 表示在月牙内）
%
%   g = crescentMargin(x, y, R_OUT, R_IN, CY2)
%
% 月牙 = 大盘(半径 R_OUT, 圆心原点) 减 偏心小盘(半径 R_IN, 圆心 (0,CY2))
% 输入 x / y 为两个关节角（单位随意，只要与 R_OUT/R_IN/CY2 一致，一般用度）
%
% 返回 g = max(g_outer, g_inner)：
%   g_outer = |p| - R_OUT          必须 <= 0（在大盘内）
%   g_inner = R_IN - |p - (0,CY2)| 必须 <= 0（在小盘外）
% 于是 g<=0 就是"在月牙内"，且 g 的数值就是到边界的有符号距离，
% 可直接当 fmincon 的非线性约束用：nonlcon = @(q) deal([], crescentMargin(q(2),q(3),...))
%
% 这与 08_pyroki_sim 里 crescent_residual 的残差形式完全一致，便于两边对表。

gOuter = sqrt(x.^2 + y.^2) - R_OUT;
gInner = R_IN - sqrt(x.^2 + (y - CY2).^2);
g = max(gOuter, gInner);
end
