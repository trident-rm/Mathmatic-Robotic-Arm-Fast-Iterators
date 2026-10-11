function [lo, hi] = crescentBounds(x, R_OUT, R_IN, CY2)
%CRESCENTBOUNDS 给定一个关节角 x，返回另一个关节角 y 的可行区间 [lo, hi]
%
%   [lo, hi] = crescentBounds(x, R_OUT, R_IN, CY2)
%
% 用途：把"月牙"这个耦合可行域重参数化成"盒"。
%   任意位形 (x, y) 在月牙内  <=>  lo(x) <= y <= hi(x)
%   于是采样可以写成：  x ~ U[a,b]，u ~ U[0,1]，y = lo(x) + u*(hi(x)-lo(x))
%   这样采出来的点 100% 在月牙内，任何"只认矩形关节限位"的工具都能用。
%
% 返回 NaN 表示该 x 根本无解（|x| > R_OUT，在大盘外）。
%
% 推导：竖直线 x=const 与月牙的交集是一段
%   大盘给出     |y| <= sqrt(R_OUT^2 - x^2) = outer
%   小盘给出     y <= CY2 - sqrt(R_IN^2 - x^2)（上半支 y >= CY2 + sqrt(...)
%                在盘外，因为 CY2 + R_IN > R_OUT）
%   当 |x| > R_IN 时竖直线不穿过小盘，小盘不再形成上界

outer = sqrt(max(R_OUT^2 - x.^2, 0));
t = R_IN^2 - x.^2;
innerCut = CY2 - sqrt(max(t, 0));
innerCut(t <= 0) = inf;              % |x| > R_IN：小盘不挡
lo = -outer;
hi = min(outer, innerCut);
bad = abs(x) > R_OUT;
lo(bad) = NaN;
hi(bad) = NaN;
end
