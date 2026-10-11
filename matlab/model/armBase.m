function b = armBase()
%ARMBASE 臂的【安装基准】与【任务坐标系】换算参数（与网页配置同一份语义）
%
%   b = armBase()
%
%   ★ 唯一真源在网页侧：web/src/configs/tongji.js 的 rules.frame。
%     改那边的时候这里要跟着改，两边必须一致 —— 不一致就会出现
%     "网页说够得着、MATLAB 说够不着"这种假矛盾（t7 那个 0/200 可解就是这么来的）。
%
%   返回：
%     yaw     : 臂本体装在整车上的朝向（°）。buildChain 用它把整棵树转到整车系，
%               网页的 buildChain() 第 71 行做的是同一件事。
%     baseH   : base 原点离任务系(OXYZ)地面的高度（mm）
%     taskMap : 任务系 → 整车系的轴向约定
%               'swap'   : base.X = -OXYZ.X, base.Y = -OXYZ.Y   （绕 Z 180°）
%               'noSwap' : base.X = -OXYZ.Y, base.Y = +OXYZ.X   （绕 Z +90°）
%
%   注意 yaw 与 taskMap 是【两件独立的事】，别以为改一个另一个会跟着变：
%   当前 yaw=-90 而 taskMap='swap'（等效绕 Z 180°），两者并不相等。
%   这不是笔误，是历史上就这么定的，保持原样以免改动已有结论。

b = struct('yaw', -90, 'baseH', 170, 'taskMap', 'swap');
end
