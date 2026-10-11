# Mathmatic-Robotic-Arm-Fast-Iterators

> 机械臂**可行性与连杆快速迭代**工具集 —— 给一套「杆 + 关节 + 坐标系 + 角度限位」，
> 秒级算出**带限位的可达集**，并直接回答「这根杆改多长，比赛任务才更好做」。

零依赖单文件 Web 工具（双击即用、可离线）+ 与之交叉验证的 MATLAB 数值实现。

---

## 这是什么

做 RoboMaster / 工程机器人臂型设计时，最耗时的一环是「改一个杆长 → 等 CAD 出图 →
再算一遍能不能做到」这个循环。本仓库把这个循环压缩到**浏览器里的几十毫秒**：

| 工具 | 回答的问题 |
|---|---|
| **`build/armBuilder.html`** | 这套杆长/限位，**够得到哪些位姿**？可达集长什么样？比赛三级任务域能覆盖多少？ |
| **`build/armLink.html`** | 关节限位**一个字不动**的前提下，**哪几根杆值得改、改成多少**，任务适应性最好？ |

具体能做的事：

- **建模**：在网页里按顺序建立坐标系链（相对上一坐标系的位置 / 三轴朝向 / 转动轴 / 限位），
  实时校验右手系，模型可存成 `.json` 分享或进版本管理。
- **可达集**：正解 + 随机采样 + 体素化，画出带限位的三维可达点云（月牙状的关节耦合区域），
  而不是教科书里那种理想化的球壳。
- **任务校核**：内置比赛规则手册表 5-10 的**三级位姿范围**，直接给出位置可达率、
  高度分层、以及带插入姿态约束的 **IK 复核**成功率。
- **尺寸合规**：底盘碰撞箱、1200×1200×1100 长宽高框、直径 1500 限制球（球心距接地中心 480），
  以及**地面上真正卡人的那个圈** √(750²−480²) = 576.3 mm。
- **连杆优化**：灵敏度扫描 / 随机重启 + 坐标下降的全局搜索，在**杆长之和上限**约束下
  最大化严格合格率（位置<3mm、姿态<3°、**且最紧关节裕度>5°**）。
- **装配动画**：核心本体固定不动、只有 Q 轴组件在动的五段式插入演示。

### 一个实测结论（详见 [`docs/notes/t11_连杆改进报告.md`](docs/notes/t11_连杆改进报告.md)）

| 改动 | 杆长和 | 严格合格率 | 分数 |
|---|---|---|---|
| 基准臂 | 1064 mm | 9% | 8.0 |
| `link2.y` 290→235 · `link3.x` 85.5→45 · `gripper.z` 211→150 | **908 mm** | **20%** | **17.7** |

代价是最大伸展 985→865 mm（−12%）、可达体素 −20%，而三级任务域位置可达率仍是 100%。
另外两条反直觉的结论：**只缩工具段几乎没用，必须和横偏一起缩**；**小臂不能缩**。
（结论来自 5 套独立位姿集复核，防过拟合。）

---

## 用什么写的

**没有构建依赖，没有 npm 包，没有第三方库。** 全部手写。

| 层 | 技术 | 说明 |
|---|---|---|
| 交互工具 | **纯 HTML + CSS + 原生 JavaScript**（ES6） | 交付物是**单文件**，三维视图用 **Canvas 2D 手绘**，无 three.js / 无 CDN |
| 源码组织 | 分模块 + **一个 80 行的构建脚本**（Node，无依赖） | 源码在 `web/src/`，`node web/build.js` 合成单文件交付物 |
| 运动学 | 手写齐次变换递推 | 浏览器内正解 `T_i = T_{i-1}·Trans(offset_i)·Rot(axis_i,q_i)`，含 LM 阻尼最小二乘 IK |
| 模型格式 | **JSON** | `{name, links:[{name,pos,rot,axis,limits,type}]}`，可直接用「读取文件」载入 |
| 本地存档 | **localStorage** | 界面参数（装配位、安装 yaw、映射、采样数、底盘尺寸）刷新不丢 |
| 数值校验 | **MATLAB R2025b** + Robotics System Toolbox | `rigidBodyTree` / `generalizedInverseKinematics`；另有纯 MATLAB 参考正解（不依赖工具箱） |
| 可选 | Optimization Toolbox（`fmincon`） | 只有走约束优化时才需要 |
| 自动化自检 | **Node.js**（无框架，纯脚本） | 30 个探针，用 Edge 无头 `--dump-dom` 跑真界面、**数 canvas 像素**做断言 |

**规模**：约 12.5 MB / 111 个文件。源码 `armBuilder` = 551 + 649 + 433 行模块 + 218 行骨架。
正解已在浏览器 JS 与 MATLAB 之间**交叉验证**：位置误差 0 mm、姿态误差 1.1e-16。

---

## 目录结构（按模块功能分）

```
├── build/                    ★ 交付物：单文件 HTML，双击即用
│   ├── armBuilder.html         建模器（入库，clone 下来直接能开）
│   ├── armLink.html            连杆调参（构建产物，不入库）
│   └── armFeas.html            MATLAB 导出的快照（构建产物，不入库）
│
├── web/                      Web 工具：源码 / 自检 / 打包
│   ├── build.js                ★ 构建入口：node web/build.js
│   ├── src/
│   │   ├── apps/armBuilder.html   骨架模板（含 3 个 __MODULE 标记）
│   │   └── modules/
│   │       ├── 01-kinematics.js     运动学 / 模型 / 3D 渲染      551 行
│   │       ├── 02-task-rules.js     任务域 / 体素 / IK / 动画    649 行
│   │       ├── 03-chassis-size.js   底盘碰撞箱 / 伸展尺寸 / 面板  433 行
│   │       ├── t11_link_mod.js      连杆调参模块（追加到 armLink）
│   │       └── t11_link_css.css     连杆调参样式
│   ├── checks/                30 个无头自检探针（t*.js）+ t11_patch.js
│   ├── lib/                   共用库（assemble 合成器 / paths 路径表）
│   └── pack/                  打包分享
│
├── matlab/                   MATLAB 侧：数值实现与交叉验证
│   ├── setup.m                ★ 加搜索路径（先跑这个）
│   ├── model/                臂型定义 + 建树 + 正解
│   │   └── armSpec.m            ★ 改臂型只改这个文件
│   ├── core/                 可行性集计算、界面、导出
│   │   └── armview.html         显示层模板（uihtml 与导出共用）
│   └── checks/               10 个自检探针（t*.m / t*.py）
│
├── docs/
│   ├── notes/                t11_连杆改进报告.md · t9_IK诊断报告.md
│   ├── screenshots/          28 张界面截图
│   └── data/                 示例臂模型 JSON + 探针产物（摘要/日志/CSV）
│
├── HISTORY.md                ★ 开发日志与踩坑记录（650 行，含全部 bug 复盘）
└── README.md                 本文件
```

**分层原则**：`web/src` 是可编辑源码，`build/` 是构建产物，`docs/` 是文档与数据，
`matlab/` 是数值侧。跨目录路径只在 `web/lib/paths.js` 里定义一次。

---

## 怎么打开 / 怎么跑

### 1. 主力工具（**不需要装任何东西**）

直接**双击**：

| 文件 | 用途 |
|---|---|
| **`build/armBuilder.html`** | 建模器 —— 建坐标系链、看可达集、任务域校核、装配动画 |
| **`build/armLink.html`** | 连杆调参 —— 在限位只读的前提下搜索最优杆长，右上角 **⛓ 连杆调参** |

`build/armBuilder.html` 已经入库，**clone 下来就能双击**。
`build/armLink.html` 是构建产物，跑一次 `node web/build.js` 就有了。

打开 `armBuilder.html` 会出现开始窗口，选 **`使用示例`** 就能看到同济示例臂（7 段链）。
建议用较新版本的 Chrome / Edge / Firefox。页面**完全离线可用，断网也能跑**。

**URL 快捷入口**（接在文件名后面）：

| 后缀 | 作用 |
|---|---|
| `#demo` | 直接进示例模型（不存档） |
| `#demosave` | 载入示例并存档 |
| `#anim` | 自动播放装配动画 |
| `#joints` | 直接展开关节滑条面板 |
| `#dump` | 把浏览器算出的各坐标系数据铺满屏幕（用于和 MATLAB 对表） |

例：`build/armBuilder.html#demo`

### 2. 改源码 → 重新构建

交付物是**单文件**（双击即用），但源码是**分模块**的。
改 `web/src/modules/*.js` 之后重新合成：

```bash
node web/build.js          # 在仓库根目录跑
```

构建日志会打印每个模块的 md5，改了哪个模块一眼能看出来。
**构建是幂等的**，而且 `armBuilder.html` 的 md5 是**校验过的** —— 见下面"怎么保证没改坏"。

### 3. MATLAB 侧（可选，用于数值交叉验证 / 批量计算）

需要 **MATLAB R2025b + Robotics System Toolbox**：

```matlab
run('<本仓库>/matlab/setup.m')    % ① 加搜索路径（model / core / checks）

app = armFeasApp;                 % ② 打开界面：左控件 / 右 HTML 实时显示
f   = app.export();               % ③ 导出到 <仓库>/build/armFeas.html
P   = armFeasData(cfg);           % ④ 只要数据、不要界面
```

改臂型**只需要改 `matlab/model/armSpec.m` 一个文件**：往 `spec` 里加一项，
界面自动多出一行控件、HTML 自动多画一段杆和一条关节轴。

> ✅ **已在 MATLAB R2025b Update 4（`matlabroot = D:\`）上实跑验证过**（`matlab -batch`）：
> - `matlab/setup.m` 生效，`armSpec` / `buildChain` / `chainFK` / `armFeasApp`
>   都从新位置解析到；
> - `matlab/checks/t2_app_smoke.m` 全部断言通过，含
>   **`chainFK` vs RST `getTransform` 交叉验证：位置误差 2.13e-13 mm、姿态 3.89e-16**；
> - 界面构造出 19 个控件，5 组控件改动重算均无报错；
> - 导出确实落在 `build/armFeas.html`（321 KiB），Edge 无头打开可正常渲染。

### 4. 无头自检（可选，需要 Node.js + Edge）

**必须在仓库根目录执行**：

```bash
node web/build.js                    # 先构建（自检读的是 build/ 里的产物）
node web/checks/t10_chassis_size.js  # 39 项几何 / 面板断言
node web/checks/t10_ui_probe.js  --run   # 50 项界面断言（Edge 无头），含数 canvas 像素
node web/checks/t11_link_tune.js     # 60 项：只追加生成 / 限位只读 / 可复现 / 原功能回归
node web/checks/t11_ui_probe.js  --run   # 54 项：真点按钮（开面板→评估→搜索→应用→恢复）
node web/checks/t11_search.js --big  # 离线跑连杆搜索 → docs/data/t11_best.json
node web/checks/t11_verify.js        # 候选 × 多套独立位姿集复核（防过拟合）
node web/checks/t11_patch.js         # 只重建 armLink（需 build/armBuilder.html 已最新）
```

当前全绿：**39 + 60 + 50 + 54 = 203 项断言全部通过**。

### 5. 打包分享

```bash
node web/pack/t10_pack.js      # 打成一个 zip 放进 ../Pack/
```

---

## 怎么保证"整理"没把东西改坏

重构最容易出的事就是"整理完跑不起来了"。这次用了可验证的做法：

1. **逐字节比对**。抽模块之前先把 `armBuilder.html`、`armLink.html`、`armFeas.html`
   的 md5 存下来；合成之后 `web/build.js` **自己会断言** `armBuilder.html` 的 md5
   等于重构前的 `5077ab35c6fd...`，不等就直接报错退出。
2. **自检兜底**。203 项无头断言全部重跑，包括
   `t11_link_tune.js` 里那条"原 3 个 `<script>` 与 armLink 逐字节相同"的比对。
3. **入库门槛**。`web/build.js` 里的 `EXPECTED_ARM_BUILDER_MD5` 是**故意写死**的：
   以后谁无意中改了模块导致产物变化，构建会立刻失败，必须显式更新这个常量。
4. **行尾锁死成 LF**（`.gitattributes`）。这一条是踩出来的坑：
   交付物既然是"逐字节合成 + 断言 md5"，**行尾就不能随平台变**。
   Windows 上 `core.autocrlf=true` 会把源文件检出成 CRLF，合成出来的 HTML 就多出
   1847 个 `\r`（99912 字节 vs 98065），md5 直接对不上 ——
   **别人克隆下来第一次构建就失败**。所以：
   - `.gitattributes` 里源码/文档一律 `text eol=lf`，图片等标记 `binary`；
   - 构建链自己也不依赖行尾：`assemble.js` / `t11_patch.js` 读文件统一
     `\r\n → \n`、写文件统一 LF，自检比对前也先统一。

   > 这个 bug 在开发机上**测不出来**（当时工作区恰好还是 LF），
   > 是靠"**把仓库重新 clone 一份，在干净副本里跑构建**"才暴露的。
   > 所以每次改完构建链，都建议这样验一次，别只信本地。

---

## 目前的状态与已知边界

**已经能用的**：建模、可达集、任务域校核、IK 复核、底盘尺寸校核、连杆搜索与复核、装配动画。

**刻意不做的 / 做不到的**：

1. **没有碰撞、没有动力学、没有重力** —— 只回答「运动学上能不能到」。
   底盘碰撞箱只是**显示**，不参与任何判据；机械臂本身**没有碰撞体**，
   所以工具**不给**整机长宽高包围盒（拿骨架量出来的是假数）。
2. **关节限位只支持区间**，月牙状耦合区域由 `crescentMargin` / `crescentBounds` 单独表达。
   RST 原生只有矩形限位，直接用 `generateRobotWorkspace` 算月牙会得到**过近似**。
3. **纯限位 ≠ 一定不碰**：实测工具尖可到 z ≈ −9 mm（擦地）。
   统计最大伸展时会剔除地面以下的采样点（示例臂 3000 点里有 340 个在地下）。
4. **样本占比 ≠ 体积占比**：任务空间点云由关节空间均匀采样得到，需要精确体积请看体素数。
5. **连杆改不出「绕 Q 轴自转」**：6 个关节里没有「绕插入轴」那一级，
   绕插入轴能连续转过的弧中位只有 30°，这是轴系拓扑决定的，改长度解决不了，得补腕部 roll。
6. **界面脚本之间仍靠共享全局变量耦合**。这次只把 3 个大 `<script>` 拆成了 3 个文件
   （内容一个字节没动），**没有**动它们之间的调用方式 ——
   `MODEL` / `CHAIN` / `Q` 等仍是跨模块全局。彻底解耦需要改运行时设计，
   会破坏"逐字节相同"这条安全绳，留作后续。

完整的踩坑清单（15 条 + 每个 bug 的复现与修法）在 **[`HISTORY.md`](HISTORY.md)**。

---

## 历史

本仓库的 `README.md` 曾是 650 行的开发日志，2026-10 拆分为两部分：
面向使用者的说明留在这里（本文件），开发过程、bug 复盘与实测数据移到
**[`HISTORY.md`](HISTORY.md)**。

同月做了**模块化重整**：按功能分出 `web/` `matlab/` `docs/` `build/`，
把 `armBuilder.html` 里 1633 行内联脚本抽成 3 个模块 + 一个构建脚本，
产物与重构前**逐字节相同**。
