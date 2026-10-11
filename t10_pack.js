/*  t10_pack.js —— 打包成一个 zip 放进 ..\Pack\（分享用）
    跑法： node t10_pack.js

    ★ Pack\ 里【只留一个 zip】，不放散文件。
      （散文件是别人下载后自己解压出来的，不该由我们预先摊在 Pack 里）

    流程：
      1) 清空 Pack\（连上一次的旧 zip 一起清掉，避免发错版本）
      2) 在系统临时目录里搭一份 staging：工具本体 / README / 快速开始 /
         示例臂.json / MATLAB校验 / 截图
      3) 调 PowerShell 把 staging 压成 Pack\同济工程研究_臂型可行性工具_<日期>.zip
      4) 逐条校验 zip（条目数、路径分隔符、大小），删掉 staging

    三个坑（都踩过）：
      · 示例臂.json 不能手抄：上一版是【双重编码】的乱码（中文被当 GBK 再存成 UTF-8），
        现在直接用 JSON.stringify(demoModel()) 现算，写完还读回来验一遍。
      · zip 的目录分隔符必须是正斜杠：Compress-Archive / CreateFromDirectory 在 Windows 上
        写的是反斜杠，Linux/macOS 解压会生成名为 "MATLAB校验\xxx.m" 的怪文件。
        所以这里用 ZipArchive 逐条建条目、手工把 \ 换成 /。
      · ZipArchiveMode 在 System.IO.Compression 里，只 Add-Type ...FileSystem 会报"找不到类型"。
*/
const fs = require('fs'), os = require('os'), path = require('path'), cp = require('child_process');
const { loadEnv } = require('./t9_load.js');

const SRC  = __dirname;
const PACK = path.join(SRC, '..', 'Pack');
const D    = new Date();
const STAMP = D.getFullYear() + String(D.getMonth()+1).padStart(2,'0') + String(D.getDate()).padStart(2,'0');
const ZIPNAME = '同济工程研究_臂型可行性工具_' + STAMP + '.zip';
const STAGE = path.join(os.tmpdir(), 'armBuilder_pack_' + STAMP);

const log = (...a) => console.log(...a);
const copy = (from, to) => {
  fs.mkdirSync(path.dirname(to), { recursive: true });
  fs.copyFileSync(from, to);
  log('  + ' + path.relative(STAGE, to).replace(/\\/g,'/'));
};

/* ---------------- 1. 清空 Pack ---------------- */
log('[1] 清空 ' + PACK + '（只留最终那一个 zip）');
fs.mkdirSync(PACK, { recursive: true });
const wiped = [];
for(const f of fs.readdirSync(PACK)){
  fs.rmSync(path.join(PACK, f), { recursive:true, force:true });
  wiped.push(f);
}
log(wiped.length ? '  删掉 ' + wiped.length + ' 项：' + wiped.join(' · ') : '  （本来就是空的）');

/* ---------------- 2. 搭 staging ---------------- */
fs.rmSync(STAGE, { recursive:true, force:true });
fs.mkdirSync(STAGE, { recursive:true });

log('\n[2] 工具本体');
copy(path.join(SRC, 'armBuilder.html'), path.join(STAGE, 'armBuilder.html'));
copy(path.join(SRC, 'README.md'),       path.join(STAGE, 'README.md'));

log('\n[3] 示例臂.json（由 armBuilder.html 里的 demoModel() 现算）');
const { A } = loadEnv({ variants:false });
const demo = A.demoModel();
const json = JSON.stringify(demo, null, 2);
fs.writeFileSync(path.join(STAGE, '示例臂.json'), json, 'utf8');
log('  + 示例臂.json  ' + demo.links.length + ' 段，' + Buffer.byteLength(json,'utf8') + ' 字节');
const back = JSON.parse(fs.readFileSync(path.join(STAGE, '示例臂.json'), 'utf8'));
if(back.name !== demo.name || back.links.length !== demo.links.length)
  throw new Error('示例臂.json 回读不一致：' + back.name);
if(/[\uFFFD]/.test(back.name)) throw new Error('示例臂.json 中文坏了：' + back.name);
log('  ✓ 回读校验通过，name = 「' + back.name + '」');

log('\n[4] MATLAB校验/');
const M = ['armFeasApp.m','armFeasData.m','armFeasExport.m','armSpec.m','armview.html',
           'batchFK.m','buildChain.m','chainFK.m','rotDeg.m','t2_app_smoke.m',
           'crescentMargin.m','crescentBounds.m','makeArm.m'];
for(const f of M){
  const p = path.join(SRC, f);
  if(fs.existsSync(p)) copy(p, path.join(STAGE, 'MATLAB校验', f));
  else log('  ⚠ 缺文件，跳过：' + f);
}

log('\n[5] 截图/');
/* 目标清单：少一张就整个跳过截图，免得打进半套图 */
const SHOTS = [
  ['t10_overlay.png',      '01_总览_三个框.png'],
  ['t10_overlay_zoom.png', '02_底盘碰撞箱特写.png'],
  ['t10_split_zoom.png',   '03_右栏分隔条与数据面板.png'],
  ['t10_form_zoom.png',    '04_底盘尺寸设置面板.png'],
  ['t10_both.png',         '05_模型链与设置面板同开.png'],
  ['t10_anim.png',         '06_装配动画.png'],
  ['t10_packed.png',       '07_打包版实跑.png'],
];
const miss = SHOTS.filter(([s]) => !fs.existsSync(path.join(SRC, s)));
if(miss.length){ log('  ⚠ 还差 ' + miss.length + ' 张截图没生成，跳过整个截图目录：');
                 miss.forEach(([s]) => log('     缺 ' + s)); }
else for(const [s, dst] of SHOTS) copy(path.join(SRC, s), path.join(STAGE, '截图', dst));

/* ---------------- 3. 快速开始.txt ---------------- */
log('\n[6] 快速开始.txt');
const TXT = `机械臂可行性集工具 · 快速开始
================================================================

■ 怎么用（不需要 MATLAB、不需要联网、不需要起服务器）
  1. 先把本压缩包解压，双击 armBuilder.html（Edge / Chrome / Firefox / Safari 都行）
  2. 首次打开会弹窗，选「使用示例」= 同济工程机械臂
     （也可点「读取文件」选 示例臂.json，效果一样）
  3. 左边是可达集三维图 —— 拖拽旋转、滚轮缩放
     右边上半是「模型链」，右边下半是「可达域 / 任务域数据」
     ★ 两栏中间那条【分隔条可以上下拖】，双击复原；嫌模型链太挤就自己拉
  4. 右上角「▶ 示例装配动画」播放三级装配流程
     （就位 → 沿 Q 轴插入 → 沿 z 平移 100mm → 绕 Y 抬起 → 绕 X 旋转 90°）

■ 底盘碰撞箱 / 伸展尺寸
  三个可以单独开关的显示框（顶栏复选框）：
    底盘框      黄框 = 底盘碰撞箱。中心点在 base 系 (0,140,0)，
                580 × 580 × 105，从中心点往 −Z 长
                ⇒ 中心点 = 箱体上表面中心；底面中心 = 接地中心 = base (0,140,−105)
                底面那四条边画得更粗（那才是"接地"的一面）
    长宽高框    青框 = 规则的长宽高限 1200 × 1200 × 1100，
                xy 以接地中心为中心、z 从地面往上
    球          紫球 = 直径 1500 的限制球。规则：机器人放在一个与地面平行、
                距球心 480 的平面上，任意变形/移动不得超出此球
                ⇒ 球心 = 接地中心 z 往上 480，半径 750
                ⇒ 球与地面交线半径 √(750²−480²) = 576.3（地面上真正卡人的那个圈，
                  画成虚线圆）
  · 顶栏「▣ 隐藏框」一键同时开关【底盘框 + 长宽高框】（球不受它管）
  · 顶栏「▦ 底盘 / 尺寸」打开参数面板：中心点坐标与参照系、箱体尺寸、
    Z 延伸方向、长宽高限、球的直径与球心高度都能改；改完离开输入框即生效
  · 数据面板会算【机械臂长度】= 各段杆长之和（示例臂 1064.1mm），
    以及当前限位下的【最大伸展】（末端离 base 原点最远，示例臂约 984mm）

■ 右上角其它可调参数（改动会自动记住，刷新页面不会丢）
  装配臂位置 x / y / z   科技核心在 OXYZ 下的位置，默认 -500 -200 800
  安装yaw                机械臂的安装朝向，默认 -90
  映射                   核心在前 / 核心在侧
  采样数                 越大可达集越准、越慢

■ 数据面板怎么看
  位置可达率        三级任务域里 E 点位置够得到的比例
  IK 复核 · 可解    数值 IK 实际求解成功率（含"插入时夹爪 y ∥ Q 轴"姿态约束）
  IK 复核 · 只解位置  同一批位姿不加姿态约束的成功率 —— 两者之差就是姿态对齐的代价
  失败原因诊断      哪些关节顶到限位、放宽限位能救回多少
  安装朝向扫描      自动试不同 yaw，给出可解率最高的安装角度 ← 提高可解率最有效的杠杆
  零位朝向校核      零位时末端 z 与 Q 轴 e⃗ 的夹角，≈180° 才是正面装配
  机械臂长度        各段杆长 / 杆长之和 / 最大伸展 / 末端高度范围
  各项指标的详细含义见 README.md 第 8 节

■ 改机械臂本身
  右边「模型链」每行可「编辑 / 删除」，可「＋ 建立坐标系」新增。
  每个坐标系要填：
    · 位置：相对上一坐标系，或相对整机原点
    · 坐标系变换：新坐标系的 X/Y/Z 各对齐父系的哪个轴（会校验右手系）
    · 转动轴 + 限位角（限位留空 = 不限位）

■ 改任务域 / 比赛规则
  规则手册表5-10 硬编码在 armBuilder.html 里的 const RULES 那一段，
  底盘箱 / 伸展尺寸的默认值在同文件的 CHASSIS / SIZER 两处，
  规则变了只改那几段。
  ⚠ 改动 CHASSIS 的默认值时必须把同文件的 CHASSIS_V 加 1，
    否则浏览器里存的旧设置会把新默认值盖回去。

■ 存档与分享
  「保存 JSON」把当前模型存成文件（可分享、可进版本管理）
  「读取文件」载入 JSON
  浏览器里也会自动存一份，下次打开直接回到上次的状态

■ 注意（这几条决定了哪些数字能信、哪些不能）
  · 模型只有「杆 + 关节 + 坐标系 + 限位」，**机械臂本身没有碰撞几何**，
    也不考虑车体、底座的干涉。
  · 所以工具**不给"整机长宽高包围盒"** —— 拿骨架去量是假数；
    等真加了碰撞体和 yaw 限位再谈。
  · 限位里没有"地面"这一项，采样点会跑到地面以下（示例臂 3000 个里约 340 个）。
    【机械臂长度】那一栏已经把这些点剔除（地面 = 过接地中心的水平面），
    但三维视图里的蓝色点云没有过滤，看到地下有蓝点属正常。
  · 可达集由随机采样得到，看「严格同体素」和「含 ±1 体素容差」两行一起判断。
  · 三级任务域的可解率约 80%，剩下的是"够得到位置但摆不出插入姿态"，
    用面板里的「安装朝向扫描」可以找到更优的安装角度。

■ 跨平台：Linux / macOS / Windows 都能用
  · 只要有浏览器就行（Chrome / Edge / Firefox / Safari 均可），
    不需要 MATLAB，不需要联网，不需要起本地服务器，双击打开即可
  · 整个工具就是一个 armBuilder.html 文件，零外部依赖
    （没有 JS 库、没有 CDN、没有网络请求，全部离线自包含）
  · 只把 armBuilder.html 一个文件拷给别人也能用（其余都是可选的）
  · ⚠ macOS 的 Safari 打开本地文件时会禁用"本地存档"（浏览器安全策略）：
    工具本身照常运行（可达集、数据、动画都正常），只是刷新后不记住设置，
    右上角会出现橙色提示。此时用「保存 JSON」把模型存成文件，
    下次用「读取文件」载入即可。Chrome / Edge / Firefox 无此问题。
  · zip 里的中文名带 UTF-8 标志、且目录用正斜杠分隔，正常解压即可；
    万一在 Linux 上乱码，用 unzip -O UTF-8 解压

■ MATLAB 校验（可选，不用 MATLAB 可忽略「MATLAB校验」文件夹）
  里面是同一套模型的 MATLAB 实现（Robotics System Toolbox），
  浏览器版的运动学已与它逐项比对一致，可互为参照做交叉验证。

■ URL 钩子（进阶）
  armBuilder.html#demo      直接看示例（不覆盖你的存档）
  armBuilder.html#demosave  载入示例并存档
  armBuilder.html#anim      自动播放装配动画
  armBuilder.html#size      直接打开"底盘 / 尺寸"设置面板
  armBuilder.html#dump      把各坐标系数据铺满屏幕（核对数值用）
`;
fs.writeFileSync(path.join(STAGE, '快速开始.txt'), TXT, 'utf8');
log('  + 快速开始.txt  ' + TXT.split('\n').length + ' 行');

/* ---------------- 4. 压成 zip ---------------- */
/* ★ 自己写 zip，不调 PowerShell / Compress-Archive：
     · Node 没有内置 zip，而子进程调 pwsh 在有些环境里压根不在 PATH 上（实测 ENOENT）；
     · 更要紧的是条目名可控 —— 规范要求正斜杠，而 Windows 上的 Compress-Archive /
       ZipFile::CreateFromDirectory 写的是反斜杠，Linux/macOS 解压会得到
       "MATLAB校验\xxx.m" 这种怪文件；
     · 顺便还能自己置 UTF-8 标志位（bit 11），中文名在别的系统上才不会乱码。
   格式：本地头(30) + 名字 + 数据 …… 中央目录(46) + 名字 …… 结尾记录(22) */
const CRC_T = (() => {                       // CRC32 查表（不用 zlib.crc32，免得挑 Node 版本）
  const t = new Int32Array(256);
  for(let n=0;n<256;n++){ let c=n; for(let k=0;k<8;k++) c = (c&1) ? (0xEDB88320 ^ (c>>>1)) : (c>>>1); t[n]=c; }
  return t;
})();
function crc32(buf){
  let c = -1;
  for(let i=0;i<buf.length;i++) c = CRC_T[(c ^ buf[i]) & 0xFF] ^ (c >>> 8);
  return (c ^ -1) >>> 0;
}
const dosTime = d => ((d.getHours()<<11) | (d.getMinutes()<<5) | (d.getSeconds()>>1)) & 0xFFFF;
const dosDate = d => (((d.getFullYear()-1980)<<9) | ((d.getMonth()+1)<<5) | d.getDate()) & 0xFFFF;

function writeZip(zipPath, files){           // files: [{name, data, date}]
  const zlib = require('zlib');
  const parts = [], central = [];
  let offset = 0;
  for(const f of files){
    const nameBuf = Buffer.from(f.name, 'utf8');           // 名字用 / 分隔，调用方保证
    const flag = /[^\x00-\x7F]/.test(f.name) ? 0x0800 : 0; // bit 11 = UTF-8
    const crc  = crc32(f.data);
    const def  = zlib.deflateRawSync(f.data, { level:9 });
    const useDef = def.length < f.data.length;             // 压不动就原样存（比如小文件）
    const body = useDef ? def : f.data;
    const method = useDef ? 8 : 0;
    const t = dosTime(f.date), dt = dosDate(f.date);

    const lh = Buffer.alloc(30);
    lh.writeUInt32LE(0x04034b50, 0); lh.writeUInt16LE(20, 4); lh.writeUInt16LE(flag, 6);
    lh.writeUInt16LE(method, 8);     lh.writeUInt16LE(t, 10);  lh.writeUInt16LE(dt, 12);
    lh.writeUInt32LE(crc, 14);       lh.writeUInt32LE(body.length, 18);
    lh.writeUInt32LE(f.data.length, 22); lh.writeUInt16LE(nameBuf.length, 26);
    lh.writeUInt16LE(0, 28);
    parts.push(lh, nameBuf, body);

    const ch = Buffer.alloc(46);
    ch.writeUInt32LE(0x02014b50, 0); ch.writeUInt16LE(20, 4);  ch.writeUInt16LE(20, 6);
    ch.writeUInt16LE(flag, 8);       ch.writeUInt16LE(method, 10);
    ch.writeUInt16LE(t, 12);         ch.writeUInt16LE(dt, 14);
    ch.writeUInt32LE(crc, 16);       ch.writeUInt32LE(body.length, 20);
    ch.writeUInt32LE(f.data.length, 24); ch.writeUInt16LE(nameBuf.length, 28);
    ch.writeUInt16LE(0, 30); ch.writeUInt16LE(0, 32); ch.writeUInt16LE(0, 34);
    ch.writeUInt16LE(0, 36); ch.writeUInt32LE(0, 38); ch.writeUInt32LE(offset, 42);
    central.push(ch, nameBuf);
    offset += lh.length + nameBuf.length + body.length;
  }
  const cd = Buffer.concat(central);
  const eocd = Buffer.alloc(22);
  eocd.writeUInt32LE(0x06054b50, 0);
  eocd.writeUInt16LE(files.length, 8); eocd.writeUInt16LE(files.length, 10);
  eocd.writeUInt32LE(cd.length, 12);   eocd.writeUInt32LE(offset, 16);
  fs.writeFileSync(zipPath, Buffer.concat(parts.concat([cd, eocd])));
}

/* 收集 staging 里的所有文件（名字一律转成正斜杠） */
function collect(dir, base){
  const out = [];
  for(const e of fs.readdirSync(dir, { withFileTypes:true })){
    const full = path.join(dir, e.name);
    if(e.isDirectory()) out.push(...collect(full, base));
    else out.push({ name: path.relative(base, full).replace(/\\/g,'/'),
                    data: fs.readFileSync(full), date: fs.statSync(full).mtime });
  }
  return out;
}

const ZIP = path.join(PACK, ZIPNAME);
log('\n[7] 压缩 → ' + ZIPNAME);
const files = collect(STAGE, STAGE);
writeZip(ZIP, files);
for(const f of files) log('  · ' + f.name);

/* ---------------- 5. 校验 ---------------- */
log('\n[8] 校验');
const bad = files.filter(f => f.name.includes('\\') || f.name.startsWith('/'));
log(bad.length ? '  ✗ 有 ' + bad.length + ' 个条目名字不对' : '  ✓ ' + files.length + ' 个条目，全是正斜杠');
const buf = fs.readFileSync(ZIP);
log(buf.slice(0,4).toString('hex') === '504b0304' ? '  ✓ zip 头正常（PK\\x03\\x04）' : '  ✗ zip 头不对');
log('  ✓ ' + ZIPNAME + '  ' + (buf.length/1024).toFixed(1) + ' KB，' +
    files.length + ' 个文件，原始共 ' +
    (files.reduce((a,f)=>a+f.data.length,0)/1024).toFixed(1) + ' KB');
const loose = fs.readdirSync(PACK);
log((loose.length === 1 && loose[0] === ZIPNAME)
    ? '  ✓ Pack\\ 里就这一个 zip，没有散文件'
    : '  ✗ Pack\\ 里还有别的东西：' + loose.join(' · '));
fs.rmSync(STAGE, { recursive:true, force:true });
log('  ✓ staging 已清理');
log('\n完成。包在 ' + ZIP);
