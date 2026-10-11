/*  t11_make_rec.js —— 按一组「杆→值」生成可直接载入的模型 JSON
    跑法：
      node t11_make_rec.js                                        # 默认 = 报告里的推荐 D
      node t11_make_rec.js "示例臂_link4弄直" link4.y=0 link4.z=402.59
    做的事：改 demoModel() 的对应 offset → 逐项比对（只允许这几个分量变，限位/轴/坐标系一字不动）
            → 写盘 → 读回来验一遍（防双重编码，Pack 那次踩过）
*/
const fs = require('fs'), path = require('path');
const { loadEnv } = require('./t9_load.js');
const P = require('../lib/paths');
const { A, sandbox } = loadEnv({ variants:false, file:'armLink.html' });
A.buildChain();
const L = sandbox.LINK;
L.init(true);
const KN = L.knobs();

let tag = '示例臂_推荐连杆D', CH = { 'link2.y':235, 'link3.x':45, 'gripper.z':150 };
const argv = process.argv.slice(2);
if(argv.length) tag = argv[0];
if(argv.length > 1){
  CH = {};
  argv.slice(1).forEach(a=>{
    const m = /^([\w.]+)=(-?[\d.]+)$/.exec(a.trim());
    if(!m) throw new Error('参数看不懂：' + a + '（要 name=value，例如 link4.y=0）');
    if(!KN.some(k=>k.name === m[1])) throw new Error('没有这个旋钮：' + m[1]);
    CH[m[1]] = +m[2];
  });
}
console.log('名字：' + tag + '　　改动：' + Object.entries(CH).map(([k,v])=>k+'='+v).join(' '));

/* ---- 目标模型：只改这些 offset 分量 ---- */
const m = A.demoModel();
m.name = '同济工程臂 · ' + tag;
let n = 0;
m.links.forEach((Lk, i)=>{
  KN.forEach(k=>{ if(k.li === i && CH[k.name] !== undefined){ Lk.pos[k.c] = CH[k.name]; n++; } });
});
if(n !== Object.keys(CH).length) throw new Error('只改了 ' + n + ' 处，参数有 ' + Object.keys(CH).length + ' 个');

/* ---- 逐项比对：除这些分量外一个字不许动 ---- */
const o = A.demoModel();
const diffs = [], bad = [];
o.links.forEach((Lk, i)=>{
  if(JSON.stringify(Lk.limits) !== JSON.stringify(m.links[i].limits)) bad.push(Lk.name + ' 限位被改了');
  ['joint','type','axis'].forEach(f=>{ if(Lk[f] !== m.links[i][f]) bad.push(Lk.name + '.' + f + ' 被改了'); });
  if(JSON.stringify(Lk.axes) !== JSON.stringify(m.links[i].axes)) bad.push(Lk.name + '.axes 被改了');
  ['x','y','z'].forEach((ax,c)=>{ if(Lk.pos[c] !== m.links[i].pos[c])
    diffs.push(`${Lk.name}.pos[${c}] ${Lk.pos[c]} → ${m.links[i].pos[c]}`); });
});
if(bad.length) throw new Error('不该改的地方改了：' + bad.join('；'));
console.log('逐项比对：只动了 ' + diffs.length + ' 个 offset 分量，限位 / 轴 / 坐标系朝向一字未动');
diffs.forEach(d=>console.log('  · ' + d));

/* ---- 顺手量一下（同一套位姿，只作参考） ---- */
const vals = L.readModel();
KN.forEach((k,i)=>{ if(CH[k.name] !== undefined) vals[i] = CH[k.name]; });
const cfg = Object.assign({}, L.cfg, {nPose:48, budget:8, iters:320, seed:20260925});
const rB = L.eval(L.readModel(), '基准', cfg);
const rC = L.eval(vals, tag, cfg);
console.log('参考（48 位姿 × 8 起点 × 320 迭代，只一套位姿）：');
console.log('  基准   分数 ' + rB.score.toFixed(1) + '  合格 ' + (100*rB.pass).toFixed(0) + '%  杆长和 ' + rB.armLen.toFixed(1));
console.log('  这份   分数 ' + rC.score.toFixed(1) + '  合格 ' + (100*rC.pass).toFixed(0) + '%  杆长和 ' + rC.armLen.toFixed(1));
console.log('  （单套位姿的抽样波动可达 ±3~6 分，下结论要看 t11_verify.js 的多套复核）');
console.log('  限位未改动 = ' + (rC.limitOK && L.limSig() === L.S.limSig));

const file = path.join(P.DATA, tag + '.json');   // 产物统一放 docs/data/
fs.writeFileSync(file, JSON.stringify(m, null, 2), 'utf8');
const back = JSON.parse(fs.readFileSync(file, 'utf8'));
if(back.name !== m.name || back.links.length !== 7) throw new Error('写出去读回来不一致');
console.log('已写 ' + file + '（' + fs.statSync(file).size + ' 字节，UTF-8 无 BOM，读回来名字/段数一致）');
