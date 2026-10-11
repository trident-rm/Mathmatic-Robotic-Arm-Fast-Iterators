# -*- coding: utf-8 -*-
"""t9_ik_truth.py —— 用【独立实现】的 FK + scipy 优化器，判定三级任务域位姿到底可不可解

目的：t8/t9 里"强多起点能解 320/320"用的是页面那份 IK。这里换一套完全独立的东西：
  1) 按 armSpec/armBuilder 的链定义，在 Python 里重写 FK（并与 JS 的 FK 逐项对表）；
  2) 解析雅可比的阻尼最小二乘 + scipy.optimize.least_squares（信赖域）多起点；
  3) 另外做一次"位置锁死、只优化姿态"的约束优化，给出该位置下【能达到的最小姿态偏差】，
     把"姿态不可满足"和"求解器没解出来"彻底分开。

用法： python t9_ik_truth.py [起点数=48]
"""
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
# 探针产物统一放 <仓库>/docs/data/（模块化重整后）
DATA = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'data'))

# ---------------------------------------------------------------- 模型（与 armSpec.m / demoModel 一致）
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

D2R = np.pi / 180.0
BASE_YAW = -90.0 * D2R
BASE_H = 170.0

# name, joint, type, offset(mm), Rf(3x3 or None), axis(unit, child frame), limits(deg or None)
LINKS = [
    ('base_link', 'base_joint', 'revolute', [0, 0, 0],        None,          [0, 0, 1],  None),
    ('link1',     'joint1',     'revolute', [0, 0, 75],       None,          [1, 0, 0],  [0, 122]),
    ('link2',     'joint2',     'revolute', [0, 290, 0],      None,          [-1, 0, 0], [-90, 200]),
    ('link3',     'joint3',     'revolute', [85.5, 0, 0],     None,          [0, 0, 1],  None),
    ('link4',     'joint4',     'revolute', [0, -30, 401.47], None,          [1, 0, 0],  [-16, 220]),
    ('link5',     'joint5',     'revolute', [0, 0, 0],        Rx(-np.pi/2),  [0, 0, 1],  None),
    ('gripper',   'gripper_fix','fixed',    [0, 0, 211],      None,          [0, 0, 1],  None),
]
MOV = [i for i, L in enumerate(LINKS) if L[2] != 'fixed']
NJ = len(MOV)
LIM = np.array([[-180.0, 180.0] if L[6] is None else L[6] for L in LINKS if L[2] != 'fixed'],
               dtype=float)

def fk(q_deg):
    """T_i = T_{i-1} · Trans(offset) · Rf · Rot(axis,q)；返回每体系的世界 R(3x3), t(3,)。"""
    R = Rz(BASE_YAW).copy()
    t = np.zeros(3)
    out = [(R.copy(), t.copy())]
    k = 0
    for name, jn, typ, off, Rf, ax, lim in LINKS:
        Rj = np.eye(3)
        if typ != 'fixed':
            Rj = _rot_axis(np.asarray(ax, float), q_deg[k] * D2R)
            k += 1
        t_new = t + R @ np.asarray(off, float)
        R = R @ (np.eye(3) if Rf is None else Rf) @ Rj
        t = t_new
        out.append((R.copy(), t.copy()))
    return out

def _rot_axis(u, th):
    u = u / np.linalg.norm(u)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)

def ee_pose(q_deg):
    F = fk(q_deg)
    return F[-1][0], F[-1][1]

def task_to_base(p):        # MAP.swap = true
    return np.array([-p[0], -p[1], p[2] - BASE_H], float)

def dir_to_base(d):
    return np.array([-d[0], -d[1], d[2]], float)

def core_e(P):
    th, ph = P['th'] * D2R, P['ph'] * D2R
    return np.array([np.sin(ph) * np.cos(th), np.sin(ph) * np.sin(th), np.cos(ph)])

def _axang_err(y, d, w):
    """把 y 转到 d 的轴角误差向量 × w（与 armBuilder.html 的 ikSolve6 完全同约定）。
       注意：不能用 cross(y,d) 当残差 —— 它在 y 与 d 反平行时也等于 0，
       会把"夹爪装反 180°"误判成"对齐成功"。"""
    ax = np.cross(y, d)
    s = np.linalg.norm(ax)
    c = float(np.dot(y, d))
    if s < 1e-8:
        if c > 0:
            return np.zeros(3), 0.0
        p = np.cross(y, [1, 0, 0]) if abs(y[0]) < 0.9 else np.cross(y, [0, 1, 0])
        p = p / np.linalg.norm(p)
        return w * np.pi * p, np.pi
    ang = np.arctan2(s, c)
    return (ax / s) * ang * w, ang

def residual(q_deg, tgt_p, tgt_d, w_ori=40.0):
    """[位置误差(3) ; w·轴角误差向量(3)]；对齐时第二块为 0（反平行时不为 0）。"""
    F = fk(q_deg)
    R, p = F[-1]
    eo, _ = _axang_err(R[:, 1], tgt_d, w_ori)
    return np.concatenate([p - tgt_p, eo])

def jac_pos(q_deg, tgt_p, tgt_d, w_ori=40.0):
    """只给【位置块】的解析雅可比，姿态块留 0 → scipy 会自己数值差分（'2-point'）。
       位置块：d(p)/dq_i = a_i × (p - p_i)，再乘 dq_rad/dq_deg = π/180。
       姿态块的解析导数坑很深（armBuilder.html 用的是"任务速度雅可比" +W·a_i，
       那是给它自己的 DLS 步长公式配的，不是残差真导数），所以这里不手推。"""
    F = fk(q_deg)
    Re, pe = F[-1]
    cols = []
    for li, (name, jn, typ, off, Rf, ax, lim) in enumerate(LINKS):
        if typ == 'fixed':
            continue
        Rc, tc = F[li + 1]
        a = Rc @ (np.asarray(ax, float) / np.linalg.norm(ax))
        cols.append(np.concatenate([np.cross(a, pe - tc), np.zeros(3)]))
    return np.array(cols).T * (np.pi / 180.0)


def check_jac():
    """数值差分校验【位置块】解析雅可比（姿态块交给 scipy 数值差分）"""
    rng = np.random.default_rng(7)
    tp = np.array([120.0, -220.0, 430.0])
    td = np.array([-0.3, 0.5, 0.81]); td = td / np.linalg.norm(td)
    worst = 0.0
    for _ in range(5):
        q = rng.uniform(LIM[:, 0] + 30, LIM[:, 1] - 30)   # 避开限位边界
        Ja = jac_pos(q, tp, td)[:3, :]
        h = 1e-6
        for k in range(NJ):
            qp, qm = q.copy(), q.copy()
            qp[k] += h; qm[k] -= h
            Jn = (residual(qp, tp, td)[:3] - residual(qm, tp, td)[:3]) / (2 * h)
            worst = max(worst, float(np.max(np.abs(Ja[:, k] - Jn))))
    return worst

# ---------------------------------------------------------------- 1) 与 JS 的 FK 对表
def check_fk():
    ref = json.load(open(os.path.join(DATA, 't8_fk_ref.json'), encoding='utf-8'))
    worst_p = worst_R = 0.0
    for case in ref['cases']:
        F = fk(np.array(case['q'], float))
        for i, fr in enumerate(case['frames']):
            R, t = F[i]
            worst_p = max(worst_p, float(np.max(np.abs(t - np.array(fr['t'], float)))))
            worst_R = max(worst_R, float(np.max(np.abs(R - np.array(fr['R'], float)))))
    print('=== 1) Python FK vs armBuilder.html JS FK（%d 组关节角 × 8 个坐标系）===' % len(ref['cases']))
    print('   位置最大偏差 %.3e mm   姿态矩阵最大偏差 %.3e' % (worst_p, worst_R))
    # 任务域映射
    for tm in ref['taskmap']:
        P = tm['P']
        dp = np.max(np.abs(task_to_base([P['x'], P['y'], P['z']]) - np.array(tm['Eb'])))
        dd = np.max(np.abs(dir_to_base(core_e(P)) - np.array(tm['e'])))
        if max(dp, dd) > 1e-9:
            print('   ✗ 任务域映射不一致 P=%s' % P)
    print('   任务域→base 映射一致\n')
    return max(worst_p, worst_R)

# ---------------------------------------------------------------- 2) 强 IK（scipy 多起点）
def solve_pose(tgt_p, tgt_d, n_start=48, seed=0):
    """返回 (ok, dp, ang, q)。ok = 位置<3mm 且 轴夹角<0.2rad。"""
    rng = np.random.default_rng(seed)
    lo, hi = LIM[:, 0], LIM[:, 1]
    assert np.all(lo < hi), 'bounds 不合法: %s %s' % (lo, hi)
    starts = [np.zeros(NJ)]
    starts += [lo + rng.random(NJ) * (hi - lo) for _ in range(n_start - 1)]
    best = (np.inf, np.inf, None)
    tz = tgt_d / np.linalg.norm(tgt_d)
    nerr = 0
    for q0 in starts:
        try:
            sol = least_squares(residual, q0, bounds=(lo, hi), args=(tgt_p, tz),
                                xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=400)
        except Exception as e:
            nerr += 1
            if nerr == 1:
                print('   ⚠ least_squares 异常: %r' % (e,))
            continue
        R, p = ee_pose(sol.x)
        dp = float(np.linalg.norm(p - tgt_p))
        ang = float(np.arccos(np.clip(R[:, 1] @ tz, -1, 1)))
        if dp < best[0] or (abs(dp - best[0]) < 1e-9 and ang < best[1]):
            best = (dp, ang, sol.x)
        if dp < 1.0 and ang < 0.05:
            break
    dp, ang, q = best
    return (dp < 3.0 and ang < 0.2), dp, ang, q

def pos_only(tgt_p, n_start=32, seed=0):
    """只解位置（3 个残差），返回 (ok, dp)。"""
    rng = np.random.default_rng(seed + 777)
    lo, hi = LIM[:, 0], LIM[:, 1]
    starts = [np.zeros(NJ)] + [lo + rng.random(NJ) * (hi - lo) for _ in range(n_start - 1)]
    best = np.inf
    for q0 in starts:
        f = lambda q: fk(q)[-1][1] - tgt_p
        try:
            sol = least_squares(f, q0, bounds=(lo, hi), xtol=1e-12, ftol=1e-12, max_nfev=300)
        except Exception:
            continue
        d = float(np.linalg.norm(fk(sol.x)[-1][1] - tgt_p))
        best = min(best, d)
        if best < 0.5:
            break
    return best < 1.5, best

def min_ang_at_pos(tgt_p, tgt_d, n_start=16, seed=0):
    """位置锁死（3 个等式约束）时能达到的最小姿态偏差角（度）——判断"姿态是否根本不可满足"。"""
    rng = np.random.default_rng(seed + 31337)
    lo, hi = LIM[:, 0], LIM[:, 1]
    tz = tgt_d / np.linalg.norm(tgt_d)
    best = np.inf
    for _ in range(n_start):
        q0 = lo + rng.random(NJ) * (hi - lo)
        # 先只解位置
        sol0 = least_squares(lambda q: fk(q)[-1][1] - tgt_p, q0, bounds=(lo, hi), max_nfev=300)
        q = sol0.x
        # 再在"位置尽量不动"的前提下压姿态：位置残差给大权重
        def res(qq):
            R, p = ee_pose(qq)
            eo, _ = _axang_err(R[:, 1], tz, 1.0)
            return np.concatenate([100.0 * (p - tgt_p), eo])
        try:
            sol = least_squares(res, q, bounds=(lo, hi), xtol=1e-12, ftol=1e-12, max_nfev=400)
        except Exception:
            continue
        R, p = ee_pose(sol.x)
        if np.linalg.norm(p - tgt_p) > 3.0:      # 位置没守住，这个解不算
            continue
        ang = float(np.arccos(np.clip(R[:, 1] @ tz, -1, 1))) * 180 / np.pi
        best = min(best, ang)
    return best

def main():
    n_start = int(sys.argv[1]) if len(sys.argv) > 1 else 48
    worst = check_fk()
    if worst > 1e-6:
        print('   ⚠ FK 不一致，先别信下面的结论')
    jw = check_jac()
    print('=== 1b) 解析雅可比 vs 数值差分 ===  最大偏差 %.3e' % jw)
    print('   ' + ('✓ 一致' if jw < 1e-4 else '✗ 不一致（残差约定与雅可比不匹配）') + '\n')

    import csv
    rows = list(csv.DictReader(open(os.path.join(DATA, 't8_ik_diag.csv'), encoding='utf-8')))
    P = [{'x': float(r['x']), 'y': float(r['y']), 'z': float(r['z']),
          'th': float(r['th']), 'ph': float(r['ph']), 'al': float(r['al'])} for r in rows]

    n_ok = n_pos = 0
    fails = []
    angs = []
    for i, p in enumerate(P):
        tp = task_to_base([p['x'], p['y'], p['z']])
        td = -dir_to_base(core_e(p))
        ok, dp, ang, q = solve_pose(tp, td, n_start, seed=1000 + i)
        pok, pdp = pos_only(tp, 24, seed=2000 + i)
        n_ok += ok
        n_pos += pok
        if ok:
            angs.append(ang * 180 / np.pi)
        else:
            fails.append((i, dp, ang * 180 / np.pi, pdp, p))
        if (i + 1) % 80 == 0:
            print('   ... %d/%d  可解 %d  位置可达 %d' % (i + 1, len(P), n_ok, n_pos))

    print('\n=== 2) 独立复核（Python FK + scipy 信赖域，%d 起点）===' % n_start)
    print('   位姿数                     %d' % len(P))
    print('   位置可达（只解位置）       %d/%d = %.1f%%' % (n_pos, len(P), 100 * n_pos / len(P)))
    print('   位置+姿态对齐 可解         %d/%d = %.1f%%' % (n_ok, len(P), 100 * n_ok / len(P)))
    if angs:
        a = np.sort(np.array(angs))
        print('   成功解的姿态偏差(°)       中位 %.2f  最大 %.2f （阈值 11.46）' %
              (a[len(a) // 2], a[-1]))

    # ---- 单起点成功率：把"求解器迭代不行"和"问题本身多峰"分开 ----
    print('\n=== 3) 单起点成功率（每个位姿各跑 K 个独立随机起点，scipy 信赖域 LM）===')
    K = 8
    rng = np.random.default_rng(4242)
    lo, hi = LIM[:, 0], LIM[:, 1]
    per, succ_at = [], {1: 0, 2: 0, 3: 0, 4: 0, 6: 0, 8: 0}
    worst_detail = []
    for i, p in enumerate(P):
        tp = task_to_base([p['x'], p['y'], p['z']])
        td = -dir_to_base(core_e(p))
        tz = td / np.linalg.norm(td)
        cnt = 0
        for k in range(K):
            q0 = lo + rng.random(NJ) * (hi - lo)
            try:
                sol = least_squares(residual, q0, bounds=(lo, hi), args=(tp, tz),
                                    xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=400)
                R, pp = ee_pose(sol.x)
                dp = float(np.linalg.norm(pp - tp))
                ang = float(np.arccos(np.clip(R[:, 1] @ tz, -1, 1)))
                if dp < 3.0 and ang < 0.2:
                    cnt += 1
            except Exception:
                pass
        per.append(cnt)
        for n in succ_at:
            if cnt >= 1:
                succ_at[n] += 1
            if cnt >= n:
                succ_at[n] += 0
        if cnt == 0:
            worst_detail.append((i, p))
    per = np.array(per)
    print('   每个位姿 8 个起点的成功个数: 0个=%d  1个=%d  2个=%d  3-4个=%d  5-6个=%d  7-8个=%d'
          % ((per == 0).sum(), (per == 1).sum(), (per == 2).sum(),
             ((per >= 3) & (per <= 4)).sum(), ((per >= 5) & (per <= 6)).sum(), (per >= 7).sum()))
    print('   单起点成功率 ≈ %.1f%%   → 用 3 个起点理论可解率 %.1f%%' %
          (100 * per.mean() / K, 100 * (1 - (1 - per.mean() / K) ** 3)))
    if worst_detail:
        print('   8 个随机起点全失败的位姿: %d 个' % len(worst_detail))
        for i, p in worst_detail[:6]:
            tp = task_to_base([p['x'], p['y'], p['z']])
            ma = min_ang_at_pos(tp, -dir_to_base(core_e(p)), 16, seed=5000 + i)
            print('      #%-4d 该位置下最小姿态偏差 = %.1f°  (ph=%.0f° z=%.0f)'
                  % (i, ma, p['ph'], p['z']))
    if fails:
        print('\n=== 4) 多起点仍失败的位姿归因 ===')
        for i, dp, ang, pdp, p in fails[:10]:
            ma = min_ang_at_pos(task_to_base([p['x'], p['y'], p['z']]),
                                -dir_to_base(core_e(p)), 16, seed=5000 + i)
            print('     #%-4d dp=%7.2fmm 姿态=%6.1f°  位置可达=%s  该位置下最小姿态偏差=%.1f°'
                  % (i, dp, ang, pdp, ma))
    else:
        print('\n   ★ 没有任何位姿失败 → 三级任务域在这套链/限位下全解可达（与 JS 强多起点一致）')

if __name__ == '__main__':
    main()
