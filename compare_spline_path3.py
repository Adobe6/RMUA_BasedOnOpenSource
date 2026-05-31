#!/usr/bin/env python3
"""
Spline-3 vs Path3 详细对比分析
包含：Z偏移修正、方向反转检查、时间均匀采样对比
"""

import math
import os
import yaml


# ================= 解析 =================

def parse_splines(filepath, target_line=3):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    data_str = lines[target_line - 1].strip()
    values = list(map(float, data_str.split()))
    points = []
    for i in range(0, len(values), 3):
        points.append((values[i], values[i+1], values[i+2]))
    return points

def parse_paths_yaml(filepath, target='path3'):
    with open(filepath, 'r') as f:
        config = yaml.safe_load(f)
    return [(p[0], p[1], p[2]) for p in config['paths'][target]]

def ned_to_enu(pts):
    return [(p[0], -p[1], -p[2]) for p in pts]

def reverse(pts):
    return list(reversed(pts))

def shift_z(pts, offset):
    return [(p[0], p[1], p[2] + offset) for p in pts]

def dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def path_length(pts):
    return sum(dist(pts[i], pts[i-1]) for i in range(1, len(pts)))

def nearest_neighbor_dists(ref_pts, query_pts):
    return [min(dist(q, r) for r in ref_pts) for q in query_pts]

def chamfer(a, b):
    a2b = sum(nearest_neighbor_dists(b, a)) / len(a)
    b2a = sum(nearest_neighbor_dists(a, b)) / len(b)
    return (a2b + b2a) / 2

def dist_percentiles(ref, query):
    ds = sorted(nearest_neighbor_dists(ref, query))
    return ds[0], ds[len(ds)//2], ds[len(ds)//4*3], ds[-1]  # min, median, p75, max


# ================= 主程序 =================

def main():
    base = '/home/adobe/great-DJI-competition'
    spline_ned = parse_splines(os.path.join(base, 'Splines.txt'), target_line=3)
    path3_enu  = parse_paths_yaml(os.path.join(base, 'basic_dev/src/path_sender/config/paths.yaml'))

    # 四组对比：原始 S-ENU、反转 S-ENU；是否加 Z 偏移
    spline_enu  = ned_to_enu(spline_ned)
    spline_rev  = reverse(spline_enu)

    z_off   = sum(p[2] for p in path3_enu)/len(path3_enu) - sum(p[2] for p in spline_enu)/len(spline_enu)
    z_off_r = sum(p[2] for p in path3_enu)/len(path3_enu) - sum(p[2] for p in spline_rev)/len(spline_rev)

    print("=" * 72)
    print("  Spline-3 (NED→ENU) vs Path3 详细对比")
    print("=" * 72)

    combos = [
        ("正向 S-ENU",     spline_enu,  z_off),
        ("正向 S-ENU Z-adj", shift_z(spline_enu, z_off), 0),
        ("反向 S-ENU",     spline_rev,  z_off_r),
        ("反向 S-ENU Z-adj", shift_z(spline_rev, z_off_r), 0),
    ]

    for label, s_pts, z in combos:
        c = chamfer(s_pts, path3_enu)
        print(f"\n--- {label} (Z offset={z:+.0f}m) ---")
        print(f"  Chamfer: {c:.1f}m")
        print(f"  起点偏差: {dist(s_pts[0], path3_enu[0]):.1f}m")
        print(f"  终点偏差: {dist(s_pts[-1], path3_enu[-1]):.1f}m")

    # ===== 最佳方案：正向 + Z 偏移 详细对比 =====
    print("\n" + "=" * 72)
    print("  最佳方案: 正向 S-ENU + Z-offset 详细逐点分析")
    print("=" * 72)

    s_best = shift_z(spline_enu, z_off)  # Z-aligned
    p_best = path3_enu

    # 逐点最近邻
    s2p_dists = nearest_neighbor_dists(p_best, s_best)
    p2s_dists = nearest_neighbor_dists(s_best, p_best)

    print(f"\n  Spline→Path3 最近邻距离: ")
    s2p_sorted = sorted(s2p_dists)
    print(f"    min={s2p_sorted[0]:.1f}  median={s2p_sorted[len(s2p_sorted)//2]:.1f}  "
          f"p75={s2p_sorted[len(s2p_sorted)*3//4]:.1f}  max={s2p_sorted[-1]:.1f} m")

    print(f"  Path3→Spline 最近邻距离: ")
    p2s_sorted = sorted(p2s_dists)
    print(f"    min={p2s_sorted[0]:.1f}  median={p2s_sorted[len(p2s_sorted)//2]:.1f}  "
          f"p75={p2s_sorted[len(p2s_sorted)*3//4]:.1f}  max={p2s_sorted[-1]:.1f} m")

    # 路径长度
    len_s = path_length(s_best)
    len_p = path_length(p_best)
    print(f"\n  路径长度: Spline={len_s:.1f}m  Path3={len_p:.1f}m  "
          f"偏差={abs(len_s-len_p):.1f}m ({abs(len_s-len_p)/len_p*100:.1f}%)")

    # Z range
    sz = [p[2] for p in s_best]
    pz = [p[2] for p in p_best]
    print(f"  Z 范围: Spline=[{min(sz):.1f}, {max(sz):.1f}]  "
          f"Path3=[{min(pz):.1f}, {max(pz):.1f}]")
    print(f"  Z 均值: Spline={sum(sz)/len(sz):.1f}  "
          f"Path3={sum(pz)/len(pz):.1f}")

    # ===== 综合评估 =====
    print(f"\n{'='*72}")
    print(f"  综合评估")
    print(f"{'='*72}")
    c_best = chamfer(s_best, p_best)
    d_s = dist(s_best[0], p_best[0])
    d_e = dist(s_best[-1], p_best[-1])

    print(f"  Chamfer 距离:    {c_best:.1f} m")
    print(f"  起点偏差:         {d_s:.1f} m")
    print(f"  终点偏差:         {d_e:.1f} m")
    print(f"  路径长度偏差:     {abs(len_s-len_p)/len_p*100:.1f}%")
    print(f"  Spline 点数:      {len(spline_enu)} vs Path3 点数: {len(path3_enu)}")
    print(f"  Z 偏移量:         {z_off:+.1f} m")

    if c_best < 5 and d_s < 10 and d_e < 10:
        verdict = "高 — 路径形状高度一致"
    elif c_best < 30 and d_s < 30 and d_e < 30:
        verdict = "中 — 路径粗略一致，可能因坐标原点或插值密度导致差异"
    else:
        verdict = "低 — 路径差异较大，Spline 可能使用了不同坐标系或参考点"
    print(f"  整体匹配度:       {verdict}")


if __name__ == '__main__':
    main()
