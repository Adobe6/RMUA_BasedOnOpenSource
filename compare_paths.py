#!/usr/bin/env python3
"""对比 Splines.txt 第3条线与 paths.yaml path3 的匹配度。

坐标系: Splines.txt 为 NED (x, y, z), paths.yaml 为 ENU (x, y, z)
转换: NED → ENU: (x, -y, -z)
"""

import math
import yaml

# ── 解析 Splines.txt ──
def parse_splines(filepath):
    groups = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 3:
                # 每行是一组 spline，取所有 (x,y,z) 三元组
                points = []
                for i in range(0, len(parts) - 2, 3):
                    points.append((float(parts[i]), float(parts[i+1]), float(parts[i+2])))
                if points:
                    groups.append(points)
    return groups

# ── 解析 paths.yaml ──
def parse_path_yaml(filepath):
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    paths = {}
    for name, coords in data['paths'].items():
        paths[name] = [(p[0], p[1], p[2]) for p in coords]
    return paths

# ── NED → ENU 转换 ──
def ned2enu(points):
    return [(x, -y, -z) for x, y, z in points]

# ── 对齐 z 轴（整体平移使两线 z 均值对齐） ──
def align_z(ref, target):
    ref_mean_z = sum(p[2] for p in ref) / len(ref)
    tgt_mean_z = sum(p[2] for p in target) / len(target)
    delta = ref_mean_z - tgt_mean_z
    return [(x, y, z + delta) for x, y, z in target]

# ── 主对比逻辑 ──
def compare(spline_ned, path_enu, label_spline, label_path):
    print(f"\n{'='*70}")
    print(f"  对比: {label_spline}  vs  {label_path}")
    print(f"{'='*70}")

    spline_enu = ned2enu(spline_ned)

    print(f"\n  Spline (NED→ENU): {len(spline_enu)} 点, "
          f"起 ({spline_enu[0][0]:.1f},{spline_enu[0][1]:.1f},{spline_enu[0][2]:.1f}), "
          f"终 ({spline_enu[-1][0]:.1f},{spline_enu[-1][1]:.1f},{spline_enu[-1][2]:.1f})")

    print(f"  Path  (ENU    ): {len(path_enu)} 点, "
          f"起 ({path_enu[0][0]:.1f},{path_enu[0][1]:.1f},{path_enu[0][2]:.1f}), "
          f"终 ({path_enu[-1][0]:.1f},{path_enu[-1][1]:.1f},{path_enu[-1][2]:.1f})")

    # 起点/终点 偏差
    dx0 = spline_enu[0][0] - path_enu[0][0]
    dy0 = spline_enu[0][1] - path_enu[0][1]
    dz0 = spline_enu[0][2] - path_enu[0][2]
    d0 = math.hypot(dx0, dy0, dz0)
    dx1 = spline_enu[-1][0] - path_enu[-1][0]
    dy1 = spline_enu[-1][1] - path_enu[-1][1]
    dz1 = spline_enu[-1][2] - path_enu[-1][2]
    d1 = math.hypot(dx1, dy1, dz1)

    print(f"\n  ── 端点偏差 ──")
    print(f"  起点: Δ=({dx0:+.1f},{dy0:+.1f},{dz0:+.1f})  3D={d0:.1f}m")
    print(f"  终点: Δ=({dx1:+.1f},{dy1:+.1f},{dz1:+.1f})  3D={d1:.1f}m")

    # z 轴对齐（因为 altitude reference 可能不同）
    spline_aligned = align_z(path_enu, spline_enu)

    # ── 逐点最近邻距离 ──
    distances = []
    for sp in spline_aligned:
        min_d = min(math.hypot(sp[0]-pp[0], sp[1]-pp[1], sp[2]-pp[2]) for pp in path_enu)
        distances.append(min_d)

    avg_dist = sum(distances) / len(distances)
    max_dist = max(distances)
    min_dist = min(distances)
    within_1 = sum(1 for d in distances if d < 1.0)
    within_3 = sum(1 for d in distances if d < 3.0)
    within_5 = sum(1 for d in distances if d < 5.0)

    print(f"\n  ── 逐点最近邻距离 (Spline→Path) ──")
    print(f"  匀值: {avg_dist:.2f}m  min: {min_dist:.2f}m  max: {max_dist:.2f}m")
    print(f"  <1m: {within_1}/{len(distances)} ({100*within_1/len(distances):.0f}%), "
          f"<3m: {within_3}/{len(distances)} ({100*within_3/len(distances):.0f}%), "
          f"<5m: {within_5}/{len(distances)} ({100*within_5/len(distances):.0f}%)")

    # 反向检查 (Path→Spline)，避免 Spline 密 Path 疏的漏判
    rev_dist = []
    for pp in path_enu:
        min_d = min(math.hypot(pp[0]-sp[0], pp[1]-sp[1], pp[2]-sp[2]) for sp in spline_aligned)
        rev_dist.append(min_d)

    rev_avg = sum(rev_dist) / len(rev_dist)
    rev_max = max(rev_dist)
    rev_within_3 = sum(1 for d in rev_dist if d < 3.0)
    rev_within_5 = sum(1 for d in rev_dist if d < 5.0)

    print(f"\n  ── 逐点最近邻距离 (Path→Spline) ──")
    print(f"  匀值: {rev_avg:.2f}m  max: {rev_max:.2f}m")
    print(f"  <3m: {rev_within_3}/{len(rev_dist)} ({100*rev_within_3/len(rev_dist):.0f}%), "
          f"<5m: {rev_within_5}/{len(rev_dist)} ({100*rev_within_5/len(rev_dist):.0f}%)")

    # 路径长度对比
    def path_length(pts):
        total = 0.0
        for i in range(1, len(pts)):
            total += math.hypot(pts[i][0]-pts[i-1][0],
                                pts[i][1]-pts[i-1][1],
                                pts[i][2]-pts[i-1][2])
        return total

    len_s = path_length(spline_enu)
    len_p = path_length(path_enu)
    print(f"\n  ── 路径长度 ──")
    print(f"  Spline: {len_s:.1f}m   Path: {len_p:.1f}m   差: {abs(len_s-len_p):.1f}m")

    # 综合评估
    score = 0
    if d0 < 5 and d1 < 5:
        score += 2
    if avg_dist < 3 and rev_avg < 3:
        score += 3
    if abs(len_s - len_p) < len_p * 0.1:
        score += 2
    if within_3 > 0.8 * len(distances):
        score += 3

    rating = ["不匹配", "弱", "一般", "良好", "高度匹配"][min(score // 2, 4)]
    print(f"\n  ── 综合评估: {rating} (score={score}/10) ──")


if __name__ == "__main__":
    import os

    base = os.path.dirname(os.path.abspath(__file__))
    spline_file = os.path.join(base, "Splines.txt")
    yaml_file = os.path.join(base, "basic_dev", "src", "path_sender", "config", "paths.yaml")

    groups = parse_splines(spline_file)
    paths = parse_path_yaml(yaml_file)

    print(f"Splines.txt: {len(groups)} 条线")
    print(f"paths.yaml : {len(paths)} 条路径")
    for i, g in enumerate(groups):
        print(f"  Spline #{i+1}: {len(g)} 点")

    # 对比所有线
    path_keys = sorted(paths.keys(), key=lambda x: int(x.replace('path', '')))

    for i, g in enumerate(groups):
        pkey = f'path{i+1}'
        if pkey in paths:
            compare(g, paths[pkey], f"Spline #{i+1}", pkey)
        else:
            print(f"\n  Spline #{i+1}: {len(g)} 点 — paths.yaml 中无对应 {pkey}")
