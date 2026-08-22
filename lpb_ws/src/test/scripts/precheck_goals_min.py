#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hedef erişilebilirlik hızlı kontrolü (YAML'siz, tak-çalıştır).
Bu sürüm SENİN paylaştığın parametrelerle doldurulmuştur.
"""

import math
from typing import List, Tuple, Dict, Any

# =============================================================================
# Navigation parameters used for the goal-feasibility precheck.
# =============================================================================
PARAMS: Dict[str, Any] = {
    # Footprint geometry is used; robot_radius is retained for compatibility.
    "robot_radius": 0.0,  # Not used when footprint geometry is active.
    "footprint": [[-0.105, -0.105], [-0.105, 0.105], [0.041, 0.105], [0.041, -0.105]],
    "footprint_padding": 0.0,

    # Inflation radius used for both global and local safety checks.
    "inflation_radius_global": 0.2,
    "inflation_radius_local":  0.2,

    # Goal tolerance is reported but not used in reachability checks.
    "xy_goal_tolerance": 0.2,

    # Small safety margin.
    "pad": 0.05,
}

# Interior room bounds for the 3 m x 3 m environment.
ROOM_MIN, ROOM_MAX = 0.0, 3.0

# Static obstacles: walls and boxes.
STATIC_OBJECTS = [
    # Boxes used in duraganOrt and karmaOrt.
    {'x': 1, 'y': 1, 'width': 0.25, 'height': 0.25},
    {'x': 2, 'y': 1, 'width': 0.25, 'height': 0.25},
    {'x': 1, 'y': 2, 'width': 0.25, 'height': 0.25},
    {'x': 2, 'y': 2, 'width': 0.25, 'height': 0.25},
]

# Dynamic obstacle tracks.
DYN_SEGMENTS = [
    ("H", (0.5, 1.5), (2.5, 1.5)),  # yatay
    ("V", (1.5, 0.5), (1.5, 2.5)),  # dikey
]

# Goal poses: x, y, theta in radians; theta is informational here.
GOAL_POINTS = [
    {'x': 2.5, 'y': 2.0, 'theta':  math.radians(45)},  #1
    {'x': 0.5, 'y': 2.5, 'theta':  math.radians(180)}, #2
    {'x': 2.5, 'y': 0.5, 'theta': -math.radians(45)},  #3
    {'x': 0.5, 'y': 2.0, 'theta':  math.radians(90)},  #4
    {'x': 2.5, 'y': 1.0, 'theta': -math.radians(90)},  #5
    {'x': 1.0, 'y': 2.5, 'theta':  math.radians(0)},   #6
    {'x': 1.0, 'y': 0.5, 'theta':  math.radians(45)},  #7
    {'x': 2.0, 'y': 2.5, 'theta':  math.radians(90)},  #8
    {'x': 2.0, 'y': 0.5, 'theta':  math.radians(45)},  #9
    {'x': 0.5, 'y': 0.5, 'theta':  math.radians(180)}  #10-0 (ROBOTUN BAŞLANGIÇ NOKTASI)
]

# =============================================================================
# Helper functions.
# =============================================================================
def footprint_inscribed_radius(fp: List[List[float]]) -> float:
    """Approximate the inscribed radius from the polygon footprint."""
    if not fp:
        return 0.0
    max_abs_x = max(abs(p[0]) for p in fp)
    max_abs_y = max(abs(p[1]) for p in fp)
    return min(max_abs_x, max_abs_y)

def effective_robot_radius(prm: Dict[str, Any]) -> float:
    pad = float(prm.get("footprint_padding", 0.0))
    fp  = prm.get("footprint")
    if fp:
        return footprint_inscribed_radius(fp) + pad
    return float(prm.get("robot_radius", 0.0)) + pad

def dist_to_walls(x: float, y: float) -> float:
    return min(x - ROOM_MIN, y - ROOM_MIN, ROOM_MAX - x, ROOM_MAX - y)

def dist_point_to_rect(x: float, y: float, cx: float, cy: float, w: float, h: float) -> float:
    hx, hy = w / 2.0, h / 2.0
    dx = max(abs(x - cx) - hx, 0.0)
    dy = max(abs(y - cy) - hy, 0.0)
    return math.hypot(dx, dy)

def dist_point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> Tuple[float, Tuple[float, float]]:
    apx, apy = px - ax, py - ay
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = 0.0 if ab2 == 0 else max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy), (cx, cy)

def suggest_offset_from_line(x: float, y: float, px: float, py: float, need: float) -> Tuple[float, float]:
    nx, ny = x - px, y - py
    nrm = math.hypot(nx, ny)
    if nrm < 1e-6:
        nx, ny = 1.0, 0.0
        nrm = 1.0
    ux, uy = nx / nrm, ny / nrm
    return x + ux * need, y + uy * need

# =============================================================================
# Main flow.
# =============================================================================
def main():
    r_robot = effective_robot_radius(PARAMS)  # ≈ 0.105
    r_infl  = max(float(PARAMS["inflation_radius_global"]), float(PARAMS["inflation_radius_local"]))  # 1.0
    pad     = float(PARAMS["pad"])  # 0.05
    r_safe  = r_robot + r_infl + pad  # 1.155

    print("=== COSTMAP PARAMETRE ÖZETİ ===")
    src = "footprint+padding" if PARAMS.get("footprint") else "robot_radius(+padding)"
    print(f"robot_radius (eff.): {r_robot:.3f} m  [kaynak: {src}]")
    print(f"inflation_radius (max): {r_infl:.3f} m")
    print(f"pad: {pad:.3f} m")
    print(f"→ r_safe = {r_safe:.3f} m")
    print(f"(bilgi) xy_goal_tolerance ≈ {float(PARAMS['xy_goal_tolerance']):.3f} m\n")

    print("=== HEDEF ERİŞİLEBİLİRLİK RAPORU ===")
    for i, g in enumerate(GOAL_POINTS, start=1):
        x, y = float(g['x']), float(g['y'])
        ok = True
        notes: List[str] = []

        # 1. Wall distance.
        d_wall = dist_to_walls(x, y)
        if d_wall < r_safe:
            ok = False
            notes.append(f"Duvar enflasyonuna yakın: d_wall={d_wall:.3f} < r_safe={r_safe:.3f} (Δ={(r_safe-d_wall):.3f})")

        # 2. Static obstacles: wall AABBs and boxes.
        for o in STATIC_OBJECTS:
            d_rect = dist_point_to_rect(x, y, o['x'], o['y'], o['width'], o['height'])
            if d_rect < r_safe:
                ok = False
                notes.append(
                    f"Statik engele yakın: min_d_rect={d_rect:.3f} < r_safe={r_safe:.3f} "
                    f"(engel@({o['x']},{o['y']}) w={o['width']}, h={o['height']})"
                )
                break

        # 3. Dynamic obstacle tracks.
        worst_dyn = float('inf'); nearest_p = None
        for _, a, b in DYN_SEGMENTS:
            d_seg, p = dist_point_to_segment(x, y, a[0], a[1], b[0], b[1])
            if d_seg < worst_dyn:
                worst_dyn, nearest_p = d_seg, p
        if worst_dyn < r_safe:
            ok = False
            need = (r_safe - worst_dyn) + 0.02  # Small additional margin.
            sx, sy = suggest_offset_from_line(x, y, nearest_p[0], nearest_p[1], need)
            notes.append(
                f"Dinamik yol çizgisine yakın: d_dyn={worst_dyn:.3f} < r_safe={r_safe:.3f} "
                f"→ Öneri: ({sx:.2f},{sy:.2f})"
            )

        status = "OK" if ok else "RİSKLİ"
        print(f"Goal #{i} @ ({x:.2f},{y:.2f}) → {status}")
        if notes:
            for n in notes:
                print("  -", n)
        else:
            print("  - Tüm kontroller geçti.")
        print()

if __name__ == "__main__":
    main()
