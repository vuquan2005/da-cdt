#!/usr/bin/env python3
"""
Script to generate the simplified arena floor texture (floor.png) for robot0_gazebo.
Creates a clean floor with:
  - START Zone (X = -0.985, Y = 0.64)
  - RACK 1 (X = -1.894, Y = 0.641) + Stop Bar
  - RACK 2 (X = -1.894, Y = -0.006) + Stop Bar
  - Switching Junction (X = -0.40)
  - END / GOAL Zone (X = 0.80, Y = 0.00)
  - Navigation Track Lines connecting all locations
"""

import os
import cv2
import numpy as np


def generate_arena_floor(output_path=None, width=2000, height=1000):
    W, H = width, height
    img = np.full((H, W, 3), 252, dtype=np.uint8)  # Clean bright off-white canvas

    def w2p(wx, wy):
        """Converts Gazebo World coordinates (meters) to Texture Pixel coordinates (u, v)."""
        u = int(((wx + 2.0) / 4.0) * W)
        v = int(((1.0 - wy) / 2.0) * H)
        return (u, v)

    # 1. Subtle Outer Border (Perimeter of 4.0m x 2.0m field)
    border_pad = 25
    cv2.rectangle(img, (border_pad, border_pad), (W - border_pad, H - border_pad), (210, 210, 210), 3)

    # 2. Main Track Lines (Dark charcoal / black #1E2022)
    line_color = (25, 25, 25)
    line_thick = 14  # ~28mm wide at 500px/m resolution

    # Path 1: Row 1 along Y = 0.64 (Connects Rack 1 -> Start -> Switch -> Distribution -> Dropoff 1)
    p_r1 = w2p(-1.75, 0.64)
    p_d1 = w2p(0.55, 0.64)
    cv2.line(img, p_r1, p_d1, line_color, line_thick, cv2.LINE_AA)

    # Path 2: Row 2 along Y = 0.00 (Connects Rack 2 -> Switch -> Distribution)
    p_r2 = w2p(-1.75, 0.00)
    p_dist_mid = w2p(0.00, 0.00)
    cv2.line(img, p_r2, p_dist_mid, line_color, line_thick, cv2.LINE_AA)

    # Drop-off branch lines:
    # Zone 2 Branch (Y = 0.22)
    p_dist_z2 = w2p(0.00, 0.22)
    p_d2 = w2p(0.55, 0.22)
    cv2.line(img, p_dist_z2, p_d2, line_color, line_thick, cv2.LINE_AA)

    # Zone 3 Branch (Y = -0.22)
    p_dist_z3 = w2p(0.00, -0.22)
    p_d3 = w2p(0.55, -0.22)
    cv2.line(img, p_dist_z3, p_d3, line_color, line_thick, cv2.LINE_AA)

    # Zone 4 Branch (Y = -0.64)
    p_dist_z4 = w2p(0.00, -0.64)
    p_d4 = w2p(0.55, -0.64)
    cv2.line(img, p_dist_z4, p_d4, line_color, line_thick, cv2.LINE_AA)

    # 3. Vertical Connector Lines
    # Central Switch (X = -0.40) between Y = 0.64 and Y = 0.00
    p_sw_top = w2p(-0.40, 0.64)
    p_sw_bot = w2p(-0.40, 0.00)
    cv2.line(img, p_sw_top, p_sw_bot, line_color, line_thick, cv2.LINE_AA)

    # Distribution Trunk (X = 0.00) between Y = 0.64 and Y = -0.64
    p_dist_top = w2p(0.00, 0.64)
    p_dist_bot = w2p(0.00, -0.64)
    cv2.line(img, p_dist_top, p_dist_bot, line_color, line_thick, cv2.LINE_AA)

    # 4. Junction Dots
    junc_pts = [p_sw_top, p_sw_bot, p_dist_top, p_dist_z2, p_dist_mid, p_dist_z3, p_dist_bot]
    for pt in junc_pts:
        cv2.circle(img, pt, 8, line_color, -1, cv2.LINE_AA)

    # 5. RACK 1 (X = -1.894, Y = 0.641)
    p_rack1 = w2p(-1.894, 0.641)
    p_stop1 = w2p(-1.65, 0.641)
    cv2.line(img, (p_stop1[0], p_stop1[1] - 45), (p_stop1[0], p_stop1[1] + 45), line_color, line_thick, cv2.LINE_AA)
    rw, rh = 30, 65
    cv2.rectangle(img, (p_rack1[0] - rw, p_rack1[1] - rh), (p_rack1[0] + rw, p_rack1[1] + rh), (140, 100, 60), 3)
    cv2.putText(img, 'RACK 1', (p_rack1[0] - 100, p_rack1[1] - 80), cv2.FONT_HERSHEY_DUPLEX, 0.8, (120, 80, 40), 2, cv2.LINE_AA)
    cv2.putText(img, 'ALUMINUM / CPU', (p_rack1[0] - 110, p_rack1[1] + 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 70, 30), 1, cv2.LINE_AA)

    # 6. RACK 2 (X = -1.894, Y = -0.006)
    p_rack2 = w2p(-1.894, -0.006)
    p_stop2 = w2p(-1.65, -0.006)
    cv2.line(img, (p_stop2[0], p_stop2[1] - 45), (p_stop2[0], p_stop2[1] + 45), line_color, line_thick, cv2.LINE_AA)
    cv2.rectangle(img, (p_rack2[0] - rw, p_rack2[1] - rh), (p_rack2[0] + rw, p_rack2[1] + rh), (140, 100, 60), 3)
    cv2.putText(img, 'RACK 2', (p_rack2[0] - 100, p_rack2[1] - 80), cv2.FONT_HERSHEY_DUPLEX, 0.8, (120, 80, 40), 2, cv2.LINE_AA)
    cv2.putText(img, 'QR / CHIP', (p_rack2[0] - 80, p_rack2[1] + 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 70, 30), 1, cv2.LINE_AA)

    # 7. START ZONE (X = -0.985, Y = 0.64)
    p_start = w2p(-0.985, 0.64)
    sz = 75
    start_bg = img.copy()
    cv2.rectangle(start_bg, (p_start[0] - sz, p_start[1] - sz), (p_start[0] + sz, p_start[1] + sz), (210, 245, 210), -1)
    cv2.addWeighted(start_bg, 0.7, img, 0.3, 0, img)
    cv2.rectangle(img, (p_start[0] - sz, p_start[1] - sz), (p_start[0] + sz, p_start[1] + sz), (40, 180, 40), 4)
    cv2.putText(img, 'START', (p_start[0] - 55, p_start[1] - sz - 15), cv2.FONT_HERSHEY_DUPLEX, 0.9, (30, 150, 30), 2, cv2.LINE_AA)
    cv2.arrowedLine(img, (p_start[0] + 40, p_start[1]), (p_start[0] - 40, p_start[1]), (30, 140, 30), 5, tipLength=0.35)

    # 8. 4 DROP-OFF ZONES
    zones = [
        ('ZONE 1: ALUMINUM', (0.70, 0.64), (220, 120, 40), (245, 220, 180), 'ALUMINUM'),  # Blue
        ('ZONE 2: CPU',      (0.70, 0.22), (40, 180, 40),  (210, 245, 210), 'CPU'),       # Green
        ('ZONE 3: QR CODE',  (0.70, -0.22),(30, 170, 230), (200, 235, 250), 'QR CODE'),   # Yellow
        ('ZONE 4: CHIP',     (0.70, -0.64),(40, 40, 220),  (210, 210, 250), 'CHIP'),      # Red
    ]

    dz_w, dz_h = 75, 75
    for label, (zx, zy), border_col, bg_col, cargo_name in zones:
        pz = w2p(zx, zy)
        p_stop_z = w2p(0.55, zy)
        cv2.line(img, (p_stop_z[0], p_stop_z[1] - 35), (p_stop_z[0], p_stop_z[1] + 35), line_color, line_thick, cv2.LINE_AA)
        z_bg = img.copy()
        cv2.rectangle(z_bg, (pz[0] - dz_w, pz[1] - dz_h), (pz[0] + dz_w, pz[1] + dz_h), bg_col, -1)
        cv2.addWeighted(z_bg, 0.7, img, 0.3, 0, img)
        cv2.rectangle(img, (pz[0] - dz_w, pz[1] - dz_h), (pz[0] + dz_w, pz[1] + dz_h), border_col, 4)
        cv2.putText(img, cargo_name, (pz[0] - 65, pz[1] + 8), cv2.FONT_HERSHEY_DUPLEX, 0.7, border_col, 2, cv2.LINE_AA)
        cv2.putText(img, label.split(':')[0], (pz[0] - 50, pz[1] - dz_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, border_col, 2, cv2.LINE_AA)

    if output_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(script_dir, '..', 'models', 'arena_floor', 'materials', 'textures', 'floor.png')

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, img)
    print(f'Floor texture saved successfully to: {output_path}')


if __name__ == '__main__':
    generate_arena_floor()
