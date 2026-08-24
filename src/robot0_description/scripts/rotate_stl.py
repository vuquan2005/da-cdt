#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Utility script to rotate binary STL files around coordinate axes.
Standard ROS REP-103: +X Forward, +Y Left, +Z Up.
"""

import os
import sys
import struct
import numpy as np


def rotate_stl_z(file_path: str, angle_rad: float):
    """Rotates a binary STL file around the Z-axis by angle_rad."""
    with open(file_path, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        data = f.read()

    record_dtype = np.dtype([
        ('normal', np.float32, (3,)),
        ('v1', np.float32, (3,)),
        ('v2', np.float32, (3,)),
        ('v3', np.float32, (3,)),
        ('attr', np.uint16)
    ])

    mesh = np.frombuffer(data, dtype=record_dtype).copy()

    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    for key in ['normal', 'v1', 'v2', 'v3']:
        x = mesh[key][:, 0].copy()
        y = mesh[key][:, 1].copy()
        mesh[key][:, 0] = x * cos_a - y * sin_a
        mesh[key][:, 1] = x * sin_a + y * cos_a

    with open(file_path, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', num_triangles))
        f.write(mesh.tobytes())

    print(f"✓ Rotated {os.path.basename(file_path)} by {np.rad2deg(angle_rad):.1f}° around Z-axis.")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 rotate_stl.py <file.stl> [angle_in_degrees, default=-90]")
        sys.exit(1)

    stl_file = sys.argv[1]
    deg = float(sys.argv[2]) if len(sys.argv) > 2 else -90.0
    rotate_stl_z(stl_file, np.deg2rad(deg))
