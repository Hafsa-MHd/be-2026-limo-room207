#!/usr/bin/env python3
import os
import re

from ament_index_python.packages import get_package_share_directory
import xacro

path = os.path.join(
    get_package_share_directory("limo_car"), "gazebo", "ackermann_with_sensor.xacro"
)
xml = xacro.process_file(path).toxml()
meshes = sorted(set(re.findall(r"filename=['\"]([^'\"]+)['\"]", xml)))
print("MESH_COUNT", len(meshes))
missing = 0
for m in meshes:
    print(m)
    if m.startswith("package://"):
        rest = m[len("package://") :]
        pkg, _, rel = rest.partition("/")
        try:
            share = get_package_share_directory(pkg)
            full = os.path.join(share, rel)
            ok = os.path.exists(full)
            print("  ->", full, "EXISTS" if ok else "MISSING")
            if not ok:
                missing += 1
        except Exception as exc:
            print("  -> PKG_FAIL", exc)
            missing += 1
print("MISSING_COUNT", missing)

# Also list link status clues from URDF: which links have visuals
visuals = re.findall(
    r"<link name=\"([^\"]+)\">.*?<visual>.*?</visual>.*?</link>",
    xml,
    flags=re.DOTALL,
)
print("LINKS_WITH_VISUAL", len(visuals))
for name in visuals:
    print(" ", name)
