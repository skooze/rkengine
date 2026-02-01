#!/usr/bin/env python3
import math
import re
import sys

LOG_DEFAULT = "build_logs/rkg_play_movement.log"

RE_FLOAT = r"[-+]?(?:\d+\.\d+|\d+\.|\d+|\.\d+)(?:[eE][-+]?\d+)?"

def parse_vec2(tag, line):
    m = re.search(rf"{tag}=\(({RE_FLOAT}),({RE_FLOAT})\)", line)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2))

def parse_float(tag, line):
    m = re.search(rf"{tag}=({RE_FLOAT})", line)
    if not m:
        return None
    return float(m.group(1))

def parse_int(tag, line):
    m = re.search(rf"{tag}=([0-9]+)", line)
    if not m:
        return None
    return int(m.group(1))

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_DEFAULT
    try:
        lines = open(path, "r", encoding="utf-8").read().splitlines()
    except OSError as e:
        print(f"check_gait_log: failed to read {path}: {e}")
        return 2

    errors = 0
    cont_flips = 0
    midline_violations = 0
    radial_violations = 0
    knee_flips = 0
    knee_sign_l = None
    knee_sign_r = None
    prev_knee_lat_l = None
    prev_knee_lat_r = None

    for line in lines:
        if not line.startswith("gait entity="):
            continue

        grounded = parse_int("grounded", line)
        if grounded is None:
            continue

        knee_lat = parse_vec2("knee_lat", line)
        target_lat = parse_vec2("target_lat", line)
        target_rad = parse_vec2("target_rad", line)
        cont = parse_vec2("cont", line)
        min_side = parse_float("min_side", line)
        min_rad = parse_float("min_rad", line)

        if None in (knee_lat, target_lat, target_rad, cont, min_side, min_rad):
            # Skip older log format.
            continue

        knee_lat_l, knee_lat_r = knee_lat
        target_lat_l, target_lat_r = target_lat
        target_rad_l, target_rad_r = target_rad
        cont_l, cont_r = cont

        if cont_l < 0.2 or cont_r < 0.2:
            cont_flips += 1

        if grounded:
            # Midline clamps: left should be <= -min_side, right >= +min_side.
            if target_lat_l > -min_side:
                midline_violations += 1
            if target_lat_r < min_side:
                midline_violations += 1

            # Radial clamp in XZ.
            if target_rad_l < min_rad:
                radial_violations += 1
            if target_rad_r < min_rad:
                radial_violations += 1

            # Knee sign continuity while planted.
            if abs(knee_lat_l) > 1e-4:
                sign_l = 1 if knee_lat_l >= 0.0 else -1
                if knee_sign_l is not None and sign_l != knee_sign_l and abs(prev_knee_lat_l) > 1e-4:
                    knee_flips += 1
                knee_sign_l = sign_l
                prev_knee_lat_l = knee_lat_l
            if abs(knee_lat_r) > 1e-4:
                sign_r = 1 if knee_lat_r >= 0.0 else -1
                if knee_sign_r is not None and sign_r != knee_sign_r and abs(prev_knee_lat_r) > 1e-4:
                    knee_flips += 1
                knee_sign_r = sign_r
                prev_knee_lat_r = knee_lat_r

    if cont_flips > 0:
        print(f"check_gait_log: continuity dips detected: {cont_flips}")
        errors += 1
    if midline_violations > 0:
        print(f"check_gait_log: midline violations detected: {midline_violations}")
        errors += 1
    if radial_violations > 0:
        print(f"check_gait_log: radial violations detected: {radial_violations}")
        errors += 1
    if knee_flips > 0:
        print(f"check_gait_log: knee sign flips detected: {knee_flips}")
        errors += 1

    if errors == 0:
        print("check_gait_log: OK")
        return 0
    return 1

if __name__ == "__main__":
    sys.exit(main())
