#!/usr/bin/env python3
"""
Rotate waypoint yaw by 180 degrees for a selected point range.

Waypoint format:
  # p_x, p_y, p_z, q_x, q_y, q_z, q_w
  0.0, 0.0, -1.0, 0.001, 0.014, -0.001, 1.000

Examples:
  # Rotate points 73 through 141 in-place, with waypoint.txt.bak backup.
  python3 translate_yaw180.py --start 73 --end 141

  # Write to a new file instead of replacing waypoint.txt.
  python3 translate_yaw180.py --start 73 --end 141 --output waypoint_new.txt

  # Use 0-based data-point indexes.
  python3 translate_yaw180.py --start 72 --end 140 --index-base 0
"""

import argparse
import math
import shutil
from pathlib import Path


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("zero-length quaternion generated")

    return qx / norm, qy / norm, qz / norm, qw / norm


def parse_waypoint_line(line):
    content = line.split("#", 1)[0].strip()
    if not content:
        return None

    parts = [part.strip() for part in content.replace(",", " ").split()]
    if len(parts) != 7:
        return None

    return [float(part) for part in parts]


def format_waypoint(values):
    px, py, pz, qx, qy, qz, qw = values
    return (
        f"{px:.3f}, {py:.3f}, {pz:.3f}, "
        f"{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}\n"
    )


def rotate_yaw_180(values):
    px, py, pz, qx, qy, qz, qw = values
    roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)
    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, normalize_angle(yaw + math.pi))
    return [px, py, pz, qx, qy, qz, qw]


def rotate_file(input_path, output_path, start_idx, end_idx, make_backup):
    lines = input_path.read_text(encoding="utf-8").splitlines(keepends=True)
    output_lines = []
    data_idx = 0
    changed = 0

    for line in lines:
        values = parse_waypoint_line(line)
        if values is None:
            output_lines.append(line)
            continue

        if start_idx <= data_idx <= end_idx:
            values = rotate_yaw_180(values)
            output_lines.append(format_waypoint(values))
            changed += 1
        else:
            output_lines.append(line)

        data_idx += 1

    if changed == 0:
        raise ValueError("selected range did not match any waypoint data rows")

    if output_path == input_path and make_backup:
        backup_path = input_path.with_suffix(input_path.suffix + ".bak")
        shutil.copy2(input_path, backup_path)

    output_path.write_text("".join(output_lines), encoding="utf-8")
    return data_idx, changed


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Rotate yaw by 180 degrees for a selected waypoint range."
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="waypoint.txt",
        help="input waypoint file, default: waypoint.txt",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="output file, default: overwrite input file",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=1,
        help="first waypoint index in the selected range, default: 1",
    )
    parser.add_argument(
        "--end",
        type=int,
        help="last waypoint index in the selected range, default: last waypoint",
    )
    parser.add_argument(
        "--index-base",
        type=int,
        choices=(0, 1),
        default=1,
        help="whether --start/--end use 0-based or 1-based indexes, default: 1",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="do not create input.bak when overwriting input",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()

    input_path = Path(args.input)
    if not input_path.is_absolute():
        input_path = Path(__file__).resolve().parent / input_path
    input_path = input_path.resolve()

    if not input_path.exists():
        raise FileNotFoundError(input_path)

    start_idx = args.start - args.index_base
    if start_idx < 0:
        raise ValueError("--start is before the first waypoint")

    if args.end is None:
        end_idx = 10**18
    else:
        end_idx = args.end - args.index_base
        if end_idx < start_idx:
            raise ValueError("--end must be greater than or equal to --start")

    if args.output:
        output_path = Path(args.output)
        if not output_path.is_absolute():
            output_path = input_path.parent / output_path
        output_path = output_path.resolve()
    else:
        output_path = input_path

    total, changed = rotate_file(
        input_path=input_path,
        output_path=output_path,
        start_idx=start_idx,
        end_idx=end_idx,
        make_backup=not args.no_backup,
    )

    display_start = start_idx + args.index_base
    display_end = min(end_idx, total - 1) + args.index_base
    print(
        f"Rotated yaw by 180 deg for {changed}/{total} waypoint(s): "
        f"{display_start}..{display_end}. Saved to {output_path}"
    )


if __name__ == "__main__":
    main()
