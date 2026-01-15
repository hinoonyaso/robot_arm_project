#!/usr/bin/env python3
"""Generate basic PNG plots without external dependencies."""

import argparse
import json
import math
import struct
import zlib
from collections import Counter
from pathlib import Path


PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _chunk(chunk_type: bytes, data: bytes) -> bytes:
    length = struct.pack(">I", len(data))
    crc = struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
    return length + chunk_type + data + crc


def write_png(path: Path, width: int, height: int, pixels) -> None:
    raw = bytearray()
    for y in range(height):
        raw.append(0)  # no filter
        for x in range(width):
            raw.extend(pixels[y][x])
    compressed = zlib.compress(bytes(raw), level=9)
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    with path.open("wb") as handle:
        handle.write(PNG_SIGNATURE)
        handle.write(_chunk(b"IHDR", ihdr))
        handle.write(_chunk(b"IDAT", compressed))
        handle.write(_chunk(b"IEND", b""))


def blank_canvas(width: int, height: int, color=(255, 255, 255)):
    return [[list(color) for _ in range(width)] for _ in range(height)]


def draw_rect(canvas, x0, y0, x1, y1, color):
    height = len(canvas)
    width = len(canvas[0])
    for y in range(max(0, y0), min(height, y1)):
        for x in range(max(0, x0), min(width, x1)):
            canvas[y][x] = list(color)


def draw_axes(canvas, margin=40, color=(30, 30, 30)):
    height = len(canvas)
    width = len(canvas[0])
    draw_rect(canvas, margin, height - margin, width - margin, height - margin + 2, color)
    draw_rect(canvas, margin, margin, margin + 2, height - margin, color)


def normalize(values, max_value):
    return [v / max_value if max_value else 0 for v in values]


def plot_bars(values, out_path: Path, title_color, bar_color):
    width, height = 640, 420
    canvas = blank_canvas(width, height)
    draw_axes(canvas)
    margin = 60
    max_value = max(values) if values else 1
    norm = normalize(values, max_value)
    bar_width = int((width - 2 * margin) / max(1, len(values)))
    for idx, value in enumerate(norm):
        x0 = margin + idx * bar_width + 10
        x1 = x0 + bar_width - 20
        y1 = height - margin
        y0 = int(y1 - value * (height - 2 * margin))
        draw_rect(canvas, x0, y0, x1, y1, bar_color)
    draw_rect(canvas, 10, 10, 200, 30, title_color)
    write_png(out_path, width, height, canvas)


def plot_line(values, out_path: Path, line_color):
    width, height = 640, 420
    canvas = blank_canvas(width, height)
    draw_axes(canvas)
    margin = 60
    max_value = max(values) if values else 1
    norm = normalize(values, max_value)
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin
    for idx in range(1, len(norm)):
        x0 = margin + int((idx - 1) / max(1, len(norm) - 1) * plot_width)
        x1 = margin + int(idx / max(1, len(norm) - 1) * plot_width)
        y0 = height - margin - int(norm[idx - 1] * plot_height)
        y1 = height - margin - int(norm[idx] * plot_height)
        draw_rect(canvas, x0, min(y0, y1), x1 + 2, max(y0, y1) + 2, line_color)
    write_png(out_path, width, height, canvas)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_dir", type=str, required=True)
    parser.add_argument("--out", type=str, default="assets/plots")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    summaries = []
    for summary_path in run_dir.rglob("summary.json"):
        with summary_path.open("r", encoding="utf-8") as handle:
            summaries.append(json.load(handle))

    if not summaries:
        raise SystemExit("No summary.json files found")

    success_rates = [s.get("recovery_success_rate", 0) for s in summaries]
    planning_times = [s.get("planning_time_ms_mean", 0) for s in summaries]

    failure_counts = Counter()
    recovery_counts = Counter()
    max_joint_vel = []
    for summary in summaries:
        failure_counts.update(summary.get("failure_type_counts", {}))
        recovery_counts.update(summary.get("recovery_action_counts", {}))
        max_joint_vel.append(summary.get("max_joint_vel", 0))

    plot_line(success_rates, out_dir / "recovery_success_rate.png", (26, 115, 232))
    plot_bars(planning_times, out_dir / "planning_time_hist.png", (255, 183, 77), (91, 143, 249))
    plot_bars(list(failure_counts.values()), out_dir / "failure_type_bar.png", (255, 138, 101), (255, 192, 203))
    plot_bars(list(recovery_counts.values()), out_dir / "recovery_action_bar.png", (111, 211, 245), (90, 216, 166))
    plot_bars(max_joint_vel, out_dir / "max_joint_vel_box.png", (196, 196, 196), (120, 144, 255))


if __name__ == "__main__":
    main()
