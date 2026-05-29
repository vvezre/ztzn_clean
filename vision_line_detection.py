#!/usr/bin/env python
# coding=utf-8

from __future__ import division

import math

import numpy as np


def normalize_visual_line_angle(angle_deg):
    try:
        angle = float(angle_deg)
    except Exception:
        return 0.0
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0
    return angle


def resolve_guidance_command(line_offset, line_angle, was_tracking=False):
    if line_offset is None:
        if was_tracking:
            return {
                "mode": "tracking_lost",
                "send": True,
                "tracking": False,
                "z_speed": 0,
            }
        return {
            "mode": "no_detection",
            "send": False,
            "tracking": False,
            "z_speed": None,
        }

    try:
        offset = float(line_offset)
    except Exception:
        offset = 0.0

    angle = normalize_visual_line_angle(line_angle)
    if abs(angle) > 10.0:
        z_speed = int(round(-angle * 2.0))
    else:
        z_speed = int(round(offset))

    return {
        "mode": "detected",
        "send": True,
        "tracking": True,
        "z_speed": z_speed,
    }


class GuidanceBandTracker(object):
    def __init__(
        self,
        max_abs_offset=None,
        max_offset_jump=25,
        max_width_change=8,
        min_stable_frames=3,
    ):
        self.max_abs_offset = None if max_abs_offset is None else float(max_abs_offset)
        self.max_offset_jump = float(max_offset_jump)
        self.max_width_change = float(max_width_change)
        self.min_stable_frames = int(min_stable_frames)
        self.last_offset = None
        self.last_width = None
        self.stable_frames = 0

    def reset(self):
        self.last_offset = None
        self.last_width = None
        self.stable_frames = 0

    def update(self, line):
        if line is None or line.get("mode") != "bright_band":
            self.reset()
            return None

        try:
            offset = float(line.get("offset", 0))
            width = float(line.get("width", 0))
        except Exception:
            self.reset()
            return None

        if self.max_abs_offset is not None and abs(offset) > self.max_abs_offset:
            self.reset()
            return None

        if self.last_offset is None:
            stable = True
        else:
            stable = (
                abs(offset - self.last_offset) <= self.max_offset_jump and
                abs(width - self.last_width) <= self.max_width_change
            )

        if stable:
            self.stable_frames += 1
        else:
            self.stable_frames = 1

        self.last_offset = offset
        self.last_width = width

        if self.stable_frames < self.min_stable_frames:
            return None
        return line


def _iter_true_runs(flags):
    start = None
    for index, flag in enumerate(flags):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            yield start, index - 1
            start = None
    if start is not None:
        yield start, len(flags) - 1


def _fill_small_column_gaps(column_hits):
    if len(column_hits) < 3:
        return column_hits
    filled = column_hits.copy()
    for index in range(1, len(filled) - 1):
        if not filled[index] and filled[index - 1] and filled[index + 1]:
            filled[index] = True
    return filled


def _estimate_band_angle(region_mask, x_offset):
    ys, xs = np.where(region_mask)
    if len(xs) < 2:
        return 0.0

    ys = ys.astype(np.float32)
    xs = xs.astype(np.float32) + float(x_offset)
    slope, _ = np.polyfit(ys, xs, 1)
    return normalize_visual_line_angle(math.degrees(math.atan(float(slope))))


def _compute_bright_threshold(gray, min_bright_value):
    peak = int(np.percentile(gray, 99.5))
    return max(int(min_bright_value), int(peak * 0.72))


def find_vertical_bright_band(
    gray,
    center_x,
    min_bright_value=180,
    min_band_width=12,
    min_band_height=120,
    min_column_ratio=0.25,
    max_vertical_angle=45.0
):
    if gray is None or getattr(gray, "size", 0) == 0:
        return None

    height, width = gray.shape[:2]
    if height <= 0 or width <= 0:
        return None

    threshold = _compute_bright_threshold(gray, min_bright_value)
    bright_mask = gray >= threshold
    column_min_hits = max(int(round(height * float(min_column_ratio))), min_band_height // 2)
    column_hits = bright_mask.sum(axis=0) >= column_min_hits
    column_hits = _fill_small_column_gaps(column_hits)

    best_band = None
    best_score = None

    for start_x, end_x in _iter_true_runs(column_hits):
        band_width = end_x - start_x + 1
        if band_width < min_band_width:
            continue

        region_mask = bright_mask[:, start_x:end_x + 1]
        rows = np.where(region_mask.any(axis=1))[0]
        if len(rows) == 0:
            continue

        band_height = rows[-1] - rows[0] + 1
        if band_height < min_band_height:
            continue

        center = (start_x + end_x) / 2.0
        angle = _estimate_band_angle(region_mask, start_x)
        if abs(angle) > max_vertical_angle:
            continue

        score = (abs(center - float(center_x)), -band_width, -band_height)
        if best_score is None or score < best_score:
            best_score = score
            best_band = {
                "start_x": start_x,
                "end_x": end_x,
                "center_x": center,
                "width": band_width,
                "height": band_height,
                "angle": angle,
                "threshold": threshold,
            }

    return best_band
