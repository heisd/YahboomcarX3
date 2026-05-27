#!/usr/bin/env python3
# encoding: utf-8
"""Shared utilities for HSV-based line following.

Provides:
  - HSV file I/O (read_hsv / write_hsv)
  - hsv_from_roi(): learn an HSV range from a user-selected ROI
  - mask_with_hsv():  produce a clean binary mask given an HSV range
  - largest_contour_centroid(): pick the biggest blob and return its centroid
  - SimplePID
"""

import os
import cv2 as cv
import numpy as np


# ---------- HSV file ----------

def write_hsv(path, hsv_range):
    """hsv_range = ((Hmin,Smin,Vmin),(Hmax,Smax,Vmax))"""
    lo, hi = hsv_range
    with open(path, 'w') as f:
        f.write(','.join(str(v) for v in (*lo, *hi)))


def read_hsv(path):
    if not os.path.exists(path):
        return ()
    with open(path, 'r') as f:
        line = f.readline().strip()
    if not line:
        return ()
    parts = line.split(',')
    if len(parts) != 6:
        return ()
    v = [int(p) for p in parts]
    return ((v[0], v[1], v[2]), (v[3], v[4], v[5]))


# ---------- HSV learning from ROI ----------

def hsv_from_roi(bgr_img, roi, pad_h=5, pad_s=20, pad_v=20):
    """Compute an HSV inRange tuple from the pixels inside ROI.

    roi = (x_min, y_min, x_max, y_max) in image coords.
    pad_* widens the learned range so the mask is robust to lighting noise.
    """
    x0, y0, x1, y1 = roi
    if x1 <= x0 or y1 <= y0:
        return ()
    hsv = cv.cvtColor(bgr_img, cv.COLOR_BGR2HSV)
    patch = hsv[y0:y1, x0:x1].reshape(-1, 3)
    if patch.size == 0:
        return ()
    h_min, s_min, v_min = patch.min(axis=0)
    h_max, s_max, v_max = patch.max(axis=0)
    h_min = int(max(0,   int(h_min) - pad_h))
    h_max = int(min(179, int(h_max) + pad_h))
    s_min = int(max(0,   int(s_min) - pad_s))
    s_max = int(min(255, int(s_max) + pad_s))
    v_min = int(max(0,   int(v_min) - pad_v))
    v_max = int(min(255, int(v_max) + pad_v))
    return ((h_min, s_min, v_min), (h_max, s_max, v_max))


# ---------- masking ----------

def mask_with_hsv(bgr_img, hsv_range, blur=5, kernel=5):
    """Return a clean binary mask using inRange + close + open."""
    if not hsv_range:
        return np.zeros(bgr_img.shape[:2], dtype=np.uint8)
    img = bgr_img
    if blur and blur > 1:
        img = cv.GaussianBlur(img, (blur, blur), 0)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lo = np.array(hsv_range[0], dtype=np.uint8)
    hi = np.array(hsv_range[1], dtype=np.uint8)
    mask = cv.inRange(hsv, lo, hi)
    k = cv.getStructuringElement(cv.MORPH_RECT, (kernel, kernel))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, k)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, k)
    return mask


def count_blobs(binary_mask, min_area=200):
    """Number of contours at/above min_area -- >1 means the HSV range is
    matching more than one line (e.g. two colours), a common cause of the
    robot jumping between lines."""
    found = cv.findContours(binary_mask, cv.RETR_EXTERNAL,
                            cv.CHAIN_APPROX_SIMPLE)
    contours = found[1] if len(found) == 3 else found[0]
    return sum(1 for c in contours if cv.contourArea(c) >= min_area)


def largest_contour_centroid(binary_mask, min_area=200):
    """Return (cx, cy, area, contour) for the largest blob, or None."""
    found = cv.findContours(binary_mask, cv.RETR_EXTERNAL,
                            cv.CHAIN_APPROX_SIMPLE)
    contours = found[1] if len(found) == 3 else found[0]
    if not contours:
        return None
    cnt = max(contours, key=cv.contourArea)
    area = cv.contourArea(cnt)
    if area < min_area:
        return None
    m = cv.moments(cnt)
    if m['m00'] == 0:
        return None
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    return cx, cy, area, cnt


# ---------- PID ----------

class SimplePID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_clamp=1.0, out_clamp=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_clamp = i_clamp
        self.out_clamp = out_clamp
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, error):
        self.integral += error
        if self.i_clamp is not None:
            self.integral = max(-self.i_clamp, min(self.i_clamp, self.integral))
        d = error - self.prev_err
        self.prev_err = error
        out = self.kp * error + self.ki * self.integral + self.kd * d
        if self.out_clamp is not None:
            out = max(-self.out_clamp, min(self.out_clamp, out))
        return out
