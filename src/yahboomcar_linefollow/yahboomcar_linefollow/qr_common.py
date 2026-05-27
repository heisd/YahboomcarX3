#!/usr/bin/env python3
# encoding: utf-8
"""Shared QR-code utilities for the line-follow pipeline.

Provides:
  - QRReader: thin wrapper over cv2.QRCodeDetector (no zbar/pyzbar system dep)
  - load_actions(): read the JSON action table from disk
  - resolve_action(): turn a decoded QR payload into a normalized action dict

Fork-road semantics
--------------------
A QR code sitting at a fork tells the car what to do there. The normalized
action `type` is one of:

    left / right  -- take that branch of the fork (open-loop timed turn)
    straight      -- ignore the line briefly and drive across the fork
    station       -- this branch is a station: stop here (hold_time<=0 latches)
    stop          -- full stop / latch until re-enabled

"Stop is the other road with Station fork": at a fork one branch leads to a
station where the car must stop; the JSON behind each QR encodes which case
this particular fork is.
"""

import json
import os

import cv2 as cv


VALID_TYPES = ('left', 'right', 'straight', 'station', 'stop')


class QRReader:
    """Decode the first QR code in a BGR frame.

    detect() returns (payload, points) where payload is the decoded text
    (str) or None when nothing is found, and points are the 4 corner pts
    (or None) for optional overlay drawing.

    If the running OpenCV build does not include QR support
    (cv2.QRCodeDetector absent), the constructor sets self._det = None and
    detect() always returns (None, None) so the rest of the pipeline
    degrades gracefully rather than crashing.
    """

    def __init__(self):
        try:
            self._det = cv.QRCodeDetector()
        except AttributeError:
            self._det = None

    def available(self):
        return self._det is not None

    def detect(self, bgr_frame):
        if bgr_frame is None or self._det is None:
            return None, None
        try:
            payload, points, _ = self._det.detectAndDecode(bgr_frame)
        except (cv.error, Exception):
            return None, None
        if not payload:
            return None, points
        return payload, points


def load_actions(path):
    """Load the QR action JSON.

    Returns a dict {'actions': {...}, 'defaults': {...}}. Missing file or
    bad JSON yields an empty table (callers should log and carry on -- a
    missing table just means no QR maneuvers, not a crash).
    """
    if not path or not os.path.exists(path):
        return {'actions': {}, 'defaults': {}}
    try:
        with open(path, 'r') as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {'actions': {}, 'defaults': {}}
    if not isinstance(data, dict):
        return {'actions': {}, 'defaults': {}}
    data.setdefault('actions', {})
    data.setdefault('defaults', {})
    return data


def _normalize(action, defaults):
    """Merge an action dict over defaults and validate its type.

    Returns the normalized action dict, or None if the type is unknown.
    """
    if not isinstance(action, dict):
        return None
    a_type = str(action.get('type', '')).strip().lower()
    if a_type not in VALID_TYPES:
        return None
    merged = dict(defaults)
    merged.update(action)
    merged['type'] = a_type
    return merged


def resolve_action(payload, table):
    """Map a decoded QR payload to a normalized action dict (or None).

    Two payload shapes are accepted so the same code works whether the JSON
    lives inside the QR or in the on-disk table:
      1. The payload itself is a JSON object -> used directly as the action.
      2. The payload is a plain string key -> looked up in table['actions'].
    """
    if not payload:
        return None
    defaults = table.get('defaults', {}) if isinstance(table, dict) else {}
    actions = table.get('actions', {}) if isinstance(table, dict) else {}

    text = payload.strip()
    if text.startswith('{'):
        try:
            inline = json.loads(text)
        except ValueError:
            inline = None
        if inline is not None:
            return _normalize(inline, defaults)

    action = actions.get(text)
    if action is None:
        return None
    return _normalize(action, defaults)
