#!/usr/bin/env python3
# encoding: utf-8
"""Closed-circuit tests for the QR priority state machine.

These tests run without a physical camera, without ROS2, and without a display.
They cover the full data path:

  QR payload (string)
    -> qr_common.resolve_action()          [unit]
    -> LineTrackQRStateMachine._tick_qr()  [integration stub]
    -> Twist output / state transitions    [behavioural]

Run with:
    pytest src/yahboomcar_linefollow/test/test_qr_state_machine.py -v

or inside Docker:
    docker run --rm yahboomcar-linefollow-test
"""

import json
import os
import sys
import types
import time
import pytest

# ---------------------------------------------------------------------------
# Stub out cv2 so we can import qr_common without OpenCV installed.
# The stub makes QRCodeDetector return a known payload on demand.
# ---------------------------------------------------------------------------
_stub_payload = None   # test code sets this to control what the "camera" sees

_cv2_stub = types.ModuleType('cv2')
_cv2_stub.__version__ = '4.5.0'
_cv2_stub.error = Exception

class _FakeQRDetector:
    def detectAndDecode(self, frame):
        return (_stub_payload or '', None, None)

_cv2_stub.QRCodeDetector = _FakeQRDetector
sys.modules['cv2'] = _cv2_stub

# Now import the package modules (they live one level up from test/)
_pkg = os.path.join(os.path.dirname(__file__), '..', 'yahboomcar_linefollow')
sys.path.insert(0, os.path.dirname(_pkg))

from yahboomcar_linefollow.qr_common import (  # noqa: E402
    QRReader, load_actions, resolve_action, VALID_TYPES,
)


# ---------------------------------------------------------------------------
# Helper: a minimal QR action table
# ---------------------------------------------------------------------------
SAMPLE_TABLE = {
    'defaults': {
        'turn_speed': 0.6,
        'turn_time': 1.2,
        'cross_speed': 0.12,
        'cross_time': 0.6,
        'hold_time': 0.0,
    },
    'actions': {
        'FORK_LEFT':     {'type': 'left'},
        'FORK_RIGHT':    {'type': 'right'},
        'FORK_STRAIGHT': {'type': 'straight'},
        'STATION_A':     {'type': 'station'},
        'STATION_B':     {'type': 'station', 'hold_time': 5.0},
        'STOP':          {'type': 'stop'},
    },
}


# ===========================================================================
# Unit tests — qr_common.resolve_action()
# ===========================================================================

class TestResolveAction:
    def test_left(self):
        a = resolve_action('FORK_LEFT', SAMPLE_TABLE)
        assert a is not None
        assert a['type'] == 'left'

    def test_right(self):
        a = resolve_action('FORK_RIGHT', SAMPLE_TABLE)
        assert a['type'] == 'right'

    def test_straight(self):
        a = resolve_action('FORK_STRAIGHT', SAMPLE_TABLE)
        assert a['type'] == 'straight'

    def test_station_no_hold(self):
        a = resolve_action('STATION_A', SAMPLE_TABLE)
        assert a['type'] == 'station'
        assert a['hold_time'] == 0.0  # latched (hold_time=0)

    def test_station_with_hold(self):
        a = resolve_action('STATION_B', SAMPLE_TABLE)
        assert a['type'] == 'station'
        assert a['hold_time'] == 5.0

    def test_stop(self):
        a = resolve_action('STOP', SAMPLE_TABLE)
        assert a['type'] == 'stop'

    def test_unknown_key_returns_none(self):
        assert resolve_action('NO_SUCH_KEY', SAMPLE_TABLE) is None

    def test_empty_payload_returns_none(self):
        assert resolve_action('', SAMPLE_TABLE) is None
        assert resolve_action(None, SAMPLE_TABLE) is None

    def test_defaults_merged(self):
        a = resolve_action('FORK_LEFT', SAMPLE_TABLE)
        assert a['turn_speed'] == SAMPLE_TABLE['defaults']['turn_speed']
        assert a['turn_time'] == SAMPLE_TABLE['defaults']['turn_time']

    def test_action_overrides_default(self):
        a = resolve_action('STATION_B', SAMPLE_TABLE)
        assert a['hold_time'] == 5.0  # overrides default 0.0

    def test_inline_json_right(self):
        payload = '{"type": "right", "turn_time": 2.0}'
        a = resolve_action(payload, SAMPLE_TABLE)
        assert a['type'] == 'right'
        assert a['turn_time'] == 2.0

    def test_inline_json_station(self):
        payload = '{"type": "station", "hold_time": 3.5}'
        a = resolve_action(payload, SAMPLE_TABLE)
        assert a['type'] == 'station'
        assert a['hold_time'] == 3.5

    def test_inline_json_merged_with_defaults(self):
        payload = '{"type": "left"}'
        a = resolve_action(payload, SAMPLE_TABLE)
        assert a['turn_speed'] == SAMPLE_TABLE['defaults']['turn_speed']

    def test_inline_bad_json_falls_through_to_none(self):
        # starts with '{' but not valid JSON → falls through to key lookup → None
        result = resolve_action('{bad json}', SAMPLE_TABLE)
        assert result is None

    def test_inline_unknown_type_returns_none(self):
        result = resolve_action('{"type": "teleport"}', SAMPLE_TABLE)
        assert result is None

    def test_all_valid_types_accepted(self):
        for t in VALID_TYPES:
            a = resolve_action(f'{{"type": "{t}"}}', SAMPLE_TABLE)
            assert a is not None, f'type {t!r} should be valid'
            assert a['type'] == t


# ===========================================================================
# Unit tests — qr_common.load_actions()
# ===========================================================================

class TestLoadActions:
    def test_load_sample_file(self, tmp_path):
        p = tmp_path / 'actions.json'
        p.write_text(json.dumps(SAMPLE_TABLE))
        table = load_actions(str(p))
        assert len(table['actions']) == len(SAMPLE_TABLE['actions'])
        assert table['defaults']['turn_speed'] == 0.6

    def test_missing_file_returns_empty(self, tmp_path):
        table = load_actions(str(tmp_path / 'nonexistent.json'))
        assert table == {'actions': {}, 'defaults': {}}

    def test_empty_path_returns_empty(self):
        table = load_actions('')
        assert table == {'actions': {}, 'defaults': {}}

    def test_bad_json_returns_empty(self, tmp_path):
        p = tmp_path / 'bad.json'
        p.write_text('{not valid json}')
        table = load_actions(str(p))
        assert table == {'actions': {}, 'defaults': {}}

    def test_comment_key_preserved_but_ignored(self, tmp_path):
        data = dict(SAMPLE_TABLE)
        data['_comment'] = 'this is a comment'
        p = tmp_path / 'with_comment.json'
        p.write_text(json.dumps(data))
        table = load_actions(str(p))
        # _comment sits in root dict but actions/defaults are correct
        assert 'actions' in table
        assert 'defaults' in table


# ===========================================================================
# Unit tests — QRReader (stubbed cv2)
# ===========================================================================

class TestQRReader:
    def test_available(self):
        qr = QRReader()
        assert qr.available()

    def test_detect_none_frame(self):
        qr = QRReader()
        payload, pts = qr.detect(None)
        assert payload is None
        assert pts is None

    def test_detect_with_payload(self):
        global _stub_payload
        _stub_payload = 'FORK_LEFT'
        qr = QRReader()
        payload, _pts = qr.detect(object())  # frame doesn't matter with stub
        _stub_payload = None
        assert payload == 'FORK_LEFT'

    def test_detect_empty_returns_none(self):
        global _stub_payload
        _stub_payload = ''
        qr = QRReader()
        payload, _pts = qr.detect(object())
        _stub_payload = None
        assert payload is None


# ===========================================================================
# Behavioural / integration tests — QR state-machine logic
# (no ROS2, no camera: we simulate the state transitions directly)
# ===========================================================================

class FakeClock:
    """Monotonic clock that can be advanced manually in tests."""
    def __init__(self):
        self._t = 1_000_000_000  # 1 s in nanoseconds (arbitrary start)

    def now_ns(self):
        return self._t

    def advance(self, seconds):
        self._t += int(seconds * 1e9)


class FakeDuration:
    def __init__(self, ns):
        self._ns = ns

    def __add__(self, other):
        return FakeTime(self._ns + other._ns)


class FakeTime:
    """Comparable fake ROS time for until comparisons."""
    def __init__(self, ns):
        self._ns = ns

    def __lt__(self, other):
        return self._ns < other._ns

    def __ge__(self, other):
        return self._ns >= other._ns

    def __add__(self, other):
        ns = (other._ns if isinstance(other, FakeTime) else other)
        return FakeTime(self._ns + ns)

    def __sub__(self, other):
        return FakeTime(self._ns - other._ns)


class QRStateMachineHarness:
    """Minimal harness that replicates the QR state-machine from line_track
    without importing ROS2. Enough for behavioural testing."""

    def __init__(self, clock, table, cooldown_sec=4.0, check_every=1):
        self.clock = clock
        self.table = table
        self._state = None
        self._last_payload = None
        self._cooldown_until = FakeTime(0)
        self._cooldown_sec = cooldown_sec
        self._check_every = check_every
        self._tick_count = 0
        self._cmd_log = []     # (linear_x, angular_z) per tick
        self._log = []

    def _after(self, seconds):
        ns = self.clock.now_ns() + int(seconds * 1e9)
        return FakeTime(ns)

    def _now(self):
        return FakeTime(self.clock.now_ns())

    def _start_maneuver(self, payload, action):
        a_type = action['type']
        state = {'type': a_type, 'payload': payload}
        if a_type in ('left', 'right'):
            state['until'] = self._after(float(action.get('turn_time', 1.2)))
            state['turn_speed'] = float(action.get('turn_speed', 0.6))
            state['cross_speed'] = float(action.get('cross_speed', 0.12))
        elif a_type == 'straight':
            state['until'] = self._after(float(action.get('cross_time', 0.6)))
            state['cross_speed'] = float(action.get('cross_speed', 0.12))
        else:
            hold = float(action.get('hold_time', 0.0))
            state['until'] = self._after(hold) if hold > 0 else None
        self._state = state
        self._last_payload = payload
        self._cooldown_until = self._after(self._cooldown_sec)
        self._log.append(f'START {a_type} payload={payload}')

    def _maneuver_twist(self):
        st = self._state
        a_type = st['type']
        if a_type in ('station', 'stop'):
            if st['until'] is None:
                return (0.0, 0.0), True   # latched
            if self._now() >= st['until']:
                return (0.0, 0.0), False
            return (0.0, 0.0), True
        if self._now() >= st['until']:
            return (0.0, 0.0), False
        lx = st['cross_speed']
        az = st['turn_speed'] if a_type == 'left' else (
             -st['turn_speed'] if a_type == 'right' else 0.0)
        return (lx, az), True

    def tick(self, qr_payload=None):
        """Simulate one _tick with optional QR input. Returns (lx, az)."""
        now = self._now()
        # Detect QR
        self._tick_count += 1
        if qr_payload and self._tick_count % self._check_every == 0:
            if not (qr_payload == self._last_payload
                    and now < self._cooldown_until):
                action = resolve_action(qr_payload, self.table)
                if action is not None:
                    self._start_maneuver(qr_payload, action)

        # Execute
        if self._state is not None:
            cmd, still = self._maneuver_twist()
            if not still:
                self._log.append(f'DONE {self._state["type"]}')
                self._state = None
            self._cmd_log.append(cmd)
            return cmd
        self._cmd_log.append((0.15, 0.0))   # normal line follow
        return (0.15, 0.0)


class TestQRStateMachine:

    def _harness(self, **kw):
        return QRStateMachineHarness(FakeClock(), SAMPLE_TABLE, **kw)

    # -- Left-turn maneuver --------------------------------------------------

    def test_left_turn_starts_immediately(self):
        h = self._harness()
        cmd = h.tick('FORK_LEFT')
        assert h._state is not None
        assert h._state['type'] == 'left'
        lx, az = cmd
        assert lx == pytest.approx(0.12)   # cross_speed
        assert az > 0                       # turning left

    def test_left_turn_expires_after_turn_time(self):
        h = self._harness()
        h.tick('FORK_LEFT')
        assert h._state is not None
        h.clock.advance(1.3)   # > default turn_time=1.2
        expiry_cmd = h.tick()
        # On the tick where the maneuver expires, state is cleared and the
        # tick returns the terminal zero-Twist (not yet back to normal follow).
        assert h._state is None            # maneuver done
        assert expiry_cmd == (0.0, 0.0)   # terminal zero
        # Next tick: back to normal line follow
        resume_cmd = h.tick()
        assert resume_cmd == (0.15, 0.0)

    def test_right_turn_angular_is_negative(self):
        h = self._harness()
        _, az = h.tick('FORK_RIGHT')
        assert az < 0

    # -- Station / stop hold -------------------------------------------------

    def test_station_latched_stop_stays_stopped(self):
        h = self._harness()
        h.tick('STATION_A')     # hold_time=0 → latched
        assert h._state is not None
        assert h._state['until'] is None   # latched
        # Advance time far beyond any hold; car should still be stopped
        h.clock.advance(100)
        cmd = h.tick()
        assert cmd == (0.0, 0.0)
        assert h._state is not None        # still latched

    def test_station_timed_hold_expires(self):
        h = self._harness()
        h.tick('STATION_B')   # hold_time=5.0
        assert h._state is not None
        h.clock.advance(5.1)
        expiry_cmd = h.tick()  # maneuver expires this tick
        assert h._state is None            # state cleared
        assert expiry_cmd == (0.0, 0.0)   # terminal zero on expiry tick
        resume_cmd = h.tick()              # next tick: normal line follow
        assert resume_cmd == (0.15, 0.0)

    def test_new_qr_unlocks_latched_station(self):
        h = self._harness(cooldown_sec=0.0)  # no cooldown
        h.tick('STATION_A')    # latched
        assert h._state['until'] is None
        h.clock.advance(0.1)   # past cooldown (0s)
        h.tick('FORK_LEFT')    # new code → starts left maneuver
        assert h._state is not None
        assert h._state['type'] == 'left'

    # -- Cooldown prevents re-trigger ----------------------------------------

    def test_cooldown_suppresses_same_payload(self):
        h = self._harness(cooldown_sec=4.0)
        h.tick('STOP')
        first_state = h._state
        h.clock.advance(1.0)   # within cooldown
        h.tick('STOP')
        assert h._state is first_state     # not re-triggered

    def test_cooldown_expires_allows_retrigger(self):
        h = self._harness(cooldown_sec=2.0)
        h.tick('FORK_LEFT')
        h.clock.advance(2.5)   # > cooldown AND > turn_time
        h.tick()               # drain the timed maneuver
        assert h._state is None
        cmd = h.tick('FORK_LEFT')  # re-trigger after cooldown
        assert h._state is not None
        assert h._state['type'] == 'left'

    # -- No QR → normal line follow ------------------------------------------

    def test_no_qr_is_normal_line_follow(self):
        h = self._harness()
        for _ in range(5):
            cmd = h.tick()
            assert cmd == (0.15, 0.0)

    # -- Unknown QR → no maneuver --------------------------------------------

    def test_unknown_payload_no_maneuver(self):
        h = self._harness()
        cmd = h.tick('TOTALLY_UNKNOWN')
        assert h._state is None
        assert cmd == (0.15, 0.0)


# ===========================================================================
# Test the live qr_actions.json shipped with the package
# ===========================================================================

class TestShippedActionsFile:
    def test_file_exists_and_loads(self):
        here = os.path.dirname(__file__)
        params = os.path.join(here, '..', 'params', 'qr_actions.json')
        assert os.path.exists(params), f'qr_actions.json missing at {params}'
        table = load_actions(params)
        assert len(table['actions']) > 0

    def test_all_shipped_actions_valid(self):
        here = os.path.dirname(__file__)
        params = os.path.join(here, '..', 'params', 'qr_actions.json')
        table = load_actions(params)
        for key, act in table['actions'].items():
            merged = resolve_action(key, table)
            assert merged is not None, f'action {key!r} failed to normalize'
            assert merged['type'] in VALID_TYPES, \
                f'action {key!r} has unknown type {merged["type"]!r}'

    def test_fork_left_produces_positive_angular(self):
        here = os.path.dirname(__file__)
        params = os.path.join(here, '..', 'params', 'qr_actions.json')
        table = load_actions(params)
        a = resolve_action('FORK_LEFT', table)
        assert a['turn_speed'] > 0

    def test_fork_right_convention(self):
        here = os.path.dirname(__file__)
        params = os.path.join(here, '..', 'params', 'qr_actions.json')
        table = load_actions(params)
        a = resolve_action('FORK_RIGHT', table)
        assert a['type'] == 'right'
        # turn_speed is stored positive; caller negates for angular.z

    def test_station_a_is_latched(self):
        here = os.path.dirname(__file__)
        params = os.path.join(here, '..', 'params', 'qr_actions.json')
        table = load_actions(params)
        a = resolve_action('STATION_A', table)
        assert a['type'] == 'station'
        assert a['hold_time'] <= 0.0   # latch (hold_time=0)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
