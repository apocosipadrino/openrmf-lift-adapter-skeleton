"""Tests for the mock lift backend physics."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lift_adapter_skeleton.backends import MockLiftBackend  # noqa: E402
from lift_adapter_skeleton.protocol import (  # noqa: E402
    DOOR_CLOSED,
    DOOR_OPEN,
    MODE_AGV,
    MODE_HUMAN,
    MOTION_STOPPED,
    MOTION_UP,
)


def test_travel_timing():
    backend = MockLiftBackend("lift1", ["L1", "L2", "L3"])
    assert backend.command("L3", DOOR_CLOSED, MODE_HUMAN, 0.0)
    state = backend.get_state(2.0)
    assert state.current_floor == "L1", "one floor per 2s: still en route"
    assert state.motion_state == MOTION_UP
    state = backend.get_state(4.0)
    assert state.current_floor == "L3"
    assert state.motion_state == MOTION_STOPPED


def test_door_opens_in_agv_mode_when_stopped():
    backend = MockLiftBackend("lift1", ["L1", "L2", "L3"])
    assert backend.command("L1", DOOR_CLOSED, MODE_AGV, 0.0)
    state = backend.get_state(0.5)
    assert state.current_mode == MODE_HUMAN, "mode switch takes 1s"
    assert state.door_state == DOOR_CLOSED
    # Once AGV mode is active and the cabin is stopped, the doors are held
    # open even though the last explicit door command was CLOSED.
    state = backend.get_state(2.5)
    assert state.current_mode == MODE_AGV
    assert state.door_state == DOOR_OPEN


def test_doors_close_before_travel():
    backend = MockLiftBackend("lift1", ["L1", "L2", "L3"])
    assert backend.command("L1", DOOR_OPEN, MODE_AGV, 0.0)
    state = backend.get_state(2.5)
    assert state.door_state == DOOR_OPEN
    assert backend.command("L3", DOOR_OPEN, MODE_AGV, 2.5)
    state = backend.get_state(3.0)
    assert state.motion_state == MOTION_STOPPED, "must not move with doors open"
    state = backend.get_state(8.0)
    assert state.current_floor == "L3"


def test_unknown_floor_command_rejected():
    backend = MockLiftBackend("lift1", ["L1", "L2", "L3"])
    assert not backend.command("L9", DOOR_CLOSED, MODE_HUMAN, 0.0)
    state = backend.get_state(1.0)
    assert state.destination_floor == "L1"


def test_offline_returns_none():
    backend = MockLiftBackend("lift1", ["L1", "L2", "L3"])
    assert backend.get_state(0.0) is not None
    backend.set_offline(True)
    assert backend.get_state(1.0) is None
    backend.set_offline(False)
    assert backend.get_state(2.0) is not None
