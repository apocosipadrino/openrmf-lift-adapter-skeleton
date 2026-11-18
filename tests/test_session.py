"""Tests for the session controller, driven with a virtual clock."""

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
    REQUEST_AGV_MODE,
    REQUEST_END_SESSION,
    REQUEST_HUMAN_MODE,
    LiftRequestData,
)
from lift_adapter_skeleton.session import LiftSessionController  # noqa: E402


def make_controller():
    backend = MockLiftBackend("lift1", ["L1", "L2", "L3"])
    controller = LiftSessionController("lift1", backend)
    return backend, controller


def advance(controller, start, end, step=0.1):
    state = None
    t = start
    while t < end:
        t = min(end, t + step)
        state = controller.tick(t)
    return state


def request(session, floor, request_type=REQUEST_AGV_MODE, door=DOOR_OPEN):
    return LiftRequestData(
        lift_name="lift1",
        session_id=session,
        request_type=request_type,
        destination_floor=floor,
        door_state=door,
    )


def test_full_agv_session():
    _, controller = make_controller()
    controller.handle_request(request("fleet1", "L3"), 0.0)

    # Mode switch is still pending: the session must NOT be published yet.
    state = controller.tick(0.0)
    assert state.session_id == "", "session granted before AGV mode switch"
    assert state.current_mode == MODE_HUMAN

    # Switch completes at t=1.0; the session is granted from then on.
    state = advance(controller, 0.0, 1.5)
    assert state.session_id == "fleet1"
    assert state.current_mode == MODE_AGV

    # Travel L1 -> L3 plus door opening finishes well before t=8.
    state = advance(controller, 1.5, 8.0)
    assert state.current_floor == "L3"
    assert state.door_state == DOOR_OPEN
    assert state.motion_state == MOTION_STOPPED
    assert state.session_id == "fleet1"

    # End of session: released immediately, human mode follows the switch.
    controller.handle_request(request("fleet1", "L3", REQUEST_END_SESSION), 8.0)
    state = controller.tick(8.0)
    assert state.session_id == ""
    state = advance(controller, 8.0, 10.0)
    assert state.current_mode == MODE_HUMAN
    assert not controller.is_locked


def test_session_lock_rejects_other_requesters():
    _, controller = make_controller()
    controller.handle_request(request("fleetA", "L2"), 0.0)
    controller.handle_request(request("fleetB", "L3"), 0.0)
    state = advance(controller, 0.0, 6.0)
    assert state.destination_floor == "L2", "locked lift obeyed another session"
    assert state.current_floor == "L2"
    assert state.session_id == "fleetA"


def test_unknown_floor_rejected():
    _, controller = make_controller()
    controller.tick(0.0)  # Learn the available floors first.
    controller.handle_request(request("fleet1", "L9"), 0.0)
    state = advance(controller, 0.0, 3.0)
    assert state.destination_floor == "L1"
    assert state.current_floor == "L1"


def test_end_session_from_non_owner_ignored():
    _, controller = make_controller()
    controller.handle_request(request("fleetA", "L2"), 0.0)
    advance(controller, 0.0, 2.0)
    controller.handle_request(request("fleetB", "L2", REQUEST_END_SESSION), 2.0)
    state = advance(controller, 2.0, 4.0)
    assert state.session_id == "fleetA"
    assert controller.is_locked


def test_backend_outage_holds_last_state():
    backend, controller = make_controller()
    controller.handle_request(request("fleet1", "L2"), 0.0)
    last = advance(controller, 0.0, 2.0)
    assert last is not None
    backend.set_offline(True)
    held = controller.tick(3.0)
    assert held == last, "outage must hold the last published state"
    backend.set_offline(False)
    fresh = controller.tick(4.0)
    assert fresh is not None
    assert fresh.session_id == "fleet1", "outage must not drop the session"


def test_human_mode_door_commands():
    _, controller = make_controller()
    controller.handle_request(
        request("fleet1", "L1", REQUEST_HUMAN_MODE, DOOR_OPEN), 0.0
    )
    state = advance(controller, 0.0, 2.0)
    assert state.current_mode == MODE_HUMAN
    assert state.door_state == DOOR_OPEN
    controller.handle_request(
        request("fleet1", "L1", REQUEST_HUMAN_MODE, DOOR_CLOSED), 2.0
    )
    state = advance(controller, 2.0, 4.0)
    assert state.door_state == DOOR_CLOSED
    assert state.session_id == "fleet1"


def test_request_for_another_lift_ignored():
    _, controller = make_controller()
    other = LiftRequestData(
        lift_name="liftX",
        session_id="fleet1",
        request_type=REQUEST_AGV_MODE,
        destination_floor="L3",
        door_state=DOOR_OPEN,
    )
    controller.handle_request(other, 0.0)
    state = advance(controller, 0.0, 2.0)
    assert state.session_id == ""
    assert not controller.is_locked
