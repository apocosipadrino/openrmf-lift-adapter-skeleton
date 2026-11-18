"""Session controller: the RMF-facing side of the lift protocol.

The controller owns the rules that decide what the adapter publishes on
``lift_states`` and what it forwards to the lift backend:

1. **Session lock.** The first requester to ask for AGV or human mode owns
   the lift until it sends ``REQUEST_END_SESSION``. Requests from any other
   ``session_id`` are ignored while the lift is locked, including an
   end-session request from a non-owner.
2. **Session gating (the subtle one).** The ``session_id`` is only published
   in ``LiftState`` once the lift has actually finished switching into the
   requested mode. The fleet treats a matching ``session_id`` as "the lift
   is locked into my session and obeying my requests", so publishing it
   early lets a robot enter a lift that is still in human mode. This
   controller trusts the backend's reported ``current_mode``: the session
   is granted when the reported mode equals the requested mode.
3. **Floor validation.** A destination that is not in the last known
   ``available_floors`` is rejected; the lift keeps its previous target.
4. **Outage tolerance.** If the backend is unreachable, the last published
   state is held and the session is kept alive. A transient read failure
   must never drop a session or publish a fabricated state.

The controller is pure Python: no ROS imports, and time is injected as a
``float`` number of seconds so tests can drive it with a virtual clock.
"""

from __future__ import annotations

import logging

from .backends import LiftBackend
from .protocol import (
    DOOR_CLOSED,
    MODE_AGV,
    MODE_HUMAN,
    REQUEST_AGV_MODE,
    REQUEST_END_SESSION,
    REQUEST_HUMAN_MODE,
    LiftRequestData,
    LiftStateData,
)

_REQUESTED_MODE = {
    REQUEST_AGV_MODE: MODE_AGV,
    REQUEST_HUMAN_MODE: MODE_HUMAN,
}


class LiftSessionController:
    """Owns session state and translates requests into backend commands."""

    def __init__(
        self,
        lift_name: str,
        backend: LiftBackend,
        mode_switch_timeout: float = 30.0,
        logger: logging.Logger | None = None,
    ) -> None:
        self._lift_name = lift_name
        self._backend = backend
        self._mode_switch_timeout = mode_switch_timeout
        self._log = logger or logging.getLogger(__name__)
        self._session_id = ""
        self._session_mode: int | None = None
        self._pending_mode: int | None = None
        self._switch_started_at: float | None = None
        self._switch_timeout_logged = False
        self._last_published: LiftStateData | None = None
        self._last_floors: list[str] = []
        self._backend_down = False

    @property
    def session_id(self) -> str:
        """The session_id that currently owns the lift, or '' if free."""
        return self._session_id

    @property
    def is_locked(self) -> bool:
        return self._session_id != ""

    def handle_request(self, req: LiftRequestData, now: float) -> None:
        """Apply one LiftRequest. Unknown lifts and bad floors are ignored."""
        if req.lift_name != self._lift_name:
            self._log.debug("Ignoring request for lift %r", req.lift_name)
            return

        if req.request_type == REQUEST_END_SESSION:
            self._end_session(req, now)
            return

        target_mode = _REQUESTED_MODE.get(req.request_type)
        if target_mode is None:
            self._log.warning("Unknown request_type %r", req.request_type)
            return

        if self._session_id and req.session_id != self._session_id:
            self._log.info(
                "Lift %s locked by session %r; ignoring request from %r",
                self._lift_name,
                self._session_id,
                req.session_id,
            )
            return

        if self._last_floors and req.destination_floor not in self._last_floors:
            self._log.info(
                "Floor %r not available on lift %s",
                req.destination_floor,
                self._lift_name,
            )
            return

        starting = not self._session_id
        self._session_id = req.session_id
        if starting or self._session_mode != target_mode:
            self._session_mode = target_mode
            self._pending_mode = target_mode
            self._switch_started_at = now
            self._switch_timeout_logged = False
        if not self._backend.command(
            req.destination_floor, req.door_state, target_mode, now
        ):
            self._log.error(
                "Backend rejected command to floor %r",
                req.destination_floor,
            )

    def tick(self, now: float) -> LiftStateData | None:
        """Poll the backend and build the LiftState to publish.

        Returns None only when no state has ever been observed (for example
        the backend has been unreachable since startup).
        """
        backend_state = self._backend.get_state(now)
        if backend_state is None:
            if not self._backend_down:
                self._log.warning(
                    "Backend for lift %s unreachable; holding last state",
                    self._lift_name,
                )
                self._backend_down = True
            return self._last_published
        if self._backend_down:
            self._log.info("Backend for lift %s reachable again", self._lift_name)
            self._backend_down = False

        self._last_floors = list(backend_state.available_floors)

        if (
            self._pending_mode is not None
            and backend_state.current_mode == self._pending_mode
        ):
            self._pending_mode = None
            self._switch_started_at = None
        elif (
            self._pending_mode is not None
            and self._switch_started_at is not None
            and not self._switch_timeout_logged
            and now - self._switch_started_at > self._mode_switch_timeout
        ):
            self._log.error(
                "Lift %s has not reached mode %s after %.1fs; "
                "session stays ungranted",
                self._lift_name,
                self._pending_mode,
                self._mode_switch_timeout,
            )
            self._switch_timeout_logged = True

        # Grant the session only once the lift reports the requested mode.
        granted = (
            self._session_id != ""
            and self._pending_mode is None
            and self._session_mode is not None
            and backend_state.current_mode == self._session_mode
        )
        state = LiftStateData(
            lift_name=self._lift_name,
            available_floors=list(backend_state.available_floors),
            current_floor=backend_state.current_floor,
            destination_floor=backend_state.destination_floor,
            door_state=backend_state.door_state,
            motion_state=backend_state.motion_state,
            available_modes=[MODE_HUMAN, MODE_AGV],
            current_mode=backend_state.current_mode,
            session_id=self._session_id if granted else "",
        )
        self._last_published = state
        return state

    def _end_session(self, req: LiftRequestData, now: float) -> None:
        if self._session_id and req.session_id != self._session_id:
            self._log.info(
                "Ignoring end-session from %r; lift %s is owned by %r",
                req.session_id,
                self._lift_name,
                self._session_id,
            )
            return
        if not self._session_id:
            return
        self._log.info(
            "Session %r ended on lift %s; returning to human mode",
            self._session_id,
            self._lift_name,
        )
        self._session_id = ""
        self._session_mode = None
        self._pending_mode = None
        self._switch_started_at = None
        current = self._last_published.current_floor if self._last_published else ""
        destination = (
            self._last_published.destination_floor
            if self._last_published and self._last_published.destination_floor
            else current
        )
        if destination:
            # Finish the current trip, then idle in human mode.
            self._backend.command(destination, DOOR_CLOSED, MODE_HUMAN, now)
