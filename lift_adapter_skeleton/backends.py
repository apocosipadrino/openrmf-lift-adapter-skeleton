"""Lift backends: the vendor-facing side of the adapter.

A backend is the only part that talks to a specific lift: a REST API, a
gRPC service, a serial link, or -- for development and tests -- the
:class:`MockLiftBackend` in this module. The session controller only ever
sees the :class:`LiftBackend` interface, so swapping the mock for a real
lift means implementing three methods and nothing else.

To adapt this skeleton to a real lift, subclass :class:`LiftBackend` and
implement :meth:`get_state` and :meth:`command` against the vendor API.
Keep vendor quirks (retries, unit conversions, error mapping) inside the
backend; the protocol rules live in the session controller.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from .protocol import (
    DOOR_CLOSED,
    DOOR_MOVING,
    DOOR_OPEN,
    MODE_AGV,
    MODE_HUMAN,
    MOTION_DOWN,
    MOTION_STOPPED,
    MOTION_UP,
)


@dataclass
class BackendState:
    """What the backend reports about the lift, in protocol terms."""

    available_floors: list[str] = field(default_factory=list)
    current_floor: str = ""
    destination_floor: str = ""
    door_state: int = DOOR_CLOSED
    motion_state: int = MOTION_STOPPED
    current_mode: int = MODE_HUMAN


class LiftBackend(ABC):
    """Interface between the session controller and one physical lift."""

    @property
    @abstractmethod
    def lift_name(self) -> str:
        """Name of the lift, matching the building map."""

    @abstractmethod
    def get_state(self, now: float) -> BackendState | None:
        """Return the current lift state, or None if the lift is unreachable.

        Returning None must be transient-safe: the controller holds the last
        published state and keeps the session alive.
        """

    @abstractmethod
    def command(
        self,
        destination_floor: str,
        door_state: int,
        mode: int,
        now: float,
    ) -> bool:
        """Command the lift. Return False if the command was rejected."""


class MockLiftBackend(LiftBackend):
    """Simulated elevator driven by a virtual clock.

    Physics are deliberately simple and deterministic: one configurable
    travel time per floor, one door transit time, and a mode-switch delay.
    Time only advances when :meth:`get_state` is called, which makes the
    mock trivially testable: the caller controls the clock.

    Door behavior follows the protocol: in AGV mode the doors are held open
    whenever the cabin is stopped; in human mode the doors follow the last
    explicit door command.
    """

    def __init__(
        self,
        lift_name: str,
        floors: list[str],
        start_floor: str | None = None,
        seconds_per_floor: float = 2.0,
        door_seconds: float = 1.0,
        mode_switch_seconds: float = 1.0,
    ) -> None:
        if not floors:
            raise ValueError("floors must not be empty")
        if start_floor is not None and start_floor not in floors:
            raise ValueError(f"start_floor {start_floor!r} not in floors")
        self._lift_name = lift_name
        self._floors = list(floors)
        self._floor_idx = (
            self._floors.index(start_floor) if start_floor else 0
        )
        self._position = float(self._floor_idx)
        self._target_idx = self._floor_idx
        self._seconds_per_floor = seconds_per_floor
        self._door_seconds = door_seconds
        self._mode_switch_seconds = mode_switch_seconds
        self._door_pos = 0.0  # 0.0 closed, 1.0 open
        self._door_target = 0.0
        self._door_command = DOOR_CLOSED
        self._mode = MODE_HUMAN
        self._pending_mode: int | None = None
        self._mode_ready_at: float | None = None
        self._motion = MOTION_STOPPED
        self._last_now: float | None = None
        self._offline = False

    @property
    def lift_name(self) -> str:
        return self._lift_name

    def set_offline(self, offline: bool) -> None:
        """Simulate a comms outage: get_state() returns None while offline."""
        self._offline = offline

    def command(
        self,
        destination_floor: str,
        door_state: int,
        mode: int,
        now: float,
    ) -> bool:
        if destination_floor not in self._floors:
            return False
        self._target_idx = self._floors.index(destination_floor)
        self._door_command = door_state
        if mode != self._mode and mode != self._pending_mode:
            self._pending_mode = mode
            self._mode_ready_at = now + self._mode_switch_seconds
        self._step(now)
        return True

    def get_state(self, now: float) -> BackendState | None:
        if self._offline:
            return None
        self._step(now)
        return BackendState(
            available_floors=list(self._floors),
            current_floor=self._floors[self._floor_idx],
            destination_floor=self._floors[self._target_idx],
            door_state=self._door_state(),
            motion_state=self._motion,
            current_mode=self._mode,
        )

    def _door_state(self) -> int:
        if self._door_pos >= 1.0:
            return DOOR_OPEN
        if self._door_pos <= 0.0:
            return DOOR_CLOSED
        return DOOR_MOVING

    def _step(self, now: float) -> None:
        if self._last_now is None:
            self._last_now = now
            return
        dt = max(0.0, now - self._last_now)
        self._last_now = now
        if dt == 0.0:
            return

        # Mode switch completes on its own schedule.
        if (
            self._pending_mode is not None
            and self._mode_ready_at is not None
            and now >= self._mode_ready_at
        ):
            self._mode = self._pending_mode
            self._pending_mode = None
            self._mode_ready_at = None

        # Doors move toward their target. In AGV mode the target is always
        # open while stopped; in human mode it follows the last command.
        if self._mode == MODE_AGV and self._motion == MOTION_STOPPED:
            self._door_target = 1.0
        elif self._mode != MODE_AGV:
            self._door_target = (
                1.0 if self._door_command == DOOR_OPEN else 0.0
            )
        if self._target_idx != self._floor_idx:
            # Never travel with the doors open: close first.
            self._door_target = 0.0
        step = dt / self._door_seconds if self._door_seconds > 0 else 1.0
        if self._door_pos < self._door_target:
            self._door_pos = min(1.0, self._door_pos + step)
        elif self._door_pos > self._door_target:
            self._door_pos = max(0.0, self._door_pos - step)

        # Travel only with the doors fully closed.
        if self._target_idx == self._floor_idx or self._door_pos > 0.0:
            self._motion = MOTION_STOPPED
            return
        direction = 1 if self._target_idx > self._position else -1
        self._motion = MOTION_UP if direction > 0 else MOTION_DOWN
        travel = dt / self._seconds_per_floor
        self._position += direction * travel
        if (direction > 0 and self._position >= self._target_idx) or (
            direction < 0 and self._position <= self._target_idx
        ):
            self._position = float(self._target_idx)
            self._floor_idx = self._target_idx
            self._motion = MOTION_STOPPED
