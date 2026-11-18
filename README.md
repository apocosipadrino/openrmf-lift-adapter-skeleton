# OpenRMF Lift Adapter Skeleton

A skeleton [OpenRMF](https://github.com/open-rmf) lift adapter: a ROS 2 node
that speaks `rmf_lift_msgs` on one side and drives a lift on the other,
with the session protocol handled by a small, dependency-free core that can
be developed and tested without ROS.

This is a community skeleton, not part of OpenRMF. It was written from the
public `rmf_lift_msgs` definitions and the OpenRMF workshop examples; it
contains no vendor or employer code.

## What it is

- A **session controller** implementing the lift protocol rules: session
  lock, session gating (see below), floor validation, and outage tolerance.
- A **backend interface** (`LiftBackend`): implement three methods against
  a vendor API (REST, gRPC, serial, ...) and the adapter works unchanged.
- A **mock lift backend** with deterministic, clock-driven physics, so the
  whole loop runs and is testable on a laptop with no elevator and no ROS.
- A thin **ROS 2 node** (`rclpy`) that only converts messages, owns the
  timers, and wires parameters to the core.

## What it is not

- Not a production adapter: no retries, no metrics, no multi-lift fan-out
  (run one node per lift), no safety certification of any kind.
- Not yet run against a live ROS 2 / OpenRMF stack. The core logic is unit
  tested and the node wiring is tested against stubs (see Testing); live
  validation against `rmf_demos` is still to do.

## Where this fits: adapter vs node

In OpenRMF, "lift adapter" names a role, not a process: adapting one
specific lift to RMF's lift protocol. In a split deployment that role
spans two processes:

```
Fleet adapters ──LiftRequest──► Lift Adapter ──LiftRequest──► Lift Node ──API──► Lift Controller
                                (RMF-facing)   ◄──LiftState──  (hardware-facing)
```

This repository is the left half: the part that faces RMF, owns sessions,
and publishes states. The right half — the Lift Node that owns the vendor
API and the physical controller — is a separate process, and together the
pair *is* the lift adapter for that elevator. The companion repository
simulates the whole right-hand side so this half can be tested without
hardware; see `docs/ARCHITECTURE.md` for the wiring.

One vocabulary note: the `LiftRequest` messages are published by the fleet
adapter, not by RMF Core itself.

## The one rule that matters

The fleet treats a `LiftState` whose `session_id` matches its request as
"the lift is locked into my session and obeying my requests". If the
adapter publishes the `session_id` before the lift has actually finished
switching into AGV mode, a robot can enter a lift that is still in human
mode. This skeleton therefore **withholds the `session_id` until the
backend reports the requested mode**. See `docs/ARCHITECTURE.md` for the
full protocol rules and their rationale.

## Quickstart

Requires ROS 2 Humble or newer with `rmf_lift_msgs` (from OpenRMF's
`rmf_internal_msgs` repository) installed.

```bash
# Inside a ROS 2 workspace:
cd ~/ros2_ws/src
git clone https://github.com/apocosipadrino/openrmf-lift-adapter-skeleton.git
cd ~/ros2_ws
colcon build --packages-select lift_adapter_skeleton
source install/setup.bash
ros2 launch lift_adapter_skeleton lift_adapter.launch.py
```

Drive it from another terminal:

```bash
# Request AGV mode, session "demo", destination L3:
ros2 topic pub --once /lift_requests rmf_lift_msgs/msg/LiftRequest \
  "{lift_name: lift1, session_id: demo, request_type: 1, destination_floor: L3, door_state: 2}"

# Watch the states (note the session_id appearing only after the mode switch):
ros2 topic echo /lift_states

# End the session:
ros2 topic pub --once /lift_requests rmf_lift_msgs/msg/LiftRequest \
  "{lift_name: lift1, session_id: demo, request_type: 0, destination_floor: L3, door_state: 0}"
```

`request_type`: 0 = end session, 1 = AGV mode, 2 = human mode.
`door_state`: 0 = closed, 2 = open.

## Layout

```
lift_adapter_skeleton/
  protocol.py      Message shapes mirroring rmf_lift_msgs (no ROS imports)
  session.py       Session controller: lock, gating, validation, outages
  backends.py      LiftBackend interface + MockLiftBackend (simulated lift)
  adapter_node.py  rclpy node: topics, timers, parameters, conversion
launch/            Launch file (mock backend, 3 floors)
tests/             Unit tests + stub wiring test, runnable without ROS
docs/              Architecture notes and protocol rationale
```

## Adapting to a real lift

1. Subclass `LiftBackend` (`get_state`, `command`, `lift_name`) against the
   vendor API. Keep vendor quirks inside the backend.
2. Return it from `build_backend()` in `adapter_node.py`, reading whatever
   parameters (address, credentials, timeouts) it needs.
3. Set `lift_name` to match the building map, and `floors` to the floors the
   lift serves. Run one node per lift.

The controller never assumes the backend is fast or reliable: `get_state`
may return `None`, and a command may be rejected. Both are handled.

## Testing

```bash
python3 tests/run_tests.py   # no ROS, no pytest needed
pytest tests/                # also works
```

The suite covers the full session lifecycle, the session lock, floor
validation, non-owner end-session, backend outages, human-mode door
commands, mock physics, and the node wiring (against stub `rclpy` /
`rmf_lift_msgs` modules).

## Companion repository

[openrmf-lift-sim-skeleton](https://github.com/apocosipadrino/openrmf-lift-sim-skeleton)
is the other side of the same conversation: a simulated lift (cabin,
doors, and modes speaking `lift_requests` / `lift_states`) that stands in
for the Lift Node plus Lift Controller in the diagram above. Run the two
together to exercise an adapter end to end without Gazebo or a physical
elevator: run the sim on namespaced topics (`sim/lift_requests`,
`sim/lift_states`) and give this adapter a backend that bridges to them —
see `docs/ARCHITECTURE.md`.

## References

- `rmf_lift_msgs` definitions: `rmf_lift_msgs/msg/LiftState.msg` and
  `LiftRequest.msg` in
  [open-rmf/rmf_internal_msgs](https://github.com/open-rmf/rmf_internal_msgs).
- Adapter pattern reference:
  [open-rmf/roscon_workshop](https://github.com/open-rmf/roscon_workshop)
  (`roscon_lift_adapter`: adapter / API / manager split).
- Session-gating rationale: OpenRMF Discourse, "Lift session issues — robot
  enters lift while lift is still in human mode (#513)".
- Paper-level background: OpenRMF, "Robot Middleware Framework"
  ([open-rmf.org](https://www.open-rmf.org/)).

## License

MIT. See `LICENSE`.
