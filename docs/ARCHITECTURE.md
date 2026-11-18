# Architecture

## Components

```
  rmf_lift_msgs topics                this repo                    the lift
  ─────────────────────   ───────────────────────────────────   ────────────

  lift_requests  ────────►  adapter_node  ──►  session controller ──► backend ──► vendor API
  (LiftRequest)             (rclpy glue)      (protocol rules)      (3 methods)   (REST/gRPC/…)
  lift_states  ◄────────  adapter_node  ◄──  session controller ◄── backend ◄── vendor API
  (LiftState)
```

| Component | Responsibility | Depends on ROS? |
|---|---|---|
| `protocol.py` | Message shapes + constants mirroring `rmf_lift_msgs` | No |
| `session.py` | Session lock, session gating, floor validation, outage holding | No |
| `backends.py` | `LiftBackend` interface; `MockLiftBackend` physics | No |
| `adapter_node.py` | Topics, timers, parameters, ROS/dataclass conversion | Yes (`rclpy`) |

The split is deliberate: everything that can be wrong about the *protocol*
lives in code that runs under plain `python3`, and the ROS node is too thin
to hide a protocol bug.

## Adapter vs node: where this repo fits

"lift adapter" names a role, not a process: adapting one specific lift to
RMF's lift protocol. In a split deployment that role spans two processes:

```
Fleet adapters ──LiftRequest──► Lift Adapter ──LiftRequest──► Lift Node ──API──► Lift Controller
                                (RMF-facing)   ◄──LiftState──  (hardware-facing)
```

- The **Lift Adapter** faces RMF: it receives requests, owns sessions,
  and publishes states. That is this repository.
- The **Lift Node** faces the hardware: it owns the vendor API and the
  physical controller.
- Together, the pair *is* the lift adapter for that elevator. Calling
  either half "the adapter" on its own invites exactly the confusion this
  split creates: both halves participate in adapting the lift, but neither
  is the whole adapter.

In the classic single-process pattern (the `roscon_workshop` reference),
one node speaks RMF messages on one side and the vendor API on the other,
with a manager behind a REST API; the diagram above is what that looks
like when the two halves run as separate processes and happen to exchange
`rmf_lift_msgs` between them.

Vocabulary note: the `LiftRequest` messages are published by the fleet
adapter, not by RMF Core itself. RMF Core (task dispatch, traffic
scheduling) decides *that* a robot needs the lift; the fleet adapter is
the component that speaks the lift protocol.

## Wiring the adapter to the simulated lift

The companion
[openrmf-lift-sim-skeleton](https://github.com/apocosipadrino/openrmf-lift-sim-skeleton)
repository simulates the whole right-hand side — Lift Node plus Lift
Controller — as one node. To chain it behind this adapter:

1. Run the sim on namespaced topics (`sim/lift_requests`,
   `sim/lift_states`) via its `request_topic` / `state_topic` parameters,
   so its messages never collide with the real lift topics.
2. Implement a bridging backend for this adapter: a `LiftBackend` that
   holds a subscription to `sim/lift_states` (caching the latest state
   for `get_state`) and a publisher to `sim/lift_requests` (translating
   `command` calls into `LiftRequest` messages). Construct it with the
   node in `build_backend()`; the node's executor drives its callbacks.

The bridge backend is deliberately not included here: it is ROS-aware by
nature, and the `LiftBackend` interface stays ROS-free so the session
controller and the mock backend remain testable without ROS. The topic
parameters on both sides already allow the wiring.

## Session lifecycle

```
Fleet / requester              Adapter                        Backend (lift)
      │                            │                                │
      │  AGV, session S, floor F   │                                │
      │ ──────────────────────────►│  lock S, command(F, AGV)       │
      │                            │ ──────────────────────────────►│
      │                            │  (mode switch in progress)     │
      │  LiftState, session ""     │                                │
      │ ◄──────────────────────────│  session withheld: mode != AGV │
      │                            │                                │
      │                            │  backend reports MODE_AGV      │
      │  LiftState, session S      │ ◄──────────────────────────────│
      │ ◄──────────────────────────│  session granted               │
      │                            │                                │
      │  ... cabin travels, doors open on arrival ...               │
      │                            │                                │
      │  END_SESSION, session S    │                                │
      │ ──────────────────────────►│  release S, command(HUMAN)     │
      │                            │ ──────────────────────────────►│
      │  LiftState, session ""     │                                │
      │ ◄──────────────────────────│                                │
```

## Protocol rules and rationale

1. **Session lock.** `LiftState.session_id` "records the session_id that
   has been granted control of the lift until it sends a request with a
   request_type of REQUEST_END_SESSION" (`LiftState.msg`). While a session
   is active, requests from any other `session_id` are ignored — including
   an end-session request. Only the owner can release the lift.
2. **Session gating.** The fleet enters the lift when it sees its own
   `session_id` together with the right floor, open doors, and AGV mode.
   Publishing the `session_id` before the mode switch completes lets a
   robot enter a lift that is still in human mode. The controller grants
   the session only when the backend *reports* the requested mode. The
   controller does not use its own timer for this: the backend's reported
   mode is the truth. A configurable timeout only logs an error; it never
   grants a session early.
3. **Floor validation.** `destination_floor` "must be one of the values
   returned in a LiftState" (`LiftRequest.msg`). Unknown floors are
   rejected before reaching the backend.
4. **Door semantics.** In AGV mode "the doors are always open when the lift
   is stopped" (`LiftRequest.msg`); explicit door requests are only
   meaningful in human mode. The mock backend implements exactly this; a
   real backend should preserve the distinction.
5. **Outage tolerance.** A `None` backend read holds the last published
   state and keeps the session. Publishing a fabricated state (or dropping
   a session) on a transient failure would be worse than going quiet for a
   cycle: the fleet keeps waiting on stale-but-true data instead of acting
   on invented data.

## Design decisions

- **One lift per node.** The controller is single-lift on purpose. Scaling
  to many lifts is a launch-file problem (one node per lift), not a
  reason to complicate the session logic.
- **Polling, not callbacks, for state.** The node polls the backend on a
  timer and publishes on a second timer, matching the workshop adapter.
  Polling keeps the "hold last state on outage" rule trivial and bounds
  publish rate independently of backend chatter.
- **Virtual clock in the core.** `tick(now)` / `get_state(now)` take time as
  a float. Tests drive the whole lifecycle deterministically with no
  sleeps and no ROS clock. The node supplies ROS time at the boundary.
- **Mock physics are honest about sequencing.** Doors close before travel,
  travel takes time, doors open after arrival, mode switches take time.
  A mock that teleports would let protocol bugs hide; this one cannot.

## Failure handling summary

| Failure | Behavior |
|---|---|
| Backend read returns `None` | Hold last state, keep session, log on transition |
| Backend rejects a command | Log; session and previous target unchanged |
| Request for another lift | Ignored |
| Request from a non-owner session | Ignored |
| End-session from a non-owner | Ignored |
| Unknown floor | Ignored, log |
| Unknown `request_type` | Ignored, log |
| Mode switch never completes | Session never granted; error logged after timeout |

## What to add for production

Retries with backoff in the backend, health metrics, multi-lift
supervision, persistent session recovery across restarts, and whatever the
vendor API demands (authentication, TLS, rate limits). None of those belong
in the session controller; they belong in the backend or in deployment.
