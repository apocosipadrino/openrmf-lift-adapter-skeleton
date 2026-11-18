"""ROS 2 node: the thin glue between RMF topics and the session controller.

All protocol decisions live in :mod:`lift_adapter_skeleton.session` and all
lift specifics live behind :mod:`lift_adapter_skeleton.backends`. This node
only converts messages, owns the timers, and wires parameters to the core,
which is what makes the core testable without ROS.

Topics (defaults, overridable by parameter):
  * subscribes ``lift_requests`` (rmf_lift_msgs/msg/LiftRequest)
  * publishes ``lift_states`` (rmf_lift_msgs/msg/LiftState)

To use a real lift, implement :class:`LiftBackend` for the vendor API and
return it from :func:`build_backend` instead of the mock.
"""

from __future__ import annotations

import logging
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rmf_lift_msgs.msg import LiftRequest, LiftState

from .backends import LiftBackend, MockLiftBackend
from .protocol import LiftRequestData, LiftStateData
from .session import LiftSessionController


def build_backend(node: Node) -> LiftBackend:
    """Construct the lift backend from node parameters.

    The skeleton ships only the mock backend. Point this function at a real
    backend implementation to drive a physical lift; nothing else changes.
    """
    return MockLiftBackend(
        lift_name=node.get_parameter("lift_name").value,
        floors=node.get_parameter("floors").value,
        seconds_per_floor=node.get_parameter("seconds_per_floor").value,
        door_seconds=node.get_parameter("door_seconds").value,
        mode_switch_seconds=node.get_parameter("mode_switch_seconds").value,
    )


def ros_to_data(msg: LiftRequest) -> LiftRequestData:
    return LiftRequestData(
        lift_name=msg.lift_name,
        session_id=msg.session_id,
        request_type=msg.request_type,
        destination_floor=msg.destination_floor,
        door_state=msg.door_state,
    )


def data_to_ros(state: LiftStateData, stamp) -> LiftState:
    msg = LiftState()
    msg.lift_time = stamp
    msg.lift_name = state.lift_name
    msg.available_floors = state.available_floors
    msg.current_floor = state.current_floor
    msg.destination_floor = state.destination_floor
    msg.door_state = state.door_state
    msg.motion_state = state.motion_state
    msg.available_modes = state.available_modes
    msg.current_mode = state.current_mode
    msg.session_id = state.session_id
    return msg


class LiftAdapterNode(Node):
    """Subscribes to lift requests, publishes lift states."""

    def __init__(self) -> None:
        super().__init__("lift_adapter")
        self.declare_parameter("lift_name", "lift1")
        self.declare_parameter("floors", ["L1", "L2", "L3"])
        self.declare_parameter("seconds_per_floor", 2.0)
        self.declare_parameter("door_seconds", 1.0)
        self.declare_parameter("mode_switch_seconds", 1.0)
        self.declare_parameter("mode_switch_timeout", 30.0)
        self.declare_parameter("update_period", 0.1)
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("request_topic", "lift_requests")
        self.declare_parameter("state_topic", "lift_states")

        backend = build_backend(self)
        self._controller = LiftSessionController(
            lift_name=self.get_parameter("lift_name").value,
            backend=backend,
            mode_switch_timeout=self.get_parameter("mode_switch_timeout").value,
            logger=logging.getLogger("lift_adapter"),
        )
        self._latest: LiftStateData | None = None

        self._state_pub = self.create_publisher(
            LiftState,
            self.get_parameter("state_topic").value,
            qos_profile_system_default,
        )
        self._request_sub = self.create_subscription(
            LiftRequest,
            self.get_parameter("request_topic").value,
            self._request_callback,
            qos_profile_system_default,
        )
        self._update_timer = self.create_timer(
            self.get_parameter("update_period").value, self._update_callback
        )
        self._publish_timer = self.create_timer(
            self.get_parameter("publish_period").value, self._publish_callback
        )
        self.get_logger().info(
            "Lift adapter running for lift '%s'",
            self.get_parameter("lift_name").value,
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _request_callback(self, msg: LiftRequest) -> None:
        self._controller.handle_request(ros_to_data(msg), self._now())

    def _update_callback(self) -> None:
        state = self._controller.tick(self._now())
        if state is not None:
            self._latest = state

    def _publish_callback(self) -> None:
        if self._latest is None:
            return
        self._state_pub.publish(
            data_to_ros(self._latest, self.get_clock().now().to_msg())
        )


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv or sys.argv)
    node = LiftAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
