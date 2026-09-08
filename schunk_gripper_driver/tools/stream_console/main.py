#!/usr/bin/env python3
"""Minimal console tool for testing the target position stream.

Usage:
    main.py [gripper_id]

If gripper_id is omitted, the first gripper reported on
/schunk/driver/connection_state is used.

Controls:
    Left/Right arrow  -- decrease/increase target position by 1 mm
    a                 -- acknowledge
    q                 -- quit

Requires a sourced ROS 2 environment (rclpy). No extra pip packages needed.
"""
import curses
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from schunk_gripper_interfaces.msg import ConnectionState
from schunk_gripper_interfaces.srv import Stop, StopWithGPE
from std_srvs.srv import SetBool, Trigger

PUBLISH_RATE_HZ = 60.0
MAX_TARGET_SPEED_MPS = 0.005
TARGET_SPEED_RAMP_SEC = 0.25


class StreamConsole(Node):
    def __init__(self, gripper_id: str):
        super().__init__("stream_console")
        self.gripper_id = gripper_id
        self.target_position = 0.0
        self.actual_position = 0.0
        self._target_position_seeded = False
        self.target_publishing = True
        self.last_key = ""
        self.streaming_enabled = False

        self.publisher = self.create_publisher(
            Float32, f"/schunk/driver/{gripper_id}/stream/target_position", 1
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            f"/schunk/driver/{gripper_id}/joint_states",
            self._joint_state_cb,
            1,
        )
        self.publisher_timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self.publish_target_position
        )
        self.streaming_client = self.create_client(
            SetBool, f"/schunk/driver/{gripper_id}/set_stream"
        )
        self.acknowledge_client = self.create_client(
            Trigger, f"/schunk/driver/{gripper_id}/acknowledge"
        )
        self.stop_client = None

    def _joint_state_cb(self, msg: JointState) -> None:
        if not msg.position:
            return
        self.actual_position = msg.position[0]
        # Seed the initial target position only once; further updates must
        # come from key presses, not the (lagging) actual position.
        if not self._target_position_seeded:
            self.target_position = msg.position[0]
            self._target_position_seeded = True

    def enable_streaming(self) -> bool:
        return self.set_streaming(True)

    def disable_streaming(self) -> bool:
        return self.set_streaming(False)

    def set_streaming(self, enabled: bool) -> bool:
        if not self.streaming_client.wait_for_service(timeout_sec=5.0):
            return False
        request = SetBool.Request()
        request.data = enabled
        future = self.streaming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return False
        self.streaming_enabled = future.result().success
        if not self.streaming_enabled:
            return False
        self.streaming_enabled = enabled
        return True

    def publish_target_position(self) -> None:
        if not self._target_position_seeded or not self.target_publishing:
            return
        msg = Float32()
        msg.data = self.target_position
        self.publisher.publish(msg)

    def acknowledge(self) -> bool:
        if not self.acknowledge_client.wait_for_service(timeout_sec=1.0):
            return False
        future = self.acknowledge_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success

    def stop(self) -> bool:
        service_name = f"/schunk/driver/{self.gripper_id}/stop"
        service_types = dict(self.get_service_names_and_types()).get(service_name, [])
        if "schunk_gripper_interfaces/srv/Stop" in service_types:
            self.stop_client = self.create_client(Stop, service_name)
            request = Stop.Request()
        elif "schunk_gripper_interfaces/srv/StopWithGPE" in service_types:
            self.stop_client = self.create_client(StopWithGPE, service_name)
            request = StopWithGPE.Request()
        else:
            return False

        if not self.stop_client.wait_for_service(timeout_sec=1.0):
            return False
        future = self.stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success


def run_console(stdscr, node: StreamConsole) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)
    target_velocity_mps = 0.0
    last_update_time = time.monotonic()

    while rclpy.ok():
        key = stdscr.getch()
        now = time.monotonic()
        elapsed_sec = now - last_update_time
        last_update_time = now
        requested_velocity_mps = 0.0

        if key == curses.KEY_LEFT:
            node.target_publishing = True
            requested_velocity_mps = -MAX_TARGET_SPEED_MPS
            node.last_key = "LEFT"
        elif key == curses.KEY_RIGHT:
            node.target_publishing = True
            requested_velocity_mps = MAX_TARGET_SPEED_MPS
            node.last_key = "RIGHT"
        elif key in (ord("q"), ord("Q")):
            break
        elif key in (ord("a"), ord("A")):
            node.target_publishing = False
            target_velocity_mps = 0.0
            node.last_key = "A (acknowledge)"
            node.acknowledge()
        elif key in (ord("s"), ord("S")):
            node.target_publishing = False
            target_velocity_mps = 0.0
            node.last_key = "S (stop)"
            node.stop()
        elif key in (ord("e"), ord("E")):
            node.target_publishing = False
            target_velocity_mps = 0.0
            node.last_key = "E (enable stream)"
            node.enable_streaming()
        elif key in (ord("d"), ord("D")):
            node.target_publishing = False
            target_velocity_mps = 0.0
            node.last_key = "D (disable stream)"
            node.disable_streaming()
        elif key != -1:
            node.target_publishing = False
            target_velocity_mps = 0.0

        max_velocity_change = MAX_TARGET_SPEED_MPS * elapsed_sec / TARGET_SPEED_RAMP_SEC
        velocity_difference = requested_velocity_mps - target_velocity_mps
        velocity_change = max(-max_velocity_change, min(max_velocity_change, velocity_difference))
        target_velocity_mps += velocity_change
        node.target_position += target_velocity_mps * elapsed_sec

        stdscr.erase()
        stdscr.addstr(0, 0, f"Gripper: {node.gripper_id}")
        stdscr.addstr(1, 0, f"Streaming enabled: {node.streaming_enabled}")
        stdscr.addstr(3, 0, f"Target position: {node.target_position * 1000:.1f} mm ({PUBLISH_RATE_HZ:.1f} Hz)")
        stdscr.addstr(4, 0, f"Actual position: {node.actual_position * 1000:.1f} mm (2.0 Hz)")
        stdscr.addstr(6, 0, f"Last key: {node.last_key}")
        stdscr.addstr(8, 0, "Left/Right arrow: move")
        stdscr.addstr(9, 0, "Press 'a' key to acknowledge")
        stdscr.addstr(10, 0, "Press 's' key to stop")
        stdscr.addstr(11, 0, "Press 'e'/'d' key to enable/disable stream")
        stdscr.addstr(12, 0, "Press 'q' key to quit")
        stdscr.refresh()


def discover_first_gripper(timeout_sec: float = 5.0) -> str | None:
    node = rclpy.create_node("stream_console_discovery")
    found: list[str] = []

    def cb(msg: ConnectionState) -> None:
        if msg.grippers:
            found.append(msg.grippers[0])

    node.create_subscription(ConnectionState, "/schunk/driver/connection_state", cb, 1)

    deadline = time.time() + timeout_sec
    while rclpy.ok() and not found and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    return found[0] if found else None


def main() -> None:
    if len(sys.argv) > 2:
        print("Usage: main.py [gripper_id]")
        sys.exit(1)

    rclpy.init()

    if len(sys.argv) == 2:
        gripper_id = sys.argv[1]
    else:
        print("No gripper_id given, looking up first gripper from /schunk/driver/connection_state...")
        discovered = discover_first_gripper()
        if discovered is None:
            print("Error: no gripper found on /schunk/driver/connection_state.")
            rclpy.shutdown()
            sys.exit(1)
        gripper_id = discovered
        print(f"Using gripper '{gripper_id}'")

    node = StreamConsole(gripper_id)

    print(f"Enabling streaming mode for '{gripper_id}'...")
    if not node.enable_streaming():
        print("Failed to enable streaming mode. Continuing anyway.")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(run_console, node)
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
