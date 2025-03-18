#!/usr/bin/env python3
# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

import rclpy

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from schunk_gripper_library.driver import Driver as GripperDriver
from std_srvs.srv import Trigger
import asyncio


class Driver(Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter("port", rclpy.Parameter.Type.STRING)
        self.declare_parameter("device_id", rclpy.Parameter.Type.INTEGER)
        self.port = self.get_parameter_or("port", "/dev/ttyUSB0").value
        self.gripper = GripperDriver()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        self.get_logger().info(f"Connecting on port {self.port}")
        if not self.gripper.connect(
            host="",
            port=self.get_parameter_or("port", "/dev/ttyUSB0").value,
            device_id=self.get_parameter_or("device_id", 12).value,
        ):
            self.get_logger().warn("Gripper connect failed")
            return TransitionCallbackReturn.FAILURE

        # Services
        self.acknowledge_srv = self.create_service(
            Trigger, "~/acknowledge", self._acknowledge_cb
        )
        self.fast_stop_srv = self.create_service(
            Trigger, "~/fast_stop", self._fast_stop_cb
        )

        # State update
        self.timer = self.create_timer(0.5, self.status_update)

        # Clear control bits
        self.gripper.clear_plc_output()
        self.gripper.set_control_bit(bit=0, value=True)
        self.gripper.send_plc_output()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")
        self.gripper.disconnect()

        # Release services
        if not self.destroy_service(self.acknowledge_srv):
            return TransitionCallbackReturn.FAILURE
        if not self.destroy_service(self.fast_stop_srv):
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")
        return TransitionCallbackReturn.SUCCESS

    def status_update(self):
        self.get_logger().info(f"---> Status update: {self.gripper.get_plc_input()}")

    # Service callbacks
    def _acknowledge_cb(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("---> Acknowledge")
        response.success = asyncio.run(self.gripper.acknowledge())
        response.message = self.gripper.get_status_diagnostics()
        return response

    def _fast_stop_cb(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("---> Fast stop")
        response.success = asyncio.run(self.gripper.fast_stop())
        response.message = self.gripper.get_status_diagnostics()
        return response


def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    driver = Driver("driver")
    executor.add_node(driver)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        driver.destroy_node()


if __name__ == "__main__":
    main()
