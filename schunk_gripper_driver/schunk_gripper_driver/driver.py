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

from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from schunk_gripper_library.driver import Driver as GripperDriver
from schunk_gripper_interfaces.srv import ListGrippers  # type: ignore [attr-defined]
from std_srvs.srv import Trigger
import asyncio
from threading import Thread
import time
from rclpy.service import Service
from functools import partial
from schunk_gripper_library.utility import Scheduler


class Driver(Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter("host", "")
        self.declare_parameter("port", 80)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("device_id", 12)
        self.scheduler: Scheduler = Scheduler()
        self.grippers = []
        gripper = {
            "host": self.get_parameter("host").value,
            "port": self.get_parameter("port").value,
            "serial_port": self.get_parameter("serial_port").value,
            "device_id": self.get_parameter("device_id").value,
            "driver": None,
            "gripper_id": None,
        }
        self.grippers.append(gripper)
        self.gripper_services: list[Service] = []

    def list_grippers(self) -> list[str]:
        devices = []
        for gripper in self.grippers:
            id = gripper["gripper_id"]
            if id:
                devices.append(id)
        return devices

    def needs_synchronize(self, gripper: dict[str, str]) -> bool:
        serial_ports = [gripper["serial_port"] for gripper in self.grippers]
        if serial_ports.count(gripper["serial_port"]) > 1:
            return True
        return False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        self.scheduler.start()

        # Connect each gripper
        for idx, gripper in enumerate(self.grippers):
            driver = GripperDriver()
            if not driver.connect(
                host=gripper["host"],
                port=gripper["port"],
                serial_port=gripper["serial_port"],
                device_id=gripper["device_id"],
            ):
                self.get_logger().warn(f"Gripper connect failed: {gripper}")
                return TransitionCallbackReturn.FAILURE
            else:
                self.grippers[idx]["driver"] = driver

        # Set unique gripper IDs
        devices = []
        for idx, gripper in enumerate(self.grippers):
            id = f"{gripper['driver'].module_type}_1"
            while id in devices:
                count = int(id.split("_")[-1]) + 1
                id = id[:-2] + f"_{count}"
            devices.append(id)
            self.grippers[idx]["gripper_id"] = id

        # Driver-wide services
        self.list_grippers_srv = self.create_service(
            ListGrippers, "~/list_grippers", self._list_grippers_cb
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")

        # Gripper-specific services
        for idx, _ in enumerate(self.grippers):
            gripper = self.grippers[idx]
            self.gripper_services.append(
                self.create_service(
                    Trigger,
                    f"~/{gripper['gripper_id']}/acknowledge",
                    partial(self._acknowledge_cb, gripper=gripper["driver"]),
                )
            )
            self.gripper_services.append(
                self.create_service(
                    Trigger,
                    f"~/{gripper['gripper_id']}/fast_stop",
                    partial(self._fast_stop_cb, gripper=gripper["driver"]),
                )
            )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")

        # Release gripper-specific services
        for idx, _ in enumerate(self.gripper_services):
            self.destroy_service(self.gripper_services[idx])
        self.gripper_services.clear()

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")
        self.scheduler.stop()
        for gripper in self.grippers:
            gripper["driver"].disconnect()
            gripper["driver"] = None

        # Release driver-wide services
        if not self.destroy_service(self.list_grippers_srv):
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")

        def kill() -> None:
            time.sleep(0.1)
            self.destroy_node()
            rclpy.shutdown()

        destroy = Thread(target=kill, daemon=True)
        destroy.start()
        return TransitionCallbackReturn.SUCCESS

    # Service callbacks
    def _list_grippers_cb(
        self, request: ListGrippers.Request, response: ListGrippers.Response
    ):
        self.get_logger().info("---> List gripper IDs")
        response.grippers = self.list_grippers()
        return response

    def _acknowledge_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: GripperDriver,
    ):
        self.get_logger().info("---> Acknowledge")
        response.success = asyncio.run(gripper.acknowledge())
        response.message = gripper.get_status_diagnostics()
        return response

    def _fast_stop_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: GripperDriver,
    ):
        self.get_logger().info("---> Fast stop")
        response.success = asyncio.run(gripper.fast_stop())
        response.message = gripper.get_status_diagnostics()
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
