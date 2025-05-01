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
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ListGrippers,
    AddGripper,
    MoveToAbsolutePosition,
    Grip,
    Release,
)
from std_srvs.srv import Trigger
import asyncio
from threading import Thread
import time
from rclpy.service import Service
from functools import partial
from schunk_gripper_library.utility import Scheduler
from typing import TypedDict


class Gripper(TypedDict):
    host: str
    port: int
    serial_port: str
    device_id: int
    driver: GripperDriver
    gripper_id: str


class Driver(Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter("host", "")
        self.declare_parameter("port", 80)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("device_id", 12)
        self.scheduler: Scheduler = Scheduler()
        self.grippers: list[Gripper] = []
        gripper: Gripper = {
            "host": self.get_parameter("host").value,
            "port": self.get_parameter("port").value,
            "serial_port": self.get_parameter("serial_port").value,
            "device_id": self.get_parameter("device_id").value,
            "driver": GripperDriver(),
            "gripper_id": "",
        }
        self.grippers.append(gripper)
        self.gripper_services: list[Service] = []

        # Setup services
        self.add_gripper_srv = self.create_service(
            AddGripper, "~/add_gripper", self._add_gripper_cb
        )
        self.reset_grippers_srv = self.create_service(
            Trigger, "~/reset_grippers", self._reset_grippers_cb
        )

    def list_grippers(self) -> list[str]:
        devices = []
        for gripper in self.grippers:
            id = gripper["gripper_id"]
            if id:
                devices.append(id)
        return devices

    def add_gripper(
        self, host: str = "", port: int = 0, serial_port: str = "", device_id: int = 0
    ) -> bool:
        if not any([host, port, serial_port, device_id]):
            return False
        if (host and not port) or (port and not host):
            return False
        if (serial_port and not device_id) or (device_id and not serial_port):
            return False
        for gripper in self.grippers:
            if host:
                if host == gripper["host"] and port == gripper["port"]:
                    return False
            else:
                if (
                    serial_port == gripper["serial_port"]
                    and device_id == gripper["device_id"]
                ):
                    return False
        self.grippers.append(
            {
                "host": host,
                "port": port,
                "serial_port": serial_port,
                "device_id": device_id,
                "driver": GripperDriver(),
                "gripper_id": "",
            }
        )
        return True

    def reset_grippers(self) -> bool:
        self.grippers.clear()
        return True

    def needs_synchronize(self, gripper: Gripper) -> bool:
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
            if self.needs_synchronize(gripper):
                update_cycle = None
                self.scheduler.cyclic_execute(
                    func=partial(driver.receive_plc_input), cycle_time=0.05
                )
            else:
                update_cycle = 0.05
            if not driver.connect(
                host=gripper["host"],
                port=gripper["port"],
                serial_port=gripper["serial_port"],
                device_id=gripper["device_id"],
                update_cycle=update_cycle,
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

        # Start info services
        self.list_grippers_srv = self.create_service(
            ListGrippers, "~/list_grippers", self._list_grippers_cb
        )

        # Deactivate setup services
        self.destroy_service(self.add_gripper_srv)
        self.destroy_service(self.reset_grippers_srv)

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
                    partial(self._acknowledge_cb, gripper=gripper),
                )
            )
            self.gripper_services.append(
                self.create_service(
                    Trigger,
                    f"~/{gripper['gripper_id']}/fast_stop",
                    partial(self._fast_stop_cb, gripper=gripper),
                )
            )
            self.gripper_services.append(
                self.create_service(
                    MoveToAbsolutePosition,
                    f"~/{gripper['gripper_id']}/move_to_absolute_position",
                    partial(self._move_to_absolute_position_cb, gripper=gripper),
                )
            )
            self.gripper_services.append(
                self.create_service(
                    Grip,
                    f"~/{gripper['gripper_id']}/grip",
                    partial(self._grip_cb, gripper=gripper),
                )
            )
            self.gripper_services.append(
                self.create_service(
                    Release,
                    f"~/{gripper['gripper_id']}/release",
                    partial(self._release_cb, gripper=gripper),
                )
            )

        # Get every gripper ready to go
        for idx, gripper in enumerate(self.grippers):
            if self.needs_synchronize(gripper):
                success = asyncio.run(
                    self.grippers[idx]["driver"].acknowledge(scheduler=self.scheduler)
                )
            else:
                success = asyncio.run(self.grippers[idx]["driver"].acknowledge())
            if not success:
                return TransitionCallbackReturn.FAILURE

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
            gripper["driver"] = GripperDriver()

        # Release info services
        if not self.destroy_service(self.list_grippers_srv):
            return TransitionCallbackReturn.FAILURE

        # Reactivate setup services
        self.add_gripper_srv = self.create_service(
            AddGripper, "~/add_gripper", self._add_gripper_cb
        )
        self.reset_grippers_srv = self.create_service(
            Trigger, "~/reset_grippers", self._reset_grippers_cb
        )

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
    def _add_gripper_cb(
        self, request: AddGripper.Request, response: AddGripper.Response
    ):
        response.success = self.add_gripper(
            host=request.host,
            port=request.port,
            serial_port=request.serial_port,
            device_id=request.device_id,
        )
        return response

    def _reset_grippers_cb(self, request: Trigger.Request, response: Trigger.Response):
        response.success = self.reset_grippers()
        return response

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
        gripper: Gripper,
    ):
        self.get_logger().info("---> Acknowledge")
        if self.needs_synchronize(gripper):
            response.success = asyncio.run(
                gripper["driver"].acknowledge(scheduler=self.scheduler)
            )
        else:
            response.success = asyncio.run(gripper["driver"].acknowledge())
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _fast_stop_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().info("---> Fast stop")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].fast_stop(scheduler=self.scheduler)
        else:
            response.success = asyncio.run(gripper["driver"].fast_stop())
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _move_to_absolute_position_cb(
        self,
        request: MoveToAbsolutePosition.Request,
        response: MoveToAbsolutePosition.Response,
        gripper: Gripper,
    ):
        self.get_logger().info("---> Move to absolute position")
        position = int(request.position * 1e6)
        velocity = int(request.velocity * 1e6)
        if self.needs_synchronize(gripper):
            response.success = asyncio.run(
                gripper["driver"].move_to_absolute_position(
                    position=position,
                    velocity=velocity,
                    use_gpe=request.use_gpe,
                    scheduler=self.scheduler,
                )
            )
        else:
            response.success = asyncio.run(
                gripper["driver"].move_to_absolute_position(
                    position=position, velocity=velocity, use_gpe=request.use_gpe
                )
            )
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _grip_cb(
        self,
        request: Grip.Request,
        response: Grip.Response,
        gripper: Gripper,
    ):
        self.get_logger().info("---> Grip")
        if self.needs_synchronize(gripper):
            response.success = asyncio.run(
                gripper["driver"].grip(
                    force=request.force,
                    use_gpe=request.use_gpe,
                    outward=request.outward,
                    scheduler=self.scheduler,
                )
            )
        else:
            response.success = asyncio.run(
                gripper["driver"].grip(
                    force=request.force,
                    use_gpe=request.use_gpe,
                    outward=request.outward,
                )
            )
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _release_cb(
        self,
        request: Release.Request,
        response: Release.Response,
        gripper: Gripper,
    ):
        self.get_logger().info("---> Release")
        if self.needs_synchronize(gripper):
            response.success = asyncio.run(
                gripper["driver"].release(
                    use_gpe=request.use_gpe,
                    scheduler=self.scheduler,
                )
            )
        else:
            response.success = asyncio.run(
                gripper["driver"].release(
                    use_gpe=request.use_gpe,
                )
            )
        response.message = gripper["driver"].get_status_diagnostics()
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
