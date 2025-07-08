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
import rclpy.logging
from schunk_gripper_library.driver import AsyncDriver as GripperDriver
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ListGrippers,
    AddGripper,
    MoveToAbsolutePosition,
    Grip,
    Release,
    ShowConfiguration,
    ShowGripperSpecification,
)
from schunk_gripper_interfaces.msg import (  # type: ignore [attr-defined]
    Gripper as GripperConfig,
    GripperState,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import asyncio
from threading import Thread, Timer as Countdown
from rclpy.service import Service
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from functools import partial
from schunk_gripper_library.utility import Scheduler
from typing import TypedDict
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult


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
        self.declare_parameter("log_level", "INFO")

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
        self.joint_state_publishers: dict[str, Publisher] = {}
        self.gripper_state_publishers: dict[str, Publisher] = {}
        self.gripper_timers: list[Timer] = []

        # Setup services
        self.add_gripper_srv = self.create_service(
            AddGripper, "~/add_gripper", self._add_gripper_cb
        )
        self.reset_grippers_srv = self.create_service(
            Trigger, "~/reset_grippers", self._reset_grippers_cb
        )
        self.show_configuration_srv = self.create_service(
            ShowConfiguration, "~/show_configuration", self._show_configuration_cb
        )
        self.add_on_set_parameters_callback(self._param_cb)

        # For concurrently running publishers
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Process async calls in the background
        self.loop = asyncio.new_event_loop()
        self.loop_thread = Thread(target=self.loop.run_forever, daemon=True)
        self.loop_thread.start()

    def list_grippers(self) -> list[str]:
        devices = []
        for gripper in self.grippers:
            id = gripper["gripper_id"]
            if id:
                devices.append(id)
        return devices

    def show_configuration(self) -> list[GripperConfig]:
        configuration = []
        for gripper in self.grippers:
            cfg = GripperConfig()
            cfg.host = gripper["host"]
            cfg.port = gripper["port"]
            cfg.serial_port = gripper["serial_port"]
            cfg.device_id = gripper["device_id"]
            configuration.append(cfg)
        return configuration

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
        self.get_logger().debug("on_configure() is called.")
        if not self.grippers:
            return TransitionCallbackReturn.FAILURE
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
            if not asyncio.run_coroutine_threadsafe(
                driver.connect(
                    host=gripper["host"],
                    port=gripper["port"],
                    serial_port=gripper["serial_port"],
                    device_id=gripper["device_id"],
                    update_cycle=update_cycle,
                ),
                self.loop,
            ).result():
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
        self.destroy_service(self.show_configuration_srv)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")

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

            self.gripper_services.append(
                self.create_service(
                    ShowGripperSpecification,
                    f"~/{gripper['gripper_id']}/show_gripper_specification",
                    partial(self._show_gripper_specification_cb, gripper=gripper),
                )
            )

        # Publishers for each gripper
        self.gripper_timers.clear()
        for idx, _ in enumerate(self.grippers):
            gripper = self.grippers[idx]
            gripper_id = gripper["gripper_id"]

            # Joint states
            self.joint_state_publishers[gripper_id] = self.create_publisher(
                msg_type=JointState,
                topic=f"~/{gripper_id}/joint_states",
                qos_profile=1,
                callback_group=self.callback_group,
            )
            self.gripper_timers.append(
                self.create_timer(
                    timer_period_sec=gripper["driver"].update_cycle,
                    callback=partial(self._publish_joint_states, gripper=gripper),
                    callback_group=self.callback_group,
                )
            )
            # Gripper state
            self.gripper_state_publishers[gripper_id] = self.create_publisher(
                msg_type=GripperState,
                topic=f"~/{gripper_id}/gripper_state",
                qos_profile=1,
                callback_group=self.callback_group,
            )
            self.gripper_timers.append(
                self.create_timer(
                    timer_period_sec=gripper["driver"].update_cycle,
                    callback=partial(self._publish_gripper_state, gripper=gripper),
                    callback_group=self.callback_group,
                )
            )

        # Get every gripper ready to go
        for idx, gripper in enumerate(self.grippers):
            if self.needs_synchronize(gripper):
                success = asyncio.run_coroutine_threadsafe(
                    self.grippers[idx]["driver"].acknowledge(scheduler=self.scheduler),
                    self.loop,
                ).result()
            else:
                success = asyncio.run_coroutine_threadsafe(
                    self.grippers[idx]["driver"].acknowledge(), self.loop
                ).result()
            if not success:
                return TransitionCallbackReturn.FAILURE

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")

        # Release gripper-specific services
        for idx, _ in enumerate(self.gripper_services):
            self.destroy_service(self.gripper_services[idx])
        self.gripper_services.clear()

        # Remove gripper-specific publishers
        for idx, _ in enumerate(self.gripper_timers):
            self.gripper_timers[idx].cancel()
        for gripper in self.list_grippers():
            self.destroy_publisher(self.joint_state_publishers.pop(gripper))
            self.destroy_publisher(self.gripper_state_publishers.pop(gripper))

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_cleanup() is called.")
        self.scheduler.stop()
        for gripper in self.grippers:
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].disconnect(), self.loop
            ).result()
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
        self.show_configuration_srv = self.create_service(
            ShowConfiguration, "~/show_configuration", self._show_configuration_cb
        )

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_shutdown() is called.")
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join()
        self.loop.close()
        if state is None:
            return TransitionCallbackReturn.SUCCESS

        Countdown(0.5, function=rclpy.try_shutdown).start()
        return TransitionCallbackReturn.SUCCESS

    def _param_cb(self, params):
        valid_log_levels = [
            "DEBUG",
            "INFO",
            "WARN",
            "ERROR",
            "FATAL",
        ]
        for p in params:
            if p.name == "log_level":
                if p.value not in valid_log_levels:
                    self.get_logger().error(
                        f"Invalid log level: {str(p.value)}. "
                        f"Valid options are: {valid_log_levels}"
                    )
                    return SetParametersResult(successful=False)
                level = rclpy.logging.get_logging_severity_from_string(p.value)
                self.get_logger().set_level(level)
                self.get_logger().debug(f"Log level changed to {p.value}")
                return SetParametersResult(successful=True)

        return SetParametersResult(successful=True)

    def _publish_joint_states(self, gripper: Gripper) -> None:
        msg = JointState()
        gripper_id = gripper["gripper_id"]
        msg.header.frame_id = gripper_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name.append(gripper_id)
        msg.position.append(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_actual_position(), self.loop
            ).result()
            / 1e6
        )
        if gripper_id in self.joint_state_publishers:
            self.joint_state_publishers[gripper_id].publish(msg)

    def _publish_gripper_state(self, gripper: Gripper) -> None:
        msg = GripperState()
        gripper_id = gripper["gripper_id"]
        msg.header.frame_id = gripper_id
        msg.header.stamp = self.get_clock().now().to_msg()

        status = (
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_diagnostics(), self.loop
            )
            .result()
            .split(",")
        )
        msg.error_code = status[0].strip()
        msg.warning_code = status[1].strip()
        msg.additional_code = status[2].strip()

        msg.bit0_ready_for_operation = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=0), self.loop
            ).result()
        )
        msg.bit1_control_authority_fieldbus = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=1), self.loop
            ).result()
        )
        msg.bit2_ready_for_shutdown = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=2), self.loop
            ).result()
        )
        msg.bit3_not_feasible = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=3), self.loop
            ).result()
        )
        msg.bit4_command_successfully_processed = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=4), self.loop
            ).result()
        )
        msg.bit5_command_received_toggle = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=5), self.loop
            ).result()
        )
        msg.bit6_warning = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=6), self.loop
            ).result()
        )
        msg.bit7_error = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=7), self.loop
            ).result()
        )
        msg.bit8_released_for_manual_movement = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=8), self.loop
            ).result()
        )
        msg.bit9_software_limit_reached = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=9), self.loop
            ).result()
        )
        msg.bit11_no_workpiece_detected = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=11), self.loop
            ).result()
        )
        msg.bit12_workpiece_gripped = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=12), self.loop
            ).result()
        )
        msg.bit13_position_reached = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=13), self.loop
            ).result()
        )
        msg.bit14_workpiece_pre_grip_started = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=14), self.loop
            ).result()
        )
        msg.bit16_workpiece_lost = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=16), self.loop
            ).result()
        )
        msg.bit17_wrong_workpiece_gripped = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=17), self.loop
            ).result()
        )
        msg.bit31_grip_force_and_position_maintenance_activated = bool(
            asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_bit(bit=31), self.loop
            ).result()
        )

        if gripper_id in self.gripper_state_publishers:
            self.gripper_state_publishers[gripper_id].publish(msg)

    # Service callbacks
    def _add_gripper_cb(
        self, request: AddGripper.Request, response: AddGripper.Response
    ):
        response.success = self.add_gripper(
            host=request.gripper.host,
            port=request.gripper.port,
            serial_port=request.gripper.serial_port,
            device_id=request.gripper.device_id,
        )
        return response

    def _reset_grippers_cb(self, request: Trigger.Request, response: Trigger.Response):
        response.success = self.reset_grippers()
        return response

    def _show_configuration_cb(
        self, request: ShowConfiguration.Request, response: ShowConfiguration.Response
    ):
        response.configuration = self.show_configuration()
        return response

    def _list_grippers_cb(
        self, request: ListGrippers.Request, response: ListGrippers.Response
    ):
        self.get_logger().debug("---> List gripper IDs")
        response.grippers = self.list_grippers()
        return response

    def _show_gripper_specification_cb(
        self,
        request: ShowGripperSpecification.Request,
        response: ShowGripperSpecification.Response,
        gripper: Gripper,
    ):
        self.get_logger().info("---> Show Specification")
        spec = gripper["driver"].show_gripper_specification()
        if not spec:
            response.success = False
            response.message = asyncio.run_coroutine_threadsafe(
                gripper["driver"].get_status_diagnostics(), self.loop
            ).result()
            return response

        response.specification.max_stroke = spec["max_stroke"]
        response.specification.max_speed = spec["max_speed"]
        response.specification.max_force = spec["max_force"]
        response.specification.serial_number = spec["serial_number"]
        response.specification.firmware_version = spec["firmware_version"]
        response.success = True
        response.message = asyncio.run_coroutine_threadsafe(
            gripper["driver"].get_status_diagnostics(), self.loop
        ).result()
        return response

    def _acknowledge_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Acknowledge")
        if self.needs_synchronize(gripper):
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].acknowledge(scheduler=self.scheduler), self.loop
            ).result()
        else:
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].acknowledge(), self.loop
            ).result()
        response.message = asyncio.run_coroutine_threadsafe(
            gripper["driver"].get_status_diagnostics(), self.loop
        ).result()
        return response

    def _fast_stop_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Fast stop")
        if self.needs_synchronize(gripper):
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].fast_stop(scheduler=self.scheduler), self.loop
            ).result()
        else:
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].fast_stop(), self.loop
            ).result()
        response.message = asyncio.run_coroutine_threadsafe(
            gripper["driver"].get_status_diagnostics(), self.loop
        ).result()
        return response

    def _move_to_absolute_position_cb(
        self,
        request: MoveToAbsolutePosition.Request,
        response: MoveToAbsolutePosition.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Move to absolute position")
        position = int(request.position * 1e6)
        velocity = int(request.velocity * 1e6)
        if self.needs_synchronize(gripper):
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].move_to_absolute_position(
                    position=position,
                    velocity=velocity,
                    use_gpe=request.use_gpe,
                    scheduler=self.scheduler,
                ),
                self.loop,
            ).result()
        else:
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].move_to_absolute_position(
                    position=position, velocity=velocity, use_gpe=request.use_gpe
                ),
                self.loop,
            ).result()
        response.message = asyncio.run_coroutine_threadsafe(
            gripper["driver"].get_status_diagnostics(), self.loop
        ).result()
        return response

    def _grip_cb(
        self,
        request: Grip.Request,
        response: Grip.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Grip")
        if self.needs_synchronize(gripper):
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].grip(
                    force=request.force,
                    use_gpe=request.use_gpe,
                    outward=request.outward,
                    scheduler=self.scheduler,
                ),
                self.loop,
            ).result()
        else:
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].grip(
                    force=request.force,
                    use_gpe=request.use_gpe,
                    outward=request.outward,
                ),
                self.loop,
            ).result()
        response.message = asyncio.run_coroutine_threadsafe(
            gripper["driver"].get_status_diagnostics(), self.loop
        ).result()
        return response

    def _release_cb(
        self,
        request: Release.Request,
        response: Release.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Release")
        if self.needs_synchronize(gripper):
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].release(
                    use_gpe=request.use_gpe,
                    scheduler=self.scheduler,
                ),
                self.loop,
            ).result()
        else:
            response.success = asyncio.run_coroutine_threadsafe(
                gripper["driver"].release(
                    use_gpe=request.use_gpe,
                ),
                self.loop,
            ).result()
        response.message = asyncio.run_coroutine_threadsafe(
            gripper["driver"].get_status_diagnostics(), self.loop
        ).result()
        return response


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    driver = Driver("driver")
    executor.add_node(driver)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        executor.shutdown()
        driver.destroy_node()


if __name__ == "__main__":
    main()
