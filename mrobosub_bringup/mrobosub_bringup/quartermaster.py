from dataclasses import dataclass
from enum import Enum
import time
import os
import multiprocessing

import rclpy
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool, Trigger
from rclpy.executors import SingleThreadedExecutor

from mrobosub_lib import Node
from . import constants as const
from .launch_manager import LaunchManager


class RobotState(Enum):
    AMBIENT = 0
    READY = 1
    RUNNING = 2


class Led(Enum):
    STRANGE = 1
    CHARM = 2
    ON = 3


@dataclass
class HallEffectState:
    strange: bool
    charm: bool


@dataclass
class LedState:
    strange: bool
    charm: bool
    led_on: bool


class Quartermaster(Node):
    def __init__(self) -> None:
        super().__init__("quartermaster")
        self.current_state = RobotState.AMBIENT
        self.hall_effect_triggered = HallEffectState(strange=False, charm=False)
        self.hall_effect_last = HallEffectState(strange=False, charm=False)
        self.led_states = LedState(strange=False, charm=False, led_on=False)
        self.timeout_time: float = 0.0
        self.timeout_disabled_led: Led | None = None

        qos_profile = QoSProfile(depth=1)
        self.led_strange_pub = self.create_publisher(Bool, "/led/strange", qos_profile)
        self.led_charm_pub = self.create_publisher(Bool, "/led/charm", qos_profile)
        self.led_on_pub = self.create_publisher(Bool, "/led/on", qos_profile)

        self.strange_sub = self.create_subscription(
            Bool, "/buttons/strange", self.handle_strange_change, 1
        )
        self.charm_sub = self.create_subscription(
            Bool, "/buttons/charm", self.handle_charm_change, 1
        )

        executor = rclpy.get_global_executor()
        complete_future = executor.create_task(lambda: None)
        self.thruster_mixing_srv = self.create_client(
            SetBool, "/thruster_mixing/enable"
        )
        self.thruster_mixing_future = complete_future
        self.zero_state_srv = self.create_client(Trigger, "/localization/zero_state")
        self.zero_state_future = complete_future
        self.soft_stop_srv = self.create_client(Trigger, "/captain/soft_stop")
        self.soft_stop_future = complete_future

        planning_pkg_path = get_package_share_directory("mrobosub_planning")
        captain_file_path = os.path.join(
            planning_pkg_path, "launch", "captain_launch.xml"
        )

        self.captain_launcher = LaunchManager(captain_file_path)

        # Seed initial values.
        self.pub_strange_led(False)
        self.pub_charm_led(False)
        self.pub_on_led(False)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def destroy_node(self) -> None:
        if self.captain_launcher:
            self.captain_launcher.stop()  # Ensure the launch manager is stopped before the node is destroyed.
        super().destroy_node()

    def pub_strange_led(self, val: bool) -> None:
        b = Bool()
        b.data = val
        self.led_strange_pub.publish(b)
        self.led_states.strange = val

    def pub_charm_led(self, val: bool) -> None:
        b = Bool()
        b.data = val
        self.led_charm_pub.publish(b)
        self.led_states.charm = val

    def pub_on_led(self, val: bool) -> None:
        b = Bool()
        b.data = val
        self.led_on_pub.publish(b)
        self.led_states.led_on = val

    def pub_led(self, led: Led, val: bool) -> None:
        self.get_logger().info(f"setting {led} to {val}")
        if led == Led.STRANGE:
            self.pub_strange_led(val)
        if led == Led.CHARM:
            self.pub_charm_led(val)
        if led == Led.ON:
            self.pub_on_led(val)

    def handle_strange_change(self, new_value: Bool) -> None:
        if new_value.data is False and self.hall_effect_last.strange is True:
            self.hall_effect_triggered.strange = True
            self.get_logger().info("strange changed")
        self.hall_effect_last.strange = new_value.data

    def handle_charm_change(self, new_value: Bool) -> None:
        if new_value.data is False and self.hall_effect_last.charm is True:
            self.hall_effect_triggered.charm = True
            self.get_logger().info("charm changed")
        self.hall_effect_last.charm = new_value.data

    async def call_thruster_mixing_srv(self) -> None:
        service_live = self.thruster_mixing_srv.wait_for_service(
            timeout_sec=const.SERVICE_TIMEOUT_DURATION
        )

        if not service_live:
            self.get_logger().error(f"Thruster mixing service server not available")
            return

        self.get_logger().info(f"Thruster mixing ready")

        request = SetBool.Request(data=True)
        response = await self.thruster_mixing_srv.call_async(request)

        if response is None:
            self.get_logger().error("Thruster mixing service call failed")
            return

        self.get_logger().info(
            f"Thruster mixing service response: success={response.success}, message='{response.message}'"
        )

    async def call_zero_state_srv(self) -> None:
        service_live = self.zero_state_srv.wait_for_service(
            timeout_sec=const.SERVICE_TIMEOUT_DURATION
        )

        if not service_live:
            self.get_logger().error(f"Zero state service server not available")
            return

        self.get_logger().info(f"Zero state service ready")

        request = Trigger.Request()
        response = await self.zero_state_srv.call_async(request)

        if response is None:
            self.get_logger().error("Zero state service call failed")
            return

        self.get_logger().info(
            f"Service response: success={response.success}, message='{response.message}'"
        )

    async def call_soft_stop_srv(self) -> None:
        service_live = self.soft_stop_srv.wait_for_service(
            timeout_sec=const.SERVICE_TIMEOUT_DURATION
        )
        if not service_live:
            self.get_logger().error(f"Soft stop service server not available")
            return

        self.get_logger().info(f"Soft stop ready")

        request = Trigger.Request()
        response = await self.soft_stop_srv.call_async(request)

        if response is None:
            self.get_logger().error("Soft stop service call failed")
            return

        self.get_logger().info(
            f"Service response: success={response.success}, message='{response.message}'"
        )

    def timer_callback(self) -> None:
        # In order to visually ensure quartermaster has started, turn on the LED
        self.pub_on_led(True)

        # If less than const.DEBOUNCE_DURATION seconds has elapsed, zero the hall_effects. This is so that
        # we don't poll the hall_effects too quickly.
        if (self.get_clock().now().nanoseconds / 1e9) < self.timeout_time:
            self.hall_effect_triggered.strange = False
            self.hall_effect_triggered.charm = False
            return

        # We want the LED showing that we have triggered a specific hall effect sensor to turn off
        # after 5 seconds.
        if self.timeout_disabled_led is not None:
            self.pub_led(self.timeout_disabled_led, False)
            self.timeout_disabled_led = None

        # We don't want to advance the state machine if both are false. Only on the rising edge.
        if (self.hall_effect_triggered.strange is False) and (
            self.hall_effect_triggered.charm is False
        ):
            return

        # We would have only gotten here if it has been const.DEBOUNCE_DURATION seconds since last timeout
        # and one of the hall effects is on
        self.timeout_time = (
            self.get_clock().now().nanoseconds / 1e9
        ) + const.DEBOUNCE_DURATION

        if self.hall_effect_triggered.strange:
            self.pub_strange_led(True)
            self.timeout_disabled_led = Led.STRANGE
        elif self.hall_effect_triggered.charm:
            self.pub_charm_led(True)
            self.timeout_disabled_led = Led.CHARM

        # Zero hall effects.
        self.hall_effect_triggered.strange = False
        self.hall_effect_triggered.charm = False

        # On each rising edge of the hall effect, the state machine advances a step.
        if self.current_state == RobotState.AMBIENT:
            self.get_logger().info("reached state machine")
            # Needed to ensure the service isn't called before the future is returned
            if self.thruster_mixing_future.done():
                self.thruster_mixing_future = self.executor.create_task(
                    self.call_thruster_mixing_srv()
                )

            if self.zero_state_future.done():
                self.zero_state_future = self.executor.create_task(
                    self.call_zero_state_srv()
                )

            self.current_state = RobotState.READY

        elif self.current_state == RobotState.READY:
            self.get_logger().info("starting state machine")
            self.captain_launcher.start()
            self.current_state = RobotState.RUNNING

        elif self.current_state == RobotState.RUNNING:
            self.get_logger().info("soft stopping state machine")
            if self.soft_stop_future.done():
                self.soft_stop_future = self.executor.create_task(
                    self.call_soft_stop_srv()
                )

            # TODO: There should probably be some sort of `time.sleep()` here so there is enough time for the soft stop service to be completed.
            #       Or maybe that could be part of the callback for the future?
            self.captain_launcher.stop()

            if self.thruster_mixing_future.done():
                self.thruster_mixing_future = self.executor.create_task(
                    self.call_thruster_mixing_srv()
                )

            self.current_state = RobotState.AMBIENT


def main() -> None:
    multiprocessing.set_start_method("spawn")
    rclpy.init()
    quartermaster = Quartermaster()
    try:
        rclpy.spin(quartermaster)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
