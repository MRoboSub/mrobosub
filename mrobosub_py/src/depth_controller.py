#!/usr/bin/env python

"""
Node responsible for controlling the depth of the sub. Takes in depth requests
from other nodes to request absolute depths and uses the barometer
data in a PID loop to smooth our vertical motion.
"""

from types import Any, FunctionType, List, Optional, TypedDict

import rospy
import util
from std_msgs.msg import Bool, Float32
from util.PID import PIDController

from controller import Controller, RosPublisher, RosSubscriber

ITERATION_RATE = 100  # Hz


class DepthController(Controller):
    def __init__(self):
        self.pid = PIDController(
            Kp=6,
            Ki=0,
            Kd=1,
            windup_limit=5,
            upper_limit=100.0,
            lower_limit=-200.0,
        )

        self.pid.launch()

        # input
        self.should_control = True
        self.current_altitude = 0
        self.target_altitude = 0

        # output
        self.power = 0

        super().__init__(
            "depth_control",
            ITERATION_RATE,
            [
                {
                    "topic": "/mrobosub/altitude",
                    "data_type": Float32,
                    "callback": self._altitude,
                },
                {
                    "topic": "/mrobosub/depth_request",
                    "data_type": Float32,
                    "callback": self._depth_request,
                },
                {
                    "topic": "/mrobosub/object_centering_control",
                    "data_type": Bool,
                    "callback": self._object_centering_control,
                },
            ],
            [{"topic": "mrobosub/out/depth", "data_type": Flaot32, "name": "depth"}],
        )

    def _object_centering_control(self, object_centering_control: Bool):
        """
        Callback function for the /mrobosub/object_centering_control ROS topic.
        This node (depth_control) must take control of depth when object_centering
        is disabled.

        params:
            object_centering_control: determines if object_centering will control the sub
        """

        self.should_control = not object_centering_control

        # If object_centering was just turned off, maintain current depth.
        if self.should_control:
            self.target_altitude = self.current_altitude

    def _altitude(self, msg: Float32):
        """
        Callback function for the /mrobosub/altitude ROS topic.

        params:
            msg: the altitude that was recieved
        """

        self.current_altitude = msg.data
        self.pid.state = self.current_altitude

    def _depth_request(self, request: Float32):
        """
        Callback function for the /mrobosub/depth_request ROS topic.

        params:
            request: the depth request that was received
        """

        if request.data > 0:
            print("depth_control: WARNING: Depth request > 0, making negative.")
            request.data *= -1

        self.target_altitude = request.data

    def control(self):
        FEED_FORWARD = 55

        if not self.should_control:
            while not self.should_control and not rospy.is_shutdown():
                rospy.sleep(1)  # 1 sec

        self.pid.setpoint = self.target_altitude
        self.pid.enabled = True

        self.power = self.pid.control_effort - FEED_FORWARD
        self.publisher["depth"].publish(self.power)

        logstr = "target: {0}deg\tcurrent: {1}deg\toutput:{2}".format(
            self.target_altitude, self.current_altitude, self.power
        )

        print(logstr)


if __name__ == "__main__":
    print("depth_control: Starting")

    depth_controller = DepthController()

    depth_controller.run()
