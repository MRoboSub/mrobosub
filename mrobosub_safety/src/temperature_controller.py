import os
import time
import typing

import rospy
from std_msgs.msg import Float64

import psutil
class TemperatureController:
    def __init__(self):
        rospy.init_node("temperature_controller")

    def is_overheating(self) -> typing.Optional[str]:
        for sensor, temperatures in psutil.sensors_temperatures().items():
            if any(temp.current >= temp.critical for temp in temperatures):
                return sensor

    def shutdown(self, sensor: str):
        for axis in ('surge', 'sway', 'heave', 'yaw', 'roll', 'pitch'):
            rospy.Publisher(f"/output_wrench/{axis}", Float64, queue_size=1).publish(0)
            rospy.Publisher(f"/target_twist/{axis}", Float64, queue_size=1).publish(0)
        rospy.signal_shutdown(f"Jetson Sensor '{sensor}' Overheating Detected")
        time.sleep(2)
        os.system("sudo shutdown -h now")  # should run without needing password: %shutdown ALL=(root) NOPASSWD: /sbin/shutdown

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            overheating_sensor = self.is_overheating()
            if overheating_sensor is not None:
                self.shutdown(overheating_sensor)
            rate.sleep()

if __name__ == "__main__":
    TemperatureController().run()
