import os
import time
import typing

import rospy
import psutil

from std_msgs.msg import Float64


OVERHEATING_TIME = 3  # seconds 

class TemperatureController:
    def __init__(self):
        rospy.init_node("temperature_controller")

    def is_overheating(self) -> bool:
        for _, temperatures in psutil.sensors_temperatures().items():
            if any(temp.critical is not None and temp.current >= temp.critical for temp in temperatures):
                return True
        return False

    def shutdown(self):
        for axis in ('surge', 'sway', 'heave', 'yaw', 'roll', 'pitch'):
            rospy.Publisher(f"/output_wrench/{axis}", Float64, queue_size=1).publish(0)
            rospy.Publisher(f"/target_twist/{axis}", Float64, queue_size=1).publish(0)
        rospy.signal_shutdown(f"Jetson Sensor Overheating Detected")
        time.sleep(2)
        os.system("sudo shutdown -h now")  # should run without needing password: %shutdown ALL=(root) NOPASSWD: /sbin/shutdown

    def run(self):
        rate = rospy.Rate(5)

        last_check = None
        while not rospy.is_shutdown():
            if self.is_overheating():
               last_check = last_check or rospy.get_time()
                if rospy.get_time() - last_check > OVERHEATING_TIME:
                    self.shutdown()
            else:
                last_check = None
            rate.sleep()

if __name__ == "__main__":
    TemperatureController().run()
