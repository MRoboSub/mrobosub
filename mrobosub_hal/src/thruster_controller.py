#!/usr/bin/env python

import rospy

from mrobosub_lib.lib import Node
from std_msgs.msg import Float64
from serial import Serial
from typing_extensions import Callable

NUM_MOTORS = 8

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        print("Launched thruster_controller node")
        self.port = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00467345-if00"
        # self.port = '/dev/ttyACM0'
        self.get_errors() # clear errors at the start
        
        self.motor_subs = {
           i: rospy.Subscriber(
               f"/motor_output/{i}", Float64, self.make_motor_callback(i)
           )
           for i in range(NUM_MOTORS)
        }
        rospy.spin()
    
    # pwm_raw ranges from -1 to 1
    # pwm_val ranges from 4000 to 8000
    def convert_pwm_signal(self, pwm_raw: Float64) -> int:
        if(pwm_raw.data<-1 or pwm_raw.data>1):
            print(f"Thruster Controller [ERROR]: PWM value {pwm_raw} out of range (should be in [-1, 1])")
            return -1
        return int((pwm_raw.data * 2000)+6000)
    
    # in case of invalid PWM or motor number parameters, does not send any updated signal to the motor controller and returns -1
    def send_signal(self, motor: int, pwm_raw: Float64) -> int:
        pwm_val: int = self.convert_pwm_signal(pwm_raw)
        if pwm_val == -1:
            return -1
        
        if(motor<0 or motor >= NUM_MOTORS):
            print(f"Thruster Controller [ERROR]: motor number {motor} out of range (should be in [0-{NUM_MOTORS-1}])")
            return -1

        LSBs = pwm_val % (2**7)
        MSBs = int(pwm_val/(2**7))

        self.get_errors()
        try:
            with Serial(self.port) as s:
                s.write(bytearray([0xAA, 0x0C, 0x04, motor, LSBs, MSBs]))
        except:
            self.get_errors() #giving it one more chance to clear errors, just in case (sometimes it's weird)
            with Serial(self.port) as s:
                s.write(bytearray([0xAA, 0x0C, 0x04, motor, LSBs, MSBs]))
        
        print(f"Thruster controller: sent pwm value {pwm_val} to motor {motor}")

        return 0

    def make_motor_callback(self, i) -> Callable[[Float64], None]:
        def motor_callback(pwm_raw: Float64):
            self.send_signal(i, pwm_raw)

        return motor_callback

    def get_errors(self):
        # gets errors from thruster controller hardware (which automatically clears the errors too)
        with Serial(self.port) as s:
            s.write(bytearray([0xAA, 0x0C, 0x21]))
            error = s.read(2)
            error_code = int.from_bytes(error, "little")
            if(error_code != 0):
                print(f"Thruster controller: error code = {error_code}")
            # eg: error_code 16 means 00010000 which is the 5th error bit set


def main():
    ThrusterController()

if __name__ == "__main__":
    main()