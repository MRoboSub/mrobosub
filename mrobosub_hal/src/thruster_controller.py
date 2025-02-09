#!/usr/bin/env python

import rospy

from mrobosub_lib.lib import Node
from std_msgs.msg import Float64
from serial import Serial

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        print("Launched thruster_controller node")
        self.port = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00467345-if00"
        # self.port = '/dev/ttyACM0'
        self.get_errors() #just to clear errors at the start
        
        rospy.Subscriber('/motor_pwm/0', Float64, self.motor_0_callback)
        rospy.Subscriber('/motor_pwm/1', Float64, self.motor_1_callback)
        rospy.Subscriber('/motor_pwm/2', Float64, self.motor_2_callback)
        rospy.Subscriber('/motor_pwm/3', Float64, self.motor_3_callback)
        rospy.Subscriber('/motor_pwm/4', Float64, self.motor_4_callback)
        rospy.Subscriber('/motor_pwm/5', Float64, self.motor_5_callback)
        rospy.Subscriber('/motor_pwm/6', Float64, self.motor_6_callback)
        rospy.Subscriber('/motor_pwm/7', Float64, self.motor_7_callback)
        rospy.spin()
    
    # pwm_raw ranges from -1 to 1
    # pwm_val ranges from 4000 to 8000
    def convert_pwm_signal(self, pwm_raw: Float64) -> int:
        if(pwm_raw.data<-1 or pwm_raw.data>1):
            print(f"Thruster Controller Error: PWM value {pwm_raw} out of range (should be between -1 and 1 (both inclusive))")
            return -1
        return int((pwm_raw.data * 2000)+6000)
    
    # in case of invalid PWM or motor number parameters, does not send any updated signal to the motor controller and returns -1
    def send_signal(self, motor: int, pwm_raw: Float64) -> int:
        pwm_val: int = self.convert_pwm_signal(pwm_raw)
        if pwm_val == -1:
            return -1
    
        print(f"Thruster controller: send pwm value {pwm_val} to motor {motor}")
        
        if(motor<0 or motor >7):
            print(f"Thruster Controller [ERROR]: motor number {motor} out of range (should be [0-7])")
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

        return 0

    def motor_0_callback(self, pwm_raw: Float64):
        self.send_signal(0, pwm_raw)
    
    def motor_1_callback(self, pwm_raw: Float64):
        self.send_signal(1, pwm_raw)

    def motor_2_callback(self, pwm_raw: Float64):
        self.send_signal(2, pwm_raw)

    def motor_3_callback(self, pwm_raw: Float64):
        self.send_signal(3, pwm_raw)

    def motor_4_callback(self, pwm_raw: Float64):
        self.send_signal(4, pwm_raw)

    def motor_5_callback(self, pwm_raw: Float64):
        self.send_signal(5, pwm_raw)

    def motor_6_callback(self, pwm_raw: Float64):
        self.send_signal(6, pwm_raw)

    def motor_7_callback(self, pwm_raw: Float64):
        self.send_signal(7, pwm_raw)

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