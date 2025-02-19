#!/usr/bin/env python

import rospy

from mrobosub_lib.lib import Node
from std_msgs.msg import Float32
from serial import Serial
from mrobosub_msgs.msg import MotorState
from std_srvs.srv import SetBool, SetBoolResponse

NUM_MOTORS = 8

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        print("Launched thruster_controller node")
        self.port = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00467345-if00"
        # self.port = '/dev/ttyACM0'
        self.emergency_stop = False
        self.serial = Serial(self.port)
        self.get_errors() # clear errors at the start
        
        self.object_position_service = rospy.Service("emergency_stop_motors", SetBool, self.handle_emergency_stop)
        self.motor_sub = rospy.Subscriber('/motor_output', MotorState, self.motor_callback)
        rospy.spin()
    
    def handle_emergency_stop(self, _):
        self.emergency_stop = True
        self.get_errors()
        self.serial.write(bytearray([0xAA, 0x0C, 0x22]))
        r = SetBoolResponse()
        r.success = True
        return r

    # pwm_raw ranges from -1 to 1
    # pwm_val ranges from 4000 to 8000
    def convert_pwm_signal(self, pwm_raw: float) -> int:
        if(pwm_raw<-1 or pwm_raw>1):
            print(f"Thruster Controller [ERROR]: PWM value {pwm_raw} out of range (should be in [-1, 1])")
            return -1
        return int((pwm_raw * 2000)+6000)
    
    # in case of invalid PWM or motor number parameters, does not send any updated signal to the motor controller and returns -1
    def send_signal(self, motor: int, pwm_raw: Float32) -> int:
        pwm_val: int = self.convert_pwm_signal(pwm_raw.data)
        if pwm_val == -1:
            return -1
        
        if(motor<0 or motor >= NUM_MOTORS):
            print(f"Thruster Controller [ERROR]: motor number {motor} out of range (should be in [0-{NUM_MOTORS-1}])")
            return -1

        LSBs = pwm_val % (2**7)
        MSBs = int(pwm_val/(2**7))

        self.get_errors()
        self.serial.write(bytearray([0xAA, 0x0C, 0x04, motor, LSBs, MSBs]))
        
        print(f"Thruster controller: sent pwm value {pwm_val} to motor {motor}")

        return 0

    def motor_callback(self, msg: MotorState):
        if not self.emergency_stop:
            for i in range(NUM_MOTORS):
                motor_name = f"motor{i}"
                self.send_signal(i, msg.motor_name) # not sure if this works

    def get_errors(self):
        # gets errors from thruster controller hardware (which automatically clears the errors too)
        self.serial.write(bytearray([0xAA, 0x0C, 0x21]))
        error = self.serial.read(2)
        error_code = int.from_bytes(error, "little")
        if(error_code != 0):
            print(f"Thruster controller: error code = {error_code}")
            # eg: error_code 16 means 00010000 which is the 5th error bit set



def main():
    ThrusterController()

if __name__ == "__main__":
    main()