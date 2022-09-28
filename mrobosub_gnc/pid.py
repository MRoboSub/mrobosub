#!/usr/bin/env python

"""
Helper PID function file.
"""
import rospy
import control_math
from typing import Optional

class PIDController:
    """
    A simple PID Controller.
    """

    def __init__(self, kp: float, ki: float, kd: float, imin: Optional[float] = None, imax: Optional[float] = None):
        """
        Initializes this PID Controller. Both imin and imax must be supplied for integral windup limiting.

        Arguments:
        - kp - the proportional gain
        - ki - the integral gain
        - kd - the derivative gain
        - imin - integral windup minimum
        - imax - integral windup maximum
        """
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        self.imin: Optional[float] = imin
        self.imax: Optional[float] = imax

        self.prev_time: float = rospy.get_time()
        self.last_error: float = 0.0
        self.cumulative_error: float = 0.0

    def calculate_from_error(self, error: float) -> float:
        """
        Calculates next PID value from a given error

        Arguments:
        - error - the current error at this timestep
        """
        curr_time = rospy.get_time()
        dt = curr_time - self.prev_time

        # Proportional
        pterm = self.kp * error
        
        # Integral
        self.cumulative_error += error * dt
        iterm = self.ki * self.cumulative_error 
        
        # Integral windup limiting
        if self.imin is not None and self.imax is not None:
            iterm = control_math.clamp(iterm, self.imin, self.imax)

        # Derivative
        rate = (error - self.last_error) / dt 
        self.last_error = error
        dterm = self.kd * rate

        return pterm + iterm + dterm

    def calculate_from_setpoint(self, setpoint: float, current: float) -> float:
        """
        Calculates next PID value from a given setpoint and current state

        Arguments:
        - setpoint - the desired target to reach
        - current - the current state to control towards setpoint
        """
        return self.calculate(setpoint - current)

