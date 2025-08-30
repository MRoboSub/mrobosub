#!/usr/bin/env python

def clamp(x: float, min: float = -1, max: float = 1) -> float:
    """
    Restricts x to the range min, max. If x leaves the range then the closer extreme is returned. If min > max, then they are swapped.

    Arguments
    - x - the value to clamp
    - min - one end of the range to restrict to
    - max - one end of the range to restrict to
    """
    if min < max:
        if x > max: 
            return max
        if x < min:
            return min
    else:
        if x > min: 
            return min
        if x < max: 
            return max
    return x

def clamp_abs(x: float, abslimit: float = 1) -> float:
    """
    Restricts x to the range -abslimit, abslimit. If x leaves the range then the closer extreme is returned.

    Arguments
    - x - the value to clamp
    - abslimit - the negative/positive limit 
    """
    return clamp(x, -abslimit, abslimit)

def deadband(x: float, limit: float = 0.1) -> float:
    """
    If x is small enough - that is, |x| < |limit| - then returns 0.

    Arguments
    - x - the value to deadband
    - limit - the value which 0 should be returned when it is under its absolute value
    """
    if abs(x) < abs(limit):
        return 0
    return x