#!/usr/bin/env python
"""
@author: S. Bertrand

# all variables in SI unit
"""

# **** TO DO: put encoderResolution, wheel diameter and inter wheels dist as parameters (YAML)


class Wheel:
    
    def __init__(self, diameter, encoderResolution):
        self.diameter = diameter
        self.v = 0.0
        self.omega = 0.0
        self.omegaRef = 0.0
        self.encoderCount = 0
        self.encoderCountPrev = 0
        self.lastTimeCountChange = 0.0
        self.encoderResolution = encoderResolution #6*120 #nb of counts per revolution (120 : reducteur)
        

class Robot:
    
    def __init__(self, interWheelDistance=0.15, wheelDiameter=0.07, x0=0.0, y0=0.0, theta0=0.0, encoderResolution=6*120):
    # zero inital speed and angular velocity assumed
        self.x0 = x0
        self.y0 = y0
        self.x = x0
        self.y = y0
        self.theta0 = theta0
        self.theta = theta0
        self.v = 0.0
        self.vRef = 0.0
        self.omega = 0.0
        self.omegaRef = 0.0
        self.interWheelDistance = interWheelDistance
        self.leftWheel = Wheel(wheelDiameter, encoderResolution)
        self.rightWheel = Wheel(wheelDiameter, encoderResolution)


if __name__=='__main__':
    ardupiRobot = Robot(0.15, 0.05)
