#!/usr/bin/env python
"""
@author: S. Bertrand & A.Reis

# main script: robot driver PID

# all variables in SI unit

"""

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped
from green_robot.msg import Int32Stamped # for receiving navdata feedback
import numpy as np
import math
import robotClass
from dynamic_reconfigure.server import Server
from green_robot.cfg import WheelPIDCFGConfig


# node init
# ----------
rospy.init_node('wheel_PID', anonymous=True)

# static parameters
# ------------------
interWheelDistance_ = rospy.get_param('interWheelDistance', 0.15)
wheelDiameter_ = rospy.get_param('wheelDiameter', 0.07)
x0_ = rospy.get_param('x0', 0.)
y0_ = rospy.get_param('y0', 0.)
theta0_ = rospy.get_param('theta0', 0.)
encoderResolution_ = rospy.get_param('encoderResolution', 6*120)
#rospy.loginfo("dist=%f" % interWheelDistance_)

# robot object
# ------------
robot = robotClass.Robot(interWheelDistance=interWheelDistance_, wheelDiameter=wheelDiameter_, x0=x0_, y0=y0_, theta0=theta0_, encoderResolution=encoderResolution_)


emergencyStopFlag = False


# publishers
# -----------

# publish on these topics to send control values to the robot  (in [-255,+255])
pubLeftMotorCmd = rospy.Publisher('green_robot/cmdMotor/left', Int32Stamped, queue_size=10)
pubRightMotorCmd = rospy.Publisher('green_robot/cmdMotor/right', Int32Stamped, queue_size=10)

# frequency of PID control of wheel angular velocities
fe = 20.0
Te = 1/fe
motorCmdPubRate = rospy.Rate(fe)


# dynamic parameters
#--------------------

# ----------------------------------------------------------------------------
def callbackDynParam(config, level):
# -----------------------------------------------------------------------------

    global kp
    global ki
    global kd

    kp = float("""{kp}""".format(**config))
    ki = float("""{ki}""".format(**config))
    kd = float("""{kd}""".format(**config))

    return config


# server for dyamic parameters
srv = Server(WheelPIDCFGConfig, callbackDynParam)

#init dynamic parameters
kp = rospy.get_param('/wheelPID/kp', 1.85)
ki = rospy.get_param('/wheelPID/ki', 2.25)
kd = rospy.get_param('/wheelPID/kd', 0.55)



# -----------------------------------------------------------------------------
def callBackLeftOmegaRef(data):
# -----------------------------------------------------------------------------
    global robot
    # assign desired wheel speed
    robot.leftWheel.omegaRef =  data.data
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackRightOmegaRef(data):
# -----------------------------------------------------------------------------
    global robot
    # assign desired wheel speed
    robot.rightWheel.omegaRef =  data.data
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackLeftOmega(data):
# -----------------------------------------------------------------------------
    global robot
    # assign wheel speed
    robot.leftWheel.omega =  data.twist.angular.z
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackRightOmega(data):
# -----------------------------------------------------------------------------
    global robot
    # assign  wheel speed
    robot.rightWheel.omega =  data.twist.angular.z
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackEmergencyStop(data):
# -----------------------------------------------------------------------------
    global emergencyStopFlag
    emergencyStop =  data.data
# -----------------------------------------------------------------------------


# subscribers
# ------------
rospy.Subscriber("wheelSpeedRef/left", Float32, callBackLeftOmegaRef)
rospy.Subscriber("wheelSpeedRef/right", Float32, callBackRightOmegaRef)
rospy.Subscriber('wheelSpeed/left', TwistStamped, callBackLeftOmega)
rospy.Subscriber('wheelSpeed/right', TwistStamped, callBackRightOmega)
rospy.Subscriber("emergencyStop", Bool, callBackEmergencyStop)

# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    

    # for PID control    
    uLeftMsg = Int32Stamped()
    uRightMsg = Int32Stamped()
    epsilonPrecLeft = 0.0
    epsilonPrecRight = 0.0
    integralLeft = 0.0
    integralRight = 0.0
    uLeftPrec = 0.0
    uRightPrec = 0.0
    
    #print Te, kp, ki, kd


    while not rospy.is_shutdown():

         # PID for wheel speed regulation
         epsilonLeft = robot.leftWheel.omegaRef - robot.leftWheel.omega
         epsilonRight = robot.rightWheel.omegaRef - robot.rightWheel.omega
         epsilonDLeft = (epsilonLeft-epsilonPrecLeft)/Te
         epsilonDRight = (epsilonRight-epsilonPrecRight)/Te

         integralLeft = integralLeft + Te*(epsilonLeft - epsilonPrecLeft)         
         integralRight = integralRight + Te*(epsilonRight - epsilonPrecRight)
         
         
         if (robot.leftWheel.omegaRef==0):
             uLeft = 0
         else:
             uLeft = uLeftPrec + kp*epsilonLeft + ki*integralLeft + kd*epsilonDLeft

         if (robot.rightWheel.omegaRef==0):
             uRight = 0
         else:
             uRight = uRightPrec + kp*epsilonRight + ki*integralRight + kd*epsilonDRight
         
         # saturations
         uLeft = np.clip(uLeft, -180, 180)
         uRight = np.clip(uRight, -180, 180)

         # deadzone
         if (math.fabs(uLeft)<=10):
             uLeft = 0
         if (math.fabs(uRight)<=10):
             uRight = 0

         # emergency stop
         if (emergencyStopFlag):
			 uLeft = 0
			 uRight = 0

         # control msgs        
         uLeftMsg.header.seq = uLeftMsg.header.seq + 1
         uLeftMsg.header.stamp = rospy.Time.now()
         uLeftMsg.data = uLeft         
         uRightMsg.header.seq = uRightMsg.header.seq + 1
         uRightMsg.header.stamp = rospy.Time.now()
         uRightMsg.data = uRight

         # msgs publications
         pubLeftMotorCmd.publish(uLeftMsg)
         pubRightMotorCmd.publish(uRightMsg)

         
         # update for next iteration
         epsilonPrecLeft = epsilonLeft
         epsilonPrecRight = epsilonRight         
         uLeftPrec = uLeft
         uRightPrec = uRight

         motorCmdPubRate.sleep()

# -----------------------------------------------------------------------------
