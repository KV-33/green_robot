#!/usr/bin/env python
"""
@author: S. Bertrand & A.Reis

# main script: robot Wheel Speed Estimation 

# all variables in SI unit

"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped
from green_robot.msg import Int32Stamped # for receiving navdata feedback
import numpy as np
import robotClass
from dynamic_reconfigure.server import Server
from green_robot.cfg import WheelSpeedFilterCFGConfig


# node init
# ----------
rospy.init_node('wheelSpeedFilterNode', anonymous=True)


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


# publishers
# -----------
# wheel speed estimated from wheel encoders (linear and angular)
pubLeftWheelSpeed = rospy.Publisher('wheelSpeed/left', TwistStamped, queue_size=10)
pubRightWheelSpeed = rospy.Publisher('wheelSpeed/right', TwistStamped, queue_size=10)

# frequency of filter execution
fe = 20.0
Te = 1/fe
wheelSpeedPubRate = rospy.Rate(fe)


# dynamic parameters
#--------------------

# -----------------------------------------------------------------------------
def callbackDynParam(config, level):
# -----------------------------------------------------------------------------
    #rospy.loginfo("""Reconfigure Request: {alpha}""".format(**config))
    global alpha

    alpha = float("""{alpha}""".format(**config))

    return config

# server for dynamic parameters
srv = Server(WheelSpeedFilterCFGConfig, callbackDynParam)

#init dynamic parameters
alpha = rospy.get_param('/wheelSpeedFilter/alpha', 0.15)
#rospy.loginfo("alpha=%f" % alpha)


# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackLeftEncoderCount(data):
# -----------------------------------------------------------------------------
    global robot
    
    robot.leftWheel.encoderCount = data.data 
    t = data.header.stamp.secs + 1.E-9*data.header.stamp.nsecs
    deltaT = t-robot.leftWheel.lastTimeCountChange
    
    # angular speed computation
    if (deltaT>0.0):
        # first order filter
        omegaOld = robot.leftWheel.omega        
        omega = (robot.leftWheel.encoderCount - robot.leftWheel.encoderCountPrev)*(2.0*np.pi/robot.leftWheel.encoderResolution) / deltaT      
        robot.leftWheel.omega = alpha*omega + (1.0-alpha)*omegaOld
        
        # updates for next iteration
        robot.leftWheel.lastTimeCountChange = t
        robot.leftWheel.encoderCountPrev = robot.leftWheel.encoderCount
        
        # message to be published
        msgTwist = TwistStamped()    
        msgTwist.header = data.header
        msgTwist.twist.angular.z = robot.leftWheel.omega
        msgTwist.twist.linear.x = robot.leftWheel.omega * robot.leftWheel.diameter/2.0  # TO DO: sign to be checked
        # publication
        pubLeftWheelSpeed.publish(msgTwist)
        
# -----------------------------------------------------------------------------       
def callBackRightEncoderCount(data):
# -----------------------------------------------------------------------------
    global robot
    
    robot.rightWheel.encoderCount = data.data 
    t = data.header.stamp.secs + 1.E-9*data.header.stamp.nsecs
    deltaT = t-robot.rightWheel.lastTimeCountChange
#    rospy.loginfo(rospy.get_name() + " - time: %f" % t)
    
    # angular speed computation
    if (deltaT>0.0):
        # first order filter
        omegaOld = robot.rightWheel.omega        
        omega = (robot.rightWheel.encoderCount - robot.rightWheel.encoderCountPrev)*(2.0*np.pi/robot.rightWheel.encoderResolution) / deltaT      
        robot.rightWheel.omega = alpha*omega + (1.0-alpha)*omegaOld
        
        # updates for next iteration
        robot.rightWheel.lastTimeCountChange = t
        robot.rightWheel.encoderCountPrev = robot.rightWheel.encoderCount
        
        # message to be published
        msgTwist = TwistStamped()    
        msgTwist.header = data.header
        msgTwist.twist.angular.z = robot.rightWheel.omega
        msgTwist.twist.linear.x = robot.rightWheel.omega * robot.rightWheel.diameter/2.0 # TO DO: sign to be checked
        # publication
        pubRightWheelSpeed.publish(msgTwist)
# -----------------------------------------------------------------------------


# subscribers
# ------------
rospy.Subscriber("green_robot/encoderCount/left", Int32Stamped, callBackLeftEncoderCount)
rospy.Subscriber("green_robot/encoderCount/right", Int32Stamped, callBackRightEncoderCount)


# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()

    while not rospy.is_shutdown():

       wheelSpeedPubRate.sleep()

# -----------------------------------------------------------------------------
