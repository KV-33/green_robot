#!/usr/bin/env python
"""
@author: S. Bertrand

# orientation control of mobile robot

# all variables in SI unit

"""

import rospy
from geometry_msgs.msg import Twist, Quaternion, Point32
import numpy as np
import robotClass
import tf
import math
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from green_robot.cfg import ControlPositionCFGConfig
from std_msgs.msg import Float32

# node init
# ----------
rospy.init_node('control_orientation', anonymous=True)

# static parameters
# ------------------
interWheelDistance_ = rospy.get_param('interWheelDistance', 0.15)
wheelDiameter_ = rospy.get_param('wheelDiameter', 0.07)
x0_ = rospy.get_param('x0', 0.)
y0_ = rospy.get_param('y0', 0.)
theta0_ = rospy.get_param('theta0', 0.0)
encoderResolution_ = rospy.get_param('encoderResolution', 720) #6*120
VMax = rospy.get_param('Vmax', 0.15)
OmegaMax = rospy.get_param('OmegaMax', 2.0)


# robot object
# ------------
robot = robotClass.Robot(interWheelDistance=interWheelDistance_, wheelDiameter=wheelDiameter_, x0=x0_, y0=y0_, theta0=theta0_, encoderResolution=encoderResolution_)


# publishers
# -----------
pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pubTheta = rospy.Publisher('theta', Float32, queue_size=10)

# frequency of orientation conrol
# --------------------------------
feAngle = 10.0
TeAngle = 1/feAngle
angleCmdPubRate = rospy.Rate(feAngle)


# dynamic parameters
#--------------------

# ----------------------------------------------------------------------------
def callbackDynParam(config, level):
# -----------------------------------------------------------------------------

    global kAngle

    kAngle = float("""{kAngle}""".format(**config))


    return config


# server for dyamic parameters
srv = Server(ControlPositionCFGConfig, callbackDynParam)


#init dynamic parameters
kAngle = rospy.get_param('/controlPosition/kAngle', 1.5)


thetaRef = 0.0


# -----------------------------------------------------------------------------
def callBackOdometry(data):
# -----------------------------------------------------------------------------
    global robot
    # assign robot coordinates
    robot.x =  data.pose.pose.position.x
    robot.y =  data.pose.pose.position.y
    [rool, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    robot.theta = yaw
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackRefAngle(data):
# -----------------------------------------------------------------------------
    global thetaRef
    # assign angular ref to robot, convert degree order in radian
    thetaRef = math.radians(data.data)
# -----------------------------------------------------------------------------


# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdometry)
rospy.Subscriber("thetaRef", Float32, callBackRefAngle)

# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    

    # control variables    
    cmdVelMsg = Twist()
    cmdVelMsg.linear.x = 0.0
    cmdVelMsg.linear.y = 0.0
    cmdVelMsg.linear.z = 0.0
    cmdVelMsg.angular.x = 0.0
    cmdVelMsg.angular.y = 0.0
    cmdVelMsg.angular.z = 0.0
    Omega = 0.0
 

    while not rospy.is_shutdown():


        # angular velocity
        
        if ( math.fabs(thetaRef - robot.theta)*180./3.14 < 10.0): # deadzone
           Omega = 0.0
        else:
           Omega = kAngle * (thetaRef - robot.theta) # proportional control
           
           if ( math.fabs(Omega)>OmegaMax ): # saturation
			   if (Omega>0):
				   Omega = OmegaMax
			   else:
				   Omega = -OmegaMax


        rospy.loginfo("kAngle=%f thetaRef=%f theta=%f Omega=%f angularDist=%f" % ( kAngle, thetaRef, robot.theta, Omega, math.fabs(thetaRef - robot.theta) ) )

        # control msg        
        cmdVelMsg.angular.z = Omega
         
         
        # msg publication
        pubCmdVel.publish(cmdVelMsg)
        pubTheta.publish(robot.theta)


        angleCmdPubRate.sleep()

# -----------------------------------------------------------------------------
