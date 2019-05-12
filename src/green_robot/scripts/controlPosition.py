#!/usr/bin/env python
"""
@author: S. Bertrand

# position control of mobile robot

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
rospy.init_node('control_position', anonymous=True)

# static parameters
# ------------------
interWheelDistance_ = rospy.get_param('interWheelDistance', 0.15)
wheelDiameter_ = rospy.get_param('wheelDiameter', 0.07)
x0_ = rospy.get_param('x0', 0.)
y0_ = rospy.get_param('y0', 0.)
theta0_ = rospy.get_param('theta0', 0.0)
encoderResolution_ = rospy.get_param('encoderResolution', 720) #6*120
VMax = rospy.get_param('Vmax', 0.15)
OmegaMax = rospy.get_param('OmegaMax', 2)
#rospy.loginfo("dist=%f" % interWheelDistance_)

# robot object
# ------------
robot = robotClass.Robot(interWheelDistance=interWheelDistance_, wheelDiameter=wheelDiameter_, x0=x0_, y0=y0_, theta0=theta0_, encoderResolution=encoderResolution_)


# publishers
# -----------
pubTheta = rospy.Publisher('theta', Float32, queue_size=10)
pubThetaRef = rospy.Publisher('thetaRef', Float32, queue_size=10)
pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# frequency of position control
# -------------------------------
fePos = 5.0
TePos = 1/fePos
positionCmdPubRate = rospy.Rate(fePos)


# frequency of orientation conrol (multiple of fePos)
# ----------------------------------------------------
feAngle = 10.0
feRatioAngleOverPos = int( feAngle/fePos )


# dynamic parameters
#--------------------

# ----------------------------------------------------------------------------
def callbackDynParam(config, level):
# -----------------------------------------------------------------------------

    global kPos
    global kAngle
    #global xRef
    #global yRef

    kPos = float("""{kPos}""".format(**config))
    kAngle = float("""{kAngle}""".format(**config))
    #xRef = float("""{xRef}""".format(**config))
    #yRef = float("""{yRef}""".format(**config))

    return config


# server for dyamic parameters
srv = Server(ControlPositionCFGConfig, callbackDynParam)


#init dynamic parameters
kPos = rospy.get_param('/controlPosition/kPos', 0.5)
kAngle = rospy.get_param('/controlPosition/kAngle', 1.2)
#xRef = rospy.get_param('/controlPosition/xRef', 0.0)
#yRef = rospy.get_param('/controlPosition/yRef', 0.0)

xRef = x0_
yRef = y0_

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
def callBackWP(data):
# -----------------------------------------------------------------------------
    global xRef, yRef
    # assign robot coordinates
    xRef = data.x
    yRef = data.y
# -----------------------------------------------------------------------------


# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdometry)
rospy.Subscriber("xyRef", Point32, callBackWP)

# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    

    # control variables    
    cmdVelMsg = Twist()
    V = 0.0
    thetaRef = 0.0
    Omega = 0.0
    
    countFreqRatio = 0
	
    

    while not rospy.is_shutdown():


         if ( math.sqrt((xRef-robot.x)**2 + (yRef-robot.y)**2) > 0.15 ):

             # velocity
             Vx = xRef - robot.x
             Vy = yRef - robot.y
             V =  kPos * math.sqrt( Vx*Vx + Vy*Vy  )
			 
             # saturation
             V = 0.3#min(V, VMax)


			 
             if (countFreqRatio>=feRatioAngleOverPos-1):
			 
                 countFreqRatio = 0
			 
                 # reference angle
                 if (robot.x == xRef):
                     if (robot.y == yRef):
                         thetaRef = robot.theta
                     elif (robot.y < yRef):
                         thetaRef = math.pi/2
                     elif (robot.y > yRef):
                         thetaRef = -math.pi/2
                 else:
                     thetaRef = math.atan2(yRef-robot.y, xRef-robot.x)
				 # to ensure continuity
                 if math.fabs(robot.theta-thetaRef)>math.pi:
                     thetaRef = thetaRef + math.copysign(2*math.pi,robot.theta)   
				 
				 # angular velocity
                 if ( math.fabs(thetaRef - robot.theta)*180./3.14 < 7): # deadzone
                     Omega = 0.0
                 else:
                     Omega = kAngle * (thetaRef - robot.theta) # proportional control
           
                     if ( math.fabs(Omega)>OmegaMax ): # saturation
                         if (Omega>0):
                             Omega = OmegaMax
                         else:
                             Omega = -OmegaMax

         else:
            V = 0.0
            thetaRef = 0.0
            Omega = 0.0


         rospy.loginfo("kPos=%f kAngle=%f V=%f thetaRef=%f Omega=%f x=%f y=%f xRef=%f yRef=%f dist=%f" % (kPos, kAngle, V, thetaRef, Omega, robot.x, robot.y, xRef, yRef, math.sqrt((xRef-robot.x)**2 + (yRef-robot.y)**2)))

         # control msg        
         cmdVelMsg.linear.x = V
         cmdVelMsg.linear.y = 0.0
         cmdVelMsg.linear.z = 0.0
         cmdVelMsg.angular.x = 0.0
         cmdVelMsg.angular.y = 0.0
         cmdVelMsg.angular.z = Omega
         
         
         # msg publication
         pubCmdVel.publish(cmdVelMsg)
         pubTheta.publish(robot.theta)
         pubThetaRef.publish(thetaRef)

         countFreqRatio += 1

         positionCmdPubRate.sleep()

# -----------------------------------------------------------------------------
