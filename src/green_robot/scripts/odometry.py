#!/usr/bin/env python
"""
@author: S. Bertrand & A.Reis

# main script: robot driver odometry

# all variables in SI unit

"""

import rospy
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from green_robot.msg import Int32Stamped # for receiving navdata feedback
import numpy as np
import robotClass



# node init
# ----------
rospy.init_node('odometry', anonymous=True)


# static parameters
# ------------------
interWheelDistance_ = rospy.get_param('interWheelDistance', 0.15)
wheelDiameter_ = rospy.get_param('wheelDiameter', 0.07)
x0_ = rospy.get_param('x0', 0.)
y0_ = rospy.get_param('y0', 0.)
theta0_ = rospy.get_param('theta0', 0.)
encoderResolution_ = rospy.get_param('encoderResolution', 720.0)
#rospy.loginfo("dist=%f" % interWheelDistance_)

# robot object
# ------------
robot = robotClass.Robot(interWheelDistance=interWheelDistance_, wheelDiameter=wheelDiameter_, x0=x0_, y0=y0_, theta0=theta0_, encoderResolution=encoderResolution_)



# publishers
# -----------
# odometry estimated from wheel encoders 
pubOdometry = rospy.Publisher('odom', Odometry, queue_size=50)

# frequency of odometry
fe = 10.
Te = 1./fe
odomPubRate = rospy.Rate(fe)


# tf broadcaster
# ---------------
odomTFBroadcaster = tf.TransformBroadcaster()


# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackLeftEncoderCount(data):

    global robot
    
    robot.leftWheel.encoderCount = data.data 
    t = data.header.stamp.secs + 1.E-9*data.header.stamp.nsecs
    deltaT = t-robot.leftWheel.lastTimeCountChange
    
    if (deltaT>0.0):
        # updates for next iteration
        robot.leftWheel.lastTimeCountChange = t
        robot.leftWheel.encoderCountPrev = robot.leftWheel.encoderCount
        
# -----------------------------------------------------------------------------        
        
        
# -----------------------------------------------------------------------------
def callBackRightEncoderCount(data):

    global robot
    
    robot.rightWheel.encoderCount = data.data 
    t = data.header.stamp.secs + 1.E-9*data.header.stamp.nsecs
    deltaT = t-robot.rightWheel.lastTimeCountChange
    
    if (deltaT>0.0):
        # updates for next iteration
        robot.rightWheel.lastTimeCountChange = t
        robot.rightWheel.encoderCountPrev = robot.rightWheel.encoderCount
        
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
    #global robot


    odomMsg = Odometry()
    odomMsg.header.frame_id ='world'
    odomMsg.child_frame_id = 'robot'
    
    t = rospy.get_time()
    tPrec = t
    leftWheelTheta = 0.0
    leftWheelThetaPrec = 0.0    
    rightWheelTheta = 0.0
    rightWheelThetaPrec = 0.0
    rL = robot.leftWheel.diameter / 2.0
    rR = robot.rightWheel.diameter / 2.0
    quaternion = Quaternion()

    #rospy.loginfo("robot x=%f y=%f theta=%f rL=%f rR=%f res=%f=%f" % (robot.x, robot.y, robot.theta, rL, rR, robot.leftWheel.encoderResolution, robot.rightWheel.encoderResolution))

    while not rospy.is_shutdown():

         # delta angles for the wheels
         leftWheelTheta = robot.leftWheel.encoderCount * 2.0*np.pi / robot.leftWheel.encoderResolution
         leftWheelDeltaTheta = leftWheelTheta - leftWheelThetaPrec         
         rightWheelTheta = robot.rightWheel.encoderCount * 2.0*np.pi / robot.rightWheel.encoderResolution
         rightWheelDeltaTheta = rightWheelTheta - rightWheelThetaPrec
         #rospy.loginfo("LWTheta=%f LWDeltaTheta=%f RWTheta=%f RWDeltaTheta=%f " % (leftWheelTheta,leftWheelDeltaTheta,rightWheelTheta,rightWheelDeltaTheta))
         
         # time
         t = rospy.get_time()
         deltaT = t - tPrec
         
         # pose and velocities estimates
         if (deltaT>0):
             # pose
             robot.x += ( (rR/2.0)*rightWheelDeltaTheta + (rL/2.0)*leftWheelDeltaTheta ) * np.cos(robot.theta)
             robot.y += ( (rR/2.0)*rightWheelDeltaTheta + (rL/2.0)*leftWheelDeltaTheta ) * np.sin(robot.theta)
             robot.theta += ( rR*rightWheelDeltaTheta  -  rL*leftWheelDeltaTheta ) / robot.interWheelDistance

             #rospy.loginfo("robot x=%f y=%f thetaDeg=%f" % (robot.x, robot.y, robot.theta*180./3.14))

             quat = tf.transformations.quaternion_from_euler(0.0, 0.0, robot.theta)
             quaternion = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, robot.theta))
             # linear and angular velocities
             robot.v = ( (rR/2.0)*rightWheelDeltaTheta + (rL/2.0)*leftWheelDeltaTheta ) / deltaT
             robot.omega = ( rR*rightWheelDeltaTheta  -  rL*leftWheelDeltaTheta ) / ( robot.interWheelDistance * deltaT )


         timeNow = rospy.Time.now()

         # broadcast TF
         odomTFBroadcaster.sendTransform( (robot.x, robot.y, 0.),  quat, timeNow, "robot", "world")#"odom", "world" )



         # odometry msgs        
         odomMsg.header.seq = odomMsg.header.seq + 1
         odomMsg.header.stamp = timeNow
         
         odomMsg.pose.pose.position.x = robot.x
         odomMsg.pose.pose.position.y = robot.y
         odomMsg.pose.pose.position.z = 0.0
         odomMsg.pose.pose.orientation.x = quaternion.x
         odomMsg.pose.pose.orientation.y = quaternion.y
         odomMsg.pose.pose.orientation.z = quaternion.z
         odomMsg.pose.pose.orientation.w = quaternion.w
         
         odomMsg.twist.twist.linear.x = robot.v
         odomMsg.twist.twist.angular.z = robot.omega
 
         
         # msgs publications
         pubOdometry.publish(odomMsg)
         
         
         # update for next iteration
         leftWheelThetaPrec = leftWheelTheta
         rightWheelThetaPrec = rightWheelTheta
         tPrec = t


         odomPubRate.sleep()

# -----------------------------------------------------------------------------
