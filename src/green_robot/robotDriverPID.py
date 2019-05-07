#!/usr/bin/env python
"""
@author: S. Bertrand

# main script: robot driver PID

# all variables in SI unit

"""

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from green_robot.msg import Int32Stamped # for receiving navdata feedback
import numpy as np
import robotClass


# node init
# ----------
rospy.init_node('green_robot_driver_PID', anonymous=True)


# robot object
# ------------
# **** TO DO: put encoderResolution, wheel diameter and inter wheels dist as parameters (YAML)
robot = robotClass.Robot(interWheelDistance=0.138, wheelDiameter=0.065, x0=0.0, y0=0.0, theta0=0.0)



# publishers
# -----------
# wheel speed estimated from wheel encoders (linear and angular)
pubLeftWheelSpeed = rospy.Publisher('green_robot/wheelSpeed/left', TwistStamped, queue_size=10)
pubRightWheelSpeed = rospy.Publisher('green_robot/wheelSpeed/right', TwistStamped, queue_size=10)
# publish on these topics to send control values to the robot  (in [-255,+255])
pubLeftMotorCmd = rospy.Publisher('green_robot/cmdMotor/left', Int32Stamped, queue_size=10)
pubRightMotorCmd = rospy.Publisher('green_robot/cmdMotor/right', Int32Stamped, queue_size=10)
# frequency of PID control of wheel angular velocities
fe = 10
Te = 1/fe
motorCmdPubRate = rospy.Rate(fe)


# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackLeftEncoderCount(data):
# -----------------------------------------------------------------------------
    global robot
    
    robot.leftWheel.encoderCount = data.data 
    t = data.header.stamp.secs + 1.E-9*data.header.stamp.nsecs
    deltaT = t-robot.leftWheel.lastTimeCountChange
#    rospy.loginfo(rospy.get_name() + " - time: %f" % t)
    
    # angular speed computation
    if (deltaT>0.0):
        # first order filter
        omegaOld = robot.leftWheel.omega        
        omega = (robot.leftWheel.encoderCount - robot.leftWheel.encoderCountPrev)*(2.0*np.pi/robot.leftWheel.encoderResolution) / deltaT      
        alpha = 0.3
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
        alpha = 0.3
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


# -----------------------------------------------------------------------------
def callBackCmdVel(data):
# -----------------------------------------------------------------------------
    global robot

    # get linear and angular velocities
    robot.Vref = data.linear.x
    robot.omegaRef = data.angular.z
    
    # wheel radius and distance between wheels
    r = 0.5*(robot.leftWheel.diameter/2.0 + robot.rightWheel.diameter/2.0)
    d = robot.interWheelDistance

    # assign desired wheel speeds
    robot.rightWheel.omegaRef = 0.5 * (2.0*robot.Vref + d*robot.omegaRef)/r    
    robot.leftWheel.omegaRef =  0.5 * (2.0*robot.Vref - d*robot.omegaRef)/r    

    rospy.loginfo(rospy.get_name() + " (%f , %f)" % (robot.leftWheel.omegaRef , robot.rightWheel.omegaRef))
# -----------------------------------------------------------------------------
   
   
# subscribers
# ------------
rospy.Subscriber("green_robot/countEncoder/left", Int32Stamped, callBackLeftEncoderCount)
rospy.Subscriber("green_robot/countEncoder/right", Int32Stamped, callBackRightEncoderCount)
rospy.Subscriber("cmd_vel", Twist, callBackCmdVel)




# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    

    # for PID control    
    uLeftMsg = Int32Stamped()
    uRightMsg = Int32Stamped()
    kp = 1.4 #50.0  #value chosen to ensure good ctrl values u for v~10cm/s and omega~45deg/s
    ki = 0.15 #0.0
    epsilonPrecLeft = 0.0
    epsilonPrecRight = 0.0
    integralLeft = 0.0
    integralRight = 0.0


    while not rospy.is_shutdown():

         # PID for wheel speed regulation
         epsilonLeft = robot.leftWheel.omegaRef - robot.leftWheel.omega
         epsilonRight = robot.rightWheel.omegaRef - robot.rightWheel.omega

         integralLeft = integralLeft + Te*(epsilonLeft - epsilonPrecLeft)         
         integralRight = integralRight + Te*(epsilonRight - epsilonPrecRight)
         
         uLeft = kp*epsilonLeft + ki*integralLeft   
         uRight = kp*epsilonRight + ki*integralRight
         
         # saturations
         uLeft = np.clip(uLeft, -255, 255)
         uRight = np.clip(uRight, -255, 255)

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


         motorCmdPubRate.sleep()

# -----------------------------------------------------------------------------
