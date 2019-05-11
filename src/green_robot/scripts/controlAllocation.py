#!/usr/bin/env python
"""
@author: S. Bertrand

# transform robot twist into wheel speeds

# all variables in SI unit

"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import robotClass


# node init
# ----------
rospy.init_node('position_allocation', anonymous=True)

# static parameters
# ------------------
interWheelDistance_ = rospy.get_param('interWheelDistance', 0.15)
wheelDiameter_ = rospy.get_param('wheelDiameter', 0.07)

# wheel radius
rL = wheelDiameter_/2.0
rR = wheelDiameter_/2.0
# inter wheel distance
d = interWheelDistance_

# publishers
# -----------
pubWheelSpeedRefLeft = rospy.Publisher('wheelSpeedRef/left', Float32, queue_size=10)
pubWheelSpeedRefRight = rospy.Publisher('wheelSpeedRef/right', Float32, queue_size=10)



# -----------------------------------------------------------------------------
def callBackCmdVel(data):
# -----------------------------------------------------------------------------
    # assign robot coordinates
    V = data.linear.x
    Omega = data.angular.z
    
    wheelSpeedRefLeftMsg = Float32()
    wheelSpeedRefRightMsg = Float32()
    
    wheelSpeedRefLeftMsg.data = 0.5*(2.0*V - d*Omega)/rL
    wheelSpeedRefRightMsg.data = 0.5*(2.0*V + d*Omega)/rR
    
    pubWheelSpeedRefLeft.publish(wheelSpeedRefLeftMsg)
    pubWheelSpeedRefRight.publish(wheelSpeedRefRightMsg)
    
# -----------------------------------------------------------------------------

# subscribers
# ------------
rospy.Subscriber("cmd_vel", Twist, callBackCmdVel)


# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    rospy.spin()    
# -----------------------------------------------------------------------------
