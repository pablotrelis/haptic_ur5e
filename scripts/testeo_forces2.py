#!/usr/bin/env python

import sys
import copy
import rospy
import roslib
from geomagic_control.msg import DeviceFeedback
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Bool


##########################################
# Joint trayectory control UR5e Geomagic #
##########################################

############
# Var init #
############


#################
# Main function #
#################

    # Inicializo nodo - haptic_jointpose -
rospy.init_node('testing', anonymous=False)

r = rospy.Rate(10)

pub = rospy.Publisher('/Geomagic/force_feedback', DeviceFeedback, queue_size=1)

df=DeviceFeedback()
df.force=Vector3()
df.lock=[False]

while not rospy.is_shutdown():
    df.force.x=0
    df.force.y=0
    df.force.z=0
    pub.publish(df)
    r.sleep()
