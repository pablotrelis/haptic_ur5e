#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String, Int16, Float64
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3
from geomagic_control.msg import DeviceButtonEvent, DeviceFeedback
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

##########################################
# Joint trayectory control UR5e Geomagic #
##########################################

############
# Var init #
############
# Botones
boton_gris = Int16()

# Forces vars
force_x = Float64()
force_y = Float64()
force_z = Float64()
# Math vars
pi = math.radians(180)

#########################################
# Callback button from /Geomagic/button #
#########################################
def Callback_botones(botones):
    boton_gris.data = botones.grey_button
    #rospy.loginfo(boton_gris)

##############################
# Callback pose from /wrench #
##############################
def Callback_force(forces):
    force_x.data = forces.wrench.force.x
    force_y.data = forces.wrench.force.y
    force_z.data = forces.wrench.force.z

#################
# Main function #
#################
def main():
    # Inicializo nodo - haptic_jointpose -
    rospy.init_node('testeo_forces', anonymous=False)
    # Moveit commander, robot commander y planning scene
    moveit_commander.roscpp_initialize(sys.argv)
    # Subscribo topics /Geomagic/button y /Geomagic/joint_status
    sub_botones = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, Callback_botones)
    # Subscribo topic fuerzas /wrench
    sub_force = rospy.Subscriber("/wrench", WrenchStamped, Callback_force)
    # Publish force feedback
    pub = rospy.Publisher('/Geomagic/force_feedback', DeviceFeedback, queue_size=1)
    r = rospy.Rate(10)
    # Instancio MoveGroupCommander, para UR5e - manipulator - (Robot planning group)

    # Publico topic /move_group/display_planned_path

    # Inicializo joint vars


    df=DeviceFeedback()
    df.force=Vector3()
    ############################
    # Main while rospy running #
    ############################
    while not rospy.is_shutdown():
        r.sleep()
        #print df
        ######################
        # Push button action #
        ######################
        df.force.x=force_x.data
        df.force.y=force_y.data
        df.force.z=force_z.data
        print(df)
        if(boton_gris.data == 1):

            print('Boton activado')
            df.force.x=1
            df.force.y=1
            df.force.z=1
            pub.publish(df)

        ##### Final if grey button #####



###############
# try excepts #
###############
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
