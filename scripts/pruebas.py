#!/usr/bin/env python

import rospy
from geomagic_control.msg import DeviceButtonEvent
from std_msgs.msg import Int16, Float64
import sys
import tf
import moveit_commander
from moveit_commander.conversions import pose_to_list
import random
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

##########################################
# Pose trayectory control UR5e Geomagic #
##########################################

############
# Var init #
############
# Botones
boton_gris = Int16()
# Pose vars
pose_x = Float64()
pose_y = Float64()
pose_z = Float64()
ori_x = Float64()
ori_y = Float64()
ori_z = Float64()
ori_w = Float64()
# Joints vars
waist = Float64()
shoulder = Float64()
elbow = Float64()
yaw = Float64()
pitch = Float64()
roll = Float64()
# Math vars
pi = math.radians(180)

#########################################
# Callback button from /Geomagic/button #
#########################################
def Callback_botones(botones):
    boton_gris.data = botones.grey_button
    #rospy.loginfo(boton_gris)

#####################################
# Callback pose from /Geomagic/pose #
#####################################
def Callback_pose(posicion):
    pose_x.data = posicion.pose.position.x
    pose_y.data = posicion.pose.position.y
    pose_z.data = posicion.pose.position.z
    #rospy.loginfo(pose_x)
    ori_x.data = posicion.pose.orientation.x
    ori_y.data = posicion.pose.orientation.y
    ori_z.data = posicion.pose.orientation.z
    ori_w.data = posicion.pose.orientation.w

#############################################
# Callback pose from /Geomagic/joint_states #
#############################################
def Callback_joints(joints):
    waist.data = joints.position[0]
    shoulder.data = joints.position[1]
    elbow.data = joints.position[2]
    yaw.data = joints.position[3]
    pitch.data = joints.position[4]
    roll.data = joints.position[5]
    #rospy.loginfo(posicion_juntas)

#################
# Main function #
#################
def main():
    # Inicializo nodo - lectura_botones -
    rospy.init_node("haptic_jointpose", anonymous=False)
    # Subscribo topics /Geomagic/button, /Geomagic/pose y /Geomagic/joint_status
    sub_btn = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, Callback_botones)
    sub_pose = rospy.Subscriber("/Geomagic/pose", PoseStamped, Callback_pose)
    sub_joints = rospy.Subscriber("/Geomagic/joint_states", JointState, Callback_joints)
    #pub = rospy.Publisher("boton_gris",Int16,queue_size=1)
    #r = rospy.Rate(1)
    # Creo el pose_goal para moveit
    pose_goal = Pose()
    # Instancio MoveGroupCommander, para UR5e - manipulator - (Robot planning group)
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    # Creo el joint_goal para moveit
    joint_goal = move_group.get_current_joint_values()

    ######################
    # Joint move source  #
    ######################
    # Envio el brazo a la pose de inicio mediante joint position
    joint_goal = [0,-23.7*pi/180,60.59*pi/180,-218.28*pi/180,pi*3/2,0]
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    #######################
    # Default pose values #
    #######################
    pose_goal.position.x = -0.8
    pose_goal.position.y = 0.133
    pose_goal.position.z = 0.2

    ############################
    # Main while rospy running #
    ############################
    while not rospy.is_shutdown():
        ######################
        # Push button action #
        ######################
        if(boton_gris.data == 1):
            # Orientacion en quartenions
            pose_goal.orientation.x = -ori_z.data
            pose_goal.orientation.y = -ori_x.data
            pose_goal.orientation.z = ori_y.data
            pose_goal.orientation.w = ori_w.data
            # Posicion tool x, y, z
            pose_goal.position.x = -(pose_z.data*1.9-471)/1000
            pose_goal.position.y = -(pose_x.data*1.62-135)/1000
            pose_goal.position.z = (pose_y.data*1+250)/1000
            print(pose_goal.position)
        ##### Final if grey button #####

        # Add pose target
        move_group.set_pose_target(pose_goal)
        # Llamo planner para plan y execute
        plan=move_group.go(wait=True)
        # Con stop elimino movimiento residual
        move_group.stop()
        # Limpio los targets antes del siguiente planning
        move_group.clear_pose_targets()
    ##### Final while rospy running #####

###############
# try excepts #
###############
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
