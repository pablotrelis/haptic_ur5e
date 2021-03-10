#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geomagic_control.msg import DeviceButtonEvent
from std_msgs.msg import Int16, Float64
import sys
import tf
import moveit_commander
from moveit_commander.conversions import pose_to_list
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

boton_gris = Int16()
pose_x = Float64()
pose_y = Float64()
pose_z = Float64()

def Callback_botones(botones):
    boton_gris.data = botones.grey_button
    #rospy.loginfo(boton_gris)

def Callback_pose(posicion):
    pose_x.data = posicion.pose.position.x
    pose_y.data = posicion.pose.position.y
    pose_z.data = posicion.pose.position.z
    #rospy.loginfo(pose_x)

def main():
    rospy.init_node("lectura_botones")
    sub_botones = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, Callback_botones)
    sub_pose = rospy.Subscriber("/Geomagic/pose", PoseStamped, Callback_pose)
    pub = rospy.Publisher("boton_gris",Int16,queue_size=1)
    r = rospy.Rate(1)

    pose_goal = Pose()
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    pose_goal.position.x = -0.8
    pose_goal.position.y = 0.133
    pose_goal.position.z = 0.2
    while not rospy.is_shutdown():
        if(boton_gris.data == 1):
            #pose_goal.orientation.w = 0.0
            pose_goal.position.x = -(pose_z.data*1.9-471)/1000
            pose_goal.position.y = -(pose_x.data*1.62-135)/1000
            pose_goal.position.z = (pose_y.data*1+250)/1000
            print(pose_goal.position.x)
            #pose_goal.position.y = pose_y.data/100
            #pose_goal.position.z = pose_z.data/100
        #group[0].set_pose_target(pose_goal)
        #group[0].go(True)
        #print(pose_x.data)


        move_group.set_pose_target(pose_goal)
        plan=move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
