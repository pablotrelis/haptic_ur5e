#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geomagic_control.msg import DeviceButtonEvent
from std_msgs.msg import Int16, Float64
import sys
import tf
import moveit_commander
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
    group = [moveit_commander.MoveGroupCommander("manipulator")]

    while not rospy.is_shutdown():
         if(boton_gris.data == 1):
           pose_goal.orientation.w = 0.0
           pose_goal.position.x = pose_x.data/100
           pose_goal.position.y = pose_y.data/100
           pose_goal.position.z = pose_z.data/100
           group[0].set_pose_target(pose_goal)
           group[0].go(True)

         else:
             print("hola")
           pose_goal.orientation.x = 2.15
           pose_goal.orientation.y = 3.097
           pose_goal.orientation.z = -1.505
           pose_goal.position.x = -0.6
           pose_goal.position.y = 0.14
           pose_goal.position.z = 0.106
           group[0].set_pose_target(pose_goal)
           group[0].go(True)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
