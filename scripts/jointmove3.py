#!/usr/bin/env python

import sys
#import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String, Int16, Float64
#from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped
from geomagic_control.msg import DeviceButtonEvent
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

boton_gris = Int16()
waist = Float64()
shoulder = Float64()
elbow = Float64()
yaw = Float64()
pitch = Float64()
roll = Float64()
joint_haptic  = []
pi = math.radians(180)

def Callback_botones(botones):
    boton_gris.data = botones.grey_button
    #rospy.loginfo(boton_gris)

def Callback_joints(joints):
    waist.data = joints.position[0]
    shoulder.data = joints.position[1]
    elbow.data = joints.position[2]
    yaw.data = joints.position[3]
    pitch.data = joints.position[4]
    roll.data = joints.position[5]
    #rospy.loginfo(posicion_juntas)

def main():
    rospy.init_node('haptic_jointpose', anonymous=True)
    #joint_goal = get_current_joint_values()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    sub_botones = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, Callback_botones)
    sub_joints = rospy.Subscriber("/Geomagic/joint_states", JointState, Callback_joints)

    r = rospy.Rate(10000000)

    move_group = moveit_commander.MoveGroupCommander("manipulator")


    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_haptic = [0,0,0,0,0,0]
    joint_goal = [pi,-pi/4,0.6397+pi/2,-pi+pi/5,pi+pi/2,pi]
    while not rospy.is_shutdown():
        #joint_haptic = [0,0,0,0,0,0]
        if(boton_gris.data == 1):
            joint_haptic = [waist.data, shoulder.data, elbow.data, yaw.data, pitch.data, roll.data]
            print joint_haptic
            #r.sleep()
            joint_goal[0] = joint_haptic[0]+pi
            joint_goal[1] = -joint_haptic[1]-pi/4
            joint_goal[2] = -joint_haptic[2]+pi/2
            joint_goal[3] = joint_haptic[4]+pi/5
            joint_goal[4] = joint_haptic[3]+pi/2
            joint_goal[5] = joint_haptic[5]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
