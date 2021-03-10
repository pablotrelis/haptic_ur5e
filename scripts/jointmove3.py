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
from geometry_msgs.msg import PoseStamped, Pose
from geomagic_control.msg import DeviceButtonEvent
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

boton_gris = Int16()
x = Float64()
y = Float64()
z = Float64()

pose_haptic  = []
pi = math.radians(180)

def Callback_botones(botones):
    boton_gris.data = botones.grey_button
    #rospy.loginfo(boton_gris)

def Callback_pose(posicion):
    x.data = posicion.pose.position.x
    y.data = posicion.pose.position.y
    z.data = posicion.pose.position.z

    #rospy.loginfo(posicion_juntas)

def main():
    rospy.init_node('haptic_jointpose', anonymous=False)
    #joint_goal = get_current_joint_values()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    sub_botones = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, Callback_botones)
    sub_pose = rospy.Subscriber("/Geomagic/pose", PoseStamped, Callback_pose)

    r = rospy.Rate(10)


    pose_goal=Pose()
    move_group = moveit_commander.MoveGroupCommander("manipulator")


    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)



    # We can get the joint values from the group and adjust some of the values:
    #joint_goal = move_group.get_current_joint_values()

    while not rospy.is_shutdown():
        #joint_haptic = [0,0,0,0,0,0]
        if(boton_gris.data == 1):
            pose_haptic = [x.data, y.data, z.data]
            print pose_haptic

            pose_goal.position.x=x.data
            #r.sleep()
        move_group[0].set_pose_target(pose_goal)
        move_group[0].go(True)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        #move_group.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
