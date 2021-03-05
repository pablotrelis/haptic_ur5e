#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geomagic_control.msg import DeviceButtonEvent
from std_msgs.msg import Int16, Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import numpy as np

boton_gris = Int16()
waist = Float64()
shoulder = Float64()
elbow = Float64()
yaw = Float64()
pitch = Float64()
roll = Float64()
joint_haptic  = []
#pi = math.radians(180)

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
    rospy.init_node("haptic_ur5e")
    sub_botones = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, Callback_botones)
    sub_joints = rospy.Subscriber("/Geomagic/joint_states", JointState, Callback_joints)
    pub = rospy.Publisher('/scaled_pos_traj_controller/command' ,JointTrajectory, queue_size=10)
    r = rospy.Rate(1)
    jt = JointTrajectory()
    jtp = JointTrajectoryPoint()

    while not rospy.is_shutdown():
        joint_haptic = [waist.data, shoulder.data, elbow.data, yaw.data, pitch.data, roll.data]
        if(boton_gris.data == 1):

            #jt.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
            #              'wrist_1_joint','wrist_2_joint','wrist_3_joint']
            #jtp.positions = [joint_haptic[0],math.radians(-70), math.radians(90), math.radians(-45), math.radians(-90), 0]
            #jtp.velocities = [1,1,1,1,1,1]
            #jtp.accelerations = [1,1,1,1,1,1]
            #jtp.time_from_start = rospy.Duration(2)

            #jt.points.append(jtp)

            #pub.publish(jt)

            print joint_haptic
            r.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
