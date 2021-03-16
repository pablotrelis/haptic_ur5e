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
from geometry_msgs.msg import PoseStamped, WrenchStamped
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
# Joints vars
waist = Float64()
shoulder = Float64()
elbow = Float64()
yaw = Float64()
pitch = Float64()
roll = Float64()
# Force vars
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
    rospy.init_node('jointmove4', anonymous=False)

    jt_pub_ur5e = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=1)
    pub_force = rospy.Publisher('/Geomagic/force_feedback', DeviceFeedback, queue_size=1)
    # Subscribo topics /Geomagic/button y /Geomagic/joint_status
    sub_botones = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, Callback_botones)
    sub_joints = rospy.Subscriber("/Geomagic/joint_states", JointState, Callback_joints)

    sub_force = rospy.Subscriber("/wrench", WrenchStamped, Callback_force)
    r = rospy.Rate(10)

    jt_ur5e = JointTrajectory()
    jt_ur5e.points = [JointTrajectoryPoint()]
    jt_ur5e.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                  'wrist_1_joint','wrist_2_joint','wrist_3_joint']
    # Inicializo joint vars
    joint_goal = [pi/2,-0.2689*2,0.6397+pi/2,-pi+pi/5,-pi/2,pi]
    joint_haptic = [0,0,0,0,0,0]
    jt_ur5e.points[0].positions=joint_goal
    #jt_ur5e.points[0].velocities = [0.5,0.5,0.5,0.5,0.5,0.5]
    #jt_ur5e.points[0].accelerations = [0.5,0.5,0.5,0.5,0.5,0.5]
    jt_ur5e.points[0].time_from_start.secs = 1
    # Force vars
    f_const=10
    f_cap=0.6
    df=DeviceFeedback()
    df.lock=[False]

    ############################
    # Main while rospy running #
    ############################
    while not rospy.is_shutdown():
        ##################
        # Forces control #
        ##################
        df.force.x=force_x.data/f_const
        if(df.force.x<f_cap and df.force.x>-f_cap ):
            df.force.x=0
        df.force.y=-force_y.data/f_const
        if(df.force.y<f_cap and df.force.y>-f_cap ):
            df.force.y=0
        df.force.z=-force_z.data/f_const-1
        if(df.force.z<f_cap and df.force.z>-f_cap ):
            df.force.z=0
        #if(df.force.x!=0 or df.force.y!=0 or df.force.z!=0):
        pub_force.publish(df)

        ######################
        # Push button action #
        ######################
        if(boton_gris.data == 1):
            joint_haptic = [waist.data, shoulder.data, elbow.data,
                                            yaw.data, pitch.data, roll.data]
            print joint_haptic
            #r.sleep()
            joint_goal[0] = joint_haptic[0]+pi/2
            joint_goal[1] = -joint_haptic[1]-0.2689
            joint_goal[2] = -joint_haptic[2]+pi/2
            joint_goal[3] = joint_haptic[4]+pi/5
            joint_goal[4] = joint_haptic[3]-pi-pi/2
            joint_goal[5] = joint_haptic[5]
        ##### Final if grey button #####
        jt_ur5e.points[0].positions=joint_goal
        jt_pub_ur5e.publish(jt_ur5e)
        r.sleep()
    ##### Final while rospy running #####

###############
# try excepts #
###############
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
