#!/usr/bin/env python
import sys
import copy
import rospy
import geometry_msgs.msg
import math
from std_msgs.msg import String, Int16, Float64
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, WrenchStamped
from geomagic_control.msg import DeviceButtonEvent, DeviceFeedback
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import keyboard  # using module keyboard

###################################################
# Joint trayectory control UR5e Geomagic RealTime #
###################################################

############
# Var init #
############
# Botones
boton_gris = Int16()
boton_blanco = Int16()
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
vel = Float64()
# Math vars
pi = math.radians(180)


#########################################
# Callback button from /Geomagic/button #
#########################################
def Callback_botones(botones):
    boton_gris.data = botones.grey_button
    boton_blanco.data = botones.white_button
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

###############################
# Callback force from /wrench #
###############################
def Callback_force(forces):
    force_x.data = forces.wrench.force.x
    force_y.data = forces.wrench.force.y
    force_z.data = forces.wrench.force.z

def Callback_speed(speed):
    vel.data = speed.data
#################
# Main function #
#################
def main():
    # Inicializo nodo - haptic_jointpose -
    rospy.init_node('hello_bye', anonymous=False)
    r = rospy.Rate(10) #Rate 500Hz
    # Pub  /Geomagic/force_feedback ,/scaled_pos_traj_controller/command
    jt_pub_ur5e = rospy.Publisher('/scaled_pos_traj_controller/command',
                            JointTrajectory, queue_size=1, latch=True)
    pub_force = rospy.Publisher('/Geomagic/force_feedback',
                            DeviceFeedback, queue_size=1, latch=True)
    # Subscribo topics /Geomagic/button y /Geomagic/joint_status
    sub_botones = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent,
                                                        Callback_botones)
    sub_joints = rospy.Subscriber("/Geomagic/joint_states", JointState,
                                                        Callback_joints)
    # Subscribo forces de UR5e /wrench
    sub_force = rospy.Subscriber("/wrench", WrenchStamped, Callback_force)
    # Subscribo valocidad robot /speed_scaling_factor
    sub_speed = rospy.Subscriber("/speed_scaling_factor",Float64,Callback_speed)

    # Joint msg
    jt_ur5e = JointTrajectory()
    jt_ur5e.points = [JointTrajectoryPoint()]
    jt_ur5e.joint_names = ['shoulder_pan_joint','shoulder_lift_joint',
                'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    # Inicializo joint vars
    joint_goal = [pi/2,-0.2689*2,0.6397+pi/2,-pi+pi/5,-pi/2,pi]
    joint_haptic = [0,0,0,0,0,0]
    joint_des = math.radians(0) #Desfase por acceleration y velocities
    ac=0 #accelerations
    ve=0 #velocities
    flj=0
    jt_ur5e.points[0].accelerations=[ac,ac,ac,ac,ac,ac]
    jt_ur5e.points[0].velocities=[ve,ve,ve,ve,ve,ve]
    # TIME FROM START CONTROL
        # Control por tiempo en nano segundos (CUIDADO nsecs muy bajo)
    jt_ur5e.points[0].time_from_start.secs = 4

    ############################
    # Main while rospy running #
    ############################
    rospy.sleep(2)
    print'\033[1;33;38m go \033[1;37;0m'
    while not rospy.is_shutdown():
        
        assert vel.data<0.61, "Velocidad muy elevada"
        if(boton_blanco.data == 1): #Posicion guardad
            jt_ur5e.points[0].positions=[pi/2,-pi/2,pi/2+pi/4,-pi+pi/5,-pi/2,pi]
            jt_pub_ur5e.publish(jt_ur5e)
            rospy.sleep(3)
            joint_goal=[math.radians(-27),math.radians(-63),math.radians(141),
                    math.radians(-165),math.radians(-90),math.radians(180)]
            jt_ur5e.points[0].positions=joint_goal
            jt_pub_ur5e.publish(jt_ur5e)
            rospy.sleep(3)
        ######################
        # Push button action #
        ######################
        if(boton_gris.data == 1): #Posicion init
            jt_ur5e.points[0].positions=[pi/2,-pi/2,pi/2+pi/4,-pi+pi/5,-pi/2,pi]
            jt_pub_ur5e.publish(jt_ur5e)
            rospy.sleep(3)
            joint_goal=[pi/2,-0.2689*2,0.6397+pi/2,-pi+pi/5,-pi/2,pi]
            jt_ur5e.points[0].positions=joint_goal
            jt_pub_ur5e.publish(jt_ur5e)
            rospy.sleep(3)
        ##### Final if grey button #####


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
