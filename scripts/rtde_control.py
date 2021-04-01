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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#Install numpy & spicy
import numpy as np
from scipy.spatial.transform import Rotation as R
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

pose_x = Float64()
pose_y = Float64()
pose_z = Float64()
pose_w = Float64()
# Speed vars
vel = Float64()
# Math vars
pi = math.radians(180)
# Output force
output_stream = sys.stdout

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

def Callback_pose(geo_pose):
    pose_x.data = geo_pose.pose.orientation.x
    pose_y.data = geo_pose.pose.orientation.y
    pose_z.data = geo_pose.pose.orientation.z
    pose_w.data = geo_pose.pose.orientation.w

def Callback_speed(speed):
    vel.data = speed.data

#################
# Main function #
#################
def main():
    # Inicializo nodo - haptic_jointpose -
    rospy.init_node('haptic_jointpose', anonymous=False)
    r = rospy.Rate(500) #Rate Hz
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

    sub_pose = rospy.Subscriber('Geomagic/pose', PoseStamped, Callback_pose)
    # Subscribo valocidad robot /speed_scaling_factor
    sub_speed = rospy.Subscriber("/speed_scaling_factor",Float64,Callback_speed)

    # Joint msg /trayectory_msg/JointTrajectory.msg
    jt_ur5e = JointTrajectory()
    jt_ur5e.points = [JointTrajectoryPoint()]
    jt_ur5e.joint_names = ['shoulder_pan_joint','shoulder_lift_joint',
                'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    # Inicializo joint vars
    joint_goal = [pi/2,-0.2689*2,0.6397+pi/2,-pi+pi/5,-pi/2,pi] #init
    joint_haptic = [0,0,0,0,0,0]
    joint_des = math.radians(0) #Desfase por acceleration y velocities
    ac=0 #accelerations
    ve=0 #velocities
    flg=0 #Flag reajuste time_from_start
    jt_ur5e.points[0].accelerations=[ac,ac,ac,ac,ac,ac]
    jt_ur5e.points[0].velocities=[ve,ve,ve,ve,ve,ve]
    # TIME FROM START CONTROL
    # Control por tiempo en nano segundos (CUIDADO nsecs muy bajo)
    jt_ur5e.points[0].time_from_start.nsecs = 2000000000
    tfs=jt_ur5e.points[0].time_from_start.nsecs/1000000000.0
    # Force vars
    f_const=8 # Force mult (Multiplicador intensidad fuerza)
    f_cap=5 # Force sensitivity (Marge de error de fuerza)
    df=DeviceFeedback()
    df.lock=[False]
    force_flag=0 # force_flag != 0 , 1 -> Disable force btn
    # Force init
    df.force.x=0
    df.force.y=0
    df.force.z=0
    pub_force.publish(df)
    rospy.sleep(3)
    jt_ur5e.points[0].positions = joint_goal

    ####################
    # Print robot info #
    ####################
    print ('\033[1;32;38m ##### Ready ##### \033[1;37;0m \n')
    r.sleep()
    print '\033[1;33;38m Robot Speed: \033[1;37;0m', vel.data*100,'%'
    assert vel.data<0.61, "Velocidad muy elevada"
    print '\033[1;33;38m Time From Start: \033[1;37;0m', tfs,'s'
    if tfs<0.5:
        print '\033[1;31;38m WARN: Time From Start < 0.5s \033[1;37;0m'
    print '\033[1;37;38m ==================== \033[1;37;0m'

    ############################
    # Main while rospy running #
    ############################
    while not rospy.is_shutdown():
        # Activar/Desactivar fuerzas
        if(force_flag==0 and boton_blanco.data == 1 ):
            force_flag=1
            print ('\033[1;34;38m Control de fuerzas \033[1;32;38m '
                                            'activado \033[1;37;0m')
            while(boton_blanco.data == 1):
                r.sleep()
        elif(force_flag==1 and boton_blanco.data == 1 ):
            force_flag=0
            print ('\033[1;34;38m Control de fuerzas \033[1;31;38m '
                                            'desactivado \033[1;37;0m')
            df.force.x=0
            df.force.y=0
            df.force.z=0
            pub_force.publish(df)
            while(boton_blanco.data == 1):
                r.sleep()
        ##################
        # Forces control # (BETA)
        ##################
        if(force_flag==1):
            # Print fuerza
            output_stream.write(' Fuerza aplicada: %.4f\r' % -force_z.data)
            output_stream.flush()
            '''
            #Rigth-Left haptic
            df.force.x=force_x.data/f_const
            if(df.force.x<f_cap and df.force.x>-f_cap ):
                df.force.x=0
            #Up-Down haptic
            df.force.y=-force_z.data/f_const
            if(df.force.y<f_cap and df.force.y>-f_cap ):
                df.force.y=0
            #Front-Back haptic
            df.force.z=force_y.data/f_const
            if(df.force.z<f_cap+0.7 and df.force.z>-f_cap ):
                df.force.z=0
            #Publish diferencia mayor de 1
            if(force_x.data>df.force.x+1 or force_y.data>df.force.y+1 or
                                            force_z.data>df.force.z+1):
                pub_force.publish(df)
            if(force_x.data<df.force.x-1 or force_y.data<df.force.y-1 or
                                            force_z.data<df.force.z-1):
                pub_force.publish(df)
            '''
            ###############
            # Force calcs #
            ###############
            force_per=(-force_z.data-f_cap)*0.04*f_const
            if force_per<0:
                force_per=0
            rot = R.from_quat([pose_x.data,pose_y.data,pose_z.data,pose_w.data])
            (X,Y,Z)= rot.as_euler('zyx', degrees=False)
            ##### Fuerzas ejes XYZ #####
            #Fuerza x
            Fx=Y/(pi/2)
            #Fuerza y
            if (Z<pi/2 and Z>-pi/2):
                Fy=-Z/(pi/2)
            else:
                if Z<-pi/2:
                    Fy=(Z*2/pi+2)-abs(Y/(pi/2))
                    if Fy<0:
                        Fy=0
                else:
                    Fy=(Z*2/pi-2)+abs(Y/(pi/2))
                    if Fy>0:
                        Fy=0
            #Fuerza z
            if (Z<pi/2 and Z>-pi/2):
                Fz=1-abs(Z/(pi/2))-abs(Y/(pi/2))
                if Fz<0:
                    Fz=0
            else:
                Fz=-(abs(Z)*2/pi-1)
            # Porcentajes intensidad fuerzas
            px=Fx/(abs(Fx)+abs(Fy)+abs(Fz))
            py=Fy/(abs(Fx)+abs(Fy)+abs(Fz))
            pz=Fz/(abs(Fx)+abs(Fy)+abs(Fz))
            df.force.x=px*force_per
            df.force.y=py*force_per
            df.force.z=pz*force_per
            # Publicacion fuerzas
            pub_force.publish(df)

        ######################
        # Push button action #
        ######################
        if(boton_gris.data == 1):
            #First time btn push
            if(flg==0):
                # Reescala time_from_start
                jt_ur5e.points[0].time_from_start.nsecs = 500000000 #0.5 sec
                tfs=jt_ur5e.points[0].time_from_start.nsecs/1000000000.0
                print '\033[1;33;38m Time From Start: \033[1;37;0m',tfs,'s'
                flg=1;
                r.sleep()
            #Joint pose haptic
            joint_haptic = [waist.data, shoulder.data, elbow.data,
                                    yaw.data, pitch.data, roll.data]
            #Set joint value
            joint_goal[0] = joint_haptic[0]+pi/2+joint_des
            joint_goal[1] = -joint_haptic[1]-0.2689+joint_des
            joint_goal[2] = -joint_haptic[2]+pi/2+joint_des
            joint_goal[3] = joint_haptic[4]+pi/5+joint_des
            joint_goal[4] = joint_haptic[3]-pi-pi/2+joint_des
            joint_goal[5] = joint_haptic[5]+joint_des
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
