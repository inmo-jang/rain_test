#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState # To receive the current state

import time
import numpy as np

from rain_unity.msg import rain_system  as RainMsg # To receive the system status from Unity
from rain_unity.msg import Human_orion  as LeapMsg # for LEAP in Unity

############ User Setting ##############
pub_rate = 250
xyzrpy = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]    # (m/s) XYZ velocity // (rad) Amplitudes of Roll Pitch Yaw in End Effector Frame
moving_period = 2     # Repeatative moving period (sec)
########################################

def fake_leap_generator():   

    pub = rospy.Publisher('/rain/leap_motion',LeapMsg,queue_size=10)
    pub_rain = rospy.Publisher('/rain/status',RainMsg, queue_size=10)
            
 
    rate = rospy.Rate(pub_rate)

    # Initalisation Msgs
    Rain_Mode = RainMsg()
    Human_operator = LeapMsg()

    palm_normal_0 = [[0], [0], [1]] # Initial Desired Palm Normal (based on Base frame)
    palm_direction_0 = [[0], [1], [0]] # Initial Desired Palm Directional (based on Base frame)

    print("Waiting for ROS Time")
    while rospy.get_time() == 0: # Waiting for receiving the first message from /clock
        pass
    time_0 = rospy.get_rostime()
    print("Got ROS Time")

    # Generate X,Y,Z and Euler Angle Variation
    while not rospy.is_shutdown():
        
        time_now = rospy.get_rostime()

        time_step = (time_now - time_0).to_sec()

        x = xyzrpy[0]*np.sin(2*np.pi*time_step/moving_period)
        y = xyzrpy[1]*np.sin(2*np.pi*time_step/moving_period)
        z = xyzrpy[2]*np.sin(2*np.pi*time_step/moving_period)

        roll = xyzrpy[3]*np.sin(2*np.pi*time_step/moving_period)
        pitch = xyzrpy[4]*np.sin(2*np.pi*time_step/moving_period)
        yaw = xyzrpy[5]*np.sin(2*np.pi*time_step/moving_period)

        a11 = np.cos(pitch)*np.cos(yaw)
        a12 = -np.cos(pitch)*np.sin(yaw)
        a13 = np.sin(pitch)
        a21 = np.cos(roll)*np.sin(yaw)+np.cos(yaw)*np.sin(roll)*np.sin(pitch)
        a22 = np.cos(roll)*np.cos(yaw)-np.sin(roll)*np.sin(pitch)*np.sin(yaw)
        a23 = -np.cos(pitch)*np.sin(roll)
        a31 = np.sin(roll)*np.sin(yaw)-np.cos(roll)*np.cos(yaw)*np.sin(pitch)
        a32 = np.cos(yaw)*np.sin(roll)+np.cos(roll)*np.sin(pitch)*np.sin(yaw)
        a33 = np.cos(roll)*np.cos(pitch)
        
                 
        T = np.matrix([[a11, a12, a13],[a21, a22, a23],[a31, a32, a33]])
        palm_normal = np.matmul(T, palm_normal_0)
        palm_direction = np.matmul(T, palm_direction_0)
        ## Palm Normal 
        Human_operator.right_hand.palm_normal.x = palm_normal[0]
        Human_operator.right_hand.palm_normal.y = palm_normal[1]
        Human_operator.right_hand.palm_normal.z = palm_normal[2]
        ## Palm Direction
        Human_operator.right_hand.palm_direction.x = palm_direction[0]
        Human_operator.right_hand.palm_direction.y = palm_direction[1]
        Human_operator.right_hand.palm_direction.z = palm_direction[2]
        ## Current Time
        # time_now = rospy.get_rostime()
        Human_operator.header.stamp = time_now
        
        ## Palm Center Velocity
        Human_operator.right_hand.palm_velocity = [x, y, z]

        ## Etc default info
        Human_operator.right_hand.is_present = True
        Human_operator.right_hand.grab_strength = 1.0          
        
        pub.publish(Human_operator)

        Rain_Mode.teleoperation_mode = "MODE_1"
        pub_rain.publish(Rain_Mode)

        rate.sleep()
    

def main():
    try:   
        rospy.init_node('fake_leap_generator', anonymous=True ,disable_signals=True)    
 
        fake_leap_generator()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()



############################
# 2018 Nov 30 by Inmo Jang
# This code generates a stream of fake leap motion information (palm_normal palm_direction) to test any of control node. 
############################