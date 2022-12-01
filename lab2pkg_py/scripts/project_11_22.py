#!/usr/bin/env python

'''
inspirations for this project come from ECE 470

to launch gazebo: set directory and source terminal then
roslaunch ur3_driver ur3_gazebo.launch

to run program: in different terminal, source it
rosrun lab2pkg_py project.py
'''
import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from project_header import *
import lab2_spawn
import project_funcs


# 20Hz
SPIN_RATE = 20

#Reset Position
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])
mid = np.radians([136,-87.74,108.34,-110.51,-90,15.69])
above1 = np.radians([123,-72,94,-111,-90,0])
above3 = np.radians([160,-103,128,-114,-90,40])

# Hanoi tower location 1
Q11 = [129.5*pi/180.0, -70*pi/180.0, 117*pi/180.0, -137*pi/180.0, -90*pi/180.0, 9.8*pi/180.0]
Q12 = [129.5*pi/180.0, -62.5*pi/180.0, 119.5*pi/180.0, -147*pi/180.0, -90*pi/180.0, 9.8*pi/180.0]
Q13 = [129.5*pi/180.0, -55.3*pi/180.0, 120.3*pi/180.0, -155*pi/180.0, -90*pi/180.0, 9.8*pi/180.0]

# Tower Location 2

Q21 = [135.7*pi/180.0, -74*pi/180.0, 123.8*pi/180.0, -139.6*pi/180.0, -90*pi/180.0, 16*pi/180.0]
Q22 = [135.7*pi/180.0, -66.6*pi/180.0, 126.2*pi/180.0, -149.5*pi/180.0, -90*pi/180.0, 16*pi/180.0]
Q23 = [135.7*pi/180.0, -58.3*pi/180.0, 127.2*pi/180.0, -158.8*pi/180.0, -90*pi/180.0, 16*pi/180.0]


# Tower Location 3

Q31 = [144*pi/180.0, -77.6*pi/180.0, 128.4*pi/180.0, -140.6*pi/180.0, -90*pi/180.0, 24.3*pi/180.0]
Q32 = [144*pi/180.0, -69*pi/180.0, 131.1*pi/180.0, -152*pi/180.0, -90*pi/180.0, 24.3*pi/180.0]
Q33 = [144*pi/180.0, -60*pi/180.0, 132.2*pi/180.0, -162*pi/180.0, -90*pi/180.0, 24.3*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False
image_var = None



# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

#initialize Q
Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]

#Callback Functions
def gripper_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN


def camera_callback(msg):
    global image_var 
    image_var = msg


def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True

#Defining Functions
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


#Move Block Function
'''     
#checking suctioning: add this to move block???
        if digital_in_0 == 0:
            rospy.loginfo("No block suctioned! Exiting")
            gripper(pub_cmd,loop_rate,suction_off)
            move_arm(pub_cmd,loop_rate,home,4.0,4.0) 
            sys.exit()
'''
def move_block(pub_cmd, loop_rate, start_loc, \
               end_loc, aboveloc):
    global go_away
    
    
    move_arm(pub_cmd,loop_rate,start_loc,4.0,4.0)
    gripper(pub_cmd,loop_rate,suction_on)
    time.sleep(1)
    move_arm(pub_cmd,loop_rate,aboveloc,4.0,4.0)
    move_arm(pub_cmd,loop_rate,end_loc,4.0,4.0)
    gripper(pub_cmd,loop_rate,suction_off)
    time.sleep(1)
    move_arm(pub_cmd,loop_rate,aboveloc,4.0,4.0)
    error = 0



    return error



#image conversion class/function
class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/cv_camera_node/image_raw", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!") 

    def image_callback(self, msg):

        global cv_image 
        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)
	
	#displaying the live image
        cv_image = cv2.flip(raw_image, -1)
        cv2.imshow("Sensor View", cv_image)
	#change the rate at which the live feed refreshes
        cv2.waitKey(3)
	
        



#main function block

def main():

    global home
    global Q
    global SPIN_RATE
    global cv_image
    

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

  
    # Defining Subscribers
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    sub_camera = rospy.Subscriber('cv_camera_node/image_raw', Image, camera_callback) 
    


   #Calling the image conversion for video sensing
    ic = ImageConverter(SPIN_RATE)
    
    time.sleep(5)

   #Calling to spawn blocks, posititions are changed in the yaml file
    lab2_spawn.spawn()
            

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is SHUTDOWN!")
    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)
   
#Defining moves
    move_block(pub_command,loop_rate,project_funcs.lab_invk(0.3,0,0.04,-90),project_funcs.lab_invk(0.2,0.2,0.04,-90),project_funcs.lab_invk(0.2,0.2,0.15,-90))
    move_block(pub_command,loop_rate,project_funcs.lab_invk(0.2,0,0.04,-90),project_funcs.lab_invk(0.2,-0.2,0.04,-90),project_funcs.lab_invk(0.2,-0.2,0.15,-90))   
        

   
#ending sequence

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
