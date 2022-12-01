#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import yaml
import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from project import *

global movespawn
global randarray
global movespawn1

def spawn(blob,it):
   
    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            # Load block position
            block_xy_pos = data['block_xy_pos']
            
        except:
            sys.exit()


    # Initialize ROS pack
    rospack = rospkg.RosPack()
    # Get path to block
    ur_path = rospack.get_path('ur_description')
    block_path = os.path.join(ur_path, 'urdf', 'block.urdf')
    block1_path = os.path.join(ur_path, 'urdf', 'red_cylinder.urdf')
    block2_path = os.path.join(ur_path, 'urdf', 'prism_yellow.urdf')
    block3_path = os.path.join(ur_path, 'urdf', 'block_green_big.urdf')
    block_paths = [block1_path, block2_path, block3_path]
    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    # Delete previous blocks not working rn
    #for i in range(2):
        #block_name = 'block' + str(i+1)
        #delete(block_name)

    # 0-indexed and spawning equality
    starting_location = 0
    missing_block = str('n')    
    missing_block = (missing_block == 'y')
    
    print(blob,'firstspawn')

	# Spawn three blocks
    if not missing_block:
        '''
        for height in range(3):
            block_name = 'block' + str(height + 1)
            pose = Pose(Point(block_xy_pos[starting_location][height][0], 
                            block_xy_pos[starting_location][height][1], block_xy_pos[starting_location][height][2]),Quaternion(0, 0, 0, 0) )
            spawn(block_name, open(block_paths[2-height], 'r').read(), 'block', pose, 'world')
            print('blockspawn',height)    

'''

        if it == 1:
            height = blob
            block_name = 'block' + str(height + 1)
            pose = Pose(Point(block_xy_pos[starting_location][height][0], 
                            block_xy_pos[starting_location][height][1], block_xy_pos[starting_location][height][2]),Quaternion(0, 0, 0, 0) )
            spawn(block_name, open(block_paths[2-height], 'r').read(), 'block', pose, 'world')
            print('blockspawn',height)     
        if it ==2:
            height = blob
            block_name = 'block' + str(height + 4)
            pose = Pose(Point(block_xy_pos[starting_location][height][0], 
                            block_xy_pos[starting_location][height][1], block_xy_pos[starting_location][height][2]),Quaternion(0, 0, 0, 0) )
            spawn(block_name, open(block_paths[2-height], 'r').read(), 'block', pose, 'world')
            print('blockspawn',height)

