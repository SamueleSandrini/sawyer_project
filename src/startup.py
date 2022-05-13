#! /usr/bin/env python3

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform, Vector3, TransformStamped
import sys 
import intera_interface              # intera_interface - Sawyer Python API
import yaml
import os

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

USER_MESSAGE_POSITION = GREEN + "Move the robot to {} position and press enter..." + END
USER_QUESTION = YELLOW + "Do you want to give another position?" + END
DEFINED_POSITION = ["approach_pick", "pick", "approach_leave", "leave"]

POSITION_CONFIG_FILE = "objects_distribution.yaml"

def save_to_config_file(data_to_add):
    script_dir = os.path.dirname(os.path.realpath('__file__'))
    foldername = script_dir + "/src/sawyer_project/config/"
    print(foldername+POSITION_CONFIG_FILE)
    with open(foldername+POSITION_CONFIG_FILE,'w') as yamlfile:
        yaml.safe_dump(data_to_add, yamlfile) 

def main():
    rospy.init_node('startup', anonymous=True)
    rospy.loginfo("Initial program")
    
    limb = intera_interface.Limb('right')   # create an instance of intera_interface's Limb class

    another_position = True
    position_list = dict()
    position_list["objects"] = []
    
    while another_position:                                 # until user want another position
        actual_position_config = dict()     
        for position_question in DEFINED_POSITION:
            input(USER_MESSAGE_POSITION.format(position_question))
            
            #leggere topic 
            #verificare che sufficientemente diversa da prima altrimenti non salvarla
            
            actual_pose = limb.joint_angles()
            print(actual_pose)
            print(type(actual_pose))
            actual_position_config[position_question] = actual_pose         # fill dictionary at key = position_question
        position_list["objects"].append(actual_position_config)             # append to a list of all objects
        another_position_question = input(USER_QUESTION)                    # ask another position 
        if another_position_question == "n":
            another_position = False
    
    save_to_config_file(position_list)                                    # save to file
    rospy.loginfo(position_list)                                                   
    
    
if __name__ == "__main__":
    main()