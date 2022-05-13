#! /usr/bin/env python3

import rospy

# from std_srvs.srv import SetBool,SetBoolResponse
# from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform, Vector3, TransformStamped
from Gripper import Gripper

import sys 
import intera_interface              # intera_interface - Sawyer Python API
import yaml
import os

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

OBJECT_PARAM = "objects"
PARAM_NOT_DEFINED_ERROR = RED + "Parameter : {} not defined" + END
DEFINED_SEQUENCE = ["approach_pick", "pick", "approach_leave", "leave","approach_leave"]
DEFINED_ROBOTIQ_COMMAND = {"approach_pick": "open","pick": "close", "leave": "open"}
COMMAND_TO_GRIPPER = {"open":"o", "close":"c", "reset":"r","activate":"a"}

def main():
    rospy.init_node('pick_plance_node', anonymous=True)
    rospy.loginfo(GREEN + "Let's start..." + END)

    # Reset Gripper
    gripper = Gripper()
    gripper.genCommand(COMMAND_TO_GRIPPER["reset"])
    gripper.genCommand(COMMAND_TO_GRIPPER["activate"])

    rospy.sleep(1)
    gripper.genCommand(COMMAND_TO_GRIPPER["close"])
    rospy.sleep(1)
    gripper.genCommand(COMMAND_TO_GRIPPER["open"])


    
if __name__ == "__main__":
    main()