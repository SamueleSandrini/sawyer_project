#! /usr/bin/env python3

import rospy
# import robotiq_2f_gripper_control.Robotiq2FGripperSimpleController 

# from std_srvs.srv import SetBool,SetBoolResponse
# from std_msgs.msg import String
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

OBJECT_PARAM = "object_distribution"
PARAM_NOT_DEFINED_ERROR = RED + "Parameter : {} not defined" + END
DEFINED_SEQUENCE = ["approach_pick", "pick", "approach_leave", "leave","approach_leave"]
DEFINED_ROBOTIQ_COMMAND = {"approach_pick": "open","pick": "close", "leave": "open"}

def main():
    rospy.init_node('pick_plance_node', anonymous=True)
    rospy.loginfo(GREEN + "Let's start..." + END)
    
    try:
        object_distribution = rospy.get_param(OBJECT_PARAM)
    except KeyError:        
        rospy.logerr(PARAM_NOT_DEFINED_ERROR.format(OBJECT_PARAM))
        return 0    
    
    limb = intera_interface.Limb('right')   # create an instance of intera_interface's Limb class
    
    # Initial set-up robot
    rospy.loginfo("Moving to robot-neutral position with low speed...")
    limb.set_joint_position_speed(speed=0.3)
    limb.move_to_neutral()
    limb.set_joint_position_speed(speed=1)
    rospy.loginfo("Robot speed set to maximum velocity...")
    
    # Reset pinza
    for object in object_distribution:
        for position_to_reach in DEFINED_SEQUENCE:
            if position_to_reach in object:
                set_point_position = object[position_to_reach]
                limb.move_to_joint_positions(set_point_position)
            if position_to_reach in DEFINED_ROBOTIQ_COMMAND:
                #aziona la pinza con il comando DEFINED_ROBOTIQ_COMMAND[position_to_reach]
                pass
    

    
if __name__ == "__main__":
    main()