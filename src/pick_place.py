#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform, Vector3, TransformStamped
from Gripper import Gripper
import sys
import intera_interface              # intera_interface - Sawyer Python API
import yaml
import os

# import moveit_commander
# import moveit_msgs.msg
# from moveit_commander.conversions import pose_to_list

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

OBJECT_PARAM = "objects"
PARAM_NOT_DEFINED_ERROR = RED + "Parameter : {} not defined" + END
DEFINED_SEQUENCE = ["approach_pick", "pick", "leave_pick", "approach_leave", "leave","approach_leave"]
DEFINED_ROBOTIQ_COMMAND = {"approach_pick": "open","pick": "close", "leave": "open"}
COMMAND_TO_GRIPPER = {"open":"o", "close":"c", "reset":"r","activate":"a"}

def main():
    rospy.init_node('pick_plance_node', anonymous=True)
    rospy.loginfo(GREEN + "Let's start..." + END)

    try:
        object_distribution = rospy.get_param(OBJECT_PARAM)
    except KeyError:
        rospy.logerr(PARAM_NOT_DEFINED_ERROR.format(OBJECT_PARAM))
        return 0

    try:
        nominal_speed_ratio = rospy.get_param("nominal_speed_ratio")
    except KeyError:
        nominal_speed_ratio = 0.5
        rospy.logerr(RED + "nominal_speed_ratio not set, set to 0.5" + END)
        return 0

    limb = intera_interface.Limb('right')   # create an instance of intera_interface's Limb class

    # Reset Gripper
    gripper = Gripper()
    gripper.genCommand(COMMAND_TO_GRIPPER["reset"])
    gripper.genCommand(COMMAND_TO_GRIPPER["activate"]) # da decommentare se non funziona

    ############## Move it
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = "right_arm"
    # group = moveit_commander.MoveGroupCommander(group_name)
    ##############

    # Initial set-up robot
    rospy.loginfo("Moving to robot-neutral position with low speed...")
    limb.set_joint_position_speed(speed=0.1)
    limb.move_to_neutral()
    limb.set_joint_position_speed(speed=nominal_speed_ratio)
    rospy.loginfo("Robot speed set to maximum velocity...")

    t_start = rospy.Time.now()
    for object in object_distribution:
        for position_to_reach in DEFINED_SEQUENCE:
            if position_to_reach in object:
                set_point_position = object[position_to_reach]
                limb.move_to_joint_positions(set_point_position)


                # With moveit
                # pose_goal = limb.joint_angles_to_cartesian_pose(set_point_position,end_point='right_hand')
                # group.set_pose_target(pose_goal)
                # plan = group.go(wait=True)
                # # Calling `stop()` ensures that there is no residual movement
                # group.stop()
                # group.clear_pose_targets()
                # print(cartesian_pose)
                # print(type(cartesian_pose))
            if position_to_reach in DEFINED_ROBOTIQ_COMMAND:
                # Send command to the gripper
                gripper.genCommand(COMMAND_TO_GRIPPER[DEFINED_ROBOTIQ_COMMAND[position_to_reach]])
            rospy.sleep(2)      #P Pase
    t_end = (rospy.Time.now() - t_start).to_sec()
    rospy.loginfo(GREEN + "Execution time: {} seconds".format(t_end) +END)

if __name__ == "__main__":
    main()
