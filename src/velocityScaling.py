#! /usr/bin/env python3

import rospy

# from std_srvs.srv import SetBool,SetBoolResponse
# from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform, Vector3, TransformStamped
from std_msgs.msg import Float64
import tf
import moveit_commander
import moveit_msgs.msg
import sys 
import numpy as np

SKELETON_FILTERED = '/skeleton_filtered'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

HEAD_KEYPOINTS = [0,1,2,3,4,5,6,7,8,9] 
LEFT_ARM_KEYPOINTS = [11, 13, 15]
RIGHT_ARM_KEYPOINTS = [12, 14, 16]
RIGHT_HAND_KEYPOINTS = [16,18,20,22]
LEFT_HAND_KEYPOINTS = [15,17,19,21]

def callback():
    pass

def main():
    
    
    rospy.init_node('velocity_scaling', anonymous=True)
    
    pub_speed_ratio = rospy.Publisher(
        '/robot/limb/right/set_speed_ratio',
        Float64,
        latch=True,
        queue_size=10)
    
    rate = rospy.Rate(50) # frequency of tf reader

    listener = tf.TransformListener()   
    
    ########################### all this part only to recive all robot-sawyer link, can be replaced by ["right...","righ"]
    # with all robot frame that you want to check distance from human
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # robot_link_to_check = []
    # robot_link_to_check.append(move_group.get_end_effector_link())
    # robot_link_to_check = robot_link_to_check.extend(move_group.get_joints())
    
    robot_link_to_check = move_group.get_joints()
    robot_link_to_check.remove("right_arm_mount")
    print(robot_link_to_check)
    input("asp")
    robot_tf_link_to_check = ["reference/"+str(single_link) for single_link in robot_link_to_check]
    robot_tf_link_to_check = [single_link.replace("right_j","right_l") for single_link in robot_tf_link_to_check]
    robot_tf_link_to_check = ["reference/right_l6"]
    print(robot_tf_link_to_check)

    ################################
    
    
    human_keypoints = [16]              # list of human keypoint to check distance from robot 
    
    # Scaling parameters
    scaling_time = 0.2                  # time required to scale the speed
    scaling_pub_freq = 100              # scaling pub frequency 
    rate_scaling_factor = rospy.Rate(scaling_pub_freq)   # rate of sleep of pub. scaling
    dt = 1/scaling_pub_freq                             
     
    actual_speed_ratio = 1 # da leggere da qualche parte
    
    while not rospy.is_shutdown():
        rospy.loginfo("New reading...")

        distances = []
        time = rospy.Time.now()
        for human_keypoint in human_keypoints:
            for tf_name_robot_link in robot_tf_link_to_check:
                try:
                    # print(tf_name_robot_link)
                    # print("id_"+str(human_keypoint))
                    # print(type(trans))
                    #(trans_base_human,rot_base_human) = listener.lookupTransform( "id_"+str(human_keypoint), "base", rospy.Time())
                    (trans_base_human,rot_base_human) = listener.lookupTransform( "base" , "id_"+str(human_keypoint), rospy.Time())
                    
                    (trans_robot_base,rot_robot_base) =  listener.lookupTransform( "base", tf_name_robot_link, rospy.Time())
                    trans_base_human_mat = tf.transformations.translation_matrix(trans_base_human)
                    # rot_base_human_mat = tf.transformations.quaternion_matrix(rot_base_human)
                    trans_robot_base_mat = tf.transformations.translation_matrix(trans_robot_base)
                    # rot_robot_base_mat = tf.transformations.quaternion_matrix(rot_robot_base)
                    
                    print(trans_base_human_mat)
                    
                    print(trans_robot_base_mat)
                    
                    # M_base_human = np.dot(trans_base_human_mat,rot_base_human_mat)
                    # M_robot_base = np.dot(trans_robot_base_mat,rot_robot_base_mat)
                    
                    # M_robot_human = np.dot(M_robot_base,M_base_human)
                    # # print(M_robot_human)
                    
                    # trans_robot_human = tf.transformations.translation_from_matrix(M_robot_human)
                    # print(type(trans_robot_human))
                    # print(trans_robot_human)
                    #relative_distance = np.linalg.norm(trans_robot_human)
                    relative_distance = np.linalg.norm(trans_robot_base_mat[0:3,3] - trans_base_human_mat[0:3,3])
                    
                    relative_distance -= 0.1
                    
                    distances.append(relative_distance)
                    
                    print(relative_distance)
                    # if(relative_distance<0.5):
                    #     pub_speed_ratio.publish(0.5)
                    #trans = tfBuffer.lookup_transform("camera_color_optical_frame", "id_"+str(human_keypoint), rospy.Time())
                    # print(rospy.Time.now().to_sec()-trans_robot_base.header.stamp.to_sec)
                    if True:
                    # if rospy.Time.now().to_sec()-trans_robot_base.header.stamp.to_sec()<0.5:
                    #     rospy.loginfo("si")
                    #     rospy.loginfo("Human keyp " + str(human_keypoint)+ "robot link" + tf_name_robot_link)
                    #     rospy.loginfo(trans_robot_base)
                        pass
                        
                    else:
                        rospy.loginfo("Troppo vecchia")
                        break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print("Non  ce")
                    #break
                    #rate.sleep()     
        time_now = (rospy.Time.now() - time).to_sec()
        print(time_now)
        if not distances:
            rate.sleep()
            continue
        min_distance = min(distances)
        if min_distance>=1:
            set_point = 1
        elif min_distance>=0.7 and min_distance<1:
            set_point = 0.6
        elif min_distance>=0.3 and min_distance<0.7:
            set_point = 0.3
        elif min_distance<0.3:
            set_point = 0.0
        
        time = rospy.Time.now()
        
        t = 0
        while t < scaling_time:
            actual_speed_ratio = (set_point-actual_speed_ratio)/(scaling_time) * t + actual_speed_ratio
            t += dt
            pub_speed_ratio.publish(actual_speed_ratio)
            rate_scaling_factor.sleep()
        
        time_now = (rospy.Time.now() - time).to_sec()

        actual_speed_ratio = set_point
        pub_speed_ratio.publish(actual_speed_ratio)
        
        print(time_now)
        rate.sleep()
    #rospy.Subscriber("/skeleton_filtered", Marker, callback)
    
    
    
if __name__ == "__main__":
    main()