#! /usr/bin/env python3
#!/usr/bin/env python  
import roslib
import rospy

from geometry_msgs.msg import TransformStamped
import tf2_ros

def main():
    rospy.init_node('camera_calibration_parameter')
    
    br = tf2_ros.TransformBroadcaster()
    


    while True:
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.77235
        t.transform.translation.y = -0.528889
        t.transform.translation.z = 0.982745
        t.transform.rotation.x = -0.22946 
        t.transform.rotation.y = 0.156673 
        t.transform.rotation.z = 0.638313 
        t.transform.rotation.w = 0.71788
        br.sendTransform(t)


    rospy.spin()

if __name__ == '__main__':
    main()
