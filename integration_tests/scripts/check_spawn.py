#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose


# Define the correct pose
correct_pose = Pose()
correct_pose.position.x = 2.0
correct_pose.position.y = 2.0
correct_pose.position.z = 0.0
correct_pose.orientation.x = 0.0
correct_pose.orientation.y = 0.0
correct_pose.orientation.z = 0.0
correct_pose.orientation.w = 1.0
threshold = 0.1

def pose_callback(msg):
    if (abs(msg.position.x - correct_pose.position.x) < threshold and abs(msg.position.y - correct_pose.position.y) < threshold):
        rospy.loginfo("Robot spawned in the correct pose.")
    else:
        rospy.logerr("Robot spawned in the incorrect pose.")
    # shutdown the node
    rospy.signal_shutdown("Robot pose checked.")

def check_spawn():
    rospy.init_node('check_spawn', anonymous=True)
    rospy.Subscriber('/mur620/mir_pose_simple', Pose, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    check_spawn()