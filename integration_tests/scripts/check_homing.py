#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import math



# Define the target pose
target_pose = PoseStamped()
target_pose.pose.position.x = 1.5498441496303177
target_pose.pose.position.y = 1.6819193002735842
target_pose.pose.position.z = 0.8967507864660563
target_pose.pose.orientation.x = 0.0
target_pose.pose.orientation.y = 0.0
target_pose.pose.orientation.z = 1.0
target_pose.pose.orientation.w = 0.0

# Define the threshold
position_threshold = 0.01  # Example threshold for position
orientation_threshold = 0.01  # Example threshold for orientation

def pose_callback(data):
    rospy.loginfo("Received Pose: %s", data)
    
    # Compare position
    position_diff = math.sqrt(
        (data.pose.position.x - target_pose.pose.position.x) ** 2 +
        (data.pose.position.y - target_pose.pose.position.y) ** 2 +
        (data.pose.position.z - target_pose.pose.position.z) ** 2
    )
    
    # Compare orientation (using Euclidean distance for simplicity)
    orientation_diff = math.sqrt(
        (data.pose.orientation.x - target_pose.pose.orientation.x) ** 2 +
        (data.pose.orientation.y - target_pose.pose.orientation.y) ** 2 +
        (data.pose.orientation.z - target_pose.pose.orientation.z) ** 2 +
        (data.pose.orientation.w - target_pose.pose.orientation.w) ** 2
    )
    
    if position_diff <= position_threshold and orientation_diff <= orientation_threshold:
        rospy.loginfo("Pose is within the threshold.")
    else:
        rospy.logerr("Pose is outside the threshold.")
        rospy.logerr("Position difference: %f", position_diff)
        rospy.logerr("Orientation difference: %f", orientation_diff)
    # shutdown the node
    rospy.signal_shutdown("Pose received")

def check_homing():
    rospy.init_node('check_homing', anonymous=True)
    rospy.Subscriber('/mur620/UR10_r/global_tcp_pose', PoseStamped, pose_callback)
    rospy.loginfo("Subscribed to /mur620/UR10_r/global_tcp_pose")
    rospy.spin()

if __name__ == '__main__':
    try:
        check_homing()
    except rospy.ROSInterruptException:
        pass