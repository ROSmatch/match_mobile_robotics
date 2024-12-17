#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def set_target_pose():
    # Initialize the ROS node
    rospy.init_node('set_virtual_object_pose')

    # Create a publisher for the target pose
    pub = rospy.Publisher('/virtual_object/set_pose', PoseStamped, queue_size=10)

    rospy.sleep(1)

    # Create a PoseStamped message
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'  # Set the frame ID
    target_pose.pose.position.x = 0.0  # Set the position (x-coordinate)
    target_pose.pose.position.y = 0.0  # Set the position (y-coordinate)
    target_pose.pose.position.z = 1.5  # Set the position (z-coordinate)
    target_pose.pose.orientation.x = 0.0  # Set the orientation (x-coordinate)
    target_pose.pose.orientation.y = 0.0  # Set the orientation (y-coordinate)
    target_pose.pose.orientation.z = 0.0  # Set the orientation (z-coordinate)
    target_pose.pose.orientation.w = 1.0  # Set the orientation (w-coordinate)

    # Publish the target pose
    pub.publish(target_pose)

    # Spin the ROS node to send the message
    rospy.sleep(1)

if __name__ == '__main__':
    set_target_pose()