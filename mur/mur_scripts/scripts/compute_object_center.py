#!/usr/bin/env python3

# gets the pose of every robot and computes the average position of all robot TCPs to get the object center

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from tf import transformations
import math


class ComputeObjectCenter:

    def config(self):
        self.robot_names = rospy.get_param('~robot_names', ['mur620a','mur620b','mur620c','mur620d'])
        self.ur_l_prefix = rospy.get_param('~UR_l_prefix', 'UR10_l')
        self.ur_r_prefix = rospy.get_param('~UR_r_prefix', 'UR10_r')
        self.set_object_pose_topic = rospy.get_param('~set_object_pose_topic', '/virtual_object/set_pose')

    def __init__(self):
        self.config()
        self.robot_poses = [[None, None] for i in range(len(self.robot_names))]

        self.object_pose_pub = rospy.Publisher(self.set_object_pose_topic, PoseStamped, queue_size=1)
        rospy.sleep(1) # wait for publisher to be registered
        # subscribe to the pose topics of the robots
        for i in range(len(self.robot_names)):
            rospy.Subscriber('/' + self.robot_names[i] + '/' + self.ur_l_prefix + '/global_tcp_pose', PoseStamped, self.robot_pose_callback, [i, 0])
            rospy.Subscriber('/' + self.robot_names[i] + '/' + self.ur_r_prefix + '/global_tcp_pose', PoseStamped, self.robot_pose_callback, [i, 1])

        # wait for messages
        rospy.loginfo('Waiting for messages')
        for i in range(len(self.robot_names)):
            rospy.wait_for_message('/' + self.robot_names[i] + '/' + self.ur_l_prefix + '/global_tcp_pose', PoseStamped)
            rospy.wait_for_message('/' + self.robot_names[i] + '/' + self.ur_r_prefix + '/global_tcp_pose', PoseStamped)
        rospy.loginfo('Received all messages')

        self.compute_object_center()

        # shutdown node
        rospy.signal_shutdown('Object center published')

    def robot_pose_callback(self, msg, index):
        self.robot_poses[index[0]][index[1]] = msg.pose

    def compute_object_center(self):
        print(self.robot_poses)
        # compute center position
        object_center = np.zeros(3)
        for i in range(len(self.robot_names)):
            object_center += np.array([self.robot_poses[i][0].position.x, self.robot_poses[i][0].position.y, self.robot_poses[i][0].position.z])
            object_center += np.array([self.robot_poses[i][1].position.x, self.robot_poses[i][1].position.y, self.robot_poses[i][1].position.z])
        object_center /= (len(self.robot_names) * 2)

        #compute center orientation
        object_orientation = np.zeros(4)
        for i in range(len(self.robot_names)):
            object_orientation += np.array([self.robot_poses[i][0].orientation.x, self.robot_poses[i][0].orientation.y, self.robot_poses[i][0].orientation.z, self.robot_poses[i][0].orientation.w])
            object_orientation += np.array([self.robot_poses[i][1].orientation.x, self.robot_poses[i][1].orientation.y, self.robot_poses[i][1].orientation.z, self.robot_poses[i][1].orientation.w])
        object_orientation /= len(self.robot_names) * 2

        # normalize orientation
        object_orientation /= np.linalg.norm(object_orientation)

        rospy.loginfo('Object center: ' + str(object_center))
        rospy.loginfo('Object orientation: ' + str(object_orientation))
        # compute relative position to each robot TCP    
        for i in range(len(self.robot_names)):
            for j in range(2):
                relative_position = Pose()
                relative_position.position.x = self.robot_poses[i][j].position.x - object_center[0]
                relative_position.position.y = self.robot_poses[i][j].position.y - object_center[1]
                relative_position.position.z = self.robot_poses[i][j].position.z - object_center[2]
                rospy.loginfo('Relative position of ' + self.robot_names[i] + ' ' + str(j) + ': ' + str(relative_position))
                                
        


        # rotate the quaternion about its x-axis by 180 degrees
        euler = transformations.euler_from_quaternion(object_orientation)
        object_orientation = transformations.quaternion_from_euler(euler[0] + math.pi, euler[1], euler[2])
        # rotate the quaternion about its z-axis by - 90 degrees
        object_orientation = transformations.quaternion_multiply(object_orientation, transformations.quaternion_from_euler(0, 0, -math.pi / 2))


        # publish object center
        object_pose = PoseStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.header.frame_id = 'map'
        object_pose.pose.position.x = object_center[0]
        object_pose.pose.position.y = object_center[1]
        object_pose.pose.position.z = object_center[2]
        object_pose.pose.orientation.x = object_orientation[0]
        object_pose.pose.orientation.y = object_orientation[1]
        object_pose.pose.orientation.z = object_orientation[2]
        object_pose.pose.orientation.w = object_orientation[3]
        # object_pose.pose.orientation.x = 0
        # object_pose.pose.orientation.y = 0
        # object_pose.pose.orientation.z = 0
        # object_pose.pose.orientation.w = 1
        self.object_pose_pub.publish(object_pose)
        rospy.sleep(1)

        # close node
        rospy.loginfo('Object center published')
        rospy.signal_shutdown('Object center published')

    



if __name__ == '__main__':
    rospy.init_node('compute_object_center')
    compute_object_center = ComputeObjectCenter()
    rospy.spin()




