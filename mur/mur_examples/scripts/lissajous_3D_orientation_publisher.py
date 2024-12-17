#!/usr/bin/env python3

# this node creates a lissajous curve and uses it to derive the target object orientation

import rospy


from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
from copy import deepcopy
from tf import broadcaster


class LissajousResponsePublisher:

    def config(self):
        self.a = rospy.get_param("~a", 1.5)
        self.b = rospy.get_param("~b", 2.0)
        self.c = rospy.get_param("~c", 1.0)
        self.omega_x = rospy.get_param("~omega_x", 2.0)
        self.omega_y = rospy.get_param("~omega_y", 3.0)
        self.omega_z = rospy.get_param("~omega_z", 4.0)
        self.delta_x = rospy.get_param("~delta_x", 0.0)
        self.delta_y = rospy.get_param("~delta_y", 0.0)
        self.delta_z = rospy.get_param("~delta_z", 0.0)
        self.velocity = rospy.get_param("~velocity", 0.0006)
        self.end_point_tolerance = rospy.get_param("~end_point_tolerance", 0.01)
        self.number_of_loops = rospy.get_param("~number_of_loops", 40)
        self.lissajous_path_topic = rospy.get_param("~lissajous_path_topic", "/lissajous_path")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/virtual_object/object_cmd_vel")

        

    def __init__(self):
        self.config()
        self.path_out = Path()
        self.path_out.header.frame_id = "map"
        self.initial_pose = PoseStamped()
        self.run_counter = 0
        

        self.cmd_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.path_publisher = rospy.Publisher(self.lissajous_path_topic, Path, queue_size=10, latch=True)
        rospy.sleep(1) # wait for the publisher to be registered

    def run(self):
        seq = 0
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        for t in range(0, int(10 / self.velocity)):
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = self.a * math.sin(self.omega_x * t * self.velocity) + self.delta_x
            pose_stamped.pose.position.y = self.b * math.sin(self.omega_y * t * self.velocity) + self.delta_y
            pose_stamped.pose.position.z = self.c * math.sin(self.omega_z * t * self.velocity) + self.delta_z
            pose_stamped.pose.orientation.w = 1
            pose_stamped.pose.orientation.x = 0
            pose_stamped.pose.orientation.y = 0
            pose_stamped.pose.orientation.z = 0

            self.path_out.poses.append(deepcopy(pose_stamped)) 

               
        self.path_out.header.stamp = rospy.Time.now()
        self.path_publisher.publish(self.path_out)
        rospy.loginfo("Published lissajous path")
        rospy.sleep(1)

        # compute velocity commands
        twist = Twist()

        self.initial_pose.pose = self.path_out.poses[0].pose

        # compute the target velocity by differentiating the lissajous curve
        self.cmd = Twist()

        rate = rospy.Rate(100)
        # phi_old = 0.0
        for i in range(0,len(self.path_out.poses)-1):
            dx = self.path_out.poses[i+1].pose.position.x - self.path_out.poses[i].pose.position.x
            dy = self.path_out.poses[i+1].pose.position.y - self.path_out.poses[i].pose.position.y
            dz = self.path_out.poses[i+1].pose.position.z - self.path_out.poses[i].pose.position.z
            
            sleep_dur = rate.sleep_dur.to_sec()
            self.cmd.angular.x = dx / sleep_dur * 0.1
            self.cmd.angular.y = dy / sleep_dur * 0.1
            self.cmd.angular.z = dz / sleep_dur * 0.1

            # broadcast the target pose
            br = broadcaster.TransformBroadcaster()
            br.sendTransform((self.path_out.poses[i].pose.position.x, self.path_out.poses[i].pose.position.y, self.path_out.poses[i].pose.position.z), (self.path_out.poses[i].pose.orientation.x, self.path_out.poses[i].pose.orientation.y, self.path_out.poses[i].pose.orientation.z, self.path_out.poses[i].pose.orientation.w), rospy.Time.now(), "target_pose", "map")

            self.cmd_publisher.publish(self.cmd)
            rate.sleep()

            # compute distance to initial pose to determine when to stop
            distance = math.sqrt((self.initial_pose.pose.position.x - self.path_out.poses[i].pose.position.x)**2 + (self.initial_pose.pose.position.y - self.path_out.poses[i].pose.position.y)**2 + (self.initial_pose.pose.position.z - self.path_out.poses[i].pose.position.z)**2)

            if rospy.is_shutdown():
                break




if __name__ == '__main__':
    rospy.init_node('lissajous_response_publisher')
    lissajous_response_publisher = LissajousResponsePublisher().run()
    rospy.spin()