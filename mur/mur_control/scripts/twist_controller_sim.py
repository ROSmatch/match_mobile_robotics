#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from math import sin, cos, pi
import moveit_commander
from geometry_msgs.msg import Twist


class TwistControllerSim():

    def config(self):
        self.UR_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.rate = rospy.get_param('~rate', 20)
        self.robot_name = rospy.get_param('~robot_name', 'mur620a')
        self.group_vel_controller_topic = rospy.get_param('~group_vel_controller_topic', '/mur620a/joint_group_vel_controller_l/unsafe/command')
        self.commanded_twist_topic = rospy.get_param('~commanded_twist_topic', '/mur620a/UR10_l/commanded_twist')
        self.move_group_name = rospy.get_param('~move_group_name', 'UR_arm_l')
        self.time_out = rospy.get_param('~time_out', 0.1)

    def __init__(self) -> None:
        self.config()
        self.commanded_twist  = Twist()
        self.last_command_time = rospy.Time.now()

        # Initialize the moveit_commander module
        moveit_commander.roscpp_initialize([])
        robot = moveit_commander.RobotCommander(robot_description="/"+ self.robot_name + '/robot_description')

        self.group = moveit_commander.MoveGroupCommander(self.move_group_name , robot_description="/" + self.robot_name + '/robot_description')

        # initialize the publisher for the joint velocities
        self.group_vel_controller_pub = rospy.Publisher(self.group_vel_controller_topic, Float64MultiArray, queue_size=1)

        # initialize the subscriber for the commanded twist
        rospy.Subscriber(self.commanded_twist_topic, Twist, self.commanded_twist_callback)

    # Compute the transformation matrix from the base frame to the end-effector frame
    def compute_jacobian(self):
        # get joint values from move_group
        joint_values = self.group.get_current_joint_values()       
        J = self.group.get_jacobian_matrix(joint_values)

        return J
    

    # Compute the inverse differential kinematics using the Jacobian matrix
    def compute_inverse_differential_kinematics(self,jacobian, commanded_twist):
        desired_twist = np.array([-0.0, -0.0, -0.0, 0, 0, 0])
        desired_twist[0] = commanded_twist.linear.x
        desired_twist[1] = commanded_twist.linear.y
        desired_twist[2] = commanded_twist.linear.z
        desired_twist[3] = commanded_twist.angular.x
        desired_twist[4] = commanded_twist.angular.y
        desired_twist[5] = commanded_twist.angular.z
        joint_velocities = np.linalg.pinv(jacobian) @ desired_twist

        return joint_velocities
        

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            if rospy.Time.now() - self.last_command_time > rospy.Duration(self.time_out):
                self.commanded_twist = Twist()

            jacobian = self.compute_jacobian()
            joint_velocities = self.compute_inverse_differential_kinematics(jacobian, self.commanded_twist)

            # publish the joint velocities
            joint_velocities_msg = Float64MultiArray()
            joint_velocities_msg.data = joint_velocities
            self.group_vel_controller_pub.publish(joint_velocities_msg)
            rate.sleep()

    def commanded_twist_callback(self, msg):
        self.commanded_twist = msg
        self.last_command_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('twist_controller_sim')
    twist_controller = TwistControllerSim()
    rospy.sleep(1)
    twist_controller.run()