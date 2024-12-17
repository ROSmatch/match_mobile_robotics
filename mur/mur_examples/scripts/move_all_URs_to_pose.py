#!/usr/bin/env python3
import rospy
import moveit_commander

# Specify the robot names and arm prefixes
robot_names = rospy.get_param('~robot_names', ['mur620a', 'mur620b', 'mur620c', 'mur620d'])
UR_prefix = rospy.get_param('~UR_prefix', ['UR10_l', 'UR10_r'])
target_pose_name = rospy.get_param('~target_pose_name', 'handling_position_wide')


# Initialize the moveit_commander module
moveit_commander.roscpp_initialize([])

# Iterate over the list of robots and arms
for robot_name in robot_names:
    # Create a moveit_commander.RobotCommander instance for the current robot
    robot = moveit_commander.RobotCommander(robot_description=robot_name + '/robot_description', ns=robot_name)

    # Create moveit_commander.MoveGroupCommander instances for the left and right arms
    left_arm_group = moveit_commander.MoveGroupCommander("UR_arm_l" , robot_description=robot_name + '/robot_description', ns=robot_name)
    right_arm_group = moveit_commander.MoveGroupCommander("UR_arm_r" , robot_description=robot_name + '/robot_description', ns=robot_name)

    # Set the target pose for the arms
    target_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with your desired pose

    # Move the left arm to the target pose
    left_arm_group.set_named_target(target_pose_name)
    left_arm_group.go(wait=True)

    # Move the right arm to the target pose
    right_arm_group.set_named_target(target_pose_name)
    right_arm_group.go(wait=False)

# Shutdown the moveit_commander module
moveit_commander.roscpp_shutdown()



