# Mir Hardware Helper
## 0. Package origin
This package came originally from (mir_robot)[https://github.com/dfki-ric/mir_robot].

## 1. Package overview
* `mir_description`: URDF description of the MiR robot
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_launch_sim`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files
* `mir_launch_hardware`  : Standalone launchfiles for some different tasks

## 2. Installation
### Install dependencies
```
git submodule init
git submodule update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build system to build the downloaded packages



## 3. Example - Launch the Cooperative Object Transport Example
To begin, launch the object transport example with the following command:
```
roslaunch mir_examples object_transport.launch
```

Next, move the "virtual_leader/base_link" to the correct position. The "virtual_leader/base_link" serves as a reference point for the other robots and should ideally be positioned in the middle of the formation, equidistant between all robots.

You can move the "virtual_leader/base_link" using "rqt_robot_steer" on the "/virtual_leader/cmd_vel" topic, or by publishing the desired pose on the "/virtual_leader/set_pose topic". In this example, we’ll position the "virtual_leader/base_link" 1 meter in front and to the side of robot "mur620a" using the following script:

```
roslaunch virtual_leader set_leader_pose.launch relative_pose:=[1.0,1.0,0.0] robot_pose_topic:=/mur620a/mir_pose_stamped_simple
```

### Start the Leader-Follower Formation Controllers

Once the leader's position is set, you can start the leader-follower formation controllers. Each robot's controller will receive the position of the "virtual_leader/base_link" and a unique "relative_pose" that it must maintain relative to the leader. When you launch the controllers, the robots will slightly adjust their positions to match their assigned target poses.

```
roslaunch formation_controller multi_robot_formation_control_sim.launch
```
If everything is set up correctly, the robots should appear as shown in the image below::
![Alt text](mir_documentation/RVIZ_formation_controller_running.png?raw=true "All leader-follower controllers are running. Formation is ready to move")

### Control the Formation

To control the robot formation, you only need to move the virtual leader. This can be done either by using the "rqt_robot_steering" tool or by running a predefined node.

#### Option 1: Using "rqt_robot_steering"
```
rosrun rqt_robot_steering rqt_robot_steering
```
Then, set the target topic to "/virtual_leader/cmd_vel" to control the virtual leader's movement.

#### Option 2: Using a Pre-Existing Node

If you prefer to use a pre-existing node to automate the control, run:
```
rosrun mir_examples lissajous_response_pub.py
```