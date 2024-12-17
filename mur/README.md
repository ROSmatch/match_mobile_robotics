# How to use perform a cooperative handling sequence

## 📝 Table of Contents
- [Setup Simulation](#setup_sim)
- [Robot Control](#robot_control)


## 1. Setup Simulation <a name = "setup_sim"></a>

1. Launch the Simulation World
Start by launching the world:
``` 
roslaunch match_gazebo big_square.launch 
``` 
2.  Spawn Multiple MuRs with Different Namespaces
Next, spawn MuRs with different namespaces:
``` 
roslaunch mur_examples multi_mur620_handling.launch
``` 
Ensure that the move_group has successfully loaded. If necessary, you may need to respawn the robots while Gazebo is running (delete them in Gazebo and use Ctrl + C in the terminal where the launch file is running). When move_group is successful, all manipulators should move to their home positions. If everything starts correctly, the robots will appear as shown in the image below:

![Alt text](mur_documentation/murs_spawned.png?raw=true "Four MuRs spawned successfully")

3. Move Robots to Handling Position
Move the robots to their handling positions by running the following command:
``` 
rosrun mur_examples move_all_URs_to_pose.py
``` 
The robots are moved using move_group, which may take some time to complete.
![Alt text](mur_documentation/MuRs_in_handling_pose.png?raw=true "All MuRs in handling pose")

4. Turn On the Controllers
Activate the admittance controllers:
``` 
roslaunch mur_control multi_decentralized_admittance_controller_sim.launch
``` 
This will launch eight admittance controller nodes simultaneously. Each robot will receive its relative target pose through the controller. Check RViz to ensure that the current pose roughly matches the target pose. The setup should look like the image below:
![Alt text](mur_documentation/RVIZ_ready.png?raw=true "All admittance controllers are running. Robots are almost in target pose.")

5. Switch the Robots to Twist Control
```
rosrun mur_examples switch_URs_to_twist_control.py
```
Note: The official UR ROS package does not currently support a twist controller in Gazebo (only in hardware). Therefore, a custom twist controller based on inverse differential kinematics and the joint_group_vel controller is used instead.

With the controllers running, the robots should move into their target positions as shown below:
![Alt text](mur_documentation/Controller_ready.png?raw=true "Controllers running. Handling simulation is fully setup")
The setup is now complete, and the controllers are ready to accept commands.


## 1. Control the Robots <a name = "robot_control"></a>

Each robot is controlled by a decentralized admittance controller, and all controllers share the same reference frame: virtual_object/base_link. To control the robot formation, simply move the "virtual_object/base_link" by sending commands through the "/virtual_object/object_cmd_vel" twist interface.

```
rostopic pub /virtual_object/object_cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.05
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r10
```
You can also use one of the existing Lissajous trajectory publishers:
```
rosrun mur_examples lissajous_3D_combined_publisher.py
```