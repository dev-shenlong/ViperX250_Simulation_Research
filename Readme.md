
# VIPER X250 Operations and Findings
The **viper x250** is a robotic arm from trossenrobotics, it is a 5 DOF arm with a span of 1300 mm, and a payload of 450 g.

It has support for multiple libraries to control and simulate, but I will be using **ROS2 Humble** and **Gazebo** in order to simulate the robot.

##  Installation of The required Libraries

In order to install the required libraries for simulation please run the following commands

    sudo apt install curl
    curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh 
    chmod +x xsarm_amd64_install.sh
    ./xsarm_amd64_install.sh -d humble

> Note: Make sure to include the setup script in the ~/.bashrc 
> If Gazebo file not launching make sure gazebo is setup in the ~/.bashrc script

## Launching the Gazebo File

In order to use the simulation files you must use the simulations present in the interbotix package. The simulation can be launched with the following command

    ros2 launch interbotix_xsarm_sim xsarm_gz_classic.launch.py robot_model:=vx250 use_position_controllers:=true dof:=5

Here there are a few arguments that have been used namely


| Argument | Function |
|---|---|
| robot_model | Used to specify which model of the robotic arm we will be working with, here it is the Viper X 250 |
|use_position_controllers| Set to true if you want to have access to the controllers in order to manipulate the arm|
|use_trajectory_controllers|instead of directly setting the position of the servos as in the previous command, we can use this to set the servos to move in a certain trajectory|
|dof| used to specify the number of degrees of freedom of the robot|

> **Note** in order to manipulate the robot the physics must be unpaused, it can be done so by the following command

    ros2 service call /unpause_physics std_srvs/srv/Empty "{}"

## To launch the Arm Controller GUI

first build the package by going to parent directory
```
colcon build
```
Then source the package using 
```
source install/setup.sh
```
Finally run the package using the following command
```
ros2 run interbotix_vx250_sim vx250_custom_controller 
```

## Topics Associated with VX250
the list of topics that are associated with the simulation are as follows

- /vx250/arm_controller/controller_state
- /vx250/arm_controller/joint_trajectory 
- /vx250/arm_controller/state
-  /vx250/arm_controller/transition_event
- /vx250/controller_manager/robot_description
- /vx250/dynamic_joint_states
- /vx250/gripper_controller/controller_state
-  /vx250/gripper_controller/joint_trajectory
-  /vx250/gripper_controller/state
-  /vx250/gripper_controller/transition_event
-  /vx250/joint_state_broadcaster/transition_event /vx250/joint_states
-  /vx250/robot_description

#### Some Important Topics and Uses

**/vx250/joint_states**
displays the current positions, velocities and efforts applied by the servos of the simulation, an example of this is as follows
```

       header:
      stamp:
        sec: 631
        nanosec: 128000000
      frame_id: ''
    name:
    - shoulder
    - elbow
    - wrist_angle
    - wrist_rotate
    - waist
    - gripper
    - left_finger
    - right_finger
    position:
    - 2.1949094293205462e-06
    - 4.0854038930504544e-05
    - -1.8792373852605238e-06
    - 2.437798106758038e-05
    - -1.1330012217314334e-07
    - -0.037943843266324606
    - 0.015000030716140362
    - -0.015000062423322007
    velocity:
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    effort:
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    - .nan
    ---
```
Data Type of the topic is

    sensor_msgs/msg/JointState
More information can be found [here](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html)
#### /vx250/arm_controller/joint_trajectory 
has 1 subscription, mostly for controlling and setting the trajectory of the robotic arm.
Has a data type of the form 

    trajectory_msgs/msg/JointTrajectory


More information can be found [here](http://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectory.html)

> The topics under /vx250/gripper_controller also function.

###  Arm Control
to control the arm, a node must be used to publish information to the 
```
/vx250/arm_controller/joint_trajectory
```
##### The trajectory has multiple fields that must be entered namely

 - Header 
	 - It consists of information like the frame id, the time stamp, etc.
- Name
	- It requires a set or sequence of strings with the various joint names
	- specifically waist, shoulder, elbow, wrist and wrist rotate
		- ['waist' , 'shoulder' , 'elbow' , 'wrist' , 'wrist_rotate' ].
- Points
	- Defines the various points the robotic arm must change to
	- It is an array of points of type : 
	  ```
        trajectory_msgs/JointTrajectoryPoint.msg
      ```
	 - The JointTrajectoryPoint also requires multiple values
		 - positions: defines the required end positions of the servos
		 - velocities : the velocities of the servos  (Could not find use)
		 - effort: aka force ig???
		 - time : 
	> The number of positions, velocities or effort must be the same as the number of joints named in the **Name** field.
	
	> The number of points is independent of the DOF but depends on the trajectory to be followed.

### Gripper Control
Have not tested yet and do not have much idea

