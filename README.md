# Final Project SSY261_Arena (2WheeledRobot_WS)

**NOTE**: to see the figures and good visualization of this and the other README files, in your Vscode, do:

```
ctrl + shift +v
```

## Table of contents
1. [Problem Description](#problem-description)
2. [Tasks](#tasks)
3. [Getting Started](#getting-started)
4. [Run the Demo](#run-the-demo)
5. [Bonus Tasks (Optional)](#bonus-tasks-optional)

---

## Problem description

The final project is based on the SSY261 Arena shown in the following figure (SSY261 Arena):

![ssy261_arena](/resources/ssy261_2024_arena/ssy261_arena_gazebo.png "SSY261 Arena")

This arena includes a two-wheeled robot with a differential drive that controls the left and right wheel velocities. The differential drive also provides dead rockoning based on odometry. In other words, the differential drive provides the current pose of the robot with respect to the odom frame (/odom). 

The robot has different sensors, namely, IMU, sonar, encoders, lidar, see Figure (Robot Sensors).

![ssy261_sensors](/resources/ssy261_2024_arena/ssy261_arena_gazebo_sensors.png "Robot Sensors")

In the arena, there are multiple targets (blue cylinders). Also, the arena has a top-mounted camera. This camera provides the position of each target relative to a camera coordinate frame (/camera), which is attached to the camera (green cube, see first figure). The camera system generates on-request a random path defined by an ordered sequence of targets. This list of targets are relative to the camera frame.

The task is to use the list of targets and generate a trajectory that moves the robot to each target in a specific time. The motion of the robot is point-to-point. This means, the robot stops in each target. The trajectory must be generated relative to the /odom frame. 

An example of the expected robot behavior can be seen in the video on Canvas inside 

Modules -> Study week 8 -> Final Project -> ssy261_arena_results.mp4

This arena is implemented using ROS2. The 2_wheeled_robot workspace provide various ros2 packages to solve this task.

``` 
├── 2_wheeledrobot_ws
│   └── src
│       ├── robot_2w_cam: Camera node that provides a Path relative to the camera frame. 
│       ├── robot_2w_ctrl: Control node that calculates a pose error between the current and target robot position. 
|       |                   This pose error is relative to the robot's frame. Then, using this pose error, it 
|       |                   calculates a commanded velocity for the differential drive.
│       ├── robot_2w_description: Provides the URDF/Xacro file with the robot description to load it gazebo and visualize it in rviz.
│       ├── robot_2w_interfaces: It provides the communication interfaces needed by the different nodes. 
|       |                         In particular, it provides the service definitions to request information.
│       └── robot_2w_path_generator: Trajectory generator node that provides continuos target positions which are used by the 
                                      controller to calculate the pose error.  
```

Each of this ros2 packages includes a readme.md file with detailed information. Please take a look at them and read them carefully!

The packages provide ros nodes that define a communication network to solve the task. This communication network is depicted in Figure (ROS2 Graph).

![ssy261_graph](/resources/ssy261_2024_arena/rosgraph_300.png "ROS2 Graph")


The control pipe-line is as follows:

1. The two-wheeled robot is simulated in Gazebo using the URDF/Xacro files provided by the robot_2w_description package (blue box). This simulated robot includes nodes that publish the different sensor information, e.g., /laserscan, /imu_plugin, etc. It also includes a node with the differential drive (/diff_drive). This node publishes the robot odometry (/odom) and subscribes to the /cmd_vel topic. This topic defines the commanded velocity of the robot. This commanded velocity is defined by the linear velocity in x-, and angular velocity in z- axes, relative to the robots frame (/base_footprint). The node /diff_drive computes the wheel velocities (left and right) to move the robot along the path.   
1. The path client node (/path_client) requests a path (/get_poses_from_camera) from the camera node (/robot_2w_cam). The camera node provides the sequence of target poses (path) relative to the /camera frame. To define this path, the camera node subscribes to the /odom topic and gets the current robot position. The camera node also publishes the path (wrt /camera) to visualize it in rviz (green path, see Video ssy261_arena_results.mp4). 
2. The path client uses the path provided by the camera node to generate a request to the path generator node (/path_generator). To generate the request, the path client must 
  - Map the path relative to the /camera frame to the /odom frame. To do this, the path client should request the Homogeneous Transformation of the camera wrt odom frame (H_cam_odom). The camera node has a service to provide that transformation (/get_tf_camera_odom).
  - Once the path is relative to the /odom frame, the path client should include the reaching time for each target position in the path. 
  - Using these two lists (target poses and times), the path client request a trajectory to the node /path_generator (/set_desired_poses).
3. The path generator node uses the list of target poses and times to calculate Spline functions that generate the target positions for the robot controller. These target positions are the desired positions in the x, and y-axes for the robot. The target positions are published in a topic (/path_generator/trajectory). To verify the trajectory is correct, the path generator node also publishes (/path_generator/robot_2w_path) the target path (wrt /odom) to visualize it in rviz (red path, see video ssy261_arena_results.mp4). 
4. The control node (/robot_2w_control) subscribes to the topics (/path_generator/trajectory) and (/odom). Both topics are relative to the /odom frame. Using that information in calculates a pose error relative to the robot's frame. This pose error (wrt robot's frame) is transformed to commanded velocities (Twist wrt robot's frame) using a simple P-controller, i.e., v=K*e. This twist information is published in the topic (/cmd_vel), which is listened by the /diff_drive node to move the robot. To simplify the gain tunning, the control node also publishes the /pose_error and /pose topics. 

The Figure (ROS2 Graph) illustrates all these nodes, the services, and the topics used to communicate. The information relative to the camera frame is depicted in red color. The information relative to the odom frame is shown in orange color. The information relative to the robot's frame is highlighted in brown color. In purple color, you can identify the configuration files (*.yaml files) used to provide the parameters for each node. Finally, the nodes that you need to write/modify/complete are shown in green color.

[back](#table-of-contents)

---

## Tasks

Each ros2 package contains a readme.md file with detailed information on the package and the tasks you need to complete. Please read careful each readme file and follow the tasks marked as (TODO:) in each of the source files. A brief summary of the tasks is presented in the following list. The awarded points per each task are indicated in brackets **[]**.

1. robot_2w_cam: 
  - You don't need to modify anything in this ros2 package. 

2. robot_2w_ctrl **[18]**: 
  - **robot_2w_ctrl/configs/robot_2w_ctrl.yaml**: 
    - tune the control gains: ctrl_gains
    - For debugging the variable *publish_pose_error* is 'False'. You can change it to 'True' to publish and print the tracking errors. 
  - **robot_2w_ctrl/src/Robot2wCtrl.cpp**: 
    - Complete the **TODO:** inside this file.
3. robot_2w_description **[5]**:
  - Complete **Assignment 04**!!!! and copy the needed files into the package robot_2w_description. The visualization of the robot must be correct, see video and Figures.
4. robot_2w_interface **[2]**: 
  - Get familiar with the three services and understand the request and response messages. Provide the output of the command *ros2 interface show* for each of the services. Give a brief explanation of the services in an additional file (you can include screenshots to explain).
5. robot_2w_path_generator **[5]**:
  - **robot_2w_path_generator/robot_2w_path_generator/trajectory_generator_module.py**:
Currently, the PathGenerator class receives the list of target positions and 
adds the initial robot position at the beginning of the list. In this way, the 
trajectory generator will compute a trajectory that starts at the initial 
robot position and ends at the last target position requested by a client. 
Your task is to modify the code to append the robot initial position also at the
end of the list of target positions, and append a new reaching time in the list 
reaching times. Therefore, the trajectory will start at the initial robot position
and end at the same initial robot position, i.e., it will generate a cyclic path
(round trip), see and work on the **TODO:** inside this file.
6. Live Demo **[5]**: Run the path_client three times and show that your robot can follow the different paths.  

[back](#table-of-contents)

---

## Bonus Tasks Optional

This task accounts for **[15]** additional points!!!!!

As can be seen in the video: 

Modules -> Study week 8 -> Final Project -> ssy261_arena_results.mp4

The robot passes through the obstacles (big blue cylinders). Of course, this is not a correct behavior. Ideally, the robot should avoid the obstacles while following the path as best as possible. 

Then, the bonus tasks is to use the information of the sensors, e.g., Lidar, sonar, IMU, or a combination of sensors to change the commanded velocity (/cmd_vel) to avoid the obstacles and follow the path. This is a free-task, there is not template and you can decide to implement your solution as you want! 

**Hint:** You need to implement a new node that listens to the /cmd_vel generated by the control node, and the topics generated by the sensor nodes. Then, based on the information of cmd_vel and the sensors, compute a new /cmd_vel message that takes into account the collision avoidance. Don't forget to map the output of the control node, i.e., /cmd_vel to a different topic, e.g., /cmd_vel_ctrl to avoid two nodes (control node and your new node) publishing to the same topic (/cmd_vel)!!!! 



## Getting started

Unzip the template file "2_wheeledrobot_arena_ws.zip" and place it inside your workspace folder.

Note that the unziped folder "2_wheeledrobot_arena_ws" will contain a new workspace, therefore, make sure you paste the new folder inside your workspace folder.

This is how you verify you have the new workspace in the correct place:


```
karinne@Karinne-Lat7440:~$ cd workspaces/
karinne@Karinne-Lat7440:~/workspaces$
karinne@Karinne-Lat7440:~/workspaces$ cd 2_wheeledrobot_arena_ws
karinne@Karinne-Lat7440:~/workspaces/2_wheeledrobot_arena_ws$ ls
README.md   resources   src
```

*NOTE* this time we have one README.md file, on folder resources which contains figures and the forlder src which contains the packages we will modify for the final project.

### Install the following ros & Ubuntu packages:

```
sudo apt update

sudo apt install ros-dev-tools ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-ros-gz ros-humble-gazebo-ros-pkgs ros-humble-tf2-tools ros-humble-rqt-tf-tree libignition-launch5 ros-humble-ros2-control ros-humble-control-toolbox ros-humble-topic-based-ros2-control ros-humble-moveit-ros-control-interface ros-humble-moveit-setup-controllers ros-humble-kinematics-interface ros-humble-ign-ros2-control ros-humble-ros2controlcli ros-humble-control-msgs ros-humble-gazebo-ros2-control ros-humble-ros2-control-test-assets ros-humble-gazebo-ros2-control-demos ros-humble-ign-ros2-control-demos ros-humble-controller-interface ros-humble-effort-controllers ros-humble-position-controllers ros-humble-velocity-controllers ros-humble-controller-manager-msgs ros-humble-forward-command-controller ros-humble-controller-manager ros-humble-ros2-controllers ros-humble-moveit-simple-controller-manager ros-humble-diff-drive-controller ros-humble-tricycle-controller ros-humble-joint-trajectory-controller ros-humble-admittance-controller ros-humble-ackermann-steering-controller ros-humble-joint-limits ros-humble-range-sensor-broadcaster ros-humble-imu-sensor-broadcaster ros-humble-force-torque-sensor-broadcaster ros-humble-steering-controllers-library ros-humble-cartographer-ros ros-humble-nav2-map-server lcov libbenchmark-dev libbenchmark1 libcommon-sense-perl libgd-perl libjson-perl libjson-xs-perl libomp-14-dev libomp-dev libomp5-14 libperlio-gzip-perl libtypes-serialiser-perl libxtensor-dev nlohmann-json3-dev ros-humble-behaviortree-cpp-v3 ros-humble-costmap-queue ros-humble-diagnostic-updater ros-humble-dwb-core ros-humble-dwb-critics ros-humble-dwb-msgs ros-humble-dwb-plugins ros-humble-nav-2d-msgs ros-humble-nav-2d-utils ros-humble-nav2-amcl ros-humble-nav2-amcl-dbgsym ros-humble-nav2-behavior-tree ros-humble-nav2-behavior-tree-dbgsym ros-humble-nav2-behaviors ros-humble-nav2-behaviors-dbgsym ros-humble-nav2-bringup ros-humble-nav2-bt-navigator ros-humble-nav2-bt-navigator-dbgsym ros-humble-nav2-collision-monitor ros-humble-nav2-collision-monitor-dbgsym ros-humble-nav2-constrained-smoother ros-humble-nav2-constrained-smoother-dbgsym ros-humble-nav2-controller ros-humble-nav2-controller-dbgsym ros-humble-nav2-core ros-humble-nav2-costmap-2d ros-humble-nav2-costmap-2d-dbgsym ros-humble-nav2-dwb-controller ros-humble-nav2-lifecycle-manager ros-humble-nav2-lifecycle-manager-dbgsym ros-humble-nav2-map-server-dbgsym ros-humble-nav2-mppi-controller ros-humble-nav2-mppi-controller-dbgsym ros-humble-nav2-msgs-dbgsym ros-humble-nav2-navfn-planner ros-humble-nav2-navfn-planner-dbgsym ros-humble-nav2-planner ros-humble-nav2-planner-dbgsym ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-nav2-regulated-pure-pursuit-controller-dbgsym ros-humble-nav2-rotation-shim-controller ros-humble-nav2-rotation-shim-controller-dbgsym ros-humble-nav2-rviz-plugins ros-humble-nav2-rviz-plugins-dbgsym ros-humble-nav2-simple-commander ros-humble-nav2-smac-planner ros-humble-nav2-smac-planner-dbgsym ros-humble-nav2-smoother ros-humble-nav2-smoother-dbgsym ros-humble-nav2-system-tests ros-humble-nav2-system-tests-dbgsym ros-humble-nav2-theta-star-planner ros-humble-nav2-theta-star-planner-dbgsym ros-humble-nav2-util-dbgsym ros-humble-nav2-velocity-smoother ros-humble-nav2-velocity-smoother-dbgsym ros-humble-nav2-voxel-grid ros-humble-nav2-voxel-grid-dbgsym ros-humble-nav2-waypoint-follower ros-humble-nav2-waypoint-follower-dbgsym ros-humble-navigation2 ros-humble-ompl ros-humble-slam-toolbox xtl-dev liburdfdom-tools ros-humble-ros2-controllers-test-nodes python3-rospkg ros-humble-mocap4r2-control ros-humble-mocap4r2-control-msgs ros-humble-ros-image-to-qimage ros-humble-rqt-controller-manager ros-humble-rqt-gauges ros-humble-rqt-gui-cpp-dbgsym ros-humble-rqt-image-overlay ros-humble-rqt-image-overlay-dbgsym ros-humble-rqt-image-overlay-layer ros-humble-rqt-image-view-dbgsym ros-humble-rqt-joint-trajectory-controller ros-humble-rqt-mocap4r2-control ros-humble-rqt-mocap4r2-control-dbgsym ros-humble-rqt-moveit ros-humble-rqt-robot-dashboard ros-humble-rqt-robot-monitor ros-humble-rqt-robot-steering ros-humble-rqt-runtime-monitor ros-humble-plotjuggler-ros -y

```

### Compile the workspace

Once, you have created your new workspace, source ROS2 and compile the new workspace

```
karinne@Karinne-Lat7440:~/workspaces/2_wheeledrobot_arena_ws$ source /opt/ros/humble/setup.bash
karinne@Karinne-Lat7440:~/workspaces/2_wheeledrobot_arena_ws$ colcon build  --symlink-install --merge-install 
```

[back](#table-of-contents)

---

## Run the demo

Now, you can run the demo. 

**NOTE**: The robot won't move until you complete all the tasks. You can test each node individually by following the launch instructions in the readme.md files found inside each ros2 packages.  

**Terminal 1**

Run Rviz, Gazebo, the Trajectory Generator, and the control nodes:

```
cd 2_wheeledrobot_arena_ws
source install/setup.bash
ros2 launch robot_2w_ctrl robot_2w_ctrl_gazebo_launch.py
```

**NOTE**: Similarly to the previous tutorials, please ignore the following errors. 

```
[gazebo-1] [Err] [InsertModelWidget.cc:403] Missing model.config for model "/home/dean/exchange/SSY156_2024/2_wheeledrobot_ws/install/share/ament_index"
[gazebo-1] [Err] [InsertModelWidget.cc:403] Missing model.config for model "/home/dean/exchange/SSY156_2024/2_wheeledrobot_ws/install/share/colcon-core"
[gazebo-1] [Err] [InsertModelWidget.cc:403] Missing model.config for model "/home/dean/exchange/SSY156_2024/2_wheeledrobot_ws/install/share/robot_2w_cam"
[gazebo-1] [Err] [InsertModelWidget.cc:403] Missing model.config for model "/home/dean/exchange/SSY156_2024/2_wheeledrobot_ws/install/share/robot_2w_ctrl"
[gazebo-1] [Err] [InsertModelWidget.cc:403] Missing model.config for model "/home/dean/exchange/SSY156_2024/2_wheeledrobot_ws/install/share/robot_2w_interfaces"
[gazebo-1] [Err] [InsertModelWidget.cc:403] Missing model.config for model "/home/dean/exchange/SSY156_2024/2_wheeledrobot_ws/install/share/robot_2w_path_generator"
```

**Terminal 2**
```
cd 2_wheeledrobot_arena_ws
source install/setup.bash
ros2 run robot_2w_path_generator path_client_from_cam
```

You can run this node several times. Each time, the camera node will send a randomly selected path (out of 10 options). The robot will always start from its current pose, and should return to that current pose after finishing the path. 

**Terminal 3**

Run rqt_graph and create two plots: Plugins->Visualization->Plot. 

```
cd 2_wheeledrobot_arena_ws
source install/setup.bash
ros2 run rqt_graph rqt_graph
``` 

To see these plots, you need to set the variable  *publish_pose_error* to **True**, in the config file "robot_2w_ctrl/configs/robot_2w_ctrl.yaml".

In the first plot, select the topics:

- /robot_2w_control/pose_error/x
- /robot_2w_control/pose_error/y
- /robot_2w_control/pose_error/z

In the second plot, select the topics: 

- /robot_2w_control/pose/x
- /robot_2w_control/pose/y
- /path_generator/trajectory/x
- /path_generator/trajectory/y


**Terminal 4**

Use the ros commands to inspect the demo and understand which nodes are created, and how they are connected. For example, 

```
cd 2_wheeledrobot_ws
source install/setup.bash
ros2 node list
ros2 node info
ros2 param list
ros2 param get
```


[back](#table-of-contents)

---
