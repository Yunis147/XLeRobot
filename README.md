# Lekiwi Mobile Manipulator

## ROS 2 System Architecture & Control

The ROS2 ecosystem for the Lekiwi rover is built by utilizing the open-source STL's and  URDF files. To bridge the physical hardware with the software stack, the system integrates ST3215 motor packages managed by a custom ros2_control interface. This maps individual motors to specific joints and establishes dedicated position and velocity control modes. The rover's movement is orchestrated through three distinct controllers explicitly designed to manage the drive wheels, the robotic arm, and the end-effector gripper.Arm motion planning fully supported through a dedicated MoveIt package.


### Setup 

Create the workspace :-
```
cd ~
mkdir Rover
mkdir Rover/src
cd ~Rover/src
```
Clone the repository :-
```
git clone https://github.com/KMTI-ROBOPARADIGM/mobile-manipulator/tree/main
```
Build the Package :-
```
cd ~Rover
colcon build
source install/setup.bash
```
### Launch

Launching the Robot :- 
```
ros2 launch lekiwi_bringup launch.py 
```
Simulation:- 
```
ros2 launch lekiwi_bringup sim.launch.py
```

Moveit Launch :-
```
ros2 launch lekiwi_moveit_config demo.launch.py
```


## Nav2 

The autonomous navigation pipeline is powered by the ROS2 Nav2 framework and SLAM (Simultaneous Localization and Mapping). This is implemented by two distinct sensor , an Intel RealSense depth camera to convert the depth to a 2d LaserScan and a dedicated LiDAR sensor. SLAM is first utilized to scan and construct a static map of the operating workspace. Once the map is established, the Nav2 stack takes over for autonomous navigation. It uses AMCL to accurately localize the robot by fusing the active sensor's scan data with continuous wheel odometry provided by the ST3215 servos. You can checkout the workflow image below .

<img width="3093" height="1036" alt="Nav2" src="https://github.com/user-attachments/assets/a88e2343-2d80-44e0-a758-26cd2a50b693" />

### Launch

SLAM Toolbox (for map building) :-

Using depth Camera:-
```
ros2 launch nav2 depth_laser.py
```
Using LiDAR:-
```
ros2 launch nav2 slam.py
```

Nav2 (for navigation) :-

Using Depth Camera:-
```
ros2 launch nav2.py
```
Using LiDAR:-
```
ros2 launch lidar_nav2.py
```
(launch the bringup file or the odometry file to publish the odometry and construct proper tf links)

## Folder Structure Overview 
```
mobile-manipulator/
├── lekiwi_bringup/
│   ├── config/
│   │   └── controllers.yaml
│   ├── launch/
│   │   ├── launch.py
│   │   ├── sim.launch.py
│   │   ├── (Controllers test files)
│   ├── lekiwi_bringup/
│   │   └── motor_odom.py 
├── lekiwi_description/
│   ├── ros2_control/
│   │   ├── lekiwi_ros2_control.xacro (For Simulation)
│   │   └── lekiwi_ros2_control_hardware.xacro
│   ├── URDF/
│   │   ├── meshes/
│   │   ├── LeKiwi.urdf
│   │   ├── lekiwi.urdf.xacro
│   │   └── lekiwi_hardware.urdf.xacro
├── lekiwi_moveit_config/
│   └── lekiwi_moveit_config/
│       ├── config/
│       ├── launch/
├── nav2/
│   ├── nav2/
│   │   ├── launch/
│   │   │   ├── depth_laser.py (Using RealSence Depth Camera)
│   │   │   ├── lidar_nav2.py
│   │   │   ├── nav2.py (Nav2 Using RealSence)
│   │   │   └── slam.py (For lidar)
│   │   ├── map/ 
│   │   ├── params/
│   │   ├── temp/
│   │   └── odom.py 
├── rplidar_ros/ (ros2 package for lidar)
├── st3215_hardware_interface/
│   ├── include/
│   ├── src/
│   │   └── st3215_system.cpp 
└── README.md
```
