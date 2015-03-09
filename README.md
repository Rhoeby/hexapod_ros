Rhoeby hexapod ROS Node (hexapod_ros)
=====================================

Rhoeby is a ROS-enabled hexapod robot based on the (customized) Robotis 
Bioloid kit. It supports SLAM, Navigation and Obstacle Avoidance using 
low-cost, light-weight 2D Scanning technology available from Rhoeby 
Dynamics (see website: http://wwww.rhoeby.com).

Hardware Used
-------------

  - Robotis Bioloid "Spider" chassis with custom legs
  - Robotis Cm9.04 MCU board
  - Nexus 4 phone provides IMU (and tele-prescence camera!)
  - Rhoeby Dynamics R2D Infra-Red LiDAR Scanner
  - Bluetooth link to robot for command and status back
  - Remote laptop running ROS Indigo

Software Features
-----------------

  - 3-DOF Inverse Kinematic leg control
  - Holonomic-capable gait
  - Odometric feedback
  - ROS node for robot control

Installation
------------

ROS driver code on github:

  https://github.com/Rhoeby/hexapod_ros

CM9.04 binary on github:

  https://github.com/Rhoeby/hexapod_ros

ROS Package installation:

  1. install robot_pose_ekf
  
  2. copy hexapod_ros into catkin workspace
  
  3. do 'catkin_make'

Other:

Download SensoDuino app to phone (Nexus 4, or other Android device), see:

  https://play.google.com/store/apps/details?id=com.techbitar.android.sensoduino

Running
-------

1. Start roscore:
  
  roscore

2. Connect to phone (after starting SensoDuino):

  sdptool add --channel=22 SP
  rfcomm listen /dev/rfcomm1 22

3. Run the hexapod_ros node:

  rosrun hexapod_ros hexapod_ros

4. Start gmapping (for example):

  roslaunch src/hexapod_ros/launch/gmapping.launch model:=src/hexapod_ros/urdf/hexapod_01.urdf 
