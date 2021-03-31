# Robot Intelligence Lab_Mobile Manipulator with Lidar (RIL_MM_Lidar)
 
## Overview
This mobile manipulator(UR5 with Robotiq Gripper85, Husky and Lidar) tutorial will show you how to operate a mobile manipulator using Gazebo, RViz, MoveIt



### Before start this tutorial, please check your ROS version, this package is for ROS Melodic Ver.



## Guide

- For RIL_MM_Lidar
[UR5+Robotiq Gripper85+Husky+LMS1XX Model]  
```
$ cd ~/catkin_ws/src && git clone https://github.com/ros-industrial/universal_robot.git
$ cd ~/catkin_ws/src && git clone https://github.com/StanleyInnovation/robotiq_85_gripper.git
$ cd ~/catkin_ws/src && git clone https://github.com/husky/husky.git
$ cd ~/catkin_ws/src && git clone https://github.com/Beom0611/ril_mm.git
$ cd ~/catkin_ws && catkin_make
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
```


- Spawning RIL_MM_Lidar in Gazebo and Rviz 
```  
$ roslaunch ril_mm_lidar_gazebo ril_mm_lidar_empty_world.launch
```


**More Info**   
- **Sangbeom Park, [github]:https://github.com/Beom0611** 


## Description    

<img width="500" height="300" src="https://user-images.githubusercontent.com/78074831/111556509-247b3880-87ce-11eb-8ef8-032df90669db.png"  alt="Screenshot" title="Screenshot">
