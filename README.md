# Robot Intelligence Lab_Mobile Manipulator with Lidar (RIL_MM_Lidar)
 
## Overview
This mobile manipulator(UR5 with Robotiq Gripper85, Husky and Lidar) tutorial will show you how to operate a mobile manipulator using Gazebo, RViz, MoveIt



**Author**   
- **Sangbeom Park, [github]:https://github.com/Beom0611**  

### Before start this tutorial, please check your ROS version, this package for ROS Melodic version



## Guide

- For RIL_MM_Lidar
[UR5+Robotiq Gripper85+Husky Model]  
```
$ cd ~/catkin_ws/src && git clone https://github.com/ros-industrial/universal_robot.git
$ cd ~/catkin_ws/src && git clone https://github.com/StanleyInnovation/robotiq_85_gripper.git
$ cd ~/catkin_ws/src && git clone https://github.com/husky/husky.git
$ cd ~/catkin_ws/src && git clone https://github.com/Beom0611/ril_mm.git
$ cd ~/catkin_ws && catkin_make
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
```


## Sample code for beginner 
- Spawning RIL_MM_Lidar in Gazebo and Rviz 
```  
$ roslaunch ril_mm_lidar_gazebo ril_mm_lidar_empty_world.launch
```
- Moving RIL_MMR to the goal point  
``` 
$ cd ~/catkin_ws/src/ril_mm_lidar_python   
$ chmod +x husky_point.py
$ rosrun ril_mm_lidar_python husky_point.py 
```
- Contorlloing the mobile manipuatlor's arm and gripper   
```
$ cd ~/catkin_ws/src/ril_mm_lidar_python
$ chmod +x joint_test1.py
$ rosrun ril_mm_lidar_python joint_test1.py 
```




## Description    

<img width="500" height="300" src="https://user-images.githubusercontent.com/78074831/111556509-247b3880-87ce-11eb-8ef8-032df90669db.png"  alt="Screenshot" title="Screenshot">
