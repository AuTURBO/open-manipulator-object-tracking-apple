## Description
This git is Ros application Git.  
*Hardware Structure  
 - Open Manipulator   
 - OpneCR   
 - RealSense D435   

*Base Software information   
 - reference open manipulator git to control open manipulator.   
   https://github.com/ROBOTIS-GIT/open_manipulator.  
 - reference turtlebot3_slam_3d git to detect object and to get 3D coordination.   
   https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d  
## Environment setting   

*First, you must set open manipulator environment.   
 - http://emanual.robotis.com/docs/en/platform/openmanipulator/  
*Sencond, you must set turtlebot3_slam_3d environment.   
 - https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d  
*Third, Please download this git and modify the below part.  
 
```bash
$mkdir ~/temp_op
$cd ~/temp_op
$git clone https://github.com/AuTURBO/open-manipulator-object-tracking-apple.git
$cd open-manipulator-object-tracking-apple
$cp -r darknet_ros ~/catkin_ws/src/darknet_ros/
$cp -r open_manipulator_example ~/catkin_ws/src/open_manipulator/
$cp turtlebot3_slam_3d.launch ~/catkin_ws/src/turtlebot3_slam_3d/launch/
```

## Run 

Click image to link to YouTube video.  
[![Video Label](http://img.youtube.com/vi/kpjaOiT5Xkw/0.jpg)](https://youtu.be/kpjaOiT5Xkw?t=0s)   

* run  
```bash
$roslaunch turtlebot3_slam_3d turtlebot3_slam_3d.launch use_zed:=false
$roslaunch open_manipulator_controller open_manipulator_controller.launch
$roslaunch open_manipulator_example jjap_it.launch target_object:=apple
```
