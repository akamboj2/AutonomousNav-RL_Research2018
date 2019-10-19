# vrep-learning-env Package
## Installation and setup
1. Make sure you have vrep or download V-rep education [http://www.coppeliarobotics.com/downloads.html](http://www.coppeliarobotics.com/downloads.html)
  Also make sure that the libv_repExtRosInterface.so extension is in the vrep folder (it should be when you download it)
2. Make sure ROS is working and up to date. This package was created and tested with ROS kinetic
3. Git clone or download this package and put it in the src folder of your catkin_ws
4. Go to the src/vrep_environment.cpp and change line 79 to be _**path to your catkin_ws**/catkin_ws/src/vrep-learning-env/src/Qtable6_

## Run simulation
1. Start _roscore_
2. Start vrep in separate terminal (_/path/to/your/vrep/installation/vrep.sh_)
3. Load Scene by clicking file->open scene, then navigating to the package and clicking _drone test minus sensor.ttm_
4. Hit play the screen should go black (because rendering is off) then hit stop.
    *NOTE:* when you run the environment for the first time or if you modify the Quadricopter customization script, you have to start and stop the simulation once by hand to get the start stop topic publishers running before it works with code
5. In a separate terminal run the vrep_Qlearning node
  1. Make sure you catkin build or catkin_make your environment
  2. Make sure you source _/**path to your catkin_ws**/catkin_ws/devel/setup.bash_
  3. run the node: `rosrun vrep-learning-env vrep_environment`
6. The table we ran is already trained so the quadcopter should  already make it to the end after the first few trials. If vrep shows a black screen simply click anywhere on the screen to re enable rendering (*note:* this slows down the simulation speed)

## Resetting and Customizing the simulation
* 3 helper functions deal with saving and reading from the Q table:
 ```
void store_table();
void read_table();
void init_zeroes();
```
  In line 17 you can uncomment out the call to the helper function init_zeroes() to reset the Q_table to zeroes. Furthermore, in  the helper functions you can change the file that is being read from or stored to.
* if you want to change the initial position of the robot you have to
  - change vrep_environment.hpp pose, and currentxyz in reset function
  - change Quadricopter object on vrep
  - change Quadricopter script pose in initialization function
* if you copy the Quadricopter and paste it into a different vrep scene, all the scripts would be copied and everything should work the same (including the start and stop simulation, because the customization script is  associated to the Quadricopter). Also, make sure all the potential objects the Quadricopter can collide with are set to collidable (click on the object->tools->scene Object Properties->Common and make sure collidable is checked)
  However, if you are replacing the Quadricopter with another robot, you will have to look through scripts and copy the info that is relevant to the nodes and functionality you want. *NOTE:* The Pioneer_learning environment is an alternative differential drive ground robot that should work

## vrep_ros_interface Publishers/vrep_Qlearning.cpp Subscribers

   Topic       | Description  | Type
  -------------|--------------|------
   `collision_status` | Message True is published if robot has collided to any object | [std_msgs/Bool][bool]
   `get_pose` | vrep publishes the current position of the robot to this topic and the code reads it | [geometry_msgs/Point][point]



## vrep_ros_interface Subscribers/vrep_Qlearning.cpp Publishers

  Topic       | Description   | Type
  ------------|---------------|------
  `set_pose` | the code publishes the desired position of the robot and the vrep reads it | [geometry_msgs/Point][point]
  `start_sim` | Code publishes True and vrep reads it to start the simulation (if the simulation is off) | [std_msgs/Bool][bool]
  `stop_sim` | Code publishes True and vrep reads the message and stops (if the simulation is off) | [std_msgs/Bool][bool]


[point]: http://docs.ros.org/api/geometry_msgs/html/msg/Point.html
[bool]: http://docs.ros.org/api/std_msgs/html/msg/Bool.html


## Authors
By: Abhi Kamboj
Please contact me with any issues and I can try my best to help: akamboj2@illinois.edu
