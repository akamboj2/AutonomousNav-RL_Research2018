# Create Navigation Project
Allows the create robot to autonomously navigate mapped areas. This project is split into 2 portions. Mapping and navigation. **NOTE:** THE AStarPathPlanner.py can be used separately in any project to plan a path and will be explained below.

Other code and packages used for this project: 
Other code and packages used
 * [urg_node](http://wiki.ros.org/urg_node) (for the Hokuyo sensor)
 * [create_autonomy](http://wiki.ros.org/create_autonomy) (as a driver)
 * [slam_gmapping](http://wiki.ros.org/slam_gmapping) (to create map)
 * [map_server](http://wiki.ros.org/map_server) (to save the map created by gmapping)
 * [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) (to manually control the robot if needed)
## Setting up the robot
Make sure the create robot, and hokuyo sensor is plugged into the laptop properly. To test it, `catkin build` or `catkin_make` your workspace. 
* For the Create iRobot: type `roslaunch create_2.launch`, then in a new terminal publish messages to the cmd_vel topic, the robot should move.
* For the Hokuyo urg sensor, do `rostopic echo /scan` to see if scan messages are being published. 

If either of these don't work, refer to the respective websites given above to solve the problem.

## Mapping
After putting these packages in your catkin workspaces, make sure you build it and source (source devel/setup.bash) as necessary. You also must `rosmake gmapping` for the mapping to work. Many nodes have to be run in this portion of the project, so a simple launch file was created. 

This launch file was created to allow for easy alteration. For example, if you are using a different sensor (not the hokuyo laser scanner) just replace the lines in the launch file that runs the urg_node with whatever sensor you want to use (as long as it still publishes LaserScan messages). Same way, if you want to use a different robot, just change the create driver launch command in the launch file to whatever launch command that will run the robot you want to use (just make sure your robot's driver is compatible with gmapping ie it publishes odometry, etc) , etc. For more detail on swapping out parts of the launch file view comments of the launch file.

### Mapping the area 
1. Navigate to your project where the mapping launch file is and type `roslaunch mapping.launch`. This will run the urg_node, gmapping, a static transform publisher (from the laser scanner to the robot), the create driver launch file, and the rviz node for visiualization.
2. Navigate the robot around the area. This can be done with any controller, or manually from the keyboard after running in a new terminal: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` then pushing the keys accordingly
3. After the desired area has been navigated through kill the terminal where the mapping launch file was launched, then run the map_saver node: `rosrun map_server map_saver -f [map name]` **NOTE:** The map will be saved where you ran the map_saver node.
4. Kill all terminals when finished

### More on Visualize with Rviz
By default (in the launch file) rviz will run to visualize the map but in order to actually see the map you must add an OccupancyGrid object and have it subscribe to the /map topic. In addition, it is helpful to add an axis object to understand the map better

## Navigation 
Same ideas as in mapping--make sure to `catkin build` or `catkin_make`, make sure to source your files, a launch file was build for simplicity and modularity (ie swapping out for a different laser scanner or robot driver). 

### Navigating a mapped area
1. Find location of the map pgm file you wish to use and type that absolute location in _/**path to your workspace**/CreateNavigation/create_autonomy/ca_driver/scripts/gui.py_ line 139. Reading the next few lines and adjusting as necessary the global variables at the top of the file will allow you to define the size of the gui and how the map is displayed in it.
2. Run the map_server node with the map that you want: `map_server map_server yourMap.yaml`. This publishes the map as an OccupancyGrid to the /map topic for the AStarPath planner to read from.
3. In a new terminal navigate to the location of your launch file and launch it: `roslaunch navigation.launch` 
4. Click anywhere on the map and the robot should move the desired area. The amount of steps it takes to navigate to the next location can be changed in plot_path() function of gui. Actually, many things can be adjusted in the program, the comments should help.
5. Kill all terminals when finished

## Using the AStarPath Planner
This path planner simply reads a map from the /map topic in the form of an OccupancyGrid and given a starting and ending point it gives the optimal path. This planner is very flexible and the scale of the map as well as the resolution as shown in the yaml file for the map, can be adjusted. Since this planner is independent of the type of robot or type of laser scanner, it can be used with anything that works with gmapping, because it only relies on the map given from gmapping. It is packed into an easy to use class, here is an example:
```python
#after creating a map with gmapping just run rosrun map_server map_server yourMap.yaml then
planner=AStarPlanner(.005,3,500) #.005 is resolution in meters per pixel (given in yaml), 3 is the scale (so read every third point--it helps speed the planner up), 500 is the size in pixels (500*.005=25, so you mapped a 2.5m by 2.5m area)
plan = planner.PlanPath([0,0],[1,.75]) # give path from x,y = 0m,0m to x,y=1m,.75m
#plan will be list of adjacent points in steps of .005*3 meters
```

## Authors 
By: Abhi Kamboj
Please contact me with any issues and I can try my best to help: akamboj2@illinois.edu

