/*how to run with gmapping
source devel/setup.bash
roslaunch ca_driver create_2.launch
rosrun urg_node urg_node scan:=base_scan
rosmake gmapping
rosrun gmapping slam_gmapping scan:=base_scan _base_link:=base_footprint
#------------then either run your this code and stop it----------
rosrun ca_driver basic_obs_avoid
ctrl-c
#------------or run a static tranform publisher in terminal and move the robot in some way (publishing using keys etc)----------
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint laser 100
#run in any way you want

#now you should be able to visualize in rviz and create map
rosrun map_server map_saver -f<map_name>
*/

/*how to run amcl
source devel/setup.bash
roslaunch ca_driver create_2.launch
rosrun urg_node urg_node scan:=base_scan
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint laser 100
rosrun map_server map_server <map.yaml file>
rosrun amcl amcl scan:=base_scan _base_frame_id:=base_footrpint
#then you can move the robot and echo the amcl_pose topic to see it's estimated pose
#for some reason you have to kill and rerun the amcl node over and over again before
#each time you see the robot??
*/
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
//#include <tf2_msgs/TFMessage>
#include <tf/transform_broadcaster.h>

ros::Publisher vel;
geometry_msgs::Twist turnLeft;
geometry_msgs::Twist turnRight;
geometry_msgs::Twist goStraight;
//int count=0;
//tf::TransformBroadcaster broadcaster;
/*for some reason when I put transformbroadcaster up here it gives errors like:
You must call ros::init() before creating the first NodeHandle
and more. but it works if you put it in main after init.*/

void detectCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  /*code for testing
  std::cout<<"\nCallbacked: "<<count++<<"\n";
  int numPoints=(msg->angle_max-msg->angle_min)/msg->angle_increment;
  std::cout<<numPoints;
  for (int i=0;i<numPoints;i+=5){//view every 5th increment (don't want to see too much and spam the terminal)
    if ((msg->ranges)[i]<.3 && (msg->ranges)[i]>msg->range_min && (msg->ranges)[i]<msg->range_max){
    //must check if it's within min and max!!!
      std::cout<<"Object detected "<<(msg->ranges)[i]<<" away at angle "<<((msg->angle_increment)*i+msg->angle_min)<<"\n";
    }
  }*/
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)),ros::Time::now(),"base_footprint","laser"));

  int numPoints=(msg->angle_max-msg->angle_min)/msg->angle_increment;
  bool isObstacle=0;
  double angle;
  for (int i=0;i<numPoints;i++){
    angle = ((msg->angle_increment)*i+msg->angle_min);
    if ((msg->ranges)[i]<=.25 && (msg->ranges)[i]>msg->range_min && fabs(angle)<=M_PI/2+.1){
    //must check if it's within min and max!!!
      std::cout<<"Object detected "<<(msg->ranges)[i]<<" m away at angle "<<angle<<"\n";
      isObstacle=1;
      break;
    }
  }
  if (isObstacle){
    if (angle<0){
      std::cout<<"\tTurning left\n";
      vel.publish(turnLeft);
    }else{
      std::cout<<"\tTurning right\n";
      vel.publish(turnRight);
    }
  }else{
    vel.publish(goStraight);
  }
}

int main(int argc, char **argv){
  std::cout<<"entering main?\n";
  ros::init(argc,argv,"avoidObst");
  ros::NodeHandle nh;



  //subscribe to scan
  ros::Subscriber lidar= nh.subscribe("/base_scan", 1, detectCallback);
  //publish to cmd_vel
  vel=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  turnLeft.angular.z=.2;
  turnRight.angular.z=-.2;
  goStraight.linear.x=.2;

  while(ros::ok()){
    ros::spin();
  }
}
