#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>

using namespace std;

class environment{
  public:
    environment();
    void reset();
    void step(int a);
    geometry_msgs::Point pose;//pose is state in frozen lake/gridworld example
    bool collision_state;
    bool isDone;
    double reward;
  //private:
    ros::NodeHandle nh;
    ros::Publisher stopSim;
    ros::Publisher startSim;
    ros::Subscriber globalPos;
    ros::Publisher setPos;
    void getPos_callback(const geometry_msgs::Point::ConstPtr& msg);
    //ros::Subscriber scan;
    //void getScan_callback();
    ros::Subscriber check_col;
    void updateCol_callback(const std_msgs::Bool::ConstPtr& msg);
    std_msgs::Bool simTrue;
    double current_x,current_y,current_z;
    ros::Rate pollRate=ros::Rate(100);
};

environment::environment(){
  cout<<"Constructor called\n";
  stopSim=nh.advertise<std_msgs::Bool>("/stop_sim",1);
  startSim=nh.advertise<std_msgs::Bool>("/start_sim",1);
  globalPos=nh.subscribe("/get_pose",1,&environment::getPos_callback,this);
  setPos=nh.advertise<geometry_msgs::Point>("/set_pose",1);
  //scan=nh.subscribe("/velodyne_scan",1,&environmnet::getScan_callback,this);
  check_col=nh.subscribe("collision_status",1,&environment::updateCol_callback,this);
  simTrue.data=true;
  reset();
}

void environment::reset(){
  //cout<<"reset called\n";
  /*first make sure simulation is actually off (and if a collision just happended
  that it is done being published)*/
  while(check_col.getNumPublishers()!=0){
    stopSim.publish(simTrue);//this is already done at goal and after collision--but just to be sure we need to check
    pollRate.sleep();
  }
  while(startSim.getNumSubscribers()==0){//wait until vrep subscribes to start_sim
    pollRate.sleep();
  }
  while(startSim.getNumSubscribers()!=0){//wait until vrep is done subscribing to start_sim (which indicates simulation has started)
    startSim.publish(simTrue);
  }
  collision_state=false;
  isDone=false;
  reward=0;
  current_x=pose.x=0;
  current_y=pose.y=0;
  current_z=pose.z=.75;//this is the position it starts at in vrep
  ros::spinOnce();
}

void environment::step(int a){
  if (isDone){
    cout<<"Don't call step, if simulation is done. Must reset";
    return;
  }
  //action is either move .25 meters in direction {0,pi/2,pi,3pi/2}
  //corresponding to integers {0,1,2,3} respectively
  double step=.25;
  switch(a){
    case 0: pose.x=((pose.x+step)>4 ? 4:(pose.x+step));
    break;
    case 1: pose.y=((pose.y+step)>4 ? 4:(pose.y+step));
    break;
    case 2: pose.x=((pose.x-step)<0 ? 0:(pose.x-step));
    break;
    case 3: pose.y=((pose.y-step)<0 ? 0:(pose.y-step));
    break;
  }
  while(setPos.getNumSubscribers()==0){//wait until vrep subscribes to set_pose
    //if (isDone) return;
    pollRate.sleep();
  }
  setPos.publish(pose);
  cout<<pose;
  if (pose.x==4 && pose.y==4){//reach goal! get reward, and stop simulation
    reward=1000;
    cout<<"AT GOAL!\n";
    isDone=true;
    while(stopSim.getNumSubscribers()==0)//wait until vrep subscribes to stop_sim
      pollRate.sleep();
    stopSim.publish(simTrue);
    ros::spinOnce();
  }else if ((pose.x==0 || pose.y==0)&&!collision_state){
    reward=-1;//penalize robot for staying at edge
  }else if (!collision_state){//didn't reach goal, reward is exponential of inverse distance to goal
    //reward=10-sqrt((4-pose.x)*(4-pose.x)+(4-pose.y)*(4-pose.y));
    reward=exp(1/sqrt((4-pose.x)*(4-pose.x)+(4-pose.y)*(4-pose.y)))*10;
  }//possible collision state?
//}else{reward=0;}
  while(!isDone && (fabs(pose.x-current_x)>.1 || fabs(pose.y-current_y)>.1)){
    //don't leave this function until the quadcopter has moved a step (tolerance of .1 in each cordinate)
    setPos.publish(pose);
    ros::spinOnce();
  }
}

void environment::getPos_callback(const geometry_msgs::Point::ConstPtr& msg){
  //cout<<"getPos called!\n";
  current_x=msg->x;
  current_y=msg->y;
  current_z=msg->z;
}

void environment::updateCol_callback(const std_msgs::Bool::ConstPtr& msg){
//  cout<<"updateCol called\n";
  //cout<<msg->data<<"\n";
  if (collision_state=msg->data){
    cout<<"COLLISION!: "<<collision_state<<"\n";
    reward=-5;
    isDone=true;
    //the simulation is stopped in vrep lua script if collision happens
    //(hopefully that helps it run faster)^^
    /*while(stopSim.getNumSubscribers()==0)//wait until vrep subscribes to stop_sim
      pollRate.sleep();
    stopSim.publish(simTrue);*/
    ros::spinOnce();
  }
}
/*
Qtable.txt has lr .8, df .9, greedy, reward negative distance to goal*10, 500 goal, -100 crash
Qtable2.txt has lr .8, df .9, e=.7, reward 10 minus distance to goal, 500 goal, -100 crash
Qtable3.txt has lr.8, df .95, e=.8, reward 0 at obstacle,  reward 10 minus distance to goal, 500 at goal
--started biasing actions forward and right
--started giving penalty -1 if at edge (x==0 or y==0)
-^problem: 10-distToGoal is linear and difference between adjacent steps is too small (should get lots of reward for going forward)
Qtable4.txt has lr .8, df .95, e's don't matter, reward 1 at goal 0 else, bias 1 right and up--the og algo
-^problem: learns super slowly
***j=50 before this and =200 after
Qtable5.txt has lr.8, df .3, e=.7, reward -1 at obstacle,  reward exp(inverse distance to goal)*10, 500 at goal
--started biasing actions forward and right to 10
--started giving penalty -1 if at edge (x==0 or y==0)
-^problem: bias is so large that hitting an obstacle for potential future award becomes larger than not hitting an obstacle
Qtable6.txt has lr.8, df .9, e=.7, reward -5 at obstacle,  reward exp(inverse distance to goal)*10, 500 at goal
--started biasing actions forward and right to .5
--started giving penalty -1 if at edge (x==0 or y==0)
*/
