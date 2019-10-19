//adding boundries (line 80 and 106-110 or something) made it stop working
//also it doesn't work with radius!! maybe check dist() function

/*status 08/28/18: Trying to make it work with multiple objects--it does learn a path to dodge them
but for some reason it learns to go around the long way--trying flipping the direction of the forloop
and it didn't make a difference. Maybe it makes sense tho because nothing in the qtable links a action
choice with shortest distance (because that won't generalize well to unseen environments)
--also tried to randomize environmnets and that works... sometimes. Could use some more rigourous testing,
but I'm pretty sure Im doing some learning wrong bc a lot of cases the robot gets stuck going back and fourth
between two spots--i'm not sure which test cases would give it this conflicting decisions--like in what case
is the best option to move back to the same square u just came from?
--oh i think i get it.. if you have blocks in ur blind spots and it looks like a good decision but it's not!
--- maybe have 8 direction states and 4 movements?

status 06/25/18: a lot has happened--it works!.. on a simple case at least--see notes for more details
interesting: if you do double q_noise=Q[s][gh][x]+N(gen);instead of N(gen)*1/(i+1);
--if you make it so noise doesn't decrease over episodes after training on one Environment
--and make j<1000 than
the robot navigates better in other environments (move obs1 down one) but not as good in it's original
--like it then moves x=1,2,3,4 instead of x=1,2,1,2,1,2,1...
it probs explored more and discovered more of its table whereas originally it would exploit more and
find the most optimal path.

status 06/25/18: fundemental error in algorithm--the Qtable does not encode information related to the goal.
So when a reward is given getting it closer to goal--it does not associate that with anything related to the goal!!!
Need to add another dimension in q table that tells the heading of the robot to the goal. Like the goal
is to the right of the robot, or to the left of it.
--so far Ive tried to add the other dimension to the q table and environment, yet haven't tested it!

status 06/21/18: need to debug. Currently robot moves straight into obstacle
if you really struggle to debug it, maybe vigourously test the environment
*/
//note: use g++ -g -std=c++0x -Wall Qtable_RL.cpp -o Q_RL
#include <iostream>
#include <cmath>
#include <random>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
//include "vrep_environment.hpp"

using namespace std;

class environment{
  public:
    struct item{
      double x;
      double y;
      double r;//radius of object
    } robot, goal, obs1, obs2;
    double sensor[8];//1x4 array, at 0,pi/2,pi,3pi/2 radians
    int step_count;
    int goal_heading;
    environment(){//simple environment
      globalPos=nh.subscribe("/get_pose",1,&environment::getPos_callback,this);
      scan=nh.subscribe("/velodyne_scan",1,&environment::updateSensor,this);
      simTrue.data=true;
      reset();
    }
    void reset(){
      stopSim.publish(simTrue);
      pose.x=0; pose.y=0;pose.z=0;
      robot.x=robot.y=goal.y=0;
      robot.r=goal.r=obs1.r=obs2.r=0;
      robot.r=.5;
      goal.r=.3;//should probs change this
      obs1.r=.65;
      obs2.r=.65;
      goal.x=4;
      obs1.x=2;
      obs1.y=1;//rand()%3-1;
      obs2.x=2;//rand()%3+1;
      obs2.y=0;
      /*do{
        obs2.y=rand()%3-1;
      }while(obs2.x==obs1.x && obs2.y==obs1.y);*/
      startSim.publish(simTrue);
    }
    ros::NodeHandle nh;
    ros::Publisher stopSim=nh.advertise<std_msgs::Bool>("/stop_sim",1);
    ros::Publisher startSim=nh.advertise<std_msgs::Bool>("/start_sim",1);
    ros::Publisher setPos=nh.advertise<geometry_msgs::Point>("/set_pose",1);
    geometry_msgs::Point pose;
    ros::Subscriber globalPos;
    ros::Subscriber scan;
    std_msgs::Bool simTrue;

    double reward(double diff){//returns reward based on distance to goal and crashes
      if (dist(robot,obs1)<=0 || dist(robot,obs2)<=0){
        /*if (robot.y==0 && robot.x==1){
          cout<<"Distance between robot and obs2= "<<dist(robot,obs2)<<'\n';
          cout<<"so\n";
        }*/
        cout<<"CRASHED!\n";
        return -100;
      }
      //return diff/dist(robot,goal);//so as you get closer--the reward gets bigger
      if (dist(robot,goal)<=0){
        cout<<"REACHED GOAL!\n";
       return 500;
     }
      /*if (diff>dist(robot,goal)){
        //if you got closer
        return 1;
      }else{*/
        return -1;
      //}
    }

    int isDone(){//returns 0 if not done, 1 if crashed, 2 if at goal
      if (dist(robot,goal)<=0){
      //  cout<<"AT GOAL\n";
        return 2;
      }
      if (dist(robot,obs1)<=0 || dist(robot,obs2)<=0){
        return 1;
      }
      //cout<<"Not goal not done\n";
      return 0;
    }
    double step(int a){//inputs an action and updates the enivronment
      step_count++;
      //actions are 0 right, 1 up, 2 left, 3 down
      //returns reward
      double prev_dist=dist(robot,goal);
      switch(a){//move robot
        case 0: pose.x=robot.x+.1;
        break;
        case 1: pose.y=robot.y+.1;
        break;
        case 2: pose.x=robot.x-.1;
        break;
        case 3: pose.y=robot.y-.1;
        break;
      }
      setPos.publish(pose);
      double diff=fabs(prev_dist-dist(robot,goal));
      //update sensor
      return reward(diff);
    }
  //private:
    double dist(item i1, item i2){//double x1,double y1, double x2, double y2){
      //just a helper function to find distance between two items
      //return sqrt(pow(x1-x2,2),pow(y2-y1,2));
      return sqrt(pow(i1.x-i2.x,2)+pow(i1.y-i2.y,2)) -(i1.r+i2.r);
    }
    void updateSensor(const std_msgs::Float32MultiArray::ConstPtr& msgs){//ALSO UPDATES GOAL_HEADING!
      for (int i=0;i<8;i++){
        sensor[i]=100;
      }

    }
    void getPos_callback(const geometry_msgs::Point::ConstPtr& msg){
      robot.x=msg->x;
      robot.y=msg->y;
      //pose.z=msg->z;
    }

};


int main(int argc, char** argv){
  ros::init(argc,argv,"ReinforcementLearn");
  srand(time(0));
  std::default_random_engine gen;
  std::normal_distribution<double> N(0,1);

  double Q[256][4][4];//[64];//2x2x2x2x4x4 Q table, 2^8 states 4 headings and 4 actions
  for (int i=0;i<256;i++){
    for (int j=0;j<4;j++){
      for (int k=0;k<4;k++) Q[i][j][k]=0;
    }
  }
  double lr =.8, y =.95;
  int episodes = 100000;//10000000;
  //int c;//for status bar
  environment env;
  for (int i=0;i<episodes;i++){//training
    //if (i%100000==0) cout<<++c<<"%%\n";
    //cout<<"\nEpisode: "<<i<<"\n";
    env.reset();
    int s = (env.sensor[0]<=1)+2*((env.sensor[1]<=1)+2*((env.sensor[2]<=1)+2*((env.sensor[3]<=1)
            +2*((env.sensor[4]<=1)+2*((env.sensor[5]<=1)+2*((env.sensor[6]<=1)+2*(env.sensor[7]<=1)))))));
    //^^think of this as flattening a 4d 2x2x2x2 array---or think of it as binary encoding with the digits backwards (read left to right)
    /*unsigned int s=0;//this is a better way to do what is right above^^
    for (int l=3;l<=0;l++){//using forloop generalizes better--but slows it down significantly!
      s=(s|(env.sensor[l]<=1))<<1;
    }*/
    int gh=env.goal_heading;
    int j=0;
    while(!env.isDone() && j<100){
      j++;
      int a=0;

      if (rand()%8){//epislon greedy, chosing greedy 7/8ths of the time
        double q_noise=Q[s][gh][a]+N(gen)*1/(i+1);
  //      cout<<"Q's with noise: "<<q_noise<<" ";
        for (int x=1;x<4;x++){//chose action greedily
          q_noise=Q[s][gh][x]+N(gen)*1/(i+1);
  //        cout<<q_noise<<" ";
          if (q_noise>Q[s][gh][a]) a=x;
          //^^this is flawed--should be comparing them all with noises
          //not a noisey one against a regular
        }
        //      cout<<"\nClearly the best action is "<<a<<'\n';
      }else{
        a=rand()%4;
      }
      double r = env.step(a);//have environment do the action
      if (i==episodes-1) cout<<"Now robot is at "<<env.robot.x<<","<<env.robot.y<<"\n";
      int s1 = (env.sensor[0]<=1)+2*((env.sensor[1]<=1)+2*((env.sensor[2]<=1)+2*((env.sensor[3]<=1)
              +2*((env.sensor[4]<=1)+2*((env.sensor[5]<=1)+2*((env.sensor[6]<=1)+2*(env.sensor[7]<=1)))))));//next state
      int gh1=env.goal_heading;
//      cout<<"So it's next state is: "<<s1<<"\n";
      //double r = env.reward();//reward
//      cout<<"Current award is: "<<r<<"\n";
      double max=Q[s1][gh1][0];
//      cout<<"Now look at future rewards: "<<max<<" ";
      for (int x=1;x<4;x++){//find max future reward
//        cout<<Q[s1][x]<<" ";
        if (Q[s1][gh1][x]>max) max=Q[s1][gh1][x];
      }
//      cout<<"(max is "<<max<<")\n";
      Q[s][gh][a]=Q[s][gh][a]*(1-lr)+lr*(r+y*max);//update Q table
      s=s1; //update new state
      gh=gh1;
    }
    if (i==episodes-1){
      cout<<"Original Environment^^^\n";
      if (j==99){
        cout<<"99 trials, we done--failed to reach gaol";
      }else{
        cout<<"Environment State: "<<env.isDone()<<"\n\n";
      }
    }
  }

  //now q table is made so let's test the robot
  env.reset();
  //env.obs1.y=-1;
  //env.obs1.x=3;
  cout<<"Obstacles at "<<env.obs1.x<<","<<env.obs1.y<<" and "<<env.obs2.x<<","<<env.obs2.y<<"\n";
  cout<<"Robot starts at "<<env.robot.x<<","<<env.robot.y<<"\n";
  int j=0;
  while (!env.isDone()  && j++<100){
    int s = (env.sensor[0]<=1)+2*((env.sensor[1]<=1)+2*((env.sensor[2]<=1)+2*((env.sensor[3]<=1)
            +2*((env.sensor[4]<=1)+2*((env.sensor[5]<=1)+2*((env.sensor[6]<=1)+2*(env.sensor[7]<=1)))))));
    int gh=env.goal_heading;
    int a =0;
    for (int x=1;x<4;x++){//chose action greedily
      if (Q[s][gh][x]>Q[s][gh][a]) a=x;
    }
    env.step(a);//have environment do the action
    cout<<"Robot moved to "<<env.robot.x<<","<<env.robot.y<<"\n";
  }
  cout<<"Environment State: "<<env.isDone()<<"\n\n";
}
