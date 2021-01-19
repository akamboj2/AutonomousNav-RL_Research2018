/*status 6/18/18:  velocity vector obstacle avoidance navigation works on irobot...
with some bugs of course-- so far I hav only done simple tests of avoiding one
obstacle but this is what I have found:
- the very first trajectory is generated with no data from the sensors (EVEN THO I DO
ROS::SPIN BEFORE GENERATING THE VECTORS)
- when it gets close to the end of its waypoint sometimes it repeatedly stops and
turns and goes over an over again in a jerky motion.
- the wires get really twisted really fast
- i'm still afraid it might just miss it's vector end (next x and next y) and
just keep on going because odometry node doesn't seem too reliable

/* remember to run urg_node like: rosrun urg_node urg_node scan:=base_scan

/*status 6/15/18: After translating the code to work with the create robot, it was turning funny
and not reaching the end of its vector,etc. I spent today debugging that and fixing most of those problems.
So far it works with simple coordinates around the axes-->could use some more rigourous testing-
- also need to test it on the floor more.
- Also, would be better if I could add back the PID turning and speed
then I think i can move on to adding the sensor and testing that!
*/


/*this is a knock off of nav_waypoints in the catkin/src/vrep_apple.../src folder
except this one is written to use the create autonomy robot.*/


/*NOTE: this code fails miserably if trying to get past a wall or something much wider than itself/laser's vision.
PROBLEM: it keeps running straight into the wall repeatedly when it's supposed to get to the other side
REASON:the best vector being generated is the vector that it already went through because that is closest
and had no obstructions.
SOLUTIONS?: get rid of biasing towards in range of sensor (might generate some better vectors)
            some how change priority of which vector is chosen?
for proof of bug run in autoNav vrep environmnet

/*this program does simple waypoint navigation using next states and random velocity vectors */
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
//#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <cstdlib>
#include <time.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double currx=0;
double curry=0;
double currtheta=0;
double angleTol= .04; //about 1 degree
//double angleTol=.02125;//i believe this is about 2.5 degrees
double distTol=.4;

ros::Publisher vel;
geometry_msgs::Twist turnLeft;
geometry_msgs::Twist turnRight;
geometry_msgs::Twist goStraight;

//double v;//this is the temporary speed checked in the forloop when finding random velocity vectors
//^^it needs to be global so objectDetectCallback can use it.
int numPoints=0;//note this is half the size of ptCloud array
double* ptCloud=NULL;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  currx=msg->pose.pose.position.x;
  curry=msg->pose.pose.position.y;
  currtheta=msg->pose.pose.orientation.z*M_PI;//is this correct? idk need to test
  currtheta=(currtheta<0 ? currtheta+2*M_PI:currtheta);
  //std::cout<<"Current pose = "<<currx<<","<<curry<<" theta="<<currtheta<<"\n";
}


void objectDetectCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  //std::cout<<"objectDetect Callbacked\n";
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)),ros::Time::now(),"base_footprint","laser"));

  if (ptCloud!=NULL){
    delete [] ptCloud;
  }
  numPoints=int((msg->angle_max-msg->angle_min)/msg->angle_increment);

  ptCloud=new double[numPoints*2];
  //dynamically allocate an array of xy cordinates
  //need dynamic because array is changing size every call

  /*with these definitions the car is always facing up
  on the positive y axis with the car being at coordinates 0,0
  in the point cloud points to the right of it will have positive xs
  points to left have negative xs. Y IS NEVER NEGATIVE!*/
  double x,y;
  //double farRight=0, farLeft=0, nearY[2]={1000,0};
  int i=0,count=0;
  for (i=0;i<numPoints;i++){
    //this takes all the point and puts them in a 1d array
    double angle = ((msg->angle_increment)*i+msg->angle_min);
    double r=(msg->ranges)[i];
    if (r>msg->range_max || r<msg->range_min){
      continue;//skip points that aren't within acceptable range
    }
    y=r*cos(angle);
    x=-r*sin(angle);
    ptCloud[count++]=x;
    ptCloud[count++]=y;
    if (r<.2  && fabs(angle)<=M_PI/2+.1) std::cout<<"Added point: "<<x<<","<<y<<"  r,theta: "<<r<<' '<<angle<<"\n";
  }
  numPoints=count/2;//in case somepoints were invalid
}

/*this function is called to check if the path infront of the car
is clear given a velocity vector and using the ptCloud data from
the 2D laser scanner. Returns false if the path is not clear and true otherwise*/
bool clearPath(double v, double th){
  //std::cout<<"clearPath Called!\n";
  int i=0;
  double hw=.3;//estimated half width of car
  double hl=.3;
  double x,y,r,ptTheta,A;
  //double r,A;
  th+=(M_PI/2-currtheta);//bc point cloud assumes car at angle pi/2 facing y always
  //we have to transform theta under that assumption. it's like changing from a reference frame
  //where car is facing currtheta to reference frame where car is facing pi/2
  th= (th>2*M_PI ? th-2*M_PI:th);
  th= (th<0 ? th+2*M_PI:th);
  //std::cout<<"currx:"<<currx<<" curry:"<<curry<<"\n";
  for (i=0;i<numPoints;i++){
    //this might be easier to do with polar cordinates
    x=ptCloud[i*2];
    y=ptCloud[i*2+1];
    //r=sqrt((x-currx)*(x-currx)+(y-curry)*(y-curry));//currx and and curry in global reference frame
    r=sqrt(x*x+y*y);//in the point cloud we assume car is always at x=0, y=0 so this is distance to it
    ptTheta=atan2(y,x);
    A=asin(hw/r);//this is how far the points theta can't be from th
    /*if (isnan(A)){
      //if u are that close to something, that r value is larger than hw that means you gonna crash so ur path is not clear
      return false;
    }*/
    //std::cout<<"x:"<<x<<" y:"<<y<<"\n";
    //std::cout<<(r<=v)<<' '<<(ptTheta>th-A)<<' '<<(ptTheta<th+A)<<'\n';
    //std::cout<<"ptTheta:"<<ptTheta<<" th:"<<th<<" A:"<<A<<'\n';
    if(r-hl<=v && ptTheta>th-A && ptTheta<th+A){
    /*if ((r-hw)<v && -x/tan(th)-hw<y && -x/tan(th)+hw>y &&
        -(y+hw)*tan(th)<x && -(y-hw)*tan(th)){*/
      //std::cout<<"path not clear at point: "<<x<<","<<y;
      //std::cout<<" for velocity "<<v<<" and angle "<<th<<"\n\n";
      return false;
    }
  }
  //std::cout<<"path clear for velocity "<<v<<" and angle "<<th<<"\n\n";
  return true;
}

/*this helper function assures there are no obstacles directly infront of the car
it returns true if there are no obstacles and false if there is something it's about
to crash into*/
bool noObstacle(){
  //std::cout<<"noObstacle called\n";
  int i;
  for (i=0;i<numPoints;i++){
    double x=ptCloud[i*2],y=ptCloud[i*2+1];
    if (fabs(x)<.2 && y<.2 && y>0){
      std::cout<<"TOO CLOSE TO OBJECT! at "<<x<<","<<y<<"\n";
      return false;
    }
  }
  return true;
}

/*this function turns the car to a given newAngle
this angles assume ccw from x (the car is facing currtheta and should face newAngle)*/
void turnCar(double newAngle){
  newAngle = (newAngle<0 ? (newAngle+2*M_PI):newAngle);//make sure newAngle is positive
  double diff = newAngle-currtheta;//difference between where you want to turn and where you're currently facing
  std::cout<<"turn to angle: "<<newAngle<<" from current angle: "<<currtheta<<"\n";
  while(fabs(diff)>angleTol){//this should turn the robot
    diff = (diff<0 ? (diff+2*M_PI):diff);//make sure difference is positive
    if (diff<M_PI){//turn left if differenclearce is less than 180 degrees
      turnLeft.angular.z=.1;//diff/M_PI*2/10;
      turnLeft.linear.x=0;
      vel.publish(turnLeft);
      //std::cout<<"Turning left, at speed:"<<turnLeft.angular.z<<'\n';
    }else{//turn right SO NEGATIVE ANGULAR VELOCTIY!
      turnRight.angular.z=-.1;//-(diff-M_PI)/M_PI*2/10;
      turnRight.linear.x=0;
      vel.publish(turnRight);
      //std::cout<<"Turning right, at speed:"<<turnRight.angular.z<<'\n';
    }
    ros::spinOnce();
    //std::cout<<"Turning published left: "<<speedL.data<<" Right: "<<speedR.data<<"\n";
    //std::cout<<"\tGoal angle: "<<newAngle<<" current angle: "<<currtheta<<"\n";
    diff=newAngle-currtheta;
    //std::cout<<"newAngle:"<<newAngle<<" currtheta:"<<currtheta<<'\n';
  }
  //stop the car fr  ros::Publisherom spinning to calculate the next x and y
  goStraight.linear.x=goStraight.angular.z=0;
  vel.publish(goStraight);
  ros::spinOnce();

  std::cout<<"\tGoal angle: "<<newAngle<<" current angle: "<<currtheta<<"\n";
  std::cout<<"\tDifference: "<<diff<<"\n";
}

double destx,desty,destz;//read in the destination x and y from user
bool gotDest=false;
void getDest(const geometry_msgs::Point::ConstPtr& msg){
  //std::cout<<"\n\n\n ENTERED GETDEST\n\n\n";
  destx=msg->x;
  desty=msg->y;
  gotDest=true;
}

int main(int argc, char** argv){
  srand(time(0));

  //initialize ros node
  ros::init(argc,argv,"wayNav");
  ros::NodeHandle nh;

  //subscribe to scan and Odometry
  ros::Subscriber lidar= nh.subscribe("/base_scan", 1, objectDetectCallback);
  ros::Subscriber odometryPose=nh.subscribe("/odom",1, poseCallback);
  ros::Subscriber destinationSub=nh.subscribe("/destination",1,getDest);
  //publish to cmd_vel
  vel=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  //ros::CallbackQueue myQ;
  /*std::printf("Enter x y:");
  std::scanf("%lf %lf",&destx,&desty);
  ros::spinOnce();*/
  //NOTE: below this commented out to stop file io reading
  /*
  double destx,desty,destz;//read in the destination x and y from user
  FILE* wayPoints;
  wayPoints= fopen("/home/isler/Abhi/create_ws/src/create_autonomy/ca_driver/src/ca_waypoints.txt","r");
  if(!wayPoints){
    ROS_INFO("No waypoint file found\n");
    return 0;
  }

  while(!std::feof(wayPoints)){
    std::fscanf(wayPoints,"%lf %lf %lf",&destx,&desty,&destz);*/

  while(ros::ok()){
    ros::spinOnce();
    if (!gotDest){//this skips the rest of the loop while got dest is false
      //std::cout<<"Waiting for destination..."<<gotDest<<'\n';
      continue;
    }
    gotDest=false;

    std::cout<<"Next Waypoint: "<<destx<<","<<desty<<"\n";
    //initialize the vectors out here so you can carry over the previous loops best velocity (for same waypoint)
    //vector to the next loop if it is still the best
    double nextx,nexty,nexttheta,minDistSq=400;
    double newSpeed=0,newAngle=0;
    //int newVector=0;

  //while(ros::ok()){//i thought this would help me kill the node after the program... it didn't
    while(fabs(currx-destx)+fabs(curry-desty)>distTol){//continue while you are not at destination
      //before generating random velocity vectors make sure car is facing the correct direction
      std::cout<<"Turn car to correct direction\n";
      turnCar(atan2(desty-curry,destx-currx));

      double v =newSpeed;//initialiclearze linear velocity to velocity of last statenewSpeed,
      double w =0;//initialize angular velocity to 0 --will give same heading direction as last state(bc you are in that direction rn)
      minDistSq=4000000;//new minDistSq will be different from before so have to recacluate it
      double distToEnd=sqrt(pow(curry-desty,2)+pow(currx-destx,2));
      std::cout<<"Currently at ("<<currx<<","<<curry<<") distToEnd:"<<distToEnd<<"\n";
      std::cout<<"Generating one million random velcotiy vectors\n";

      for (int i=0;i<1000;i++){//generate 999,999 random v,w vectorsdouble v;//this is the temporary speed checked in the forloop when finding random velocity vectors
        /*note v and w are set at the end of the loop for the next loop so that the
        previous states v and w values can also be tested and carried over to be the
        best vector in this state as well.*/
        //calculate next angle and next xy states
        nexttheta=currtheta+w;
        nexttheta=(nexttheta>2*M_PI ? nexttheta-2*M_PI:nexttheta);
        nextx=currx+v*cos(nexttheta);
        nexty=curry+v*sin(nexttheta);
        //if that state is closer to the destination then any of the other 100 random samples
        //and there is nothing in its Path
        //inPath=false;
        //myQ.callAvailable();
        //ros::spinOnce();
        if(pow(nextx-destx,2)+pow(nexty-desty,2)<minDistSq && clearPath(v,nexttheta)){
          //make that your final new speed and angle.
          minDistSq=pow(nextx-destx,2)+pow(nexty-desty,2);
          newSpeed=v;
          newAngle=nexttheta;
          //newVector=0;
          //std::cout<<"RESETTING NEWVECTOR TO ZERO\n";
          //std::cout<<"newAngle is now: "<<nexttheta<<'\n';
        }
        //randomly generate a linear and angular velocity for next loop
        v=(rand()/(float)RAND_MAX)*distToEnd;//linear velocity between 0 and disttoend
        if (i<10){
          w=(rand()/(float)RAND_MAX)*(2*M_PI);//anglular velocity between 0 and 2pi
        }else{
          w=(rand()/(float)RAND_MAX)*(M_PI/2)-M_PI/4;//anglular velocity between -pi/4 and pi/4 (range of the sensor)
          w=(w<0 ? w+2*M_PI:w);//between 0 and pi/4 or 7pi/4 and 2pi
        }
      }

      //turn the robot to the new angle
      //while(ros::ok()){ //i thought this would help with killing the node--it didn't
      turnCar(newAngle);

      //set the next x and y based on current state (use new speed and your current angle after turning)
      nextx=currx+newSpeed*cos(currtheta);
      nexty=curry+newSpeed*sin(currtheta);

      //done turning so continue going straight until you get to your next point
      //std::cout<<"Straight published left: "<<speedL.data<<" Right: "<<speedR.data<<"\n";
      std::cout<<"Going from ("<<currx<<","<<curry<<") to ("<<nextx<<","<<nexty<<")\n";
      std::cout<<"\tat velocity: "<<newSpeed<<" and angle: "<<currtheta<<"\n";
      //std::cout<<"\tactual speed is "<<newSpeed/distToEnd*4.5<<"\n";

      while(fabs(currx-nextx)+fabs(curry-nexty)>distTol){
        //having a set speed doesn't work bc of random errors in simulation and transmission
        //we need constant correction
        newAngle=atan2(nexty-curry,nextx-currx);//this is needed because your new angle is always changing if you're rotating!
        newAngle=(newAngle<0 ? newAngle+2*M_PI:newAngle);
        double diff = newAngle-currtheta;
  //      std::cout<<"newAngle: "<<newAngle<<" currtheta:"<<currtheta<<"\n";
        diff = (diff<0 ? diff+2*M_PI:diff);//make sure diff is positive
        double distToNext=sqrt(pow(nextx-destx,2)+pow(nexty-desty,2));
        double setSpeed;
        //on the field use this
        setSpeed=newSpeed/distToEnd*4.5-2/distToNext;//transforms speed into a number between 0 and 5
        //and makes it decrease proportionally as you get closer to next point
        setSpeed=(setSpeed<.03 ? .03:setSpeed);//lower bound of speed is .03
        //for now lets just use
        //setSpeed=newSpeed;
        setSpeed=.1;//for testing with create
        if (fabs(diff)>angleTol){
          //if it's not going where it's supposed to
  //        std::cout<<"diff: "<<diff<<" so turn ";
          if (diff<M_PI){
            //turn left if that's closest
            //so if your at 90 degrees add 2, 45 degrees add 1, etc
  //          std::cout<<"left\n";
            turnLeft.angular.z=setSpeed*2;//diff/M_PI*4;
            turnLeft.linear.x=setSpeed;
            vel.publish(turnLeft);
          }else{//turn right SO NEGATIVE ANGULAR VELOCTIY!
  //          std::cout<<"right\n";
            turnRight.angular.z=-setSpeed*2;//-(diff-M_PI)/M_PI*4;
            turnRight.linear.x=setSpeed;
            vel.publish(turnRight);
          }
        }else{
          goStraight.angular.z=0;
          goStraight.linear.x=setSpeed;
          vel.publish(goStraight);
        }

        //std::cout<<" \tStraight published left: "<<speedL.data<<" Right: "<<speedR.data<<"\n";
        ros::spinOnce();
  //      std::cout<<"at ("<<currx<<","<<curry<<") to ("<<nextx<<","<<nexty<<")\n";

        //std::cout<<"Going from ("<<currx<<","<<curry<<") to ("<<nextx<<","<<nexty<<")\n";
        //!clearPath(sqrt(pow(nextx-currx,2)+pow(nexty-curry,2)),newAngle)
        //      if (newVector++>200 && !noObstacle() && !clearPath(sqrt(pow(nextx-currx,2)+pow(nexty-curry,2)),currtheta)){
        if (!noObstacle()){ //newVector++>300 &&
          std::cout<<"Stopping car and regenerating velocity vectors.\n";
          double backx=currx,backy=curry;
          while(sqrt(pow(backx-currx,2)+pow(backy-curry,2))<.2){//move the car backwards .2
            goStraight.angular.z=0;
            goStraight.linear.x=-.05;
            vel.publish(goStraight);
            ros::spinOnce();
          }
          break;
        }
      }

      /*you should stop the car before the next round of calculations (99 rand vectors)
      because otherwise if it was checking distances while moving, it would get different results*/
      goStraight.linear.x=goStraight.angular.z=0;
      vel.publish(goStraight);
      ros::spinOnce();
      std::cout<<std::endl;
    }
    std::cout<<"END OF WAYPOINT "<<destx<<","<<desty<<"\n\n\n";
  }

//}for ros::ok()
  goStraight.linear.x=goStraight.angular.z=0;
  vel.publish(goStraight);
  ros::spinOnce();
}
