/*still need to confirm if this is correct orientation of sensor
*/


#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include <iostream>
using namespace std;

void scancallback(const std_msgs::Float32MultiArray::ConstPtr msg){
  //cout<<(((msg->layout).dim).at(0)).label<<"\n";
  //cout<<"\t"<<msg->layout.dim[0].size<<"\n";
//  cout<<"\t"<<msg->layout.dim[0].stride<<"\n";
/*
  cout<<msg->layout.dim[1].label<<"\n";
  cout<<"\t"<<msg->layout.dim[1].size<<"\n";
  cout<<"\t"<<msg->layout.dim[1].stride<<"\n";

  cout<<msg->layout.dim[2].label<<"\n";
  cout<<"\t"<<msg->layout.dim[2].size<<"\n";
  cout<<"\t"<<msg->layout.dim[2].stride<<"\n";

  cout<<msg->layout.dim[3].label<<"\n";
  cout<<"\t"<<msg->layout.dim[3].size<<"\n";
  cout<<"\t"<<msg->layout.dim[3].stride<<"\n";
*/
  int num=msg->data.size()/3-1;
  //cout<<"SIZE: "<<num<<'\n';
  for (int i=0;i<num;i++){
    if(i%4==0) continue;
    float x,y,z;
    //under this +x is the distance infront of the laser and +yis distance to left of laser
    y=msg->data.at(3*i+1);
    x=msg->data.at(3*i+0);
    z=msg->data.at(3*i+2);
    //if (msg->data.at(i)>2) std::cout<<msg->data.at(i)<<' '<<i%3<<"\n";
    if(fabs(z)<.04 || x>4){
      printf("(%f,%f,%f)\n",x,y,z);
    }else{
      //cout<<z<<"\n";
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc,argv,"testscan");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/velodyne_scan",1,scancallback);
  ros::spin();
}
