#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
using namespace std;
bool map_made=false;
typedef struct node{
  int x,y;
  node* right,left,up,down;//neighbor nodes
  node* bptr;//back pointer
  double g; //total length of back pointer (cost till now)
  double f;//g+h (length of backptr plus heuristic)
};
node* all=NULL;
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  if (map_made) return;
  map_made=true;
  cout<<(int)(msg->data.at(0));
  for (int i=0;i<4000;i++){
    int x=i%4000,y=i/4000; //in this case y increases down, x increases right
    val=(int)(msg->data.at(i));
    node* n = new node;
    node.x=x; node.y=y;
    if (val==0 && !in(node,all)){
      if(x< 4000-1 && msg->data.at(4000*y+x+1)){
      }else if(y>0 && msg->data.at(4000*(y-1))){

      }else if(x>0) && msg->data.at(4000*y+x-1){

      }else if(x<0) && msg->data.at(4000*(y-1)+x){

      }

    }
    //cout<<i;
    //if (i==3999) cout<<"AT END!";
    //Note this won't print at end until you kill node
    //because i think the buffer or something gets full but it seems to work.
  }//cout<<msg->size()<<"\n";
}

int main(int argc, char** argv){
  ros::init(argc,argv,"mapgraph");
  ros::NodeHandle nh;

  ros::Subscriber map_subs = nh.subscribe("/map",1,mapCallback);
  ros::spin();
}
