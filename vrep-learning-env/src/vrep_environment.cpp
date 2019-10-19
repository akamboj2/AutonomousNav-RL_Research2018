#include "vrep_environment.hpp"
#include <random>
#include <fstream>

void store_table();
void read_table();
void init_zeroes();

double Q[17][17][4];
int main(int argc, char** argv){
  srand(time(0));
  //std::default_random_engine gen;
  //std::normal_distribution<double> N(0,1);
  ros::init(argc,argv,"vrep_Qlearning");
  environment env;
  //to make table zeros
  //init_zeroes();
  read_table();
  int episodes=10000;
  double lr=.5,df=.9,e=0,erate=1.175;//learning_rate, discount_factor, epsilon (for-e greedy)
  for (int i=0;i<episodes;i++){
    int j=0;
    int x=env.pose.x*4,y=env.pose.y*4;//this is the state
    while(j<200 && !env.isDone){
      int a=0;
      if((rand()/(double)RAND_MAX)>e){//be greedy with prob epsilon-1
        a=0;
        for (int z=1;z<4;z++){ //find action with greatest q value
          if (Q[x][y][z]>Q[x][y][a]) a=z;
        }
        cout<<"Greedy a: "<<a<<" at step "<<j<<'\n';
      }else{//be random with probability epsilon
        a=rand()%4;//chose random action
        cout<<"Random a: "<<a<<" at step "<<j<<'\n';
      }
      /*//regular with noise
      double q_noise[4];//the og rl algo
      for (int z=0;z<4;z++){
        q_noise[z]=Q[x][y][z]+N(gen)*1/(i+1);
      }
      for (int z=1;z<4;z++){
        if (q_noise[z]>q_noise[a]) a=z;
      }
      */
      env.step(a);
      int x_next=env.pose.x*4,y_next=env.pose.y*4, a_next=0;
      for(int z=1;z<4;z++){
        if(Q[x_next][y_next][z]>Q[x_next][y_next][a_next]) a_next=z;
      }
      //update q table
      Q[x][y][a]= (df*Q[x_next][y_next][a_next]+env.reward)*lr+ Q[x][y][a]*(1-lr);
      //cout<<"Updated: "<<x<<","<<y<<" with reward:"<<env.reward<<" and next state: "<<x_next<<","<<y_next<<"\n";
      x=x_next;
      y=y_next;
      j++;
    //  e =(e<.4?e*erate:.4);//epsilon increases every step and maxes at .6
    }
  //  e=.01;//reset epsilon

    if (j==199) cout<<"Ran out of steps :/\n";
    env.reset();
    store_table();
    //if (i%2==0) store_table();
    if (i%10==0){
      cout<<".";
    //  store_table();
    }
    if (i%1000==0){
  //    cout<<i;
    }
    cout<<"Episode: "<<i<<"\n";
  //e =(e<.6?e*erate:.85);
  }
}

void store_table(){
  cout<<"Storing Qtable...\n";
  FILE* f;
  f=fopen("/home/abhi/Abhi/catkin_ws/src/vrep-learning-env/src/Qtable6.txt","w");
  for (double y=0;y<=4;y+=.25){
    for (double x=0;x<=4;x+=.25){
      int i=x*4, j=y*4;
      fprintf(f,"(%f,%f): %f %f %f %f\n",x,y,Q[i][j][0],Q[i][j][1],
      Q[i][j][2],Q[i][j][3]);
    }
  }
  /*//if you want it to look like the map:
  for (double y=4;y>=0;y-=.25){
    for (double x=0;x<=4;x+=.25){
      int i=x*4, j=y*4;
      fprintf(f,"(%f,%f): %f %f %f %f\t",x,y,Q[i][j][0],Q[i][j][1],
      Q[i][j][2],Q[i][j][3]);
    }
  }*/
  fclose(f);
}
void read_table(){
  cout<<"Reading Qtable...\n";
  ifstream f;
  f.open("/home/abhi/Abhi/catkin_ws/src/vrep-learning-env/src/Qtable6.txt");
  for (int y=0;y<17;y++){
    for (int x=0;x<17;x++){
      string dum;
      f>>dum>>Q[x][y][0]>>Q[x][y][1]>>Q[x][y][2]>>Q[x][y][3];
    }
  }
  f.close();
}
void init_zeroes(){
  cout<<"Initializing Q table to zeroes..\n";
  for (int x=0;x<17;x++){
    for (int y=0;y<17;y++){
      Q[x][y][0]=Q[x][y][1]=.5;
      Q[x][y][2]=Q[x][y][3]=0;
    }
  }
}
/*things to note:
when you change the initial position of the robot
-change vrep_environment.hpp pose, and currentxyz in reset function
-change Quadricopter object on vrep
-change Quadricopter script pose in initialization function

when you run the environment for the first time or if
you modify the Quadricopter customization script, you have to start
and stop the simulation once by hand to get the start stop topic
publishers running before it works with code
*/

/*
//a good simulation tester
int i=0;
while(!env.isDone && i++<10){
  std::cout<<"looped\n";
  env.step(0);
  std::cout<<"after env.step()\n";
  cout<<env.isDone<<"\n";
}
cout<<"getting out here?\n";
if (env.isDone){
  cout<<"resetting env\n";
  env.reset();
}
for(int y=0;y<5;y++){
  cout<<"step: "<<y<<"\n";
  cout<<"isDone?: "<<env.isDone<<"\n";
  env.step(1);
}*/
