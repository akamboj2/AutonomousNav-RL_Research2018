#include <iostream>
#include <fstream>

using namespace std;
float Q[17][17][4];

void store_table(){
  cout<<"Storing Qtable...\n";
  FILE* f;
  f=fopen("/home/isler/Abhi/catkin_ws/src/vrep-learning-env/src/QtablePracticeStore.txt","w");
  for (double y=0;y<=4;y+=.25){
    for (double x=0;x<=4;x+=.25){
      int i=x*4, j=y*4;
      fprintf(f,"(%f,%f): %f %f %f %f\n",x,y,Q[i][j][0],Q[i][j][1],
      Q[i][j][2],Q[i][j][3]);
    }
  }
  fclose(f);
}

int main(int argc, char** argv){
  cout<<"Reading Qtable...\n";
  ifstream f;
  f.open("/home/isler/Abhi/catkin_ws/src/vrep-learning-env/src/Qtable.txt");
  for (int y=0;y<17;y++){
    for (int x=0;x<17;x++){
      string line, dum;
      f>>dum>>Q[x][y][0]>>Q[x][y][1]>>Q[x][y][2]>>Q[x][y][3];
    }
  }
  f.close();
  store_table();
}
