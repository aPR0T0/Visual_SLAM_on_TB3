#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "std_msgs/Int8MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector.h"
#include "bits/stdc++.h"

int n1;
int m;
int data1[9216];

long data_2d[96][96];
int x, y, z, w;

//--------------------------------------------------------------------------//
    /*Node for BFS*/
class node{
  public:
  std::pair<int,int> parent; // This stores index of the parent w.r.t 2d map obtained
  std::vector<std::pair<int,int>> children; // This stores the indices of all the "unoccupied children" of the current node
  node(){
    parent.first = NULL;
    parent.second = NULL;
  }

  // giving indices to the parent of the given class
  node(int i, int j){
    parent.first = i;
    parent.second = j;
  }

  void set_child(int i, int j){
    children.push_back(std::make_pair(i,j));
  }
};
//--------------------------------------------------------------------------//

//--------------------------------------------------------------------------//
    /*Functions with one time use*/

void currentMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

  n1 = msg->info.width;
  m = msg->info.height;
  for(int i = 0 ; i < n1*m ; i++){
    
    data1[i] = (msg->data[i]);

  }
}
//------------------------------------------------------------------------// 

void get_2d_map(int data1[]){
  for(int i = 0; i < n1 ; i++){
      for(int j = 0; j < (m-1) ; j++){
        data_2d[i][j] = data1[i*n1+j+1];
      }
  }
}

//--------------------------------------------------------------------------//


//--------------------------------------------------------------------------//
    /*Functions for distance and pixel indices conversion here*/
//--------------------------------------------------------------------------//

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_interpretor");

  ros::NodeHandle n;
  ros::Subscriber subscriber = n.subscribe("map",1000, currentMap);
  ros::Rate rate(10);
  
  int count = 0;
  while (ros::ok())
  {
    ros::Duration current_time(ros::Time::now().toSec()); /* current time*/

    get_2d_map(data1); //got the 2d map to operate upon

//--------------------------------------------------------------------------//
    /*Apply BFS here*/
//--------------------------------------------------------------------------//

    ros::spinOnce();

    rate.sleep();
    ++count;
  }


  return 0;
}