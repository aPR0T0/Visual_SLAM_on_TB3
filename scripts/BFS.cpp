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
int curr_x, curr_y, curr_theta;
int data1[9216];

long data_2d[96][96];
int x, y, z, w;

//--------------------------------------------------------------------------//
                              /*Node for BFS*/
class Node{
  public:
  std::pair<int,int> parent; // This stores index of the parent w.r.t 2d map obtained
  std::vector<std::pair<int,int>> children; // This stores the indices of all the "unoccupied children" of the current Node
  Node(){
    parent.first  = -1;
    parent.second = -1;
  }

  // giving indices to the parent of the given class
  Node(int i, int j){
    parent.first  = i;
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

/* We need the odometry data to know where exactly are we present in the map*/
void get_position(const nav_msgs::OdometryPtr& msg){
  curr_x = msg->pose.pose.position.x;
  curr_y = msg->pose.pose.position.y;
}

//--------------------------------------------------------------------------// 

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
std::pair<int,int> distance_to_pixel(int distance_x, int distance_y){
  // 1 pixel = 0.1 meter
  std::pair<int,int> indices;

  indices.first  = (distance_x + 5)/0.1 ;// origin of the map is position (-5,-5)
  indices.second = (distance_y + 5)/0.1 ;// origin of the map is position (-5,-5)

  return indices;
}
//--------------------------------------------------------------------------//

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_interpretor");

  ros::NodeHandle n;
  // Subscribing to map topic where all the data of the map is coming from
  ros::Subscriber subscriber = n.subscribe("map",1000, currentMap);
  // Subscribing to odometry topic to get the pose estimate
  ros::Subscriber odo_sub = n.subscribe("odom", 1000, get_position);
  ros::Rate rate(10);
  
  int count = 0;
  while (ros::ok())
  {
    ros::Duration current_time(ros::Time::now().toSec()); /* current time*/
    get_2d_map(data1); //got the 2d map to operate upon

    // Initializing the node root which is going to be the first and the only node with no parent
    std::pair<int,int> u; // pixels of the current grid cell
    u = distance_to_pixel(curr_x, curr_y);
    Node* root = new Node(u.first, u.second);
//--------------------------------------------------------------------------//
    /*Apply BFS here*/
//--------------------------------------------------------------------------//

    ros::spinOnce();

    rate.sleep();
    ++count;
  }


  return 0;
}