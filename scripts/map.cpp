#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"

int n1;
int m;
int data[147456];
int x, y, z, w;
void currentMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

  n1 = msg->info.width;
  m = msg->info.height;
  for(int i = 0 ; i < n1 ; i++){
    data[i] = msg->data[i];
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_interpretor");

  ros::NodeHandle n;

  int data[147456];
  ros::Subscriber subscriber = n.subscribe("map",1000, currentMap);
  ros::Rate rate(10);
  
  int count = 0;
  while (ros::ok())
  {
    ros::Duration current_time(ros::Time::now().toSec()); /* current time*/
    int data_2d[n1][m];

    for(int i = 0; i < n1 ; i++){
      for(int j = 0; j < m ; j++){
        data_2d[i][j] = data[j];
      }
    }

    for(int i = 0; i < n1; i++){
      
      for(int j = 0; j < m; j++){
        ROS_INFO("READING: %d", data_2d[i][j]);
      }
      
    }
    ros::spinOnce();

    rate.sleep();
    ++count;
  }


  return 0;
}