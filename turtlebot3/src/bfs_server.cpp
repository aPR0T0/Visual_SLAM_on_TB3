// ############### Including headers ################### //

#include "tf/tf.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8MultiArray.h"
#include "pp_msgs/PathPlanningPlugin.h"
#include "pp_msgs/PathPlanningPluginResponse.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "turtlebot3/Points.h"
#include "bits/stdc++.h"
#include "algorithm"
#include "bfs.h"

// #################################################### //

/*
  Function      :   makePlan - A standard node for different algorithms
  Arguments     :   Service Request, Service Response
  Funcitonality :   Publish a path for move_base
  Returns       :   bool that describes whether path is beign published or not
*/
bool make_plan(pp_msgs::PathPlanningPlugin::Request &req, pp_msgs::PathPlanningPlugin::Response &res){
  std::vector<int> costmap;
  std::vector<int> path_bfs;
  costmap = req.costmap_ros;

  int width = req.width;
  int height = req.height;

  int start_index = req.start;
  int end_index   = req.goal;
  float resolution = 0.1;

  turtlebot3::Points path_from_bfs = bfs(start_index, end_index, width, height, resolution, costmap);

  if(path_from_bfs.points.size() == 0){
    ROS_INFO("!!! NO PATH FOUND !!!\n");
    return false;
  }
  else{
    ROS_INFO("!!! PATH FOUND !!!\n");
  }

  path_bfs.clear();
  for (auto &itx : path_from_bfs.points ){

    std::pair<int,int> idx_2d = distance_to_pixel(itx.x, itx.y);
    ROS_INFO(" x : %d  y : %d \n", idx_2d.first, idx_2d.second);
    int idx_1d = idx_2d.first * width  +  idx_2d.second;
    path_bfs.push_back(idx_1d);

    std::reverse(path_bfs.begin(), path_bfs.end());
  } 

  res.plan = path_bfs;

  return true;
}

// ##################################################################################################### //
// #####################################     Main Function    ########################################## //
// ##################################################################################################### //

int main(int argc, char **argv) {
  
  // ########################## Creating Ros nodes, Subsciber, and Publishers ########################## //
  ros::init(argc, argv, "bfs_server");

  // Creating a nodehandler to make relevant subscribers and publishers
  ros::NodeHandle n;    
  
  ros::Subscriber subscriber = n.subscribe("map",1000, currentMap);   // Subscribing to map topic
  ros::Subscriber odo_sub = n.subscribe("odom", 1000, get_position);  // Subscribing to odom topic

  ros::ServiceServer service = n.advertiseService("/move_base/GlobalPlanner/make_plan", make_plan);

  ROS_INFO("Ready to publish the path\n");

  ros::spin();
  return 0;
}

// ###################################################################################################### //
// #############################################  The End  ############################################## //
// ###################################################################################################### //