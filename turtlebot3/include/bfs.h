#ifndef BFS_HPP
#define BFS_HPP

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>
#include<turtlebot3/Points.h>
#include<bits/stdc++.h>

void currentMap(const nav_msgs::OccupancyGrid::ConstPtr &);
void get_position(const nav_msgs::OdometryPtr& );
std::pair<int,int> distance_to_pixel(float , float );
std::pair<double,double> pixel_to_distance(int , int );
void BFS_stack_builder(int, int);
turtlebot3::Points bfs(int , int , int , int , float , std::vector<int>);

#endif

