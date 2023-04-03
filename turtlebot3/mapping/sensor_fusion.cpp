/*
 * This Program will help converging the data 
 * coming from Oak-D and LiDar from TB3 module
 * The output of the LiDar Map will be post - processed here
 * In Post processing the bounds of the height of the obstacle will be considered
 * Now, with the help of the current data we will be making a local map
 * Now, this map will have the range of 1m - > for more accuracy
 * It has FOV = 50°, Assumption according to the data sheet
 */

// Now Subsribed topics : IMU -> x, y, and z of the bot
                    //    PCL list -> Actual points seen at a thresholded distances
					//    map Topic 2d which will provide us the current map with resolution and coordinate system
					//    Now, with the rotation of the current point which are visible in the current threshold add them into the map and publish that map into the new topic
// Now Published topics : /map topic -> publish the overwritten map on the map topic
// What we need to do ? - Overwrite the map according to the current position of the bot and the points detected over the threshold
// Next we need to publish this new map over the already publishing map topics
#include <bfs.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

// Global Variables									
turtlebot3::Points pcl_data;		// For the realtime data

/*	
	Function 	: OakdSub
	Arguments	: Array of points through geometry_msgs
	Funtionality: Just to read the data and store it
*/
void oakd_sub(turtlebot3::Points::ConstPtr &msg){
	pcl_data = msg->points;			// Subscriber to get the current data
}

int main(int argc, char *argv[])
{
	ros::init("sensor_fusion");
	ros::NodeHandle n;

	ros::Subscriber odo_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, get_position);
	ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, currentMap);
	ros::Subscriber pcl_sub = n.subscribe<turtlebot3::Points>("/oakd/pcl", 1000, oakd_sub);

	ros::Publisher new_map = n.advertise<nav_msgs::OccupancyGrid>("/new_map", 1000);
	    
    ros::Rate r(10);

	float rotation_matrix[3][3] = {{ std::cos(curr_theta), -std::sin(curr_theta), 	0}\
								,  { std::sin(curr_theta),  std::cos(curr_theta), 	0}\
								,  {   			0		 ,  		  0	 	    ,  	1}};

    while(ros::ok()){
		
		
        ros::spinOnce();
        r.sleep();
	}

}