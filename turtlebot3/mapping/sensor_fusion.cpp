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
                    //    PCL list -> Actual coordinate_local seen at a thresholded distances
					//    map Topic 2d which will provide us the current map with resolution and coordinate system
					//    Now, with the rotation of the current point which are visible in the current threshold add them into the map and publish that map into the new topic
// Now Published topics : /map topic -> publish the overwritten map on the map topic
// What we need to do ? - Overwrite the map according to the current position of the bot and the coordinate_local detected over the threshold
// Next we need to publish this new map over the already publishing map topics
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <turtlebot3/Points.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <bfs.h>
#include <bits/stdc++.h>
#include <ros/console.h>

// Global Variables									
turtlebot3::Points pcl_data;		// For the realtime data
float global_x, global_y;			// Global x and y coordinates
float resolution = 0.1;				// Resolution 0.1 according to the gmapping
int  global_points[96][96];
int global_data1[96*96]; 

/*
  Function      :  Current Map
  Arguments     :  Message of type nav_msgs/Occupancy Grid by subscibing to the map topic
  Functionality :  Get's the Message of the Map topic and stores it in the Map data
  Returns       :  None
*/

void currentMap1(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  std::cout<<"Subscribed map\n";
  rows = msg->info.width;
  columns = msg->info.height;
  for(int i = 0 ; i < rows*columns ; i++){
    
    global_data1[i] = (msg->data[i]);
  } 
} 


/*
	Function    :   determinantOfMatrix()
	Arguments   :   a 3x3 matrix with all the elements in it
	type        :   double[][]
	Returns     :   Determinant of the given 2d matrix 
*/

void make1D()
{
	for(int i = 0 ; i < 96 ; i ++){
		for (int j = 0 ; j < 96 ; j ++){
			global_data1[ j + i*96 ] = global_points[i][j] + global_data1[j + i*96];
		}
	}
}

/*
	Function    :   matmul()
	Arguments   :   a 3x3 matrix with all the elements in it and a 3x1 matrix
	type        :   double[][]
	Returns     :   resultant array with multiplied matrices
*/
double* matmul(double MatA[3][3], double MatB[3]){
	double *ans = (double*)malloc(3*sizeof(double));

	for(int i = 0 ; i < 3 ; i++){
		ans[i] = 0;
		for(int j = 0 ; j < 3 ; j++){
			ans[i] += MatA[i][j] * MatB[j];
		}
	}

	return ans;
}
/*	
	Function 	: OakdSub
	Arguments	: Array of coordinate_local through geometry_msgs
	Funtionality: Just to read the data and store it
*/
void oakd_sub(const turtlebot3::Points::ConstPtr &msg){
	std::cout<<"Subscription was performed\n";
	pcl_data.points = msg->points;			// Subscriber to get the current data
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "sensor_fusion");
	ros::NodeHandle n;

	ros::Subscriber odo_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, get_position);
	ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, currentMap1);
	ros::Subscriber pcl_sub = n.subscribe<turtlebot3::Points>("/oakd/pcl", 1000, oakd_sub);

	ros::Publisher  new_map = n.advertise<nav_msgs::OccupancyGrid>("/new_map", 1000);
	
    ros::Rate r(10);

    while(ros::ok()){

		// Globally filled points are given value 1 and surrounding 9 pixels around it are also marked as 1 accounting for the inflation
		double coordinate_local[3], global_coordinates[3];		
		double *global;

		// Find at what index the point occurs but before that find the point
		std::cout<<odom_sub_count<<"\n";
		for(auto &i : pcl_data.points){
			if(odom_sub_count > 1){

				coordinate_local[0] = i.x;
				coordinate_local[1] = i.y;
				coordinate_local[2] = i.z;

				double rotation_matrix[3][3] = {{ std::cos(curr_theta), -std::sin(curr_theta), 	0} \
											 ,  { std::sin(curr_theta),  std::cos(curr_theta), 	0} \
											 ,  {   		0		 ,  		  0	 	    ,  	1}};

				global = matmul(rotation_matrix, coordinate_local);
				
				// These points are w.r.t the global frame
				global_coordinates[0] = global[0] + curr_x;	
				global_coordinates[1] = global[1] + curr_y;	
				global_coordinates[2] = global[2];

				//Print
				std::cout<<global_coordinates[0]<<std::endl;

				// Freeing heap allotment
				free(global);
				
				int index_m = round(global_coordinates[0]) / resolution;
				int index_n = round(global_coordinates[1]) / resolution;
				
				// Marking all the filled points / detected points in the local map 
				for ( int i = index_m - 1 ; i <= index_m + 1 ; i ++ ){
					for ( int j = index_n - 1 ; j <= index_n + 1 ; j++ ){
						if( i < rows && i >=0 && j < columns && j >= 0){
							global_points[i][j] = 1;
						}
						std::cout<<global_points[i][j]<<std::endl;
					}
				}
			}
		}

		make1D();

		nav_msgs::OccupancyGrid map_1D;

		std::cout<<"This should be working\n"<<std::endl;
		std::vector<signed char> a(global_data1, global_data1 + 96*96);

		map_1D.info.resolution = 0.1;
		map_1D.info.width = columns;
		map_1D.info.height = rows;
		map_1D.data = a; 

		new_map.publish(map_1D);

        ros::spinOnce();
        r.sleep();
	}

}