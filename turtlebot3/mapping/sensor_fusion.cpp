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
#include <bfs.h>
#include <stdio.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

// Global Variables									
turtlebot3::Points pcl_data;		// For the realtime data
float global_x, global_y;			// Global x and y coordinates
float resolution = 0.1;				// Resolution 0.1 according to the gmapping
int global_points_1D[96*96];		// creating a 1d array for publishing it to the new map topic

/*
	Function    :   determinantOfMatrix()
	Arguments   :   a 3x3 matrix with all the elements in it
	type        :   double[][]
	Returns     :   Determinant of the given 2d matrix 
*/

double make1D(int global_points[][])
{
	for(int i = 0 ; i < 96 ; i ++){
		for (int j = 0 ; j < 96 ; j ++){
			global_points_1D[ j + i*96 ] = global_points[i][j];
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
	double *ans = malloc(3*sizeof(double));

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
void oakd_sub(turtlebot3::Points::ConstPtr &msg){
	pcl_data = msg->coordinate_local;			// Subscriber to get the current data
}

int main(int argc, char *argv[])
{
	ros::init("sensor_fusion");
	ros::NodeHandle n;

	ros::Subscriber odo_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, get_position);
	ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, currentMap);
	ros::Subscriber pcl_sub = n.subscribe<turtlebot3::Points>("/oakd/pcl", 1000, oakd_sub);

	ros::Publisher  new_map = n.advertise<nav_msgs::OccupancyGrid>("/new_map", 1000);
	
    ros::Rate r(10);

	float rotation_matrix[3][3] = {{ std::cos(curr_theta), -std::sin(curr_theta), 	0}\
								,  { std::sin(curr_theta),  std::cos(curr_theta), 	0}\
								,  {   			0		 ,  		  0	 	    ,  	1}};

    while(ros::ok()){
		double coordinate_local[3], global_coordinates[3];		// Globally filled points are given value 1 and surrounding 9 pixels around it are also marked as 1 accounting for the inflation
		int  global_points[96][96] = {0};
		double *global;
		// Find at what index the point occurs but before that find the point
		for(auto &i : pcl_data){

			coordinate_local[0] = i.x;
			coordinate_local[1] = i.y;
			coordinate_local[2] = i.z;

			global = matmul(rotation_matrix, coordinate_local);
			
			// These points are w.r.t the global frame
			global_coordinates[0] = global[0] + curr_x;	
			global_coordinates[1] = global[1] + curr_y;	
			global_coordinates[2] = global[2];
			// Freeing heap allotment
			free(global);
			
			int index_m = round(global_coordinates[0]) / resolution;
			int index_n = round(global_coordinates[1]) / resolution;
			
			// Marking all the filled points / detected points in the local map 
			for ( int i = index_m - 1 ; i <= index_m + 1 ; i ++ ){
				for ( int j = index_n - 1 ; j <= index_n + 1 ; j++ ){
					global_points[i][j] = 1;
				}
			}
		}

		make1D(global_points);

		nav_msgs::OccupancyGrid map_1D;
		std::vector<signed char> a(global_points_1D, global_points_1D + 96*96);

		map_1D.data = a;

		new_map.publish(map_1D);

        ros::spinOnce();
        r.sleep();
	}

}