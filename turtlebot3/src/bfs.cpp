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
<<<<<<< HEAD
#include "bfs.h"
=======
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5

// #################################################### //

// ## Pre-initialzed Variables uses for heights and Lengths of the map ## //

int rows = 96;
int columns = 96;
int data1[96*96];

// ###################################################################### //


// ################# Position based Variables ########################### //

float curr_x, curr_y, curr_theta;
float des_x, des_y, des_theta;

// ###################################################################### //


// ########################## Counters ############################## //

int odom_sub_count = 0;        // Keeps track whether the odometry topic is subscribed or not
bool final_index_found = 0;    // Keeps track whether the final index is found or not
int path_pub_count = 0;        // Keeps track whether the path has already been created or not
int tar_sub_count = 0 ;        // Keeps track whether the final index is available or not

// ################################################################## //


// ################################ Important Data Types ########################################### //

std::vector<std::pair<int,int>> path; // This is going to be the path we traversed

std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> layer; // Will be storing parent and the current node together in the single unvisited layer

std::stack<std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>>> stack_layer;

int visited[96][96]={-1}; // Marking all nodes as unvisited

// ################################################################################################## //


// ################################ Functions for One time Use ###################################### //

/*
  Function      :  Current Map
  Arguments     :  Message of type nav_msgs/Occupancy Grid by subscibing to the map topic
  Functionality :  Get's the Message of the Map topic and stores it in the Map data
  Returns       :  None
*/
void currentMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

  rows = msg->info.width;
  columns = msg->info.height;
  for(int i = 0 ; i < rows*columns ; i++){
    
    data1[i] = (msg->data[i]);
  } 
} 

/*
  Function      :  get position
  Arguments     :  Message of tupe nav_msgs/Odometry by subscribing to the "/odom" Topic
  Functionality :  Stores the Current position of the bot in the curr_x and curr_y
  Returns       :  None
*/
void get_position(const nav_msgs::OdometryPtr& msg){
  odom_sub_count++;
  curr_x = msg->pose.pose.position.x;
  curr_y = msg->pose.pose.position.y;
}

/*
  Function      :  get 2d position
  Arguments     :  map in the form of 'array - 1D' 
  Functionality :  Converts 1D to a 2D array for better post-processing
  Returns       :  None
*/
void get_2d_map(int data1[]){

  for(int i = 0; i < rows ; i++){
    for(int j = 0; j < (columns-1) ; j++){
      visited[i][j] = data1[i*rows+j+1];
    }
  }

}

/*
  Function      :   distance to pixels
  Arguments     :   1. Float distance in x
                    2. Float distance in y
  Functionality :   Converts distance in meters to pixel indices according to the resolution of the map
  Returns       :   A pair of integers know as "Indices" 
*/
std::pair<int,int> distance_to_pixel(float distance_x, float distance_y){
  // 1 pixel = 0.1 meter
  std::pair<int,int> indices;

  indices.first  = (distance_x + 5)/0.1 ;// origin of the map is position (-5,-5)
  indices.second = (distance_y + 5)/0.1 ;// origin of the map is position (-5,-5)

  return indices;
}

/*
  Function      :   Pixels to Distance
  Arguments     :   1. integer index in rows
                    2. integer index in columns
  Functionality :   Converts  pixel indices to distance in meters according to the resolution of the map
  Returns       :   A pair of "doubles" know as "Co-ordinates" 
*/
std::pair<double,double> pixel_to_distance(int index_1, int index_2){

  std::pair<double,double> pixels;
  
  pixels.first = index_1*0.1 - 5;
  pixels.second = index_2*0.1 - 5;

  return pixels;
}

/*
  Function      :   Layer with the help of BFS
  Arguments     :   1. integer index in rows
                    2. integer index in columns
  Functionality :   Keeps a track of all the unvisited neighbours in the form of layer
  Returns       :   None 
*/                 
void BFS_stack_builder(int i, int  j){

  visited[i][j] = 1; // Marking current node as visited too

// East
  if(visited[i+1][j] == 0){  

    if( i+1 < rows ){ // Seeing whether the index is even feasible or no
<<<<<<< HEAD
      layer.push_back({{i,j},{i+1,j}}); // Parent is go ing to be its current node
=======
      layer.push_back({{i,j},{i+1,j}}); // Parent is going to be its current node
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
      visited[i+1][j] = 1;
    }

  }

<<<<<<< HEAD
// South-East
  if(visited[i+1][j+1] == 0){  

    if( i+1 < rows and j+1 < columns ){ // Seeing whether the index is even feasible or no
      layer.push_back({{i,j},{i+1,j+1}}); // Parent is go ing to be its current node
      visited[i+1][j+1] = 1;
    }

  }

// North-East
  if(visited[i+1][j-1] == 0){  

    if( i+1 < rows and j-1 >= 0 ){ // Seeing whether the index is even feasible or no
      layer.push_back({{i,j},{i+1,j-1}}); // Parent is go ing to be its current node
      visited[i+1][j-1] = 1;
    }

  }

=======
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
// South
  if(visited[i][j+1] == 0){
    
    if( j+1 < columns ){
      layer.push_back({{i,j}, {i, j+1}}); // Parent is going to be its current node
      visited[i][j+1] = 1;
    }

  }

// West
  if(visited[i-1][j] == 0 ){
    
    if( i-1 >= 0 ){
      layer.push_back({{i,j},{i-1,j}});
      visited[i-1][j] = 1;
    }

  }

<<<<<<< HEAD
// North-West
  if(visited[i-1][j-1] == 0 ){
    
    if( i-1 >= 0 and j-1 >= 0 ){
      layer.push_back({{i,j},{i-1,j-1}});
      visited[i-1][j-1] = 1;
    }

  }

// South-West
  if(visited[i-1][j+1] == 0 ){
    
    if( i-1 >= 0 and j+1 >= 0 ){
      layer.push_back({{i,j},{i-1,j+1}});
      visited[i-1][j+1] = 1;
    }

  }

=======
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
// North
  if(visited[i][j-1] == 0){
    
    if( j-1 >= 0 ){
      layer.push_back({{i,j},{i,j-1}});
      visited[i][j-1] = 1;
    }

  }
}

/*
  Function      :   Tree Building with the help of BFS layer builder
  Arguments     :   1. integer index in counter
                    2. integer index in columns
  Functionality :   Keeps a track of all the unvisited neighbours in the form of layer
  Returns       :   None 
*/   
std::pair<int,int> build_a_tree( int layer_element_count, int flag, std::pair<int,int> final_index){

  std::pair<int,int> indx;        // Current index 
  std::pair<int,int> indx_parent; // Current indexs' Parents

  while(!layer.empty()){ 

    indx = layer.front().second; 

    if(indx == final_index){

      final_index_found = 1;
      indx_parent = layer.front().first;
      break;

    }

    layer.erase(layer.begin()); // This will clear the whole layer when the whole for loop is done traversing without deleting the newly added elements of the next layer
    
    BFS_stack_builder(indx.first, indx.second); // Now building the graph further
    
    if(layer_element_count == flag){

      flag = layer.size();
      layer_element_count = 0;
      stack_layer.push(layer);

    } 

    layer_element_count++;

  } 

  return indx_parent;
}

/*
  Function      :   Build a from stack
  Arguments     :   1. Pair of integers for the final index
                    2. pair of integers for the current index's parent
  Functionality :   Build a path
  Returns       :   None 
*/   
void build_path_from_stack(std::pair<int,int> final_index, std::pair<int,int> indx_parent){

<<<<<<< HEAD
  if(path_pub_count >= 0){ // Cheching whether the path has been previously built or not

    path.push_back(final_index); // Pushing the Final Index
    stack_layer.pop();
    path.clear();
=======
  if(path_pub_count == 0){ // Cheching whether the path has been previously built or not

    path.push_back(final_index); // Pushing the Final Index
    stack_layer.pop();

>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
    std::cout<< "Final index was found\n"; // Just to know whether we have been successful or not
    std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> templayer; // This layer just stores the top of the stack elements

    while(!stack_layer.empty()){ // Till we reach the initial index

      std::cout<<"indx parent : "<<indx_parent.first<< " " << indx_parent.second<<std::endl;
      templayer = stack_layer.top();

      while(templayer.front().second != indx_parent and !templayer.empty()){

        templayer.erase(templayer.begin()); // Just a type of searching

      } 

      path.push_back(templayer.front().second);
      indx_parent = templayer.front().first;
      stack_layer.pop();

    }
  }
<<<<<<< HEAD
  layer.clear();
  path_pub_count++;

}

/*
  Function      :   Make a Plan 
  Arguments     :   start index, end index, width, height, resolution
  Functionality :   Implement BFS
  Returns       :   Path
*/

turtlebot3::Points bfs(int start_index, int end_index, int width, int height, float resolution, std::vector<int> costmap){

  ROS_INFO("+-----------------+ +++++++ +-------------------+\n");
  ROS_INFO("+-----------------+ Running +-------------------+\n");
  ROS_INFO("+-----------------+ +++++++ +-------------------+\n");

  std::pair<int,int> initial_index, final_index; // pixels of the current grid cell


  final_index.first    = (  end_index / width  ); // Need to verify this
  final_index.second   = (  end_index % height );

  initial_index.first  = ( start_index / width );
  initial_index.second = ( start_index % width );

  ROS_INFO("end : %d   , end : %d\n", final_index.first, final_index.second);
  ROS_INFO("start : %d , start : %d\n", initial_index.first, initial_index.second);
  
  get_2d_map(data1); //got the 2d map to operate upon

  turtlebot3::Points P_msg; // This is going to be the final published path
  geometry_msgs::Point  point_pub;

  if(odom_sub_count > 0){ // Just checking if the subscription is done or not

    visited[initial_index.first][initial_index.second] = 1; // Marking the Current node as visited

    layer.push_back({{-1,-1},{initial_index.first,initial_index.second}});  // Initialzing layer0 with the First node later the elements will be added into it
    stack_layer.push(layer);  // Now, Stack is a collection of all layers till the final index

    int flag = layer.size()-1; // Flag is the signal that all the possible children in the current array has been added and now u can move on to the next layer

    int layer_element_count = 0; // this will check if all the elements in the layer has been traversed or not

    // ############################################################################################### //
    std::pair<int,int> indx_parent;
    indx_parent = build_a_tree(flag, layer_element_count, final_index); // This will give us the parent of the final node 

    if(final_index_found){

      build_path_from_stack(final_index, indx_parent);

    } 
    else{
      return P_msg;
    }
    // Now as there is some path existing 

  }

  // ############### Converting the path in terms of the message that we are going to publish ############# //
  int x_t = path.size();

  geometry_msgs::Point path_pub[x_t];

  for(int i = 0; i < path.size() ; i++){
    std::pair<double,double> path_temp;
    path_temp = pixel_to_distance(path[i].first, path[i].second);
    
    point_pub.x = path_temp.first;
    point_pub.y = path_temp.second;
    path_pub[i] = point_pub; 
  }

  P_msg.points.clear();
  P_msg.end_index = des_theta;
  int index_z = 0;
  // Now declaring the messages with the help of the points
  for (auto &it : path_pub) {
      geometry_msgs::Point point;
      point.x = (it).x;
      point.y = (it).y;
      point.z = 0;
      P_msg.points.push_back(point);
      index_z++;
  }
  return P_msg;
}
=======

  path_pub_count++;

  for(auto &xt: path){
    std::cout<<xt.first<<" "<<xt.second<<std::endl;
  }
}


// ##################################################################################################### //
// #####################################     Main Function    ########################################## //
// ##################################################################################################### //

int main(int argc, char **argv) {
  
  // ########################## Creating Ros nodes, Subsciber, and Publishers ########################## //
  ros::init(argc, argv, "map_interpretor");
  ros::NodeHandle n;      // Creating a nodehandler to make relevant subscribers and publishers

  ros::Subscriber subscriber = n.subscribe("map",1000, currentMap);   // Subscribing to map topic
  ros::Subscriber odo_sub = n.subscribe("odom", 1000, get_position);  // Subscribing to odom topic

  ros::Publisher path_publisher = n.advertise<turtlebot3::Points>("path_sub", 1000); // Creating a Publishing node for the path
  ros::Rate rate(1);   // Defining a rate for overall ros operations
  int count = 0;

  // Getting the target from the user
  std::cout<<"Please Enter the Location in RVIZ and put the orientation here : \n"<<std::endl;
  std::cin>>des_x>>des_y;
  std::cin>>des_theta;

  tar_sub_count++;

  while (ros::ok())
  {

    get_2d_map(data1); //got the 2d map to operate upon
    std::pair<int,int> initial_index, final_index; // pixels of the current grid cell
    turtlebot3::Points P_msg; // This is going to be the final published path
    geometry_msgs::Point  point_pub;

    if(odom_sub_count > 0 and tar_sub_count > 0){ // Just checking if the subscription is done or not

      initial_index = distance_to_pixel(curr_x, curr_y);
      final_index = distance_to_pixel(des_x, des_y);
      ROS_INFO("START x %d START y %d, END x %d END y %d\n", initial_index.first, initial_index.second, final_index.first, final_index.second);
      visited[initial_index.first][initial_index.second] = 1; // Marking the Current node as visited

      layer.push_back({{-1,-1},{initial_index.first,initial_index.second}});  // Initialzing layer0 with the First node later the elements will be added into it
      stack_layer.push(layer);  // Now, Stack is a collection of all layers till the final index

      int flag = layer.size()-1; // Flag is the signal that all the possible children in the current array has been added and now u can move on to the next layer

      int layer_element_count = 0; // this will check if all the elements in the layer has been traversed or not

      // ############################################################################################### //
      std::pair<int,int> indx_parent;
      indx_parent = build_a_tree(flag, layer_element_count, final_index); // This will give us the parent of the final node 

      if(final_index_found){

        build_path_from_stack(final_index, indx_parent);

      } 
      else{
        std::cout<<"No path Exists"<<std::endl;
        break;
      }
      // Now as there is some path existing 

    }

    // ############### Converting the path in terms of the message that we are going to publish ############# //
    int x_t = path.size();
    std::cout<<"flag: "<<x_t<<std::endl;
    geometry_msgs::Point path_pub[x_t];

    for(int i = 0; i < path.size() ; i++){
      std::pair<double,double> path_temp;
      path_temp = pixel_to_distance(path[i].first, path[i].second);
      
      point_pub.x = path_temp.first;
      point_pub.y = path_temp.second;
      path_pub[i] = point_pub; 
    }

    P_msg.points.clear();
    P_msg.end_index = des_theta;
    int index_z = 0;
    // Now declaring the messages with the help of the points
    for (auto &it : path_pub) {
        geometry_msgs::Point point;
        point.x = (it).x;
        point.y = (it).y;
        point.z = 0;
        P_msg.points.push_back(point);
        index_z++;
    }
    for(int j = 0; j<path.size() ; j++){
        std::cout<<"x : "<<P_msg.points[j].x<<" y : "<<P_msg.points[j].y<<std::endl;
    }
  

    path_publisher.publish(P_msg);
    ros::spinOnce();

    rate.sleep();
    ++count;
  }
  return 0;
}

// ############################################################################################## //
// #######################################  The End  ############################################ //
// ############################################################################################## //
>>>>>>> b7282c4a9c56b228f485a3bc1795d231c8b17ff5
