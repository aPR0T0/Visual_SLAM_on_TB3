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
int des_x, des_y, des_theta;
int data1[9216];

long data_2d[96][96];
int x, y, z, w;
std::queue<std::pair<int,int>> Que;
int visited[96][96]={-1}; // Marking all nodes as unvisited
// -1 -> univisited
//  1 -> visited

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

std::pair<int,int> pixel_to_distance(int index_1, int index_2){
  std::pair<int,int> pixels;
  
  pixels.first = index_1*0.1 - 5;
  pixels.second = index_2*0.1 - 5;

  return pixels;
}
//--------------------------------------------------------------------------//

//--------------------------------------------------------------------------//


                      /*BFS Function 
                        Arguments:
                                  Nodes
                        Returns: 
                                  Nothing
                        Functionality:
                                  Makes the graph for the BFS
                      */

                    
void BFS_graph_builder(Node* &root, int i, int  j){
  // Now we will be checking each free node which is in the-
  // neighbourhood of range 1 pixel in the environment.
  // if(i+1 > n1 || j+1 > m){
      //This condition will be used later
  // } 
  visited[i][j] = 1; // Marking current node as visited too

  if(data_2d[i+1][j] == 0 && visited[i+1][j] == -1){
    //Setting a parent
    Node* child = new Node({i+1, j});
    child->parent = {i, j};
    //Setting a Child
    root->children.push_back({i+1,j});
    Que.push({i+1,j});
    visited[i+1][j] = 1;
  }

  if(data_2d[i][j+1] == 0 && visited[i][j+1] == -1){

    Node* child = new Node({i, j+1});
    child->parent = {i, j};

    root->children.push_back({i,j+1});
    Que.push({i,j+1});
    visited[i][j+1] = 1;
  }

  if(data_2d[i+1][j+1] == 0 && visited[i+1][j+1] == -1){

    Node* child = new Node({i+1, j+1});
    child->parent = {i, j};

    root->children.push_back({i+1,j+1});
    Que.push({i+1,j+1});
    visited[i+1][j+1] = 1;
  }

  if(data_2d[i-1][j+1] == 0 && visited[i][j+1] == -1){

    Node* child = new Node({i-1, j+1});
    child->parent = {i, j};

    root->children.push_back({i-1,j+1});
    Que.push({i-1,j+1});
    visited[i][j+1] = 1;
  }

  if(data_2d[i-1][j] == 0 && visited[i-1][j] == -1){

    Node* child = new Node({i-1, j});
    child->parent = {i, j};

    root->children.push_back({i-1,j});
    Que.push({i-1,j});
    visited[i-1][j] = 1;
  }

  if(data_2d[i][j-1] == 0 && visited[i][j-1] == -1){

    Node* child = new Node({i, j-1});
    child->parent = {i, j};

    root->children.push_back({i,j-1});
    Que.push({i,j-1});
    visited[i][j-1] = 1;
  }

  if(data_2d[i-1][j-1] == 0 && visited[i-1][j-1] == -1){

    Node* child = new Node({i-1, j-1});
    child->parent = {i, j};

    root->children.push_back({i-1,j-1});
    Que.push({i-1,j-1});
    visited[i-1][j-1] = 1;
  }

  if(data_2d[i+1][j-1] == 0 && visited[i+1][j-1] == -1){

    Node* child = new Node({i+1, j-1});
    child->parent = {i, j};

    root->children.push_back({i+1,j-1});
    Que.push({i+1,j-1});
    visited[i+1][j-1] = 1;
  }
}
//--------------------------------------------------------------------------//

//--------------------------------------------------------------------------//
                        // ## Returning Path ## //
std::vector<std::pair<int,int>> return_path(std::pair<int,int> final_index, std::pair<int,int> initial_index, Node* root){
  std::vector<std::pair<int,int>> path;
  std::pair<int,int> parentx;
  
  if(final_index.first == initial_index.first && final_index.second == initial_index.second){
    path.push_back(initial_index);
    return path;
  }
  else if(final_index.first < 0 && final_index.second < 0){
    path.clear();
    path.push_back({-1,-1});
    return path;
  }
  parentx = root->parent;
  root = new Node(root->parent.first, root->parent.second); // Now going back to the parent
  path.push_back(parentx);
  return_path(final_index, parentx, root);
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
    std::pair<int,int> u, v; // pixels of the current grid cell
    std::vector<std::pair<int,int>> path;
    
    u = distance_to_pixel(curr_x, curr_y);
    v = distance_to_pixel(des_x, des_y);
    Node* root = new Node(u.first, u.second);
     // Marking current node as visited

//--------------------------------------------------------------------------//
    /*BFS Function declared Up there
      Arguments:
                Nodes
      Returns: 
                Nothing
      Functionality:
                Makes the graph for the BFS
    */
    BFS_graph_builder(root, u.first, u.second);
    // Now all the neighbouring nodes are added to the children vector in the current node

    // Now let's check if the queue is empty or not
    if(!Que.empty()){
      
      std::pair<int,int> indx = Que.front();
      Que.pop(); // Now marking current element as visited
      
      if(indx.first == v.first && indx.second == v.second){// This means the goal is reached
        path = return_path(v, u, root);
      }

      else if(indx.first > v.first && indx.second > v.second){
        path = return_path({-1,-1}, u, root); // This will return no path 
      }

      Node* root = new Node(indx.first, indx.second);// Now going to the First Child
      
      BFS_graph_builder(root, indx.first, indx.second); // Now building the graph further

    } 

//--------------------------------------------------------------------------//

    ros::spinOnce();

    rate.sleep();
    ++count;
  }


  return 0;
}


/*
// 

  ####################################### Algorithm for the Code #####################################

//
*/