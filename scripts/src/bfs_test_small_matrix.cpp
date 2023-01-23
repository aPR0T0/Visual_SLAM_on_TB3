#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "bits/stdc++.h"

int n1 = 5;
int m = 5;
float des_x, des_y, des_theta;
int data1[25] = {1,0,0,0,0,1,0,0,0,0,0,0,1,1,0,1,0,0,0,0,0,0,0,0,0};
const int N = 1e2;
long data_2d[5][5];
std::vector<std::pair<int,int>> path;
int x, y, z, w;
std::queue<std::pair<int,int>> Que;
int visited[N][N] = {0}; // Marking all nodes as unvisited
// -1 -> univisited
//  1 -> visited

//--------------------------------------------------------------------------//
                              /*Node for BFS*/
class Node{
  public:
  std::pair<int,int> its_indx;
  std::pair<int,int> parent; // This stores index of the parent w.r.t 2d map obtained
  std::vector<std::pair<int,int>> children; // This stores the indices of all the "unoccupied children" of the current Node

  // giving indices to the parent of the given class

  Node(int i, int j){
    its_indx.first = i;
    its_indx.second = j;
  }

  Node(int i, int j, int i1, int j1){
    its_indx.first = i1;
    its_indx.second = j1;
    parent.first  = i; // initializing as parent and children still null
    parent.second = j; // initializing as parent and children still null
    // children.clear();  // When initializing a new node there is no child
  }

  void set_child(int i, int j){
    children.push_back(std::make_pair(i,j));
  }
};
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------// 

void get_2d_map(int data1[]){
  for(int i = 0; i < n1 ; i++){
    for(int j = 0; j < m ; j++){
      data_2d[i][j] = data1[i*n1+j];
    }
  }
}


//--------------------------------------------------------------------------//


//--------------------------------------------------------------------------//
    /*Functions for distance and pixel indices conversion here*/
std::pair<int,int> distance_to_pixel(float distance_x, float distance_y){
  // 1 pixel = 0.1 meter
  std::pair<int,int> indices;
  // std::cout<<distance_x<<distance_y<<std::endl;
  indices.first  = (distance_x + 0.25)/0.1 ;// origin of the map is position (-0.25,-0.25)
  indices.second = (distance_y + 0.25)/0.1 ;// origin of the map is position (-0.25,-0.25)

  return indices;
}

std::pair<float,float> pixel_to_distance(float index_1, float index_2){
  std::pair<float,float> pixels;
  
  pixels.first  = index_1*0.1 - 0.25;  // x - axis   
  pixels.second = index_2*0.1 - 0.25;  // y - axis

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
  visited[i][j] = 1; // Marking current node as visited too

  if(data_2d[i+1][j] == 0 && visited[i+1][j] == 0){
    
    if( i+1 < n1 ){ // Seeing whether the index is even feasible or not
      //Setting a parent
      Node* child = new Node(i,j,i+1, j);
      root->children.push_back({i+1,j});
      Que.push({i+1,j});
      visited[i+1][j] = 1;
    }

  }

  if(data_2d[i][j+1] == 0 && visited[i][j+1] == 0){
    
    if( j+1 < m ){
      Node* child = new Node(i,j,i, j+1);

      root->children.push_back({i,j+1});
      Que.push({i,j+1});
      visited[i][j+1] = 1;
    }
  }

  if(data_2d[i+1][j+1] == 0 && visited[i+1][j+1] == 0){
    
    if( i+1 < n1 and j+1 < m ){
      Node* child = new Node(i,j,i+1, j+1);
      root->children.push_back({i+1,j+1});
      Que.push({i+1,j+1});
      visited[i+1][j+1] = 1;
    }

  }

  if(data_2d[i-1][j+1] == 0 && visited[i][j+1] == 0){
    
    if( i-1 >= 0 and j+1 < m){
      Node* child = new Node(i,j,i-1, j+1);
      root->children.push_back({i-1,j+1});
      Que.push({i-1,j+1});
      visited[i][j+1] = 1;
    }

  }

  if(data_2d[i-1][j] == 0 && visited[i-1][j] == 0){
    
    if( i-1 >= 0 ){
      Node* child = new Node(i,j,i-1, j);
      root->children.push_back({i-1,j});
      Que.push({i-1,j});
      visited[i-1][j] = 1;
    }
  }

  if(data_2d[i][j-1] == 0 && visited[i][j-1] == 0){
    
    if( j >= 0 ){
      Node* child = new Node(i,j,i, j-1);

      root->children.push_back({i,j-1});
      Que.push({i,j-1});
      visited[i][j-1] = 1;
    }
  }

  if(data_2d[i-1][j-1] == 0 && visited[i-1][j-1] == 0){
    
    if(i-1 >= 0 and j-1 >= 0){
      Node* child = new Node(i,j,i-1, j-1);

      root->children.push_back({i-1,j-1});
      Que.push({i-1,j-1});
      visited[i-1][j-1] = 1;
    }
  }

  if(data_2d[i+1][j-1] == 0 && visited[i+1][j-1] == 0){
    
    if( i+1 < n1 and j-1 >= 0 ){
      Node* child = new Node(i,j,i+1, j-1);
      root->children.push_back({i+1,j-1});
      Que.push({i+1,j-1});
      visited[i+1][j-1] = 1;
    }
  }
}
//--------------------------------------------------------------------------//

//--------------------------------------------------------------------------//
                        // ## Returning Path ## //
std::vector<std::pair<int,int>> return_path(std::pair<int,int> curr_index, std::pair<int,int> initial_index, Node* &root){
  std::pair<int,int> indices = root->parent;
  if(curr_index == initial_index){
    path.push_back({indices.first,indices.second});
    return path;
  }

  if(curr_index.first < 0 and curr_index.second < 0){
    path.clear();
    return path;
  }

  path.push_back({indices.first,indices.second});
  curr_index = indices;
  root = new Node(curr_index.first, curr_index.second);
  path = return_path(curr_index, initial_index, root);
}
//--------------------------------------------------------------------------//

int main() {

  int count = 0;
    // ros::Duration current_time(ros::Time::now().toSec()); /* current time*/ - > As such no need for this at this point of time
  get_2d_map(data1); //got the 2d map to operate upon
  
  for(int i = 0; i < n1 ; i++){
    for(int j = 0; j < m ; j++){
      std::cout<<data_2d[i][j];
    }
    std::cout<<std::endl;
  }
  float curr_x = -0.25, curr_y = 0.2, curr_theta = 0;
  // Initializing the node root which is going to be the first and the only node with no parent
  std::pair<int,int> u;
  std::pair<int,int> v; // pixels of the current grid cell
  std::vector<std::pair<int,int>> path; // This is going to be the path we traversed

  std::cout<<"Enter the values for desired coordinates : x  y  and orientation : Theta ";
  std::cin>>des_x>>des_y>>des_theta;

  u = distance_to_pixel(curr_x, curr_y);
  v = distance_to_pixel(des_x, des_y);

  Node* root = new Node(-1,-1,u.first,u.second); // Initial root cannot have a parent
   // Marking current node as visited
  visited[u.first][u.second] = 1;
  Que.push({u.first,u.second});

//--------------------------------------------------------------------------//
  /*BFS Function declared Up there
    Arguments:
              Nodes
    Returns: 
              Nothing
    Functionality:
              Makes the graph for the BFS
  */
  
  BFS_graph_builder(root, u.first, u.second);  // Single layer has been built
  // Now all the neighbouring nodes are added to the children vector in the current node
  // Now let's check if the queue is empty or not
  if(Que.empty()){
  std::cout<<"While loop not working";
  }
  while(!Que.empty() && path.empty()){
    
    std::pair<int,int> indx = Que.front();
    
    Que.pop(); // Now marking current element as visited
    
    if(indx.first == v.first && indx.second == v.second){// This means the goal is reached
      std::cout<< "Path was found"<<std::endl;
      return_path(v, u, root);
      std::cout<< "Path was found"<<std::endl;
      std::cout<< "Path Size"<<path.size()<<std::endl;
      return 0;
      for(int i = 0; i < path.size() ; i++){
        std::cout<< "Element: "<< i+1 <<"Indexes: "<< path[i].first<<" "<< path[i].second<<std::endl;
      }
    
    }


    else if(indx.first > v.first && indx.second > v.second){
      return_path({-1,-1}, u, root); // This will return no path 
      std::cout<< "No Path was found"<<std::endl;
      return 0;
    }

    Node* root = new Node(root->parent.first,root->parent.second,indx.first, indx.second);// Now going to the First Child
    
    BFS_graph_builder(root, indx.first, indx.second); // Now building the graph further

  } 
  return 0;
}
