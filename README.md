# TurtleBot3 - Visual Slam and Path Planning with Dynamic Obstacle avoidance

## Objective of the project :
To make Localization through Visual SLAM and making a local costmap using OAKD and global costmap using LIDAR on the TurtleBot3, along with this we need to implement dynamic Obstacle Avoidance

<!-- TABLE OF CONTENTS -->
## Table of Contents

- [<span style="color:black">Project</span>](#objective-of-the-project)
  - [<span style="color:purple">Demo</span>](#demo)
  - [<span style="color:purple">Table of Contents</span>](#table-of-contents)
  - [<span style="color:purple">Stages</span>](#stages)
  - [<span style="color:purple">Demo</span>](#demo)
  - [<span style="color:purple">Algorithm Flowchart</span>](#algorithm-for-bfs)
  - [<span style="color:purple">Our Approach</span>](#pseudocode:using-stacks)
  - [<span style="color:purple">Getting Started</span>](#how-to-run-bfs-from-this-project)

### Stages
* Stage 1:
  - [x] Understanding Various Path Planning Algorithms
  - [x] Writing a brute Code for BFS
  - [x] Using LIDAR to create a map and then use MOVE_BASE along with a custom GLOBAL PLANNER for BFS
  - [x] Implement BFS in Simulation with Global/Static Map 
* Stage 2:
  - [ ] Mapping and Localization through OAK-D
  - [ ] Implementing Algorithm for Dynamic Obstacle Avoidance in simulation
  - [ ] Creating an Arena for the same


  
 
* Reference for the Model : [Robotis](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

We are Using turtle bot burger for our Path planning objective

## Demo:





https://user-images.githubusercontent.com/97826285/219851215-2b4791e2-5c9c-4097-bd83-f1cc251032d2.mp4






## Algorithm for BFS

### Basic Algorithm for BFS

Breadth-first search (BFS) is an algorithm for traversing
or searching tree or graph data structures. It starts at
the tree root (or some arbitrary node of a graph, sometimes
referred to as a 'search key') and explores the neighbor
nodes first, before moving to the next level neighbors.

![Algorithm Visualization](https://upload.wikimedia.org/wikipedia/commons/5/5d/Breadth-First-Search-Algorithm.gif)

## Pseudocode : Using Queues
`Note: That we use stack and layers instead of traditional queues to implement the BFS algorithm` 

```sh
BFS(root)

  Pre: root is the node of the BST
  Post: the nodes in the BST have been visited in breadth first order
  q ← queue

  while root = ø

    yield root.value
    if root.left = ø
      q.enqueue(root.left)
    end if

    if root.right = ø
      q.enqueue(root.right)
    end if

    if !q.isEmpty()
      root ← q.dequeue()
    else
      root ← ø
    end if

  end while

end BFS
```
* Before Moving on to the next part we need to have a better understanding of three data structures
  * **Vectors** : Vectors are sequence containers representing arrays that can change in size. [Click Here](https://cplusplus.com/reference/vector/vector/) to know more 
    * We don't specify namespace in the code because there are two main things `ros` and `std` so instead we use scope resolution operator to specify the functionality
  * **Stacks** : Stacks are a type of container adaptors with LIFO(Last In First Out) type of working, where a new element is added at one end (top) and an element is removed from that end only. [Click here](https://www.geeksforgeeks.org/stack-in-cpp-stl/) to know more.
  * **Pairs**  : Pair is used to combine together two values that may be of different data types. Pair provides a way to store two heterogeneous objects as a single unit. It is basically used if we want to store tuples. The pair container is a simple container defined in < utility > header consisting of two data elements or objects. [Click here](https://www.geeksforgeeks.org/pair-in-cpp-stl/) to know more.

## Pseudocode : Using Stacks

Some Initializations:
* **path** : vector of pair, where two elements in the pair are the ith and jth index respectively
* **layer** : vector of pair of pairs, where one pair is for the parent node indices and the other pair is for child's node index
* **stack** : stack of vector of pair of pairs, basically used to store layers

`Note : Layer contains the elements which lie in the same level.` 
```sh
BFS(root)

  Pre: root is the node of the BST
  Post: the nodes in the BST have been visited in breadth first order
  st ← stack : which will store individual layers
  layer ← layer : vector of pair of pairs, each pair contains pairs of indices of parent node and current node respectively

  // Creating a tree
  while root != final index:

    check for the neighbours in east, south, north and west directions
      layer = children which are unvisited, and then mark them visited
    
    st.push(layer) ← adding the whole layer

    clear layer ← After this new layer begins as the previous layer is already pushed into the stack and marked visited

  end while

  if root == final index:

    path.push_back(final index)
    while stack is not empty:

      temp_layer = st.top() ← get the topmost layer in the stack because it definitely will contain the final index
      x = find final index in temp layer ← this will give us the node we were looking for
      
      if (found):
        
        parent = temp_layer[x].first ← Now get the parent of the and put that in the path
        path.push_back(parent) ← parent is now in the path

      end if

      st.pop()

    end while

  end if

  return(path)

end BFS
```

#### Applications
* Finding the shortest path between two nodes u and v, with path length measured by number of edges (an advantage over depth-first search)
* Serialization/Deserialization of a binary tree vs serialization in sorted order, allows the tree to be re-constructed in an efficient manner.
* Construction of the failure function of the Aho-Corasick pattern matcher.
* Testing bipartiteness of a graph.


## Our use case
#### There are two main steps that we need to perform here:
- Storing each [node]() by connecting them to its neighbours which are unoccupied and unvisited. The subpart for this is: 
    1. Storing a single parent of current node.
    2. Storing all the unvisited neighbours as children in the form of a list.
- After storing, we just need to reach the required node, that will be set as target and will be given as the input by the user

## How to run BFS from this project

1. In the first terminal source the cloning repo in the src folder of the workspace that you have created `git clone https://github.com/aPR0T0/TurtleBot-V3-BFS.git`

```

nano ~/.bashrc
// And add these lines and the source the bashrc
alias get_tb3='source /opt/ros/noetic/setup.bash && export TURTLEBOT3_MODEL=burger && source ~/your_ws/devel/setup.bash'

```
2. As we have now created an alias so no need to repeat the previous step as you relaunch the nodes, just use the alias to source the directories!

```
// alias
get_tb3

roslaunch turtlebot3_gazebo turtlebot3_world.launch

// In 3rd terminal
get_tb3

roslaunch turtlebot3_slam turtlebot3_slam slam_method:=gmapping

// In 4th terminal
get_tb3

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 

// Now, just move the bot until whole arena is traversed and then close 4th terminal as map is not created
// Once done mapping close terminal 3 and go to next instruction

// In 5th terminal
get_tb3

roslaunch turtlebot3 map_node.launch map_file:=$HOME/map.yaml

// Now use Estimated pose Icon on GUI to give initial estimate of where bot is...
// then in 6th terminal
get_tb3

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 

// traverse the map until all particles closer and closer to the bot

```

Voila! You got the simulation done!!
Now, Just click on nav_goal_2d in RVIZ GUI and see the magic of path planning

## References

- [trekhleb/javascript-algorithms](https://github.com/trekhleb/javascript-algorithms/tree/master/src/algorithms/tree/breadth-first-search)
- [Wikipedia](https://en.wikipedia.org/wiki/Breadth-first_search)
- [Tree Traversals (Inorder, Preorder and Postorder)](https://www.geeksforgeeks.org/tree-traversals-inorder-preorder-and-postorder/)
- [BFS vs DFS](https://www.geeksforgeeks.org/bfs-vs-dfs-binary-tree/)
