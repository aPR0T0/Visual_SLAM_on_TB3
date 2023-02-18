# TurtleBot3 - BFS path planning

## Objective of the project :
To Understand path planning concepts using slam, and implementing BFS on LiDAR based TurtleBot3, and then  further increasing that knowledge and implementing more advanced path planning concepts

## Reference for the Model:- [Robotis](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

We are Using turtle bot burger for our Path planning objective

## Demo:





https://user-images.githubusercontent.com/97826285/219851215-2b4791e2-5c9c-4097-bd83-f1cc251032d2.mp4





---
## Algorithm for BFS

### Basic Algorithm for BFS

Breadth-first search (BFS) is an algorithm for traversing
or searching tree or graph data structures. It starts at
the tree root (or some arbitrary node of a graph, sometimes
referred to as a 'search key') and explores the neighbor
nodes first, before moving to the next level neighbors.

![Algorithm Visualization](https://upload.wikimedia.org/wikipedia/commons/5/5d/Breadth-First-Search-Algorithm.gif)

## Pseudocode

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

#### Applications
* Finding the shortest path between two nodes u and v, with path length measured by number of edges (an advantage over depth-first search)
* Serialization/Deserialization of a binary tree vs serialization in sorted order, allows the tree to be re-constructed in an efficient manner.
* Construction of the failure function of the Aho-Corasick pattern matcher.
* Testing bipartiteness of a graph.


----
## Our use case
#### There are two main steps that we need to perform here:
- Storing each [node]() by connecting them to its neighbours which are unoccupied and unvisited. The subpart for this is: 
    1. Storing a single parent of current node.
    2. Storing all the unvisited neighbours as children in the form of a list.
- After storing, we just need to reach the required node, that will be set as target and will be given as the input by the user

## References

- [trekhleb/javascript-algorithms](https://github.com/trekhleb/javascript-algorithms/tree/master/src/algorithms/tree/breadth-first-search)
- [Wikipedia](https://en.wikipedia.org/wiki/Breadth-first_search)
- [Tree Traversals (Inorder, Preorder and Postorder)](https://www.geeksforgeeks.org/tree-traversals-inorder-preorder-and-postorder/)
- [BFS vs DFS](https://www.geeksforgeeks.org/bfs-vs-dfs-binary-tree/)
