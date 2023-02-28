/*
 * This Program will help converging the data 
 * coming from Oak-D and LiDar from TB3 module
 * The output of the LiDar Map will be post - processed here
 * In Post processing the bounds of the height of the obstacle will be considered
 * Now, with the help of the current data we will be making a local map
 * Now, this map will have the range of 1m - > for more accuracy
 * It has FOV = 50°, Assumption according to the data sheet
 */

// Including Required headers
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <ros/console.h>


