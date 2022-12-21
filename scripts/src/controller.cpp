#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <scripts/Points.h>
#include "tf/tf.h"
#include "math.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
double desired_x[5],desired_y[5], yaw[5];
double turtle_x, turtle_y;
double prev_x = 0, prev_y = 0;
double roll, pitch, turtle_theta;
int D; // Extended Diameter
int counter = 0; // Counter so that we take the first reading as the global frame readings
const int k_x = 1, k_y = 1, k_theta = 1;
int index_x = 0;
void callback(const scripts::Points::ConstPtr& msg ){ // we need to make a pointer for each message type in roscpp

  desired_x[index_x] = msg->points[index_x].x;
  desired_y[index_x] = msg->points[index_x].y;
  
  yaw[index_x] = std::atan2( ( turtle_x - prev_x ),( turtle_y - prev_y) );

}

void callback_odo(const nav_msgs::Odometry::ConstPtr& msg ){
  turtle_x = msg->pose.pose.position.x;
  turtle_y = msg->pose.pose.position.y;

  tf::Quaternion q(
    // Does nothing but a way to store a quaternion
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z
  );

  tf::Matrix3x3(q).getRPY(roll,pitch,turtle_theta);
  // gets orientation in RPY for the given quaternions
}

int main(int argc, char **argv){

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("Command_vel", 1000);

  ros::Rate rate(10);
  
  ros::Subscriber subscriber = n.subscribe("path_sub", 10, callback);
  ros::Subscriber subs = n.subscribe("odo_sub", 10, callback_odo);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  int t = 0;
  int err_x, err_y, err_theta;
  while (ros::ok())
  {
    err_x = desired_x[t] - turtle_x;
    err_y = desired_y[t] - turtle_y;
    err_theta = yaw[t] - turtle_theta;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    // std_msgs::String msg;// No specific standard is provided because we are using two standards here one is ros and another is standard
    geometry_msgs::Twist velocity;
    
    if(0.01 >= err_x > -0.01 and -0.01<= err_y <= 0.01 and -0.01 <= err_theta < 0.01){
      index_x++;
      // The subscription is now again called for the next index until the final index is reached
      ros::Subscriber subscriber = n.subscribe("path_sub", 10, callback);
      ros::Subscriber subs = n.subscribe("odo_sub", 10, callback_odo);
    }
    // ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    velocity_pub.publish(velocity);

    ros::spinOnce();

    rate.sleep();
    ++count;
  }


  return 0;
}