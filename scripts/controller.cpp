#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "map/map.h"
#include "tf/tf.h"
#include "math.h"
#include "Time.h"
#include <sstream>

const int N = 1e9;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 *
 */
int desired_x = 0,desired_y = 0;
double turtle_x, turtle_y, yaw=0;
double roll, pitch, turtle_theta;
int D; // Extended Diameter
int counter = 0; // Counter so that we take the first reading as the global frame readings
int k_x, k_y, k_theta;

k_x = 1;
k_y = 1;
k_theta = 1;

void current_pose(const nav_msgs::OdometryConstPtr& msg){ // we need to make a pointer for each message type in roscpp
  
  turtle_x = msg->pose.pose.position.x;
  turtle_y = msg->pose.pose.position.y;

  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, turtle_theta);

}

void goal_pose(const nav_msgs::OdometryConstPtr& msg){
    desired_x = msg->pose.pose.position.x;
    desired_y = msg->pose.pose.position.y;
    tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("Command_vel", 1000);

  ros::Rate rate(10);
  
  ros::Subscriber subscriber = n.subscribe("odo_sub", 10, current_pose);

  ros::Subscriber subs = n.subscribe("goal_plan", 10, goal_pose);

  
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  int t = 0;
  int err_x, err_y, err_theta;

  while (ros::ok())
  {
    ros::Duration current_time(ros::Time::now().toSec()); // current time
    err_x = desired_x - turtle_x;
    err_y = desired_y - turtle_y;
    err_theta = yaw - turtle_theta;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    // std_msgs::String msg;// No specific standard is provided because we are using two standards here one is ros and another is standard
    geometry_msgs::Twist velocity;

    int rotation_matrix[3][3] = {{cos(turtle_theta)*cos(0), sin(turtle_theta)*cos(0), -sin(0)},{sin(0)*sin(0)*cos(turtle_theta)-cos(0)*sin(turtle_theta), sin(0)*sin(0)*sin(turtle_theta) + cos(0)*cos(turtle_theta), sin(0)*cos(0)},{cos(0)*sin(0)*cos(turtle_theta) + sin(0)*sin(turtle_theta),cos(0)*sin(0)*sin(turtle_theta) - sin(0)*cos(turtle_theta), cos(0)*cos(0)}};
    int err[3] = {err_x, err_y, err_theta};
    int result[3];
    
    // Dot Product
    for(int i = 0 ; i <= 3 ; i++){
      for(int k = 0 ; k <= 3 ; k++){
        for(int j = 0 ; j <= 3 ; j++){
          result[i] += rotation_matrix[i][k]*err[j];
        }
      }
    }
    

    velocity.linear.x = k_x * result[0];
    velocity.linear.y = k_y * result[1];
    velocity.angular.z = k_theta * result[2]; 
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