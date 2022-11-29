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
int desired_x[N] = {-1},desired_y[N] = {-1};
int turtle_x, turtle_y, yaw[N] = {-1};
tfScalar roll, pitch, turtle_theta;
int D; // Extended Diameter
int counter = 0; // Counter so that we take the first reading as the global frame readings
int k_x, k_y, k_theta;

k_x = 1;
k_y = 1;
k_theta = 1;

void current_pose(const nav_msgs::OdometryConstPtr& msg){ // we need to make a pointer for each message type in roscpp
  
  turtle_x = msg.pose.pose.position.desired_x;
  turtle_y = msg.pose.pose.position.desired_y;
  tf::Matrix3x3 mat(msg.pose.pose.orientation);
  mat.getEulerYPR(&turtle_theta, &pitch, &roll);

}

int main(int argc, char **argv){

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("Command_vel", 1000);

  ros::Rate rate(10);
  
  ros::Subscriber subscriber = n.subscribe("odo_sub", 10, current_pose);

  ros::Duration sample_time(1);
  ros::Duration prev_time(0);
  ros::Duration time(0); 
  
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
    err_x = desired_x[0] - turtle_x;
    err_y = desired_y[0] - turtle_y;
    err_theta = yaw[0] - turtle_theta;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    // std_msgs::String msg;// No specific standard is provided because we are using two standards here one is ros and another is standard
    geometry_msgs::Twist velocity;
    
    if(0.01 >= err_x > -0.01 and -0.01<= err_y <= 0.01 and -0.01 <= err_theta < 0.01){
      if(time>=sample_time){
        if( 0 <= t < sizeof(desired_x)){
          t++;
          err_x = desired_x[t] - turtle_x;
          err_y = desired_y[t] - turtle_y;
          err_theta = yaw[t] - turtle_theta;
        }
      }
      else{
        time = current_time - prev_time;
      }
    }
    else{
      prev_time = current_time;
    }

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