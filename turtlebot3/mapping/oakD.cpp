// Including Required headers
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud>
#include <pcl/conversions.h>
#include <ros/console.h>

// Global Variable of XYZ type
pcl::PointCloud<pcl::PointXYZ> cloud;

/*
    Function        : cloud_cb
    Arguments       : message of `Point cloud 2` type
    Functionality   : To convert pcl2 data in form of pcl which is human interpretable
    returns         : None
*/
void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *input, pcl_pc2);
    pcl::fromPCLPointCloud2( pcl_pc2, cloud);

}


// ##################################################################################################### //
// #####################################     Main Function    ########################################## //
// ##################################################################################################### //

int main(int argc, char **argv){

    // Initializing the oak-d node
    ros::init(argc, argv, "oakd_interpretor");
    ros::NodeHandle n;

    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud>("/oakd/pcl", 1000);
    ros::Subscriber pcl2_sub = n.subscribe("/oakd/pointcloud", 1000, cloud_cb);

    pcl_pub.publish(cloud);

    ros::spin();

    return 0;
}