// Including Required headers
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <turtlebot3/Points.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <bits/stdc++.h>
#include <ros/console.h>

// Global Variable of list type which holds xyz coordinates of the points
turtlebot3::Points pcl_msg;

/*
    Function        : cloud_cb
    Arguments       : message of `Point cloud 2` type
    Functionality   : To convert pcl2 data in form of pcl which is human interpretable
    returns         : None
*/
void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    // Declarations
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    
    // Conversions
    pcl_conversions::toPCL( *input, pcl_pc2);
    pcl::fromPCLPointCloud2( pcl_pc2, *temp_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud = *temp_cloud;
    
    geometry_msgs::Point helper;

    for (auto &i : cloud.points){
        
        if(i.z <= 0.6){
            helper.x =   i.x ;
            helper.y = - i.y ;
            helper.z =   i.z ;

            pcl_msg.points.push_back(helper);
        }
    }   

    pcl_msg.end_index = pcl_msg.points.size() - 1;
}   


// ##################################################################################################### //
// #####################################     Main Function    ########################################## //
// ##################################################################################################### //

int main(int argc, char **argv){

    // Initializing the oak-d node
    ros::init(argc, argv, "oakd_interpretor");
    ros::NodeHandle n;

    std::cout<<"Trying to connect ...."<<std::endl;

    ros::Publisher pcl_pub = n.advertise<turtlebot3::Points>("/oakd/pcl", 1000);
    ros::Subscriber pcl2_sub = n.subscribe("/oakd/pointcloud", 1000, cloud_cb);
    
    ros::Rate r(10);

    while(ros::ok()){
        
        ros::Subscriber pcl2_sub = n.subscribe("/oakd/pointcloud", 1000, cloud_cb);

        for( auto &t : pcl_msg.points){
            std::cout<<"[ X = "<< t.x <<"Y = "<< t.y << "Z = "<< t.z <<" ]"<<std::endl;
        }

        pcl_pub.publish(pcl_msg);
        pcl_msg.points.clear();

        ros::spinOnce();
        r.sleep();
    }
    
    // Note spin just loops the callback function but not the publisher 
    return 0;
}

