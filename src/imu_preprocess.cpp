#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <exception>
 
ros::Subscriber sub_from;
ros::Publisher pub_to;

ros::Subscriber sub_scan_from;
ros::Publisher pub_scan_to;
 
void ProcessOdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
{
    nav_msgs::Odometry msg_with_cov = *msg;
    
    std::vector<double> pose_cov{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025};
    std::vector<double> twist_cov{0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02};

    msg_with_cov.header.frame_id = "odom";
    msg_with_cov.child_frame_id = "base_footprint";

 
    // adding orientation covariance
    memcpy( &msg_with_cov.pose.covariance[0],
            &pose_cov[0], 
            pose_cov.size() * sizeof(double));

    memcpy( &msg_with_cov.twist.covariance[0],
            &twist_cov[0], 
            twist_cov.size() * sizeof(double));

    // republishing imu data with covariance
    pub_to.publish(msg_with_cov);
}

void ProcessScanCallback( const sensor_msgs::LaserScan::ConstPtr& msg )
{
    sensor_msgs::LaserScan new_msg_frame = *msg;
    
    new_msg_frame.header.frame_id = "laser";

    // republishing imu data with covariance
    pub_scan_to.publish(new_msg_frame);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_scan_changer");
 
    ros::NodeHandle n;
    pub_to   = n.advertise<nav_msgs::Odometry>("/convert/odom", 1000);
    sub_from = n.subscribe("/slamware_ros_sdk_server_node/odom", 1000, ProcessOdomCallback);   
    
    pub_scan_to   = n.advertise<sensor_msgs::LaserScan>("/scan", 1000);
    sub_scan_from = n.subscribe("/slamware_ros_sdk_server_node/scan", 1000, ProcessScanCallback);
    
    ros::spin();
 
    return 0;
}