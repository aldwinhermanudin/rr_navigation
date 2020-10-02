
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv){

    ros::init(argc, argv, "rosparam_test");
    ros::NodeHandle n;
    std::string ns = ros::this_node::getName();
    std::string test_param0 = ns + std::string("/test_param0");
    double s;
    if (n.getParam(test_param0, s)){
        ROS_INFO("Got param: %lf", s);
    } else {
        ROS_ERROR_STREAM("Failed to get param " << test_param0 );
    }

    std::string la_cov_param = ns + std::string("/covariance/linear_acceleration");
    std::vector<double> la_cov;
    if (n.getParam(la_cov_param, la_cov)){
        ROS_INFO_STREAM("Got param from " << la_cov_param);
        for (double val : la_cov){
            printf("%lf ", val);
        }
        std::cout << std::endl;
    } else {
        ROS_ERROR_STREAM("Failed to get param " << la_cov_param );
    }
}