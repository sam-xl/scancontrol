#include <ros/ros.h>
#include "scancontrol/driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scancontrol_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    
    // Start the driver
    scancontrol_driver::ScanControlDriver driver(node, private_nh);

    // Start services
    ros::ServiceServer get_feature_srv = private_nh.advertiseService("get_feature", &scancontrol_driver::ScanControlDriver::ServiceGetFeature, &driver);
    ros::ServiceServer set_feature_srv = private_nh.advertiseService("set_feature", &scancontrol_driver::ScanControlDriver::ServiceSetFeature, &driver);
    ros::ServiceServer get_resolution_srv = private_nh.advertiseService("get_resolution", &scancontrol_driver::ScanControlDriver::ServiceGetResolution, &driver);
    ros::ServiceServer set_resolution_srv = private_nh.advertiseService("set_resolution", &scancontrol_driver::ScanControlDriver::ServiceSetResolution, &driver); 
    ros::ServiceServer get_available_resolutions_srv = private_nh.advertiseService("get_available_resolutions", &scancontrol_driver::ScanControlDriver::ServiceGetAvailableResolutions, &driver);

    //
    ROS_INFO("Driver started");

    // Loop driver until shutdown
    driver.StartProfileTransfer();
    while(ros::ok())
    {
        ros::spinOnce();
    }
    driver.StopProfileTransfer();
    return 0;
}