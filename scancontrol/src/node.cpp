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
    ros::ServiceServer service = node.advertiseService("scancontrol_settings_service_" + driver.serial(), &scancontrol_driver::ScanControlDriver::ServiceSetFeature, &driver);
    ros::ServiceServer get_feature_srv = node.advertiseService("scancontrol_get_feature", &scancontrol_driver::ScanControlDriver::ServiceGetFeature, &driver);
    ros::ServiceServer set_feature_srv = node.advertiseService("scancontrol_set_feature", &scancontrol_driver::ScanControlDriver::ServiceSetFeature, &driver);
    ros::ServiceServer get_resolution_srv = node.advertiseService("scancontrol_get_resolution", &scancontrol_driver::ScanControlDriver::ServiceGetResolution, &driver);
    ros::ServiceServer set_resolution_srv = node.advertiseService("scancontrol_set_resolution", &scancontrol_driver::ScanControlDriver::ServiceSetResolution, &driver); 
    ros::ServiceServer get_resolutions_srv = node.advertiseService("scancontrol_get_resolutions", &scancontrol_driver::ScanControlDriver::ServiceGetResolutions, &driver);

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