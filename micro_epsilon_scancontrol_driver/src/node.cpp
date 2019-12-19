#include <ros/ros.h>
#include "micro_epsilon_scancontrol_driver/driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micro_epsilon_scancontrol_driver_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    
    // Start the driver
    try
    {
        scancontrol_driver::ScanControlDriver driver(node, private_nh);
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
    catch(const std::runtime_error& error)
    {
        ROS_FATAL_STREAM(error.what());
        ros::shutdown();
        return 0;
    }


}
