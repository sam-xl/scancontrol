#include "rclcpp/rclcpp.hpp"
#include "micro_epsilon_scancontrol_driver/driver.h"
#include "micro_epsilon_scancontrol_driver/keyboard_control.h"
#include <thread>

static const rclcpp::Logger logger = rclcpp::get_logger("scancontrol_driver");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("scancontrol_driver"); 
    rclcpp::Node::SharedPtr private_node = rclcpp::Node::make_shared("scancontrol_driver"); 

    // Start the driver
    try
    {
        scancontrol_driver::ScanControlDriver driver(node, private_node);
        RCLCPP_INFO(logger, "Driver started");

        //Turn On Laser
        driver.SetFeature(FEATURE_FUNCTION_LASERPOWER,2);

        // Loop driver until shutdown
        driver.StartProfileTransfer();
        while(rclcpp::ok())
        {
            rclcpp::spin_some(node);
        }

        //Turn Off Laser
        driver.SetFeature(FEATURE_FUNCTION_LASERPOWER,0);
        driver.StopProfileTransfer();

        return 0;
    }
    catch(const std::runtime_error& error)
    {
        RCLCPP_FATAL_STREAM(logger, error.what());
        rclcpp::shutdown();
        return 0;
    }


}
