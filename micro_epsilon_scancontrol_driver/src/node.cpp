#include "rclcpp/rclcpp.hpp"
#include "micro_epsilon_scancontrol_driver/driver.h"
#include <thread>
#include <memory>
static const rclcpp::Logger logger = rclcpp::get_logger("scancontrol_driver");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Start the driver
    try
    {
        std::shared_ptr<scancontrol_driver::ScanControlDriver> driver = std::make_shared<scancontrol_driver::ScanControlDriver>();
        RCLCPP_INFO(logger, "Driver started");

        //Turn On Laser
        driver->SetFeature(FEATURE_FUNCTION_LASERPOWER,2);

        // Loop driver until shutdown
        driver->StartProfileTransfer();
        rclcpp::spin(driver);

        //Turn Off Laser
        driver->SetFeature(FEATURE_FUNCTION_LASERPOWER,0);
        driver->StopProfileTransfer();

        return 0;
    }
    catch(const std::runtime_error& error)
    {
        RCLCPP_FATAL_STREAM(logger, error.what());
        rclcpp::shutdown();
        return 0;
    }


}
