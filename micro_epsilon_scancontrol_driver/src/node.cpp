#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "micro_epsilon_scancontrol_driver/driver.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Start the driver
  try
  {
    std::shared_ptr<scancontrol_driver::ScanControlDriver> driver =
        std::make_shared<scancontrol_driver::ScanControlDriver>();
    RCLCPP_INFO(driver->get_logger(), "Driver started");

    // Loop driver until shutdown
    driver->StartProfileTransfer();
    rclcpp::spin(driver);
    driver->StopProfileTransfer();
    return 0;
  }
  catch (const std::runtime_error& error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("micro_epsilon_scancontrol_driver"),
                        "Unable to create driver node. Error: " << error.what());
    rclcpp::shutdown();
    return 1;
  }
}
