#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "micro_epsilon_scancontrol_driver/driver.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  try
  {
    auto driver = std::make_shared<scancontrol_driver::ScanControlDriver>(options);
    exec.add_node(driver);

    driver->StartProfileTransfer();
    exec.spin();

    driver->StopProfileTransfer();
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::runtime_error& error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("micro_epsilon_scancontrol_driver"),
                        "Unable to create driver component. Error: " << error.what());
    rclcpp::shutdown();
    return 1;
  }
}
