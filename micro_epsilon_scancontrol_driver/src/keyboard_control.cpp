#include "micro_epsilon_scancontrol_driver/keyboard_control.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

static const rclcpp::Logger logger = rclcpp::get_logger("scancontrol_driver");

void handle_keyboard_input(scancontrol_driver::ScanControlDriver& driver)
{
    while (rclcpp::ok())
    {
        char command;
        std::cout << "Enter '2' to turn laser ON, '0' to turn laser OFF, or 'q' to quit: ";
        std::cin >> command;

        if (command == '2')
        {
            driver.SetFeature(FEATURE_FUNCTION_LASERPOWER, 2);
            RCLCPP_INFO(logger, "Laser turned ON");
        }
        else if (command == '0')
        {
            driver.SetFeature(FEATURE_FUNCTION_LASERPOWER, 0);
            RCLCPP_INFO(logger, "Laser turned OFF");
        }
        else if (command == 'q')
        {
            RCLCPP_INFO(logger, "Exiting...");
            rclcpp::shutdown();
            break;
        }
    }
}