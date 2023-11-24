#include "micro_epsilon_scancontrol_driver/nodelet.h"

// Register this plugin with pluginlib.
PLUGINLIB_EXPORT_CLASS(scancontrol_driver::DriverNodelet, nodelet::Nodelet)

namespace scancontrol_driver
{
void DriverNodelet::onInit()
{
  // Start scanCONTROL driver
  driver_.reset(new ScanControlDriver(getNodeHandle(), getPrivateNodeHandle()));

  // spawn device poll thread
  // running_ = true;
  // device_thread_ = boost::shared_ptr<boost::thread> (new boost::thread(boost::bind(&DriverNodelet::devicePoll,
  // this)));
}

/** @brief Device poll thread main loop. */
// void DriverNodelet::devicePoll()
// {
//     while(ros::ok())
//         {

//         }
//     running_ = false;
// }
}  // namespace scancontrol_driver
