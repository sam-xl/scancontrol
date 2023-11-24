#ifndef _SCANCONTROL_NODELET_H_
#define _SCANCONTROL_NODELET_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <string>

#include "micro_epsilon_scancontrol_driver/driver.h"
#include "micro_epsilon_scancontrol_driver/nodelet.h"

namespace scancontrol_driver
{
class DriverNodelet : public nodelet::Nodelet
{
public:
  DriverNodelet()
    : running_(false)
  {
  }
  ~DriverNodelet()
  {
    // if (running_)
    // {
    //     NODELET_INFO("Shutting down driver thread.");
    //     running_ = false;
    //     device_thread_->join();
    //     NODELET_INFO("Driver thread stopped.");
    // }
  }

private:
  virtual void onInit();

  volatile bool running_;
  boost::shared_ptr<ScanControlDriver> driver_;
};

}  // namespace scancontrol_driver

#endif  // _SCANCONTROL_NODELET_H_