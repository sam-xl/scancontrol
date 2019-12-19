#ifndef _SCANCONTROL_NODELET_H_
#define _SCANCONTROL_NODELET_H_

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "micro_epsilon_scancontrol_driver/nodelet.h"
#include "micro_epsilon_scancontrol_driver/driver.h"

namespace scancontrol_driver
{
    class DriverNodelet: public nodelet::Nodelet
    {   
        public:
            DriverNodelet():
                running_(false)
            {}
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

}

#endif // _SCANCONTROL_NODELET_H_