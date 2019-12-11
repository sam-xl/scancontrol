#ifndef _SCANCONTROL_DRIVER_H_
#define _SCANCONTROL_DRIVER_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <scancontrol/ScanControlSettingsService.h>
#include <scancontrol/GetFeature.h>
#include <scancontrol/SetFeature.h>
#include <scancontrol/GetResolution.h>
#include <scancontrol/SetResolution.h>
#include <scancontrol/GetResolutions.h>

#include <llt.h>
#include <mescan.h>

#define MAX_DEVICE_INTERFACE_COUNT 6
#define MAX_RESOLUTION_COUNT 6
#define GENERAL_FUNCTION_FAILED -1

typedef pcl::PointCloud<pcl::PointXYZI> point_cloud_t;

namespace scancontrol_driver
{
    class ScanControlDriver
    {
        public:
            // Constructor and destructor
            ScanControlDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);
            ~ScanControlDriver() {}
            
            // Profile functions
            int SetPartialProfile(int &resolution);
            int StartProfileTransfer();
            int StopProfileTransfer();
            int ProcessAndPublishProfile(const void * data, size_t data_size);
            
            // Device setting functions
            int GetFeature(unsigned int setting_id, unsigned int *value);
            int SetFeature(unsigned int setting_id, unsigned int value);

            // Get configuration parameters 
            std::string serial() const {return config_.serial;};
            int resolution() const {return config_.resolution;};

            // Service Callback
            //bool ServiceSetFeature(scancontrol::ScanControlSettingsService::Request &request, scancontrol::ScanControlSettingsService::Response &response);
            bool ServiceSetFeature(scancontrol::SetFeature::Request &request, scancontrol::SetFeature::Response &response);
            bool ServiceGetFeature(scancontrol::GetFeature::Request &request, scancontrol::GetFeature::Response &response);
            bool ServiceSetResolution(scancontrol::SetResolution::Request &request, scancontrol::SetResolution::Response &response);
            bool ServiceGetResolution(scancontrol::GetResolution::Request &request, scancontrol::GetResolution::Response &response);
            bool ServiceGetResolutions(scancontrol::GetResolutions::Request &request, scancontrol::GetResolutions::Response &response);

        private:
            // Profile functions
            int Profile2PointCloud();
            // Configuration storage
            struct
            {
                std::string frame_id;       
                std::string model;           
                std::string serial;
                std::string interface;
                std::string topic_name;
                int resolution;
                int pp_start_point;
                int pp_start_point_data;
                int pp_point_count;
                int pp_point_data_width;
            } config_;
            
            bool initialized_ = false;
            bool transfer_active_ = false;

            // ROS handles
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::ServiceServer service_server;
            ros::Publisher publisher;

            // Driver objects
            CInterfaceLLT device_interface;
            TScannerType device_type;
            TPartialProfile t_partial_profile_;
            std::vector<guint8> profile_buffer;
            std::vector<double> value_x, value_z;
            int lost_values;
            unsigned int lost_profiles;

            point_cloud_t::Ptr point_cloud_msg;

            std::map<std::string, unsigned int> feature2id = {
                {"FEATURE_FUNCTION_SERIAL", FEATURE_FUNCTION_SERIAL},
                {"FEATURE_FUNCTION_PEAKFILTER_WIDTH", FEATURE_FUNCTION_PEAKFILTER_WIDTH},
                {"FEATURE_FUNCTION_PEAKFILTER_HEIGHT", FEATURE_FUNCTION_PEAKFILTER_HEIGHT},
                {"FEATURE_FUNCTION_FREE_MEASURINGFIELD_Z", FEATURE_FUNCTION_FREE_MEASURINGFIELD_Z},
                {"FEATURE_FUNCTION_FREE_MEASURINGFIELD_X", FEATURE_FUNCTION_FREE_MEASURINGFIELD_X},
                {"FEATURE_FUNCTION_DYNAMIC_TRACK_DIVISOR", FEATURE_FUNCTION_DYNAMIC_TRACK_DIVISOR},
                {"FEATURE_FUNCTION_DYNAMIC_TRACK_FACTOR", FEATURE_FUNCTION_DYNAMIC_TRACK_FACTOR},
                {"FEATURE_FUNCTION_CALIBRATION_0", FEATURE_FUNCTION_CALIBRATION_0},
                {"FEATURE_FUNCTION_CALIBRATION_1", FEATURE_FUNCTION_CALIBRATION_1},
                {"FEATURE_FUNCTION_CALIBRATION_2", FEATURE_FUNCTION_CALIBRATION_2},
                {"FEATURE_FUNCTION_CALIBRATION_3", FEATURE_FUNCTION_CALIBRATION_3},
                {"FEATURE_FUNCTION_CALIBRATION_4", FEATURE_FUNCTION_CALIBRATION_4},
                {"FEATURE_FUNCTION_CALIBRATION_5", FEATURE_FUNCTION_CALIBRATION_5},
                {"FEATURE_FUNCTION_CALIBRATION_6", FEATURE_FUNCTION_CALIBRATION_6},
                {"FEATURE_FUNCTION_CALIBRATION_7", FEATURE_FUNCTION_CALIBRATION_7},
                {"FEATURE_FUNCTION_LASERPOWER", FEATURE_FUNCTION_LASERPOWER},
                {"FEATURE_FUNCTION_MEASURINGFIELD", FEATURE_FUNCTION_MEASURINGFIELD},
                {"FEATURE_FUNCTION_TRIGGER", FEATURE_FUNCTION_TRIGGER},
                {"FEATURE_FUNCTION_SHUTTERTIME", FEATURE_FUNCTION_SHUTTERTIME},
                {"FEATURE_FUNCTION_IDLETIME", FEATURE_FUNCTION_IDLETIME},
                {"FEATURE_FUNCTION_PROCESSING_PROFILEDATA", FEATURE_FUNCTION_PROCESSING_PROFILEDATA},
                {"FEATURE_FUNCTION_THRESHOLD", FEATURE_FUNCTION_THRESHOLD},
                {"FEATURE_FUNCTION_MAINTENANCEFUNCTIONS", FEATURE_FUNCTION_MAINTENANCEFUNCTIONS},
                {"FEATURE_FUNCTION_REARRANGEMENT_PROFILE", FEATURE_FUNCTION_REARRANGEMENT_PROFILE},
                {"FEATURE_FUNCTION_PROFILE_FILTER", FEATURE_FUNCTION_PROFILE_FILTER},
                {"FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION", FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION},
                {"FEATURE_FUNCTION_PACKET_DELAY", FEATURE_FUNCTION_PACKET_DELAY},
                {"FEATURE_FUNCTION_SHARPNESS", FEATURE_FUNCTION_SHARPNESS},
                {"FEATURE_FUNCTION_TEMPERATURE", FEATURE_FUNCTION_TEMPERATURE},
                {"FEATURE_FUNCTION_SATURATION", FEATURE_FUNCTION_SATURATION},
                {"FEATURE_FUNCTION_CAPTURE_QUALITY", FEATURE_FUNCTION_CAPTURE_QUALITY}
            };

            //
            std::list<unsigned int> features_with_corruption_risk = {FEATURE_FUNCTION_LASERPOWER, FEATURE_FUNCTION_MEASURINGFIELD, FEATURE_FUNCTION_TRIGGER}; 
    };

    void NewProfileCallback(const void * data, size_t data_size, gpointer user_data);
    void ControlLostCallback(ArvGvDevice *mydevice, gpointer user_data);

} // namespace scancontrol_driver

#endif // _SCANCONTROL_DRIVER_H_