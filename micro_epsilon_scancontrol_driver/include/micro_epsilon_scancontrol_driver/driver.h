#ifndef _SCANCONTROL_DRIVER_H_
#define _SCANCONTROL_DRIVER_H_

#include <llt.h>
#include <mescan.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <micro_epsilon_scancontrol_msgs/srv/get_available_resolutions.hpp>
#include <micro_epsilon_scancontrol_msgs/srv/get_duration.hpp>
#include <micro_epsilon_scancontrol_msgs/srv/get_feature.hpp>
#include <micro_epsilon_scancontrol_msgs/srv/get_resolution.hpp>
#include <micro_epsilon_scancontrol_msgs/srv/set_duration.hpp>
#include <micro_epsilon_scancontrol_msgs/srv/set_feature.hpp>
#include <micro_epsilon_scancontrol_msgs/srv/set_resolution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#define MAX_DEVICE_INTERFACE_COUNT 6
#define MAX_RESOLUTION_COUNT 6
#define GENERAL_FUNCTION_FAILED -1

#define DEFAULT_FRAME_ID "optical_frame"
#define TOPIC_NAME "~profiles"

typedef pcl::PointCloud<pcl::PointXYZI> point_cloud_t;

namespace scancontrol_driver
{
class ScanControlDriver : public rclcpp::Node
{
public:
  // Constructor and destructor
  explicit ScanControlDriver();
  ~ScanControlDriver() = default;

  // ROS logger
  rclcpp::Logger logger = this->get_logger();

  // Profile functions
  int SetPartialProfile(int& resolution);
  int StartProfileTransfer();
  int StopProfileTransfer();
  int ProcessAndPublishProfile(const void* data, size_t data_size);

  // Device setting functions
  int GetFeature(unsigned int setting_id, unsigned int* value);
  int SetFeature(unsigned int setting_id, unsigned int value);
  int SetDuration(unsigned int setting_id, unsigned int value);
  int GetDuration(unsigned int setting_id, unsigned int* value);

  // Get configuration parameters
  int resolution() const
  {
    return config_.resolution;
  };

  // Service Callback
  void ServiceSetFeature(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetFeature::Request> request,
                         std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetFeature::Response> response);
  void ServiceGetFeature(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetFeature::Request> request,
                         std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetFeature::Response> response);
  void ServiceSetResolution(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Request> request,
                            std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Response> response);
  void ServiceGetResolution(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Request> request,
                            std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Response> response);
  void ServiceGetAvailableResolutions(
      const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetAvailableResolutions::Request> request,
      std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetAvailableResolutions::Response> response);
  void ServiceInvertZ(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void ServiceInvertX(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void
  ServiceSetExposureDuration(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetDuration::Request> request,
                             std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetDuration::Response> response);
  void
  ServiceGetExposureDuration(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetDuration::Request> request,
                             std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetDuration::Response> response);
  void ServiceSetIdleDuration(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetDuration::Request> request,
                              std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetDuration::Response> response);
  void ServiceGetIdleDuration(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetDuration::Request> request,
                              std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetDuration::Response> response);
  void ServiceToggleLaserPower(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response);

private:
  std::vector<std::string> GetDeviceInterfaces();
  
  // Profile functions
  int Profile2PointCloud();
  // Configuration storage
  struct
  {
    std::string frame_id;
    std::string model;
    std::string interface;
    int resolution;
    int pp_start_point;
    int pp_start_point_data;
    int pp_point_count;
    int pp_point_data_width;
  } config_;
  bool initialized_ = false;
  bool transfer_active_ = false;

  // ROS handles
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::GetFeature>::SharedPtr get_feature_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::SetFeature>::SharedPtr set_feature_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::GetResolution>::SharedPtr get_resolution_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::SetResolution>::SharedPtr set_resolution_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::GetAvailableResolutions>::SharedPtr
      get_available_resolutions_srv;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr invert_z_srv;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr invert_x_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::SetDuration>::SharedPtr set_exposure_duration_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::GetDuration>::SharedPtr get_exposure_duration_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::SetDuration>::SharedPtr set_idle_duration_srv;
  rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::GetDuration>::SharedPtr get_idle_duration_srv;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_laser_srv;

  // Driver objects
  std::unique_ptr<CInterfaceLLT> device_interface_ptr;
  TScannerType device_type;
  TPartialProfile t_partial_profile_;
  std::vector<guint8> profile_buffer;
  std::vector<double> value_x, value_z;
  int lost_values;
  unsigned int lost_profiles;
  std::vector<unsigned short> maximum_intensity, threshold;

  point_cloud_t::Ptr point_cloud_msg;

  std::map<std::string, unsigned int> feature2id = {
    { "FEATURE_FUNCTION_SERIAL", FEATURE_FUNCTION_SERIAL },
    { "FEATURE_FUNCTION_PEAKFILTER_WIDTH", FEATURE_FUNCTION_PEAKFILTER_WIDTH },
    { "FEATURE_FUNCTION_PEAKFILTER_HEIGHT", FEATURE_FUNCTION_PEAKFILTER_HEIGHT },
    { "FEATURE_FUNCTION_FREE_MEASURINGFIELD_Z", FEATURE_FUNCTION_FREE_MEASURINGFIELD_Z },
    { "FEATURE_FUNCTION_FREE_MEASURINGFIELD_X", FEATURE_FUNCTION_FREE_MEASURINGFIELD_X },
    { "FEATURE_FUNCTION_DYNAMIC_TRACK_DIVISOR", FEATURE_FUNCTION_DYNAMIC_TRACK_DIVISOR },
    { "FEATURE_FUNCTION_DYNAMIC_TRACK_FACTOR", FEATURE_FUNCTION_DYNAMIC_TRACK_FACTOR },
    { "FEATURE_FUNCTION_CALIBRATION_0", FEATURE_FUNCTION_CALIBRATION_0 },
    { "FEATURE_FUNCTION_CALIBRATION_1", FEATURE_FUNCTION_CALIBRATION_1 },
    { "FEATURE_FUNCTION_CALIBRATION_2", FEATURE_FUNCTION_CALIBRATION_2 },
    { "FEATURE_FUNCTION_CALIBRATION_3", FEATURE_FUNCTION_CALIBRATION_3 },
    { "FEATURE_FUNCTION_CALIBRATION_4", FEATURE_FUNCTION_CALIBRATION_4 },
    { "FEATURE_FUNCTION_CALIBRATION_5", FEATURE_FUNCTION_CALIBRATION_5 },
    { "FEATURE_FUNCTION_CALIBRATION_6", FEATURE_FUNCTION_CALIBRATION_6 },
    { "FEATURE_FUNCTION_CALIBRATION_7", FEATURE_FUNCTION_CALIBRATION_7 },
    { "FEATURE_FUNCTION_LASERPOWER", FEATURE_FUNCTION_LASERPOWER },
    { "FEATURE_FUNCTION_MEASURINGFIELD", FEATURE_FUNCTION_MEASURINGFIELD },
    { "FEATURE_FUNCTION_TRIGGER", FEATURE_FUNCTION_TRIGGER },
    { "FEATURE_FUNCTION_SHUTTERTIME", FEATURE_FUNCTION_SHUTTERTIME },
    { "FEATURE_FUNCTION_IDLETIME", FEATURE_FUNCTION_IDLETIME },
    { "FEATURE_FUNCTION_PROCESSING_PROFILEDATA", FEATURE_FUNCTION_PROCESSING_PROFILEDATA },
    { "FEATURE_FUNCTION_THRESHOLD", FEATURE_FUNCTION_THRESHOLD },
    { "FEATURE_FUNCTION_MAINTENANCEFUNCTIONS", FEATURE_FUNCTION_MAINTENANCEFUNCTIONS },
    { "FEATURE_FUNCTION_REARRANGEMENT_PROFILE", FEATURE_FUNCTION_REARRANGEMENT_PROFILE },
    { "FEATURE_FUNCTION_PROFILE_FILTER", FEATURE_FUNCTION_PROFILE_FILTER },
    { "FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION", FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION },
    { "FEATURE_FUNCTION_PACKET_DELAY", FEATURE_FUNCTION_PACKET_DELAY },
    { "FEATURE_FUNCTION_SHARPNESS", FEATURE_FUNCTION_SHARPNESS },
    { "FEATURE_FUNCTION_TEMPERATURE", FEATURE_FUNCTION_TEMPERATURE },
    { "FEATURE_FUNCTION_SATURATION", FEATURE_FUNCTION_SATURATION },
    { "FEATURE_FUNCTION_CAPTURE_QUALITY", FEATURE_FUNCTION_CAPTURE_QUALITY }
  };

  //
  std::list<unsigned int> features_with_corruption_risk = { FEATURE_FUNCTION_LASERPOWER,
                                                            FEATURE_FUNCTION_MEASURINGFIELD, FEATURE_FUNCTION_TRIGGER };
};

void NewProfileCallback(const void* data, size_t data_size, gpointer user_data);
void ControlLostCallback(ArvGvDevice* mydevice, gpointer user_data);

}  // namespace scancontrol_driver

#endif  // _SCANCONTROL_DRIVER_H_
