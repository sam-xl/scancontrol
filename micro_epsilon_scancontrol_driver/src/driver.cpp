#include "micro_epsilon_scancontrol_driver/driver.h"

namespace scancontrol_driver
{

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("scancontrol_driver_node");  // TODO change this into a class variable. or just use this->get_logger()?

    ScanControlDriver::ScanControlDriver(const std::string& name):Node(name)
    {   
        /* 
            Extract the relevant parameters. 
        */   

        // Device settings
        this->declare_parameter<int>("resolution", -1);
        this->get_parameter_or("resolution", config_.resolution, -1);

        // Multiple device parameters        
        this->declare_parameter<std::string>("serial", std::string(""));
        this->get_parameter_or("serial", config_.serial, std::string(""));
        this->declare_parameter<std::string>("frame_id", std::string(DEFAULT_FRAME_ID));
        this->get_parameter_or("frame_id", config_.frame_id, std::string(DEFAULT_FRAME_ID));
        this->declare_parameter<std::string>("topic_name", std::string(DEFAULT_TOPIC_NAME));
        this->get_parameter_or("topic_name", config_.topic_name, std::string(DEFAULT_TOPIC_NAME));

        // TODO: Are these parameters needed?
        this->declare_parameter<int>("partial_profile_start_point", 0);
        this->get_parameter_or("partial_profile_start_point", config_.pp_start_point, 0);
        this->declare_parameter<int>("partial_profile_start_point_data", 4);
        this->get_parameter_or("partial_profile_start_point_data", config_.pp_start_point_data, 4);
        this->declare_parameter<int>("partial_profile_point_count", -1);
        this->get_parameter_or("partial_profile_point_count", config_.pp_point_count, -1);
        this->declare_parameter<int>("partial_profile_data_width", 4);
        this->get_parameter_or("partial_profile_data_width", config_.pp_point_data_width, 4);

        // Create driver interface object:
        device_interface_ptr = std::make_unique<CInterfaceLLT>();

        /*
            Search for available scanCONTROL interfaces
                The code only supports a maximum of MAX_LLT_INTERFACE_COUNT (6) devices. If
                more are connected the devices found after the 6th are not considdered. This
                is a limitation of the API, which needs a fixed number of devices to search 
                for. If 6 devices is insufficient, redefine MAX_LLT_INTERFACE_COUNT to 
                accomodate the additional devices.
        */
        gint32 return_code      = 0;
        gint32 interface_count  = 0;
        std::vector<char *> available_interfaces(MAX_DEVICE_INTERFACE_COUNT);

        return_code = device_interface_ptr->GetDeviceInterfaces(&available_interfaces[0], MAX_DEVICE_INTERFACE_COUNT);
        if (return_code == ERROR_GETDEVINTERFACE_REQUEST_COUNT){
            RCLCPP_WARN_STREAM(LOGGER, "There are more than " << MAX_DEVICE_INTERFACE_COUNT << " scanCONTROL sensors connected.");
            interface_count = MAX_DEVICE_INTERFACE_COUNT;
        } else if (return_code < 0) {
            RCLCPP_WARN_STREAM(LOGGER, "An error occured while searching for connected scanCONTROL devices. Code: " << return_code);
            interface_count = 0;
        } else {
            interface_count = return_code;
        }

        /*
            Select scanCONTROL interface
                A preffered interface can be set by means of the 'serial' parameter. 
        */
        gint8 selected_interface = -1;
        if (interface_count == 0){
            RCLCPP_WARN(LOGGER, "There is no scanCONTROL device connected. Exiting...");
            goto stop_initialization;
        } else if (interface_count == 1){
            RCLCPP_INFO(LOGGER, "There is 1 scanCONTROL device connected.");
            selected_interface = 0;

            // Check if the available device is the same as the prefered device (if a serial is provided):
            std::string interface(available_interfaces[0]);
            if ((config_.serial == "") || (interface.find(config_.serial) > -1)){
                RCLCPP_INFO_STREAM(LOGGER, "Interface found: " << interface);
            }
            else{
                RCLCPP_WARN_STREAM(LOGGER, "Interface not found! Searched for serial = " << config_.serial);
                RCLCPP_INFO_STREAM(LOGGER, "Selected interface: " << interface);
            }
        } else {
            RCLCPP_INFO_STREAM(LOGGER, "There are " << interface_count << " scanCONTROL devices connected.");

            // Select prefered device based on the defined ip or serial. If both are set, this selects the device which ip or serial is encountered first.
            if (config_.serial != ""){
                for (int i = 0; i < interface_count; i++){
                    std::string interface(available_interfaces[i]);
                    if (interface.find(config_.serial) > -1){
                        RCLCPP_INFO_STREAM(LOGGER, "Interface found: " << interface);
                        selected_interface = i;
                        break;
                    }
                }
                // Fallback if serial are not found:
                if (selected_interface == -1){
                    RCLCPP_WARN_STREAM(LOGGER, "Interface not found! Searched for serial = " << config_.serial);
                    RCLCPP_WARN(LOGGER, "Available interfaces:");
                    for (gint8 i = 0; i < interface_count; i++){
                        RCLCPP_WARN_STREAM(LOGGER, "   " << available_interfaces[i]);
                    }
                    selected_interface = 0;
                    RCLCPP_INFO_STREAM(LOGGER, "\nSelecting first available interface: " << available_interfaces[selected_interface]);
                }
            } else{
                selected_interface = 0;
                RCLCPP_INFO_STREAM(LOGGER, "No 'serial' set, selecting first interface: " << available_interfaces[selected_interface]);
            }
        }

        /*
            Set the selected device to the driver interface class and catch possible errors
        */
        config_.interface = std::string(available_interfaces[selected_interface]);
        config_.serial = std::string(config_.interface.end() - 9, config_.interface.end());
        return_code = device_interface_ptr->SetDeviceInterface(available_interfaces[selected_interface]);
        if (return_code < GENERAL_FUNCTION_OK){
            RCLCPP_FATAL_STREAM(LOGGER, "Error while setting device ID! Code: " << return_code);
            goto stop_initialization;
        }

        /*
            Connect to scanCONTROL device
        */
        return_code = device_interface_ptr->Connect();  
        if (return_code < GENERAL_FUNCTION_OK){
            RCLCPP_FATAL_STREAM(LOGGER, "Error while connecting to device! Code: " << return_code);
            goto stop_initialization;
        }

        /*
            Identify device type and store information on type and serial
        */
        return_code = device_interface_ptr->GetLLTType(&device_type);
        if (return_code < GENERAL_FUNCTION_OK){
            RCLCPP_FATAL_STREAM(LOGGER, "Error while retrieving device type! Code: " << return_code);
            goto stop_initialization;
        }
        if (device_type >= scanCONTROL27xx_25 && device_type <= scanCONTROL27xx_xxx) {
            RCLCPP_INFO_STREAM(LOGGER, "The scanCONTROL is a scanCONTROL27xx, with serial number " << config_.serial << ".");
            config_.model = std::string("scanCONTROL27xx");
        } else if (device_type >= scanCONTROL26xx_25 && device_type <= scanCONTROL26xx_xxx) {
            RCLCPP_INFO_STREAM(LOGGER, "The scanCONTROL is a scanCONTROL26xx, with serial number " << config_.serial << ".");
            config_.model = std::string("scanCONTROL26xx");
        } else if (device_type >= scanCONTROL29xx_25 && device_type <= scanCONTROL29xx_xxx) {
            RCLCPP_INFO_STREAM(LOGGER, "The scanCONTROL is a scanCONTROL29xx, with serial number " << config_.serial << ".");
            config_.model = std::string("scanCONTROL29xx");
        } else if (device_type >= scanCONTROL30xx_25 && device_type <= scanCONTROL30xx_xxx) {
            RCLCPP_INFO_STREAM(LOGGER, "The scanCONTROL is a scanCONTROL30xx, with serial number " << config_.serial << ".");
            config_.model = std::string("scanCONTROL30xx");
        } else if (device_type >= scanCONTROL25xx_25 && device_type <= scanCONTROL25xx_xxx) {
            RCLCPP_INFO_STREAM(LOGGER, "The scanCONTROL is a scanCONTROL25xx, with serial number " << config_.serial << ".");
            config_.model = std::string("scanCONTROL25xx");
        } else {
            RCLCPP_FATAL(LOGGER, "The scanCONTROL device is a undefined type.\nPlease contact Micro-Epsilon for a newer SDK or update the driver.");
            goto stop_initialization;
        }

        /*
            Set the resolution of the scanner.
        */
        // Find all available resolutions
        guint32 available_resolutions[MAX_RESOLUTION_COUNT];
        if (return_code = device_interface_ptr->GetResolutions(&available_resolutions[0], MAX_RESOLUTION_COUNT) < GENERAL_FUNCTION_OK){
            RCLCPP_FATAL_STREAM(LOGGER, "Unable to request the available resolutions of the scanCONTROL device. Code: " << return_code);
            goto stop_initialization;
        }

        // Select prefered/first found resolution
        if (config_.resolution > 0){
            gint8 selected_resolution = -1;
            for (int i = 0; i < return_code; i++){
                std::string resolution = std::to_string(available_resolutions[i]);
                if (resolution.find(config_.serial) > -1){
                    selected_resolution = i;
                    break;
                }
            }
            if (selected_resolution == -1){
                RCLCPP_WARN_STREAM(LOGGER, "Requested resolution of " << std::to_string(config_.resolution) <<" not found as available option.");
                RCLCPP_WARN(LOGGER, "Available resolutions:");
                for (int i = 0; i < return_code; i++){
                    RCLCPP_WARN_STREAM(LOGGER, "   " << std::to_string(available_resolutions[i]));
                }
                config_.resolution = available_resolutions[0];
                RCLCPP_INFO_STREAM(LOGGER, "Selecting first available resolution: " << std::to_string(config_.resolution));
            }
        } else{
            config_.resolution = available_resolutions[0];
            RCLCPP_INFO_STREAM(LOGGER, "No resolution set, selecting first available: " << std::to_string(config_.resolution));
        }

        // Set the selected resolution
        if (return_code = device_interface_ptr->SetResolution(config_.resolution) < GENERAL_FUNCTION_OK){
            RCLCPP_FATAL_STREAM(LOGGER, "Error while setting device resolution! CodeL " << return_code);
            goto stop_initialization;
        }
        
        /*
            Prepare the partial profile
        */
        if (return_code = SetPartialProfile(config_.resolution) < GENERAL_FUNCTION_OK){
            goto stop_initialization;
        }

        /*
            Register callback functions
                RegisterBufferCallback > NewProfile: Triggered when the sensor has a new profile available in the buffer.  
                RegisterControlLostCallback > ControlLostCallback: Triggered when control of the device is lost. 
        */
        if ((return_code = device_interface_ptr->RegisterBufferCallback((gpointer) &NewProfileCallback, this)) < GENERAL_FUNCTION_OK){
            RCLCPP_FATAL_STREAM(LOGGER, "Error while registering buffer callback. Code: " << return_code);
            goto stop_initialization;
        }
        if ((return_code = device_interface_ptr->RegisterControlLostCallback((gpointer) &ControlLostCallback, this)) < GENERAL_FUNCTION_OK){
            RCLCPP_FATAL_STREAM(LOGGER, "Error while registering control lost callback. Code: " << return_code);
            goto stop_initialization;
        }

        /*
            Initialization finished and successfully connected to scanCONTROL device.
        */
        initialized_ = true;

        // goto location when initialization fails
        stop_initialization:            
            if (!initialized_) {
                throw std::runtime_error("ScanControlDriver: Initialization failed.");
            }

        // Advertise topic
        publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(config_.topic_name, 10);

        using std::placeholders::_1;
        using std::placeholders::_2;

        // Advertise services
        get_feature_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::GetFeature>(
            "~/get_feature", std::bind(&ScanControlDriver::ServiceGetFeature, this, _1, _2));
        set_feature_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::SetFeature>(
            "~/set_feature", std::bind(&ScanControlDriver::ServiceSetFeature, this, _1, _2));
        get_resolution_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::GetResolution>(
            "~/get_resolution", std::bind(&ScanControlDriver::ServiceGetResolution, this, _1, _2));
        set_resolution_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::SetResolution>(
            "~/set_resolution", std::bind(&ScanControlDriver::ServiceSetResolution, this, _1, _2)); 
        get_available_resolutions_srv   = this->create_service<micro_epsilon_scancontrol_msgs::srv::GetAvailableResolutions>(
            "~/get_available_resolutions",  std::bind(&ScanControlDriver::ServiceGetAvailableResolutions, this, _1, _2));
        invert_z_srv = this->create_service<std_srvs::srv::SetBool>(
            "~/invert_z", std::bind(&ScanControlDriver::ServiceInvertZ, this, _1, _2));
        invert_x_srv = this->create_service<std_srvs::srv::SetBool>(
            "~/invert_x", std::bind(&ScanControlDriver::ServiceInvertX, this, _1, _2));
        set_exposure_time_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::SetTime>(
            "~/set_exposure_time", std::bind(&ScanControlDriver::ServiceSetExposureTime, this, _1, _2));
        get_exposure_time_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::GetTime>(
            "~/get_exposure_time", std::bind(&ScanControlDriver::ServiceGetExposureTime, this, _1, _2));
        set_idle_time_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::SetTime>(
            "~/set_idle_time", std::bind(&ScanControlDriver::ServiceSetIdleTime, this, _1, _2));
        get_idle_time_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::GetTime>(
            "~/get_idle_time", std::bind(&ScanControlDriver::ServiceGetIdleTime, this, _1, _2));

    }

    int ScanControlDriver::SetPartialProfile(int &resolution){
        gint32 return_code = 0;
        // Set profile configuration to partial profile
        if ((return_code = device_interface_ptr->SetProfileConfig(PARTIAL_PROFILE)) < GENERAL_FUNCTION_OK){
            RCLCPP_WARN_STREAM(LOGGER, "Error while setting profile config to PARTIAL_PROFILE. Code: " << return_code);
            return GENERAL_FUNCTION_FAILED;
        }

        // Create partial profile object and send to device
        t_partial_profile_.nStartPoint = config_.pp_start_point;
        t_partial_profile_.nStartPointData = config_.pp_start_point_data;
        t_partial_profile_.nPointDataWidth = config_.pp_point_data_width;
        t_partial_profile_.nPointCount = (config_.pp_point_count == -1 || config_.pp_point_count > resolution) ? resolution : config_.pp_point_count;

        // Send partial profile settings to device
        if ((return_code = device_interface_ptr->SetPartialProfile(&t_partial_profile_)) < GENERAL_FUNCTION_OK){
            // Restore nPointCount to old value 
            t_partial_profile_.nPointCount = (config_.pp_point_count == -1 || config_.pp_point_count > resolution) ? config_.resolution : config_.pp_point_count;
            
            // Send warning and return failed
            RCLCPP_WARN_STREAM(LOGGER, "Error while setting partial profile settings. Code: " << return_code);
            return GENERAL_FUNCTION_FAILED;
        }

        // Resize buffers - values contain less than the point count, due to the timestamp data
        profile_buffer.resize(t_partial_profile_.nPointCount*t_partial_profile_.nPointDataWidth);
        lost_values = (16 + t_partial_profile_.nPointDataWidth - 1)/t_partial_profile_.nPointDataWidth;
        value_x.resize(t_partial_profile_.nPointCount);
        value_z.resize(t_partial_profile_.nPointCount);
        RCLCPP_INFO_STREAM(LOGGER, "Profile is losing " << std::to_string(lost_values) << " values due to timestamp of 16 byte at the end of the profile.");

        // Prepare new point cloud message
        point_cloud_msg.reset(new point_cloud_t);
        point_cloud_msg->header.frame_id = config_.frame_id;
        point_cloud_msg->height = 1;
        point_cloud_msg->width = resolution;
        for (int i=0; i<resolution; i++){
            pcl::PointXYZI point(1.0);
            point_cloud_msg->points.push_back(point);
        }

        return GENERAL_FUNCTION_OK;
    }

    /* Start the transfer of new profiles */
    int ScanControlDriver::StartProfileTransfer(){
        int return_code = 0;
        if ((return_code = device_interface_ptr->TransferProfiles(NORMAL_TRANSFER, true)) < GENERAL_FUNCTION_OK) {
            RCLCPP_WARN_STREAM(LOGGER, "Error while starting profile transfer! Code: " << return_code);
            return return_code;
        }
        transfer_active_ = true;
        return GENERAL_FUNCTION_OK;
    }

    /* Stop the transfer of new profiles */
    int ScanControlDriver::StopProfileTransfer(){
        int return_code = 0;
        if ((return_code = device_interface_ptr->TransferProfiles(NORMAL_TRANSFER, false)) < GENERAL_FUNCTION_OK) {
            RCLCPP_WARN_STREAM(LOGGER, "Error while stopping profile transfer. Code: " << return_code);
            return return_code;
        }
        transfer_active_ = false;
        return GENERAL_FUNCTION_OK;
    }

    /* Process raw profile data and create the point cloud message */
    int ScanControlDriver::Profile2PointCloud(){

        device_interface_ptr->ConvertPartProfile2Values(&profile_buffer[0], profile_buffer.size(), &t_partial_profile_, device_type, 0, NULL, NULL, NULL, &value_x[0], &value_z[0], NULL, NULL);
        for (int i = 0; i < config_.resolution; i++){
            point_cloud_msg->points[i].x = value_x[i]/1000;
            
            // Fill in NaN if the scanner is to close or far away (sensor returns ~32.232) and for the final few points which are overwritten by the timestamp data
            if ((value_z[i] < 32.5) || (i >= config_.resolution - lost_values)){
                point_cloud_msg->points[i].z = std::numeric_limits<double>::infinity(); 
            }
            else{
                point_cloud_msg->points[i].z = value_z[i]/1000;
            }
        }
        return GENERAL_FUNCTION_OK;
    }
    
    /* Process and publish profile */
    int ScanControlDriver::ProcessAndPublishProfile(const void * data, size_t data_size){
        // Timestamp 
        pcl_conversions::toPCL(this->get_clock()->now(), point_cloud_msg->header.stamp);

        // Copy sensor data to local buffer 
        if (data != NULL && data_size == profile_buffer.size()){
            memcpy(&profile_buffer[0], data, data_size);
        }

        // Process buffer and publish point cloud
        ScanControlDriver::Profile2PointCloud();
        // TODO: Is their a better way as long as pcl_ros is not ported to ROS2
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg<pcl::PointXYZI>(*point_cloud_msg, output_msg);        
        
        publisher->publish(output_msg);

        return GENERAL_FUNCTION_OK;
    }

    /* Retrieve the current value of a setting/feature. Consult the scanCONTROL API documentation for a list of available features */ 
    int ScanControlDriver::GetFeature(unsigned int setting_id, unsigned int *value){
        int return_code = 0;
        return_code = device_interface_ptr->GetFeature(setting_id, value);
        if (return_code < GENERAL_FUNCTION_OK){
            RCLCPP_WARN_STREAM(LOGGER, "Setting could not be retrieved. Code: " << return_code);
            return return_code;
        }
        return GENERAL_FUNCTION_OK;
    }

    /*
        Attempt to set the a setting/feature to the requested value. 
            The function temporarily halts the transfer of new profiles if the setting can cause the profiles to be corrupted. 
    */ 
    int ScanControlDriver::SetFeature(unsigned int setting_id, unsigned int value){
        int return_code = 0;
        if ((std::find(features_with_corruption_risk.begin(), features_with_corruption_risk.end(), setting_id) != features_with_corruption_risk.end()) && transfer_active_){
            RCLCPP_INFO(LOGGER, "Risk of profile corruption, temporarily stopping profile transfer.");
            if (return_code = ScanControlDriver::StopProfileTransfer() < GENERAL_FUNCTION_OK){
                RCLCPP_WARN_STREAM(LOGGER, "Profile transfer could not be stopped. Code: " << return_code);
                return -1;
            }
            if (return_code = device_interface_ptr->SetFeature(setting_id, value) < GENERAL_FUNCTION_OK){
                RCLCPP_WARN_STREAM(LOGGER, "Feature could not be set. Code: " << return_code);
                return return_code;
            }
            if (return_code = ScanControlDriver::StartProfileTransfer() < GENERAL_FUNCTION_OK){
                RCLCPP_WARN_STREAM(LOGGER, "Profile transfer could not be restarted after changing feature. Code: " << return_code);
                return -1;
            }
            return GENERAL_FUNCTION_OK;
        }
        if (return_code = device_interface_ptr->SetFeature(setting_id, value) < GENERAL_FUNCTION_OK){
            RCLCPP_WARN_STREAM(LOGGER, "Feature could not be set. Code: " << return_code);
            return return_code;
        }
        return GENERAL_FUNCTION_OK;
    }

    /* Wrapper of the SetFeature call for use by the ServiceSetFeature service */
    void ScanControlDriver::ServiceSetFeature(
            const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetFeature::Request> request,
            std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetFeature::Response> response)
    {
        response->return_code = ScanControlDriver::SetFeature(request->setting, request->value);
        // return true;
    }

    /* Wrapper of the GetFeature call for use by the ServiceGetFeature service */
    void ScanControlDriver::ServiceGetFeature(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetFeature::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetFeature::Response> response)
    {
        response->return_code = ScanControlDriver::GetFeature(request->setting, &(response->value));
        // return true;
    }

    /* Wrapper of the SetResolution call for use by the ServiceSetResolution service */
    void ScanControlDriver::ServiceSetResolution(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Response> response)
    {
        response->return_code = StopProfileTransfer();
        if (response->return_code < GENERAL_FUNCTION_OK){
            RCLCPP_WARN_STREAM(LOGGER, "Error while stopping transmission! Code: " << response->return_code);
            return;
        }
        response->return_code = device_interface_ptr->SetResolution(request->resolution);
        if (response->return_code < GENERAL_FUNCTION_OK){
            RCLCPP_WARN_STREAM(LOGGER, "Error while setting device resolution! Code: " << response->return_code);
            // return true;
        }
        int temp_resolution = request->resolution;
        response->return_code = SetPartialProfile(temp_resolution);
        if (response->return_code < GENERAL_FUNCTION_OK){
            RCLCPP_WARN_STREAM(LOGGER, "Error while setting partial profile. Code: " << response->return_code);
            return;
        }
        response->return_code = StartProfileTransfer();
        if (response->return_code < GENERAL_FUNCTION_OK){
            RCLCPP_WARN_STREAM(LOGGER, "Error while starting transmission! Code: " << response->return_code);
            return;
        }

        // Change of resolution was successful
        config_.resolution = request->resolution;
        // return true;
    }

    /* Wrapper of the GetResolution call for use by the ServiceGetResolution service */
    void ScanControlDriver::ServiceGetResolution(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Response> response)
    {
        response->return_code = device_interface_ptr->GetResolution(&(response->resolution));
        // return true;
    }
    
    /* Wrapper of the GetResolutions call for use by the ServiceGetAvailableResolutions service */
    void ScanControlDriver::ServiceGetAvailableResolutions(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetAvailableResolutions::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetAvailableResolutions::Response> response
    ){
        guint32 available_resolutions[MAX_RESOLUTION_COUNT] = {0};
        response->return_code = device_interface_ptr->GetResolutions(&available_resolutions[0], MAX_RESOLUTION_COUNT);
        for (int i = 0; i < MAX_RESOLUTION_COUNT; i++){
            if (available_resolutions[i] > 0){
                response->resolutions.push_back(available_resolutions[i]);
            }  
        }
        // return true;
    }

    /* 
    Enable or disable the inversion of Z values on the scanCONTROL device:
        If request->data == true > Enable the inversion of Z values on the scanCONTROL device. (Default of the scanCONTROL device)
        If request->data == false > Disable the inversion of Z values on the scanCONTROL device.
    */
    void ScanControlDriver::ServiceInvertZ(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {   
        unsigned int value;
        int return_code; 

        // Retrieve current settings
        if (return_code = ScanControlDriver::GetFeature(FEATURE_FUNCTION_PROCESSING_PROFILEDATA ,&value) < GENERAL_FUNCTION_OK)
        {   
            // Failed to get PROCESSING feature
            response->success = false;
            response->message = std::string("Failed to get 'Profile Data Processing' feature. Error code: ") + std::to_string(return_code);
            // return true;
        }

        // Set 6th bit according to the SetBool service request
        value = value & ~(1<<6);
        if (request->data) 
        {
            value |= (1<<6);
        }

        // Send the updated settings 
        if (return_code = ScanControlDriver::SetFeature(FEATURE_FUNCTION_PROCESSING_PROFILEDATA, value) < GENERAL_FUNCTION_OK)
        {
            // Failed to set PROCESSING feature
            response->success = false;
            response->message = std::string("Failed to set 'Profile Data Processing' feature. Error code: ") + std::to_string(return_code);
            // return true;
        }

        response->success = true;

        // return true;
    }  

    /* 
    Enable or disable the inversion of X values on the scanCONTROL device:
        If request->data == true > Enable the inversion of X values on the scanCONTROL device. (Default of the scanCONTROL device)
        If request->data == false > Disable the inversion of X values on the scanCONTROL device.
    */
    void ScanControlDriver::ServiceInvertX(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {   
        unsigned int value;
        int return_code; 

        // Retrieve current settings
        if (return_code = ScanControlDriver::GetFeature(FEATURE_FUNCTION_PROCESSING_PROFILEDATA ,&value) < GENERAL_FUNCTION_OK)
        {   
            // Failed to get PROCESSING feature
            response->success = false;
            response->message = std::string("Failed to get 'Profile Data Processing' feature. Error code: ") + std::to_string(return_code);
            // return true;
        }

        // Set 6th bit according to the SetBool service request
        value = value & ~(1<<7);
        if (request->data) 
        {
            value |= (1<<7);
        }

        // Send the updated settings 
        if (return_code = ScanControlDriver::SetFeature(FEATURE_FUNCTION_PROCESSING_PROFILEDATA, value) < GENERAL_FUNCTION_OK)
        {
            // Failed to set PROCESSING feature
            response->success = false;
            response->message = std::string("Failed to set 'Profile Data Processing' feature. Error code: ") + std::to_string(return_code);
            // return true;
        }

        response->success = true;

        // return true;
    }  

    // Generic function for setting a times like exposure and idle time.
    // setting_id: 
    // value: time in microseconds. min: 1 mus, max: 40950 mus ;  eg. 1005 = 1.005ms
    int ScanControlDriver::SetTime(unsigned int setting_id, unsigned int value){
        int ret_code;

        // detailed docs about encoding and decoding here: https://samxl.atlassian.net/l/cp/3fr1eQD0

        // encoded value in 1 mus steps. 
        uint32_t remainder = ((value % 10) << 12) & 0xF000; // Remainder is left shifted first and bits 0-12 are masked. the quotient will occupy this area.
        uint32_t quotient = ((value / 10)) & 0xFFF; // take the quotient and mask bits 12-15
        uint32_t encoded_value = remainder + quotient; 

        ret_code = SetFeature(setting_id, encoded_value); 
        
        // success if value was sent AND set.
        if (ret_code < GENERAL_FUNCTION_OK){
            RCLCPP_ERROR_STREAM(LOGGER, "SetFeature failed. Return code:"<<ret_code);
            return ret_code;
        }
        
        // Check if returned value from laser matches the request
        unsigned int actual_value = 0;
        ret_code = GetTime(setting_id, &actual_value);
        if (actual_value != value){
            RCLCPP_WARN(LOGGER, "Requested value and actual value do not match. ");
            return ret_code;
        }
        return GENERAL_FUNCTION_OK;
    }

    int ScanControlDriver::GetTime(unsigned int setting_id, unsigned int* value){
        int ret_code = 0;
        ret_code = GetFeature(setting_id, value);
        
        if (ret_code < GENERAL_FUNCTION_OK){
            RCLCPP_ERROR_STREAM(LOGGER, "GetFeature failed. Return code: "<<ret_code);
            return ret_code;
        }

        // Decode
        // detailed docs about encoding and decoding here: https://samxl.atlassian.net/l/cp/3fr1eQD0

        uint32_t quotient = *value & 0xFFF;               // Extract ExposureTime / 10
        uint32_t remainder = (*value >> 12) & 0xF;        // Extract ExposureTime % 10
        *value = (quotient * 10) + remainder;  // Reconstruct ExposureTime

        return GENERAL_FUNCTION_OK;

    }


    // a wrapper on setfeature to use proper encoding 
    void ScanControlDriver::ServiceSetExposureTime(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetTime::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetTime::Response> response){
        
        int ret_code = SetTime(FEATURE_FUNCTION_EXPOSURE_TIME, request->time);
        response->success = !(ret_code < GENERAL_FUNCTION_OK);
        response->return_code = ret_code;
    }

    // a wrapper on getfeature to use proper decoding
    void ScanControlDriver::ServiceGetExposureTime(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetTime::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetTime::Response> response){
        response->return_code = GetTime(FEATURE_FUNCTION_EXPOSURE_TIME, &(response->time));
        response->success = !(response->return_code < GENERAL_FUNCTION_OK);
    }

        // a wrapper on setfeature to use proper encoding 
    void ScanControlDriver::ServiceSetIdleTime(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetTime::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetTime::Response> response){
        
        int ret_code = SetTime(FEATURE_FUNCTION_IDLE_TIME, request->time);
        response->success = !(ret_code < GENERAL_FUNCTION_OK);
        response->return_code = ret_code;
    }

    // a wrapper on getfeature to use proper decoding
    void ScanControlDriver::ServiceGetIdleTime(
        const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetTime::Request> request,
        std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetTime::Response> response){
        response->return_code = GetTime(FEATURE_FUNCTION_IDLE_TIME, &(response->time));
        response->success = !(response->return_code < GENERAL_FUNCTION_OK);
    }  

    /* Callback for when a new profile is read, for use with the scanCONTROL API. */
    void NewProfileCallback(const void * data, size_t data_size, gpointer user_data){
        // Cast user_data to a driver class pointer and process and publish point cloud
        ScanControlDriver* driver_ptr = static_cast<ScanControlDriver*>(user_data);

        // Call member function
        driver_ptr->ProcessAndPublishProfile(data, data_size);
    }

    /* Callback for when connection to the device is lost, for use with the scanCONTROL API. */
    void ControlLostCallback(ArvGvDevice *mydevice, gpointer user_data){
        RCLCPP_FATAL(LOGGER, "Conrol of scanCONTROL device lost!");
        rclcpp::shutdown();
    }

} // namespace scancontrol_driver