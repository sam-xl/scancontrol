#include <rclcpp/rclcpp.hpp>

#include <llt.h>
#include <mescan.h>
#include <vector> 
#include <micro_epsilon_scancontrol_msgs/srv/get_resolution.hpp>
#include <micro_epsilon_scancontrol_msgs/srv/set_resolution.hpp>
#include <functional>
#include <iostream>


#define MAX_INTERFACE_COUNT 5
#define MAX_RESOLUTION 6


guint32 resolution;
std::vector<guint8> profile_buffer;
TScannerType llt_type;
guint32 profile_count, needed_profile_count;
guint32 profile_data_size;

EHANDLE* event;



class TestNode : public rclcpp::Node{
    public:
        TestNode();
        void getResolutionWrapper(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Request> request, std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Response> response);
        void setResolutionWrapper(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Request> request, std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Response> response);
        // void getResolutionWrapper();
    private:
        rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::GetResolution>::SharedPtr service;
        rclcpp::Service<micro_epsilon_scancontrol_msgs::srv::SetResolution>::SharedPtr set_resolution_srv;

        std::unique_ptr<CInterfaceLLT> hLLT;

};

TestNode::TestNode():rclcpp::Node("test_node"){
    RCLCPP_INFO(this->get_logger(), "test node started");

    gint32 ret = 0;

    char *interfaces[MAX_INTERFACE_COUNT];
    guint32 resolutions[MAX_RESOLUTION];
    guint32 interface_count = 0;

    guint32 idle_time = 3900;
    guint32 exposure_time = 100;


    service = this->create_service<micro_epsilon_scancontrol_msgs::srv::GetResolution>("get_resolution2", std::bind(&TestNode::getResolutionWrapper, this, std::placeholders::_1, std::placeholders::_2));
    set_resolution_srv = this->create_service<micro_epsilon_scancontrol_msgs::srv::SetResolution>("set_resolution", std::bind(&TestNode::setResolutionWrapper, this, std::placeholders::_1, std::placeholders::_2));

    if ((ret = CInterfaceLLT::GetDeviceInterfaces(&interfaces[0], MAX_INTERFACE_COUNT)) ==
        ERROR_GETDEVINTERFACE_REQUEST_COUNT) {
        std::cout << "There are more than " << MAX_INTERFACE_COUNT << " scanCONTROL connected" << std::endl;
        interface_count = MAX_INTERFACE_COUNT;
    } else if (ret < 1) {
        std::cout << "A error occured during searching for connected scanCONTROL" << std::endl;
        interface_count = 0;
    } else {
        interface_count = ret;
    }

    if (interface_count == 0) {
        std::cout << "There is no scanCONTROL connected - Exiting" << std::endl;
        exit(0);
    } else if (interface_count == 1) {
        std::cout << "There is 1 scanCONTROL connected " << std::endl;
    } else {
        std::cout << "There are " << interface_count << " scanCONTROL connected" << std::endl;
    }

    for (guint32 i = 0; i < interface_count; i++) {
        std::cout << interfaces[i] << "" << std::endl;
    }

     // new LLT instance
    hLLT =  std::make_unique<CInterfaceLLT>();
    event = CInterfaceLLT::CreateEvent();

    if ((ret = hLLT->SetDeviceInterface(interfaces[1])) < GENERAL_FUNCTION_OK) {
        std::cout << "Error while setting dev id " << ret << "!" << std::endl;
        goto cleanup;
    }

    // connect to sensor
    if ((ret = hLLT->Connect()) < GENERAL_FUNCTION_OK) {
        std::cout << "Error while connecting to camera - Error " << ret << "!" << std::endl;
        goto cleanup;
    }

    if ((ret = hLLT->GetLLTType(&llt_type)) < GENERAL_FUNCTION_OK) {
        std::cout << "Error while GetLLTType!" << std::endl;
        goto cleanup;
    }

    if (ret == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED) {
        std::cout << "Can't decode scanCONTROL type. Please contact Micro Epsilon for a newer version of the library."
                  << std::endl;
        goto cleanup;
    }

    if (llt_type >= scanCONTROL27xx_25 && llt_type <= scanCONTROL27xx_xxx) {
        std::cout << "The scanCONTROL is a scanCONTROL27xx" << std::endl;
    } else if (llt_type >= scanCONTROL26xx_25 && llt_type <= scanCONTROL26xx_xxx) {
        std::cout << "The scanCONTROL is a scanCONTROL26xx" << std::endl;
    } else if (llt_type >= scanCONTROL29xx_25 && llt_type <= scanCONTROL29xx_xxx) {
        std::cout << "The scanCONTROL is a scanCONTROL29xx" << std::endl;
    } else if (llt_type >= scanCONTROL30xx_25 && llt_type <= scanCONTROL30xx_xxx) {
        std::cout << "The scanCONTROL is a scanCONTROL30xx" << std::endl;
    } else if (llt_type >= scanCONTROL25xx_25 && llt_type <= scanCONTROL25xx_xxx) {
        std::cout << "The scanCONTROL is a scanCONTROL25xx" << std::endl;
    } else {
        std::cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK" << std::endl;
        goto cleanup;
    }

    std::cout << "Get all possible resolutions" << std::endl;
    if ((ret = hLLT->GetResolutions(&resolutions[0], MAX_RESOLUTION)) < GENERAL_FUNCTION_OK) {
        std::cout << "Error GetResolutions!" << std::endl;
        goto cleanup;
    }
    uint32_t res;
    hLLT->GetResolution(&res);
    std::cout<<res<<std::endl;

cleanup:
    // delete hLLT;
    CInterfaceLLT::FreeEvent(event);
}

void TestNode::getResolutionWrapper(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Request> request, std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::GetResolution::Response> response){
    
    RCLCPP_INFO(this->get_logger(), "service called");
    uint32_t res;
    hLLT->GetResolution(&res);
    std::cout<<res<<std::endl;
    response->return_code = hLLT->GetResolution(&(response->resolution));

}

void TestNode::setResolutionWrapper(const std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Request> request, std::shared_ptr<micro_epsilon_scancontrol_msgs::srv::SetResolution::Response> response){
    
    RCLCPP_INFO(this->get_logger(), "set resolution srv called");

    int return_code;

    if (return_code = hLLT->SetResolution(request->resolution) < 1){
        RCLCPP_FATAL_STREAM(this->get_logger(), "Error while setting device resolution! CodeL " << return_code);
    }
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    std::shared_ptr<TestNode> node = std::make_shared<TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

}