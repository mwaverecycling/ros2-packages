// TODO: Rewrite as a component.
// Guide: https://github.com/ros2/demos/blob/master/composition/include/composition/listener_component.hpp

#include "mwave_modules/io_component.hpp"

#include "rcutils/logging_macros.h"

#include <chrono>
#include <future>


namespace mwave_modules
{
    IOComponent::IOComponent(const std::string & node_name, const std::string & namespace_, bool use_intra_proccess_comms)
        : mwave_util::BroadcastNode(node_name, namespace_, use_intra_proccess_comms) {  }

    void IOComponent::init()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing I/O Component '%s'...", this->get_name());

        /* Configure IO */
        this->config_client = this->create_client<mwave_messages::srv::FetchIOConfig>("/FetchIOConfig");
        // Allow the orchestrator 30 seconds to be avalible as all modules will be coming online together.
        if(!this->config_client->wait_for_service(std::chrono::duration<int64_t, std::ratio<1, 1>>(30)))
        {
            RCLCPP_WARN(this->get_logger(), "Configuration service not avalible after waiting");
            return;
        }
        auto config_request = std::make_shared<mwave_messages::srv::FetchIOConfig::Request>();
        config_request->node = this->get_name();
        auto future = config_client->async_send_request(config_request);
        RCLCPP_INFO(this->get_logger(), "Waiting for future...");
        auto status = future.wait_for(std::chrono::duration<int64_t, std::ratio<1, 1>>(5));
        switch (status) {
            case std::future_status::deferred :
                RCLCPP_INFO(this->get_logger(), "Haven't sent request... yest");
                return;
            case std::future_status::ready :
                RCLCPP_INFO(this->get_logger(), "Response received");
                break;
            case std::future_status::timeout :
                RCLCPP_ERROR(this->get_logger(), "Service Timeout!!");
                return;
        }
        auto result = future.get();

        /* Configure I2C Devices */
        for (mwave_messages::msg::I2CDevice device : result->devices) {
            RCLCPP_INFO(this->get_logger(), "Configuring %s device.", device.device);
            this->i2cbridge.configureDevice(device, this);
        }

        this->broadcast("state", "ready");
    }
} // namespace mwave_modules
