// Demo guide at: https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp

#include <memory>
#include <string>

#include "mwave_util/components.hpp"
#include "mwave_modules/io_component.hpp"
#include "mwave_modules/logic_component.hpp"
#include "mwave_modules/hmi_component.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

	// TODO: Create a config file to load this in from
	std::string name = "testall";
	
	rclcpp::executors::SingleThreadedExecutor exec;
	
	rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedPtr config_client = 
		std::shared_ptr<mwave_messages::srv::FetchConfigType>("/FetchConfigType");

	auto configRequest = std::makes_shared<mwave_messages::srv::FetchConfigType::Request>();
	configRequest->node_name = name;
	
	auto future = client->async_send_request(configRequest);
	future.wait();
	auto result = future.get();
	for (std::string type : result) {
		if (type == "ion") {
			exec.add_node(std::make_shared<mwave_modules::IOComponent>(name + "_ion"));				
		} else if (type == "log") {
			exec.add_node(std::make_shared<mwave_modules::LogicComponent>(name + "_log"));
		} else if (type == "hmi") {
			exec.add_node(std::make_shared<mwave_modules::HMIComponent>(name + "_hmi"));
		}			
	}
		
    exec.spin();
	
    rclcpp::shutdown();
	
    return 0;
}
