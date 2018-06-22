// Demo guide at: https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp

#include <memory>
#include <string>

#include "mwave_util/components.hpp"
#include "mwave_modules/io_component.hpp"
#include "mwave_modules/logic_component.hpp"
#include "mwave_modules/hmi_component.hpp"

#include "mwave_messages/srv/fetch_config_type.hpp"

#include "rclcpp/rclcpp.hpp"

class SeedNode : public rclcpp::Node
{
	public:
		explicit SeedNode(const std::string & name, rclcpp::executors::SingleThreadedExecutor & exec) : rclcpp::Node(name)
		{
			rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedPtr config_client = 
				this->create_client<mwave_messages::srv::FetchConfigType>("/FetchConfigType");

			auto config_request = std::make_shared<mwave_messages::srv::FetchConfigType::Request>();
			config_request->node_name = name;
			
			auto future = config_client->async_send_request(config_request);
			auto result = future.get();
			for (std::string type : result->nodes) {
				if (type == "ion") {
					exec.add_node(std::make_shared<mwave_modules::IOComponent>(name + "_ion"));				
				} else if (type == "log") {
					exec.add_node(std::make_shared<mwave_modules::LogicComponent>(name + "_log"));
				} else if (type == "hmi") {
					exec.add_node(std::make_shared<mwave_modules::HMIComponent>(name + "_hmi"));
				}			
			}
		}
};

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

	// TODO: Create a config file to load this in from
	std::string name = "testall";
	rclcpp::executors::SingleThreadedExecutor exec;
	
	auto seed_node = std::make_shared<SeedNode>(name, exec);
		
    exec.spin();
	
    rclcpp::shutdown();
	
    return 0;
}
