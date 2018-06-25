// Demo guide at: https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp

#include <memory>
#include <string>

/* 
 * The seed node is the node that spawn and executes tasks of any component of a plant board (i.e. raspberrypi).
 * This should not implement any logic outside of getting it's plant module name (e.g. compactor), 
 * contacting the orchestrator for componet list, and passing the name to the component(s).
 */

#include "mwave_util/components.hpp"
#include "mwave_modules/io_component.hpp"
#include "mwave_modules/logic_component.hpp"
#include "mwave_modules/hmi_component.hpp"

#include "mwave_messages/srv/fetch_config_type.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

	// TODO: Create a config file for plant module name
	std::string name = "testall";

	// TODO: Make this a mutlithreaded executor
	rclcpp::executors::SingleThreadedExecutor exec;
	auto seed_node = std::make_shared<rclcpp::Node>(name);
	// Get config for exec
	// spin till future

	rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedPtr config_client = 
				seed_node->create_client<mwave_messages::srv::FetchConfigType>("/FetchConfigType");

	auto config_request = std::make_shared<mwave_messages::srv::FetchConfigType::Request>();
	config_request->node_name = name;

	auto future_result = config_client->async_send_request(config_request, [&name, &exec](rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedFuture future) -> void {
		for(std::string type : future.get()->nodes) {
			if (type == "ion") {
				exec.add_node(std::make_shared<mwave_modules::IOComponent>(name + "_ion"));				
			} else if (type == "log") {
				exec.add_node(std::make_shared<mwave_modules::LogicComponent>(name + "_log"));
			} else if (type == "hmi") {
				exec.add_node(std::make_shared<mwave_modules::HMIComponent>(name + "_hmi"));
			}			
		}
		//exec.remove_node(seed_node);
	});

	exec.add_node(seed_node);
    exec.spin();
    rclcpp::shutdown();
    
    return 0;
}
