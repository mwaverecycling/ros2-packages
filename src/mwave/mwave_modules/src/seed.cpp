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

class SeedNode : public rclcpp::Node
{
	public:
		explicit SeedNode(const std::string & name, rclcpp::executors::SingleThreadedExecutor & exec) : rclcpp::Node(name)
		{
			// Create service client for getting a list of necessary components
			rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedPtr config_client = 
				this->create_client<mwave_messages::srv::FetchConfigType>("/FetchConfigType");
			auto config_request = std::make_shared<mwave_messages::srv::FetchConfigType::Request>();
			// Populate request argments and make request
			config_request->node_name = name;
			auto future = config_client->async_send_request(config_request);
			// The return is an array of char[]'s that represent the type of components: ion, input/output; log, logic; hmi, GUI.
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

	// TODO: Create a config file for plant module name
	std::string name = "testall";
	// TODO: Look into making this a multithreaded executor
	rclcpp::executors::SingleThreadedExecutor exec;
	
	auto seed_node = std::make_shared<SeedNode>(name, exec);
    
    exec.spin();
    
    rclcpp::shutdown();
    
    return 0;
}
