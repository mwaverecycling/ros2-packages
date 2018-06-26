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

void parse_config(rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedFuture future, rclcpp::Node::SharedPtr seed_node, rclcpp::executors::SingleThreadedExecutor::SharedPtr exec)
{
    std::string node_name(seed_node->get_name());
    for(std::string type : future.get()->nodes)
    {
        mwave_util::BroadcastNode::SharedPtr component_node;
        if (type == "ion") {
            component_node = std::make_shared<mwave_modules::IOComponent>(node_name + "_ion");
        } else if (type == "log") {
            component_node = std::make_shared<mwave_modules::LogicComponent>(node_name + "_log");
        } else if (type == "hmi") {
            component_node = std::make_shared<mwave_modules::HMIComponent>(node_name + "_hmi");
        }
        exec->add_node(component_node);
        component_node->init();
    }
    exec->remove_node(seed_node);
}

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    // TODO: Create a config file for plant module name
    std::string name = "testall";

    // TODO: Make this a mutlithreaded executor
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto seed_node = std::make_shared<rclcpp::Node>(name);
    // Get config for exec
    // spin till future

    rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedPtr config_client =
                seed_node->create_client<mwave_messages::srv::FetchConfigType>("/FetchConfigType");

    auto config_request = std::make_shared<mwave_messages::srv::FetchConfigType::Request>();
    config_request->node_name = name;

    //(void)config_client->async_send_request(config_request, std::bind(parse_config, std::placeholders::_1, seed_node, exec));
    (void)config_client->async_send_request(config_request, [seed_node, exec](rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedFuture future) -> void {
        parse_config(future, seed_node, exec);
    });

    exec->add_node(seed_node);
    exec->spin();
    rclcpp::shutdown();

    return 0;
}
