// Demo guide at: https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp

#include <memory>
#include <string>
#include <future>

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

void parse_config(rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedFuture future, rclcpp::Node::SharedPtr seed_node, rclcpp::executor::Executor::SharedPtr exec)
{
    //std::vector<std::future<void>> ret;

    std::string node_name(seed_node->get_name());
    RCLCPP_INFO(seed_node->get_logger(), "Received configuration for '%s'", seed_node->get_name());
    std::vector<std::string> node_types = future.get()->nodes;
    for(std::string type : node_types)
    //std::vector<std::string>::iterator type_itr;
    //for(type_itr = node_types.begin(); type_itr != node_types.end(); type_itr++)
    {
        //std::string type = *type_itr;
        RCLCPP_INFO(seed_node->get_logger(), "Component needed: %s", type.c_str());
        mwave_util::BroadcastNode::SharedPtr component_node;

        if (type == "ion") {
            component_node = std::make_shared<mwave_modules::IOComponent>(node_name + "_ion");
        } else if (type == "log") {
            component_node = std::make_shared<mwave_modules::LogicComponent>(node_name + "_log");
        } else if (type == "hmi") {
            component_node = std::make_shared<mwave_modules::HMIComponent>(node_name + "_hmi");
        }

        exec->add_node(component_node);
    	//ret.push_back(std::async(std::launch::async, [](mwave_util::BroadcastNode::SharedPtr nod) -> void { nod->init(); }, component_node));
    }
    if(node_types.size() > 0) {
        RCLCPP_INFO(seed_node->get_logger(), "'%s' configured!", seed_node->get_name());
        exec->remove_node(seed_node);
    }
    else { RCLCPP_WARN(seed_node->get_logger(), "There were no components for node '%s'", seed_node->get_name()); }

    //return ret;
}

std::vector<mwave_util::BroadcastNode::SharedPtr> config_script (rclcpp::Node::SharedPtr node) {
    rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedPtr config_client =
        node->create_client<mwave_messages::srv::FetchConfigType>("/FetchConfigType");

    auto config_request = std::make_shared<mwave_messages::srv::FetchConfigType::Request>();
    std::string name(node->get_name());
    config_request->node_name = name;

    auto future = config_client->async_send_request(config_request);

    std::vector<std::string> node_types = future.get()->nodes;
    std::vector<mwave_util::BroadcastNode::SharedPtr> ret_nodes;

    for(std::string type : node_types)
    //std::vector<std::string>::iterator type_itr;
    //for(type_itr = node_types.begin(); type_itr != node_types.end(); type_itr++)
    {
        //std::string type = *type_itr;
        RCLCPP_INFO(node->get_logger(), "Component needed: %s", type.c_str());
        mwave_util::BroadcastNode::SharedPtr component_node;

        if (type == "ion") {
            component_node = std::make_shared<mwave_modules::IOComponent>(name + "_ion");
        } else if (type == "log") {
            component_node = std::make_shared<mwave_modules::LogicComponent>(name + "_log");
        } else if (type == "hmi") {
            component_node = std::make_shared<mwave_modules::HMIComponent>(name + "_hmi");
        }

        ret_nodes.push_back(component_node);
    }
    if(node_types.size() > 0 && node_types.size() == ret_nodes.size()) {
        RCLCPP_INFO(node->get_logger(), "'%s' configured!", node->get_name());
    }
    else { RCLCPP_WARN(node->get_logger(), "There were no components for node '%s'", node->get_name()); }
    return ret_nodes;
}

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    // TODO: Create a config file for plant module name
    std::string name = "testi2c";

    auto seed_node = std::make_shared<rclcpp::Node>(name);

    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    //std::shared_future<void> script = std::async(std::launch::async, std::bind(config_script, seed_node, exec));

    //rclcpp::spin_until_future_complete(seed_node, script);

    auto component_nodes = config_script(seed_node);
    for(auto cnode : component_nodes)
    {
        cnode->init();
        exec->add_node(cnode);
    }

    exec->spin();
    /*
    // TODO: Make this a mutlithreaded executor
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto seed_node = std::make_shared<rclcpp::Node>(name);
    RCLCPP_INFO(seed_node->get_logger(), "Fetching configuration for '%s'", seed_node->get_name());

    rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedPtr config_client =
                seed_node->create_client<mwave_messages::srv::FetchConfigType>("/FetchConfigType");

    auto config_request = std::make_shared<mwave_messages::srv::FetchConfigType::Request>();
    config_request->node_name = name;

    //(void)config_client->async_send_request(config_request, std::bind(parse_config, std::placeholders::_1, seed_node, exec));
    (void)config_client->async_send_request(config_request, [seed_node, exec](rclcpp::Client<mwave_messages::srv::FetchConfigType>::SharedFuture future) -> void {
        (void)parse_config(future, seed_node, exec);
    });

    exec->add_node(seed_node);
    exec->spin();
    */

    rclcpp::shutdown();

    return 0;
}
