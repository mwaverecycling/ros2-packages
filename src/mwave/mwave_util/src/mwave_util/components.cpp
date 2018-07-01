#include "mwave_util/components.hpp"

#include <memory>

namespace mwave_util
{
    BroadcastNode::BroadcastNode(const std::string & node_name, const std::string & namespace_, bool use_intra_process_comms)
        : HandledNode (node_name, namespace_, use_intra_process_comms) {  }

    void BroadcastNode::init(rclcpp::executor::Executor::SharedPtr exec)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing BroadcastNode '%s'...", this->get_name());

        this->_bpub = this->create_publisher<mwave_messages::msg::Broadcast>("/broadcast");
        this->_bsub = this->create_subscription<mwave_messages::msg::Broadcast>("/broadcast", std::bind(&BroadcastNode::on_broadcast, this, std::placeholders::_1));

        this->broadcast("state", "ready");
    }

    void BroadcastNode::broadcast(const std::string & type, const std::string & message) {
        _bmsg->type = type;
        _bmsg->name = std::string(this->get_name());
        _bmsg->message = message;
        _bpub->publish(_bmsg);
    }

    void BroadcastNode::on_broadcast(const mwave_messages::msg::Broadcast::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Broadcast[%s]: %s", msg->type.c_str(), msg->message.c_str());
    }
}
