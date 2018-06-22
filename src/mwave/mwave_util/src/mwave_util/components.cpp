#include "mwave_util/components.hpp"

#include <memory>

namespace mwave_util
{
    BroadcastNode::BroadcastNode(const std::string & node_name, const std::string & namespace_, bool use_intra_process_comms)
        : HandledNode (node_name, namespace_, use_intra_process_comms)
    {
        this->_bpub = this->create_publisher<mwave_messages::msg::Broadcast>("/broadcast");
        this->_bsub = this->create_subscription<mwave_messages::msg::Broadcast>("/broadcast", std::bind(&BroadcastNode::on_broadcast, this, std::placeholders::_1));
        //this->_bsub = this->create_subscription<mwave_messages::msg::Broadcast>("/broadcast", [this](const mwave_messages::msg::Broadcast::SharedPtr msg) -> void {
        //    this->on_broadcast(msg);
        //});
    }

    void BroadcastNode::broadcast(std::string& type, std::string& message) {
        _bmsg->type = type;
        _bmsg->message = message;
        _bpub->publish(_bmsg);
    }

    void BroadcastNode::on_broadcast(const mwave_messages::msg::Broadcast::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Broadcast[%s]: %s", msg->type.c_str(), msg->message.c_str());
    }
}
