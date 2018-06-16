#include "mwave_util/components.hpp"
#include "mwave_util/nodes.hpp"

namespace mwave_util
{
    BroadcastLifecycleNode::BroadcastLifecycleNode(
            const std::string & node_name,
            const std::string & namespace_,
            bool use_intra_process_comms
            ) : HandledLifecycleNode (node_name, namespace_, use_intra_process_comms) {
        this->_pub_broadcast = this->create_publisher<mwave_messages::msg::Broadcast>("/broadcast");
        this->_sub_broadcast = this->create_subscription<mwave_messages::msg::Broadcast>("/broadcast", [this](const mwave_messages::msg::Broadcast::SharedPtr msg) -> void {
                this->on_broadcast(msg);
                });
    }

    void BroadcastLifecycleNode::broadcast(std::string& type, std::string& message) {
        _msg->type = type;
        _msg->message = message;
        _pub_broadcast->publish(_msg);
    };

    void BroadcastLifecycleNode::on_broadcast(const mwave_messages::msg::Broadcast::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Broadcast[%s]: %s", msg->type.c_str(), msg->message.c_str());
    };
}
