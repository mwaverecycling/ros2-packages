

#include "mwave_util/nodes.hpp"

namespace mwave_util
{
    HandledNode::HandledNode(const std::string & node_name, const std::string & namespace_, bool use_intra_process_comms)
        : Node(node_name, namespace_, use_intra_process_comms)
    {
        this->ready_future = this->ready_promise.get_future();
    }

    std::shared_future<bool> HandledNode::init()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Generic HandledNode '%s'...", this->get_name());
        this->ready_promise.set_value(true);
        return this->ready_future;
    }
}