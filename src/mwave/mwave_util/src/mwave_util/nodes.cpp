#include "mwave_util/nodes.hpp"

#include <future>

namespace mwave_util
{
    HandledNode::HandledNode(const std::string & node_name, const std::string & namespace_, bool use_intra_process_comms)
        : Node(node_name, namespace_, use_intra_process_comms) {  }

    void HandledNode::init()
    {
        RCLCPP_INFO(this->get_logger(), "Initialized Generic HandledNode '%s'", this->get_name());
    }
}