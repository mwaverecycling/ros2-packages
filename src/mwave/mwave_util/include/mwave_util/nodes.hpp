#ifndef MWAVEUTIL_HANDLEDNODE_HPP
#define MWAVEUTIL_HANDLEDNODE_HPP

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <vector>
#include <map>

namespace mwave_util
{
    class HandledNode : public rclcpp::Node
    {
        public:
            using SharedPtr = std::shared_ptr<HandledNode>;

            explicit HandledNode(
                const std::string& node_name,
                const std::string& namespace_ = "",
                bool use_intra_process_comms = false
            );

            /**
             * For initializing long-running or ROS-dependent constructors
             */
            virtual void init(rclcpp::executor::Executor::SharedPtr exec);

        protected:
    };
}

#endif
