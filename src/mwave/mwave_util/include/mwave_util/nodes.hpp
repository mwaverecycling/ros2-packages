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

            virtual std::shared_future<bool> init();

        protected:
            std::promise<bool> ready_promise;
            std::shared_future<bool> ready_future;
    };
}

#endif