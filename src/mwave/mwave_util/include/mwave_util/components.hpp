#ifndef MWAVE_UTIL__COMPONENT_HPP_
#define MWAVE_UTIL__COMPONENT_HPP_

#include "mwave_util/nodes.hpp"
#include "mwave_messages/msg/broadcast.hpp"

namespace mwave_util
{
    class BroadcastNode : public HandledNode 
    {
        public: 
            using SharedPtr = std::shared_ptr<BroadcastNode>;

            explicit BroadcastNode(
                const std::string& node_name, 
                const std::string& namespace_ = "", 
                bool use_intra_process_comms = false
            );

            virtual std::shared_future<bool> init() override;

        protected:
            void broadcast(std::string& type, std::string& message);
            virtual void on_broadcast(const mwave_messages::msg::Broadcast::SharedPtr msg);

        private:
            /** Broadcast Publisher */
            rclcpp::Publisher<mwave_messages::msg::Broadcast>::SharedPtr _bpub;
            /** Broadcast Subscriber */
            rclcpp::Subscription<mwave_messages::msg::Broadcast>::SharedPtr _bsub;
            /** Broadcast Message */
            std::shared_ptr<mwave_messages::msg::Broadcast> _bmsg = std::make_shared<mwave_messages::msg::Broadcast>();
    };
}

#endif //MWAVE_MODULE__MODULE_COMPONENT_HPP_
