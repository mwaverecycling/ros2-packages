#ifndef MWAVE_UTIL__COMPONENT_HPP_
#define MWAVE_UTIL__COMPONENT_HPP_

#include "mwave_util/nodes.hpp"
#include "mwave_messages/msg/broadcast.hpp"

namespace mwave_util
{
	class BroadcastLifecycleNode : public HandledLifecycleNode 
	{
	public: 
		using SharedPtr = std::shared_ptr<BroadcastLifecycleNode>;

		explicit BroadcastLifecycleNode(
                const std::string& node_name, 
                const std::string& namespace_ = "", 
                bool use_intra_process_comms = false
        );

	protected:
		void broadcast(std::string& type, std::string& message);
        virtual void on_broadcast(const mwave_messages::msg::Broadcast::SharedPtr msg);

	private:
		rclcpp::Publisher<mwave_messages::msg::Broadcast>::SharedPtr _pub_broadcast;
        rclcpp::Subscription<mwave_messages::msg::Broadcast>::SharedPtr _sub_broadcast;
        std::shared_ptr<mwave_messages::msg::Broadcast> _msg = std::make_shared<mwave_messages::msg::Broadcast>();
	};
} //namespace mwave_module
#endif //MWAVE_MODULE__MODULE_COMPONENT_HPP_
