#ifndef MWAVE_MODULES__IO_COMPONENT_HPP_
#define MWAVE_MODULES__IO_COMPONENT_HPP_

#include "mwave_util/components.hpp"
#include "mwave_messages/srv/fetch_logic_config.hpp"

namespace mwave_module
{
	class Logic : public mwave_util::BroadcastLifecycleNode
	{
	public: 
		using SharedPtr = std::shared_ptr<Logic>;

		explicit Logic(const std::string & node_name, const std::string & namespace = "", bool use_intra_process_comms = false)
			: mwave_util::BroadcastLifecycleNode(node_name, "", intra_proccess_comms);

	private:
		rclcpp::Client<mwave_messages::srv::FetchLogicConfig>::SharedPtr client_;
	};
} //namespace mwave_module
#endif //MWAVE_MODULES__IO_COMPONENT_HPP_