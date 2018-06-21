#ifndef MWAVE_MODULES__IO_COMPONENT_HPP_
#define MWAVE_MODULES__IO_COMPONENT_HPP_

#include "mwave_util/components.hpp"
#include "mwave_messages/srv/fetch_logic_config.hpp"

namespace mwave_module
{
	class LogicComponent : public mwave_util::BroadcastNode
	{
		public: 
			using SharedPtr = std::shared_ptr<Logic>;
	
			explicit Logic(const std::string & node_name, const std::string & namespace = "");
	
		private:
			rclcpp::Client<mwave_messages::srv::FetchLogicConfig>::SharedPtr client_;
	};
} //namespace mwave_module
#endif //MWAVE_MODULES__IO_COMPONENT_HPP_