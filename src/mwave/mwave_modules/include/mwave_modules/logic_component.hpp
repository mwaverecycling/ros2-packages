#ifndef MWAVE_MODULES__LOGIC_COMPONENT_HPP_
#define MWAVE_MODULES__LOGIC_COMPONENT_HPP_

#include "mwave_util/components.hpp"
#include "mwave_messages/srv/fetch_logic_config.hpp"

namespace mwave_modules
{
	class LogicComponent : public mwave_util::BroadcastNode
	{
		public: 
			using SharedPtr = std::shared_ptr<LogicComponent>;
	
			explicit LogicComponent(const std::string & node_name, const std::string & namespace_ = "");
	
		private:
			rclcpp::Client<mwave_messages::srv::FetchLogicConfig>::SharedPtr client_;
	};
} //namespace mwave_module
#endif //MWAVE_MODULES__LOGIC_COMPONENT_HPP_