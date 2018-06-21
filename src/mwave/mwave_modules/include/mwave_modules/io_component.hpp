#ifndef MWAVE_MODULES__IO_COMPONENT_HPP_
#define MWAVE_MODULES__IO_COMPONENT_HPP_

#include "mwave_util/components.hpp"
#include "mwave_messages/srv/fetch_io_config.hpp"

namespace mwave_module
{
	class IOComponent : public mwave_util::BroadcastNode
	{
		public: 
			using SharedPtr = std::shared_ptr<IOComponent>;
	
			explicit IOComponent(const std::string & node_name, const std::string & namespace_ = "");

		private:
			rclcpp::Client<mwave_messages::srv::FetchIOConfig>::SharedPtr client_;
	};
} //namespace mwave_module
#endif //MWAVE_MODULES__IO_COMPONENT_HPP_
