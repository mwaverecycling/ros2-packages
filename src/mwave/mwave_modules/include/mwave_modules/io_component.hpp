#ifndef MWAVE_MODULES__IO_COMPONENT_HPP_
#define MWAVE_MODULES__IO_COMPONENT_HPP_

#include "mwave_util/components.hpp"
#include "mwave_messages/srv/fetch_io_config.hpp"
#include "i2cbridge/i2cbridge.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mwave_modules
{
	class IOComponent : public mwave_util::BroadcastNode
	{
		public:
			using SharedPtr = std::shared_ptr<IOComponent>;

			explicit IOComponent(
				const std::string & node_name,
				const std::string & namespace_ = "",
				bool intra_proccess_comms = false
			);

            virtual void init(rclcpp::executor::Executor::SharedPtr exec) override;

		private:
			rclcpp::Client<mwave_messages::srv::FetchIOConfig>::SharedPtr config_client;
            I2CBridge::I2CBridge i2cbridge;
	};
} //namespace mwave_module
#endif //MWAVE_MODULES__IO_COMPONENT_HPP_
