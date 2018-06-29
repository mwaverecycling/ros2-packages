#ifndef I2CBRIDGE_CONFIGURE_HPP
#define I2CBRIDGE_CONFIGURE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "mwave_messages/msg/i2_c_device.hpp"
#include "i2cbridge/devices/pca9555bridge.hpp"
#include "i2cpp/device.hpp"

namespace I2CBridge
{
	class I2CBridge
	{
		public:
			I2CBridge();
			~I2CBridge();

			void configureDevice(const mwave_messages::msg::I2CDevice& config, rclcpp::Node* node);

		private:
			std::vector<std::shared_ptr<DeviceBridge>> devices;
	};
}


#endif // I2CBRIDGE_CONFIGURE_HPP
