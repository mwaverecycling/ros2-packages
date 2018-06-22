#include "i2cbridge/i2cbridge.hpp"

namespace I2CBridge
{
	I2CBridge::I2CBridge() {  }
	I2CBridge::~I2CBridge() {  }

	void I2CBridge::configureDevice(const mwave_messages::msg::I2CDevice& config, rclcpp::Node* node)
	{
		if(config.device == "pca9555") {
			PCA9555Bridge::SharedPtr device = std::make_shared<PCA9555Bridge>(config, node);
			this->devices.push_back(device);
		}
	}
}