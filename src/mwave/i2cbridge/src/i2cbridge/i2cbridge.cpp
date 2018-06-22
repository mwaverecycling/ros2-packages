/*

#include "i2cbridge/i2cbridge.hpp"
#include "i2cbridge/devices/pca9555.hpp"

namespace I2CROSBridge
{
	I2CBridge::I2CBridge() {  }
	I2CBridge::~I2CBridge() {  }

	template<class HandledNodeT>
	void I2CBridge::configureDevice(const mwave_messages::msg::I2CDevice& config, HandledNodeT* node)
	{
		if(config.device == "pca9555") {
			std::shared_ptr<i2cpp::PCA9555> device = ConfigurePCA9555(config, node);
			this->devices.push_back(device);
		}
	}
}

*/