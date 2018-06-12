#include "i2cbridge/i2cbridge.hpp"
#include "i2cbridge/devices/pca9555.hpp"

namespace I2CROSBridge
{
	I2CBridge::I2CBridge() {  }
	I2CBridge::~I2CBridge() {  }

	template<class HandledNodeT>
	void I2CBridge::configureDevice(const mwave_config::msg::I2CDevice& config, HandledNodeT* node)
	{
		if(config.device == "pca9555")
			this->devices.push_back(ConfigurePCA9555(config, node));

		
	}
}