#include "i2cbridge/i2cbridge.hpp"
#include "i2cbridge/devices/pca9555.hpp"

namespace I2CROSBridge
{
	I2CBridge::I2CBridge(std::shared_ptr<mwave_util::HandledNode> node) : node(node) {  }
	I2CBridge::~I2CBridge() {  }

	void I2CBridge::configureDevice(const mwave_config::msg::I2CDevice& config)
	{
		if(config.device == "pca9555")
			this->devices.push_back(ConfigurePCA9555(config, this->node));

		
	}
}