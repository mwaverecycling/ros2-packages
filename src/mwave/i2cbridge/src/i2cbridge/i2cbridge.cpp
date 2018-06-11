#include "i2cbridge/i2cbridge.hpp"
#include "i2cbridge/devices/pca9555.hpp"

namespace I2CROSBridge
{
	template<class HandledNodeT>
	I2CBridge<HandledNodeT>::I2CBridge(std::shared_ptr<HandledNodeT> node) : node(node) {  }
	template<class HandledNodeT>
	I2CBridge<HandledNodeT>::~I2CBridge() {  }

	template<class HandledNodeT>
	void I2CBridge<HandledNodeT>::configureDevice(const mwave_config::msg::I2CDevice& config)
	{
		if(config.device == "pca9555")
			this->devices.push_back(ConfigurePCA9555(config, this->node));

		
	}
}