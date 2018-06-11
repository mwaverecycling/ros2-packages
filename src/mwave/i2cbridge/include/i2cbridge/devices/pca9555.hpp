#ifndef I2CBRIDGE_PCA9555_HPP
#define I2CBRIDGE_PCA9555_HPP

#include <memory>

#include "mwave_config/msg/i2_c_device.hpp"
#include "i2cpp/devices/pca9555.hpp"

namespace I2CROSBridge
{
	template<class HandledNodeT>
	std::shared_ptr<i2cpp::PCA9555> ConfigurePCA9555(
		const mwave_config::msg::I2CDevice& device,
		std::shared_ptr<HandledNodeT> node);
}


#endif // I2CBRIDGE_PCA9555_HPP