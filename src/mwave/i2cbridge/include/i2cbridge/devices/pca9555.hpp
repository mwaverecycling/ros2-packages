#ifndef I2CBRIDGE_PCA9555_HPP
#define I2CBRIDGE_PCA9555_HPP

#include <memory>

#include "mwave_messages/msg/i2_c_device.hpp"
#include "i2cpp/devices/pca9555.hpp"

namespace I2CROSBridge
{
	template<class HandledNodeT>
	std::shared_ptr<i2cpp::PCA9555> ConfigurePCA9555(
		const mwave_messages::msg::I2CDevice& device,
		HandledNodeT* node);
}


#endif // I2CBRIDGE_PCA9555_HPP