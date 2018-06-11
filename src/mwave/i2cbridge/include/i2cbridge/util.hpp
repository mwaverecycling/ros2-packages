#ifndef I2CBRIDGE_UTIL_HPP
#define I2CBRIDGE_UTIL_HPP

#include <string>
#include <map>

#include "mwave_config/msg/i2_c_device.hpp"


namespace I2CROSBridge
{
	std::map<std::string, std::string> getOptions(const mwave_config::msg::I2CDevice& device);
}


#endif // I2CBRIDGE_UTIL_HPP