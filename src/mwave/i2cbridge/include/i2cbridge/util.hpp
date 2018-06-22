#ifndef I2CBRIDGE_UTIL_HPP
#define I2CBRIDGE_UTIL_HPP

#include <string>
#include <map>

#include "mwave_messages/msg/i2_c_device.hpp"


namespace I2CBridge
{
	std::map<std::string, std::string> getOptions(const mwave_messages::msg::I2CDevice& device);

	class DeviceBridge
	{

	};
}


#endif // I2CBRIDGE_UTIL_HPP