#include "i2cbridge/util.hpp"

#include <vector>

namespace I2CBridge
{
	std::map<std::string, std::string> getOptions(const mwave_messages::msg::I2CDevice& device)
	{
		std::map<std::string, std::string> ret;

		unsigned int i;
		for(i = 0; i < device.option_keys.size(); i++)
		{
			ret[device.option_keys[i]] = device.option_values[i];
		}

		return ret;
	}
}
