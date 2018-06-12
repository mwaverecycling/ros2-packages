#ifndef I2CBRIDGE_CONFIGURE_HPP
#define I2CBRIDGE_CONFIGURE_HPP

#include <memory>
#include <vector>

#include "mwave_config/msg/i2_c_device.hpp"
#include "i2cpp/device.hpp"

namespace I2CROSBridge
{
	class I2CBridge
	{
		public:
			I2CBridge();
			~I2CBridge();

			template<class HandledNodeT>
			void configureDevice(const mwave_config::msg::I2CDevice& config, HandledNodeT* node);

		private:
			std::vector<std::shared_ptr<i2cpp::Device>> devices;
	};
}


#endif // I2CBRIDGE_CONFIGURE_HPP