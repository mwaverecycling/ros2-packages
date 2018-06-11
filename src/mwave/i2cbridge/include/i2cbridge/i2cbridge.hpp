#ifndef I2CBRIDGE_CONFIGURE_HPP
#define I2CBRIDGE_CONFIGURE_HPP

#include <memory>
#include <vector>

#include "mwave_config/msg/i2_c_device.hpp"
#include "i2cpp/device.hpp"

namespace I2CROSBridge
{
	template<class HandledNodeT>
	class I2CBridge
	{
		public:
			I2CBridge(std::shared_ptr<HandledNodeT> node);
			~I2CBridge();

			void configureDevice(const mwave_config::msg::I2CDevice& config);

		private:
			std::shared_ptr<HandledNodeT> node;
			std::vector<std::shared_ptr<i2cpp::Device>> devices;
	};
}


#endif // I2CBRIDGE_CONFIGURE_HPP