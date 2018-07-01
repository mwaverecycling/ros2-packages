#ifndef I2CBRIDGE_PCA9555_HPP
#define I2CBRIDGE_PCA9555_HPP

#include <memory>
#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mwave_messages/msg/i2_c_device.hpp"
#include "i2cpp/devices/pca9555.hpp"
#include "i2cbridge/util.hpp"

namespace I2CBridge
{
	class PCA9555Bridge : public DeviceBridge
	{
		public:
			using SharedPtr = std::shared_ptr<PCA9555Bridge>;

			PCA9555Bridge(const mwave_messages::msg::I2CDevice& config, rclcpp::Node* node);

			void configure_input_pin(uint_fast8_t pin, const std::string & topic, rclcpp::Node* node);
			void configure_output_pin(uint_fast8_t pin, const std::string & topic, rclcpp::Node* node);
    		void output_callback(const std_msgs::msg::Bool::SharedPtr msg, uint_fast8_t pin);
		    void input_callback();

		private:
			uint_fast8_t input_pin_map[16];
			uint_fast16_t config_mask;
            uint_fast16_t prev_state;
			i2cpp::PCA9555::SharedPtr device_ref;

			std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> pubs;
			std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subs;
			rclcpp::TimerBase::SharedPtr timer;
	};
}


#endif // I2CBRIDGE_PCA9555_HPP
