#include "i2cbridge/devices/pca9555.hpp"
#include "i2cbridge/util.hpp"

#include <map>
#include <vector>
#include <chrono>
#include <cstdint>

#include "std_msgs/msg/bool.hpp"

namespace I2CROSBridge
{
	void ConfigurePCA9555OutputPin(uint_fast8_t pin, const std::string & topic, i2cpp::PCA9555::SharedPtr device, mwave_util::HandledNode::SharedPtr node)
	{
        node->add_subscription<std_msgs::msg::Bool>(topic,
            [device, node, pin](const std_msgs::msg::Bool::SharedPtr msg) -> void
        {
            bool value = msg->data;
            RCLCPP_INFO(node->get_logger(), "Writing %s to pin 0x%02x at 0x%02x", value ? "true" : "false", pin, device->get_address());
            device->write_output_pin(pin, value);
        });
	}
    void ConfigurePCA9555InputCallback(uint_fast16_t mask, std::string topics[16], i2cpp::PCA9555::SharedPtr device, mwave_util::HandledNode::SharedPtr node)
    {
        uint_fast16_t prev = device->get_state();
        uint_fast16_t result = device->read_input();
        uint_fast8_t i;
        for(i = 0; i < 16; i++)
        {
            if((mask & (1 << i)) == 0)
            {
                if((prev & (1 << i)) != (result & (1 << i))) {
                    auto msg = std::make_shared<std_msgs::msg::Bool>();
                    msg->data = (result & (1 << i)) > 0;
                    node->get_publisher<std_msgs::msg::Bool>(topics[i])->publish(msg);
                }
            }
        }
        device->set_state(result);
    };


	std::shared_ptr<i2cpp::PCA9555> ConfigurePCA9555(const mwave_config::msg::I2CDevice& config, mwave_util::HandledNode::SharedPtr node)
	{
		std::map<std::string, std::string> options = getOptions(config);
        uint_fast16_t mask = options.find("mode") != options.end()
            ? (options["mode"] == "output" ? 0x0000 : 0xffff)
            : (std::stoul(options.at("mask"), 0, 16) & 0xffff);
		i2cpp::PCA9555::SharedPtr device_ref = std::make_shared<i2cpp::PCA9555>(config.bus, config.address, mask);

        std::string input_topics[16];
		uint_fast8_t i = 0;
        for(i = 0; i < config.topics.size(); i++)
        {
            if(config.topics[i].length() == 0) { continue; }
            // I if it's an output pin
            if((mask & (1 << i)) > 0) { ConfigurePCA9555OutputPin(i, config.topics[i], device_ref, node); }
            //else { ConfigurePCA9555InputPin(i, config.topics[i], device_ref, node); }
            else {
                input_topics[i] = config.topics[i];
                node->add_publisher<std_msgs::msg::Bool>(config.topics[i]);
            }
        }

        auto callback = [mask, input_topics, device_ref, node]() -> void {
            uint_fast16_t prev = device_ref->get_state();
            uint_fast16_t result = device_ref->read_input();
            uint_fast8_t index;
            for(index = 0; index < 16; index++)
            {
                if((mask & (1 << index)) == 0)
                {
                    if((prev & (1 << index)) != (result & (1 << index))) {
                        auto msg = std::make_shared<std_msgs::msg::Bool>();
                        msg->data = (result & (1 << index)) > 0;
                        node->get_publisher<std_msgs::msg::Bool>(input_topics[index])->publish(msg);
                    }
                }
            }
            device_ref->set_state(result);
        };
        device_ref->set_state(~device_ref->read_input());
        
        // Default Sample Rate is 10 Hz
        uint_fast16_t sample_freq = config.frequency == 0 ? 10 : config.frequency;
        int_fast64_t sample_rate = int_fast64_t((double(1) / sample_freq) * 1000);
        node->add_wall_timer(std::chrono::duration<int_fast64_t, std::ratio<1, 1000000>>(sample_rate), callback);

		return device_ref;
	}
}