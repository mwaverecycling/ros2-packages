#include "i2cbridge/devices/pca9555bridge.hpp"

#include <map>
#include <chrono>
#include <cstdint>
#include <functional>

#include "rcutils/logging_macros.h"

namespace I2CBridge
{
    PCA9555Bridge::PCA9555Bridge(const mwave_messages::msg::I2CDevice& config, rclcpp::Node* node)
    {
        std::map<std::string, std::string> options = getOptions(config);
        this->config_mask = options.find("mode") != options.end()
            ? (options["mode"] == "output" ? 0x0000 : 0xffff)
            : (std::stoul(options.at("mask"), 0, 16) & 0xffff);
        this->device_ref = std::make_shared<::i2cpp::PCA9555>(int(config.bus), uint_fast8_t(config.address));
        this->device_ref->write_config(this->config_mask);

        uint_fast8_t i = 0;
        for(i = 0; i < config.topics.size(); i++)
        {
            if(config.topics[i].length() == 0) { continue; }
            // Check if Input Pin
            if((this->config_mask & (1 << i)) == 0) {
                this->configure_input_pin(i, config.topics[i], node);
            } else {
                this->configure_output_pin(i, config.topics[i], node);
            }
        }

        this->device_ref->set_state(~(this->device_ref->read_input()));

        // Default Sample Rate is 10 Hz
        uint_fast16_t sample_freq = config.frequency == 0 ? 10 : config.frequency;
        int_fast64_t sample_rate = int_fast64_t((double(1) / sample_freq) * 1000);
        this->timer = node->create_wall_timer(std::chrono::duration<int_fast64_t, std::ratio<1, 1000000>>(sample_rate),
            std::bind(&PCA9555Bridge::input_callback, this));
    }

    void PCA9555Bridge::configure_input_pin(uint_fast8_t pin, const std::string & topic, rclcpp::Node* node)
    {
        this->input_pin_map[pin] = this->pubs.size();
        auto pub = node->create_publisher<std_msgs::msg::Bool>(topic);
        this->pubs.push_back(pub);
    }
    void PCA9555Bridge::configure_output_pin(uint_fast8_t pin, const std::string & topic, rclcpp::Node* node)
    {
        auto sub = node->create_subscription<std_msgs::msg::Bool>(topic, [this, pin](const std_msgs::msg::Bool::SharedPtr msg) -> void {
            this->output_callback(msg, pin);
        });
        //auto sub = node->create_subscription<std_msgs::msg::Bool>(topic,
        //    std::bind(&PCA9555Bridge::output_callback, this, std::placeholders::_1, pin));
        this->subs.push_back(sub);
    }

    void PCA9555Bridge::output_callback(const std_msgs::msg::Bool::SharedPtr msg, uint_fast8_t pin)
    {
        bool value = msg->data;
        //RCLCPP_INFO(node->template get_logger(), "Writing %s to pin 0x%02x at 0x%02x", value ? "true" : "false", pin, device->get_address());
        this->device_ref->write_output_pin(pin, value);
    }
    void PCA9555Bridge::input_callback()
    {
        uint_fast16_t prev = this->device_ref->get_state();
        uint_fast16_t result = this->device_ref->read_input();
        uint_fast8_t i;
        for(i = 0; i < 16; i++)
        {
            if((this->config_mask & (1 << i)) == 0)
            {
                if((prev & (1 << i)) != (result & (1 << i))) {
                    auto msg = std::make_shared<std_msgs::msg::Bool>();
                    msg->data = (result & (1 << i)) > 0;
                    this->pubs[this->input_pin_map[i]]->publish(msg);
                }
            }
        }
        this->device_ref->set_state(result);
    }
}
