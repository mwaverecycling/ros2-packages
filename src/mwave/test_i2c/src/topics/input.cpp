#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/u_int16.hpp"

extern "C"
{
    #include "i2cfunc.h"
    #include "devices/pca9555.h"
}

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for test_i2c input app:\n");
  printf("input [-s sample_rate] [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-s sample_rate : How many times a second to poll I2C. Defaults to 100.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to test_i2c_input_relay\n");
}

// Create a Test_I2C_Input class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Test_I2C_Input : public rclcpp::Node
{
    public:
        explicit Test_I2C_Input(const std::string & topic_name, std::chrono::duration sample_rate) : Node("test_i2c_input")
        {
            _adapter = i2c_init(1);
            _msg = std::make_shared<std_msgs::msg::UInt16>();

            // Create a function for when messages are to be sent.
            auto poll_i2c = [this]() -> void {
                int status = pca9555_read_input(_adapter, 0x21, _buffer);
                if(status < 2) {
                    RCLCPP_WARN(this->get_logger(), "   Read Failed! [%d]", status);
                }
                _data = (_buffer[0] << 8) | _buffer[1];
                if(_msg->data != _data) {
                    RCLCPP_INFO(this->get_logger(), "Value Changed: 0x%04x -> 0x%04x", _msg->data, _data);
                    _msg->data = _data;
                // Put the message into a queue to be processed by the middleware.
                // This call is non-blocking.
                    _pub->publish(_msg);
                }
            };

            _pub = this->create_publisher<std_msgs::msg::UInt16>(topic_name, rmw_qos_profile_default);

            // Use a timer to schedule periodic message publishing.
            _timer = this->create_wall_timer(sample_rate, poll_i2c);
            RCLCPP_INFO(this->get_logger(), "Listening for I2C changes...");
        }

    private:
        int _adapter;
        uint8_t _buffer[2];
        uint16_t _data;
        std::shared_ptr<std_msgs::msg::UInt16> _msg;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr _pub;
        rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char* argv[])
{
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
        print_usage();
        return 0;
    }

    // Initialize any global resources needed by the middleware and the client library.
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Parse the command line options.
    auto topic = std::string("test_i2c_input_relay");
    if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
        topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
    }
    unsigned int sample_freq = 100;
    if (rcutils_cli_option_exist(argv, argv + argc, "-s")) {
        sample_freq = std::stoi(rcutils_cli_get_option(argv, argv + argc, "-s"));
    }
    double sample_rate_f = 1f / sample_freq;
    std::chrono::duration<double> sample_rate(sample_rate_f);

    // Create a node.
    auto node = std::make_shared<Test_I2C_Input>(topic, sample_rate);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
