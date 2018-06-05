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

void print_usage()
{
    printf("Usage for test_i2c output app:\n");
    printf("output [-t topic_name] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-t topic_name : Specify the topic on which to subscribe. Defaults to test_i2c_output_relay\n");
}

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Test_I2C_Output : public rclcpp::Node
{
    public:
        explicit Test_I2C_Output(const std::string & topic_name) : Node("test_i2c_output")
        {
            _adapter = i2c_init(1);

            // Create a callback function for when messages are received.
            // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
            auto set_i2c = [this](const std_msgs::msg::UInt16::SharedPtr msg) -> void {
                RCLCPP_INFO(this->get_logger(), "Writing 0x%04x to 0x20", msg->data);
                _buffer[0] = (msg->data >> 8) & 0xff;
                _buffer[1] = msg->data & 0xff;
                int status = pca9555_write_output(_adapter, 0x20, _buffer);
                if(status < 2) {
                    RCLCPP_WARN(this->get_logger(), "   Write Failed! [%d]", status);
                }
            };

            // Create a subscription to the topic which can be matched with one or more compatible ROS
            // publishers.
            // Note that not all publishers on the same topic with the same type will be compatible:
            // they must have compatible Quality of Service policies.
            _sub = create_subscription<std_msgs::msg::UInt16>(topic_name, set_i2c);
            RCLCPP_INFO(this->get_logger(), "Waiting to change I2C...");
        }

    private:
        int _adapter;
        uint8_t _buffer[2];
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _sub;
};

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
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
    auto topic = std::string("test_i2c_output_relay");
    if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
        topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
    }

    // Create a node.
    auto node = std::make_shared<Test_I2C_Output>(topic);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
