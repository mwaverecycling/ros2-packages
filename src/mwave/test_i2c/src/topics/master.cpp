#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <cstdlib>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/u_int16.hpp"

#define TRIAL_COUNT 4096



using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for test_i2c_master app:\n");
  printf("master [-o out-topic] [-i in-topic] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-o topic_name : Specify the topic on which to publish. Defaults to test_i2c_output_relay\n");
  printf("-i topic_name : Specify the topic on which to subscribe. Defaults to test_i2c_input_relay\n");
}

// Create a Test_I2C_Input class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Test_I2C_Master : public rclcpp::Node
{
    public:
        explicit Test_I2C_Master(const std::string & out_topic, const std::string & in_topic) : Node("test_i2c_master")
        {
            _msg = std::make_shared<std_msgs::msg::UInt16>();
            _msg->data = 0x0000;

            // Create a function for when messages are to be sent.
            auto callback = [this](const std_msgs::msg::UInt16::SharedPtr msg) -> void {
                if(msg->data != _data) {
                    RCLCPP_WARN(this->get_logger(), "Error: 0x%04x != 0x%04x", msg->data, _data);
                    errors++;
                }

                m_EndTime = std::chrono::system_clock::now();
                elapsed += std::chrono::duration_cast<std::chrono::milliseconds>(m_EndTime - m_StartTime).count();

                count++;
                if(count < TRIAL_COUNT) {
                    m_StartTime = std::chrono::system_clock::now();
                    _data = rand() & 0xffff;
                    _msg->data = _data;
                    _pub->publish(_msg);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Finished:\n   Total Time: %5.3fs\n   Average Time: %5.2fms\n   Errors: %d/%d", elapsed / 1000, elapsed / TRIAL_COUNT, errors, TRIAL_COUNT);
                }
            };

            _sub = create_subscription<std_msgs::msg::UInt16>(in_topic, callback);

            // Create a publisher with a custom Quality of Service profile.
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
            custom_qos_profile.depth = 7;
            _pub = this->create_publisher<std_msgs::msg::UInt16>(out_topic, custom_qos_profile);

            m_StartTime = std::chrono::system_clock::now();
            _pub->publish(_msg);
        }

    private:
        uint16_t _data;
        std::shared_ptr<std_msgs::msg::UInt16> _msg;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr _pub;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _sub;

        std::chrono::time_point<std::chrono::system_clock> m_StartTime;
        std::chrono::time_point<std::chrono::system_clock> m_EndTime;
        double elapsed = 0;
        uint16_t count = 0;
        uint16_t errors = 0;
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
    auto out_topic = std::string("test_i2c_output_relay");
    if (rcutils_cli_option_exist(argv, argv + argc, "-o")) {
        topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-o"));
    }
    auto in_topic = std::string("test_i2c_input_relay");
    if (rcutils_cli_option_exist(argv, argv + argc, "-i")) {
        topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-i"));
    }

    // Create a node.
    auto node = std::make_shared<Test_I2C_Master>(out_topic, in_topic);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
