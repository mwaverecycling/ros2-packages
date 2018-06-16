// Demo guide at: https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp

#include <memory>

#include "mwave_util/components.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;

    auto broadcast = std::make_shared<mwave_util::BroadcastLifecycleNode>("test");
    exec.add_node(broadcast->get_node_base_interface());

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
