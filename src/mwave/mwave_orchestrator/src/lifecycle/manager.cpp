#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transistion.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

class LifecycleServiceClient : public rclcpp::Node
{
    public:
        explicit LifecycleServiceClient(const std::string & node_name) : Node(node_name)
        {}

        void
            init()
            {
                // Get array of nodes from DB table 'ros2config:nodes'
                // {node_name}/get_state
                // {node_name}/change_state
            }

    private:
            std::shared_ptr<mariadb::account> sql_account = mariadb::account::create("127.0.0.1", "ros2", "oCXxFFUmBbVbV3gTM35DaTYveA4Ahh2P", "ros2config");
            std::shared_ptr<mariadb::connection> sql_connection = mariadb::connection::create(sql_account);
}

// Main

// TODO: Look into how to listen to the graph for new nodes coming online
// http://docs.ros2.org/beta1/api/rcl/guard__condition_8h.html -- rcl_guard_condition_init()
// 1. Verify the node with DB
// 2. Setup expected lifecycle services
// 3. Attempt to configure
