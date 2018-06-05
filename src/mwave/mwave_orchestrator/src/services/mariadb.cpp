#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// Source: https://github.com/viaduck/mariadbpp
#include <mysql/mysql.h>
#include <mariadb++/connection.hpp>
//#include "mariadb++/account.hpp"
//#include "mariadb++/statement.hpp"
//#include "mariadb++/result_set.hpp"

#include "mwave_config/msg/i2_c_device.hpp"
#include "mwave_config/srv/config_type.hpp"
#include "mwave_config/srv/config_hmi.hpp"
#include "mwave_config/srv/config_log.hpp"
#include "mwave_config/srv/config_ion.hpp"

class Maria_DB_Node : public rclcpp::Node
{
public:
    explicit Maria_DB_Node(const std::string & service_name)
       : Node("maria_db")
    {

        //Create a callback function for when nodes initiate their configuration.
        /**
         * handle_fetch_type will expect a message of ConfigType.
         * Params:
         * - node_name (String)
         * Returns:
         * - node_type (Array of 3 character strings)
         */ 
        auto handle_fetch_type =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<mwave_config::srv::ConfigType::Request> request,
                std::shared_ptr<mwave_config::srv::ConfigType::Response> response) -> void
                {
                    (void)request_header;
                    RCLCPP_INFO(this->get_logger(), "Node fetching type: %s", request->node_name.c_str());

                    std::string query = "SELECT * FROM nodes WHERE slug = '" + request->node_name + "';";
                    //std::string query = "SELECT * FROM nodes;";
                    std::shared_ptr<mariadb::result_set> sql_result = sql_connection->query(query);
                    if (sql_result->next() ) {
                        // Return first row data
                        response->err_string = ""; 
                        response->node_type = sql_result->get_string(2);
                    } else {
                        // There is no row to retrieve
                        std::string err_msg = "Node name '" + request->node_name + "' is not a registed node in the database table 'nodes'.";
                        response->err_string = err_msg;
                        response->node_type = "";
                        return;
                    }
                };
        //TODO: Create a new generic service that is the first called to return the type of node it is and thus what config service it should use
        // Types: HMI, LOGIC, IO
        // TODO: Create module specific config services
        // Create a service that will use the callback function to handle requests.
        srv_ = create_service<mwave_config::srv::ConfigType>(service_name, handle_fetch_type);
    };

private:
        rclcpp::Service<mwave_config::srv::ConfigType>::SharedPtr srv_;
        //TODO: Create configuration file for database connection info.
        std::shared_ptr<mariadb::account> sql_account = mariadb::account::create("127.0.0.1", "ros2", "oCXxFFUmBbVbV3gTM35DaTYveA4Ahh2P", "ros2config");
        std::shared_ptr<mariadb::connection> sql_connection = mariadb::connection::create(sql_account);
        
        // DB Dynamic Queries
        std::shared_ptr<mariadb::statement> query_node_name = sql_connection->create_statement("SELECT * FROM nodes WHERE slug = '?';");
        //auto query_i2c = sql_connection.create_statement("SELECT * FROM i2c_config WHERE node = '?'");
        //auto query_topics = sql_connection.create_statement("SELECT * FROM topics WHERE node = '?'");
};

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when exectued simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

    auto service_name = std::string("fetch_type");
    
    auto node = std::make_shared<Maria_DB_Node>(service_name);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
