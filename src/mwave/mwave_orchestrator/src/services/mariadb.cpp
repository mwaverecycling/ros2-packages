#include <memory>
#include <string>
#include <sstream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

// Source: https://github.com/viaduck/mariadbpp
#include <mysql/mysql.h>
#include <mariadb++/connection.hpp>

#include "mwave_messages/msg/i2_c_device.hpp"
#include "mwave_messages/srv/fetch_config_type.hpp"
#include "mwave_messages/srv/fetch_io_config.hpp"
#include "mwave_messages/srv/fetch_hmi_config.hpp"
#include "mwave_messages/srv/fetch_logic_config.hpp"

std::vector<std::string> split(const std::string& str)
{
    std::vector<std::string> ret;
    std::string toke;
    std::istringstream ss(str);

    while(std::getline(ss, toke, ',')) {
        ret.push_back(toke);
    }
    return ret;
}

class OrchestratorConfigNode : public rclcpp::Node
{
    public:
        explicit OrchestratorConfigNode() : Node("OrchestratorConfig")
        {
            int timeout_count = 5;
            this->sql_connection->connect();
            while(!this->sql_connection->connected() && (timeout_count > 0)) {
                RCLCPP_WARN(this->get_logger(), "Database Connection timedout, retrying...");
                usleep(1000000);
            }
            if(!this->sql_connection->connected()) {
                RCLCPP_ERROR(this->get_logger(), "HOLY SHIT DATABASE WONT CONNECT!");
                exit(1);
            }
            this->query_node_types = sql_connection->create_statement("SELECT * FROM node_types WHERE node=?;");
            this->query_config_i2c = sql_connection->create_statement("SELECT * FROM i2c_config WHERE node=?;");
            this->query_topics_i2c = sql_connection->create_statement("SELECT pin, topic FROM i2c_topics WHERE node=? AND bus=? AND address=?;");

            //A callback function for when nodes initiate their configuration. (Message Type: FetchConfigType)
            auto handle_fetch_config_type =
                [this](const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<mwave_messages::srv::FetchConfigType::Request> request,
                       std::shared_ptr<mwave_messages::srv::FetchConfigType::Response> response) -> void
            {
                (void)request_header;
                RCLCPP_INFO(this->get_logger(), "Request of type for node: %s", request->node_name.c_str());
                // Get types from node_types
                query_node_types->set_string(0, request->node_name);
                auto sql_result = query_node_types->query();
                // Populate ret_nodes with type names
                std::vector<std::string> ret_nodes = std::vector<std::string>();
                while (sql_result->next()) {
                    std::string type_name = sql_result->get_string("type");
                    RCLCPP_INFO(this->get_logger(), "Types: %s", type_name.c_str());
                    ret_nodes.push_back(type_name);
                }

                response->nodes = ret_nodes;
		        response->error = "";
            };
            //A callback function for when nodes configure thier input and output devices. (Message Type: FetchIOConfig)
            auto handle_fetch_io_config =
                [this](const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<mwave_messages::srv::FetchIOConfig::Request> request,
                       std::shared_ptr<mwave_messages::srv::FetchIOConfig::Response> response) -> void
            {
                (void)request_header;
                RCLCPP_INFO(this->get_logger(), "Request of input/output for node: %s", request->node.c_str());
                // Get bus, address, device, option_keys, and option_values from i2c_config table
                query_config_i2c->set_string(0, request->node);
                auto sql_result = query_config_i2c->query();
                // Populate transient object with data from i2c_config
                std::vector<mwave_messages::msg::I2CDevice> ret_devices;
                while(sql_result->next()) {
                    auto device = this->parseDevice(sql_result);
                    ret_devices.push_back(*device);
                }
                // Construct the response from the transient objects
                std::vector<mwave_messages::msg::I2CDevice>::iterator itr;
                for(itr = ret_devices.begin(); itr != ret_devices.end(); itr++) {
                    query_topics_i2c->set_string(0, request->node);
                    query_topics_i2c->set_unsigned8(1, itr->bus);
                    query_topics_i2c->set_unsigned8(2, itr->address);
                    // Reassign sql_result to table i2c_topics matching name, bus, and address
                    sql_result = query_topics_i2c->query();
                    itr->topics.resize(sql_result->row_count());
                    while(sql_result->next()) {
                        itr->topics.at(sql_result->get_unsigned8("pin")) = sql_result->get_string("slug");
                    }
                }

                response->devices = ret_devices;
            };
            //A callback function for when nodes configure their HMI(s) (Message Type: FetchHMIConfig)
            auto handle_fetch_hmi_config =
                [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<mwave_messages::srv::FetchHMIConfig::Request> request, std::shared_ptr<mwave_messages::srv::FetchHMIConfig::Response> response) -> void
                {
                    (void)request_header;
                    (void)request;
                    response->dummy_response = "Dummy Response.";
                };
            //A callback function for when nodes configure their logic modules (Message Type: FetchLogicConfig)
            auto handle_fetch_logic_config =
                [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<mwave_messages::srv::FetchLogicConfig::Request> request, std::shared_ptr<mwave_messages::srv::FetchLogicConfig::Response> response) -> void
                {
                    (void)request_header;
                    (void)request;
                    response->dummy_response = "Dummy Response.";
                };

            srv_fetch_config_type = create_service<mwave_messages::srv::FetchConfigType>("/FetchConfigType", handle_fetch_config_type);
            srv_fetch_io_config = create_service<mwave_messages::srv::FetchIOConfig>("/FetchIOConfig", handle_fetch_io_config);
            srv_fetch_hmi_config = create_service<mwave_messages::srv::FetchHMIConfig>("/FetchHMIConfig", handle_fetch_hmi_config);
            srv_fetch_logic_config = create_service<mwave_messages::srv::FetchLogicConfig>("/FetchLogicConfig", handle_fetch_logic_config);

            RCLCPP_INFO(this->get_logger(), "Listening!");
        };

        /*
         * Given a result set from the table i2c_config, this method will return a parsed version as a (I2CDevice) message.
         */
        std::shared_ptr<mwave_messages::msg::I2CDevice> parseDevice(const std::shared_ptr<mariadb::result_set>& sql_result)
        {
            mwave_messages::msg::I2CDevice config;

            config.bus = sql_result->get_unsigned8("bus");
            config.address = sql_result->get_unsigned8("address");
            config.device = sql_result->get_string("device");

            std::string option_keys_raw = sql_result->get_string("option_keys");
            std::string option_values_raw = sql_result->get_string("option_values");
            config.option_keys = split(option_keys_raw);
            config.option_values = split(option_values_raw);

            config.topics = std::vector<std::string>();
            return std::make_shared<mwave_messages::msg::I2CDevice>(config);
        };


    private:
            rclcpp::Service<mwave_messages::srv::FetchConfigType>::SharedPtr srv_fetch_config_type;
            rclcpp::Service<mwave_messages::srv::FetchIOConfig>::SharedPtr srv_fetch_io_config;
            rclcpp::Service<mwave_messages::srv::FetchHMIConfig>::SharedPtr srv_fetch_hmi_config;
            rclcpp::Service<mwave_messages::srv::FetchLogicConfig>::SharedPtr srv_fetch_logic_config;

            //TODO: Load in configs from mwave_orchestrator/private/Orchestrator.db.json
            std::shared_ptr<mariadb::account> sql_account = mariadb::account::create("127.0.0.1", "ros2", "oCXxFFUmBbVbV3gTM35DaTYveA4Ahh2P", "ros2config");
            std::shared_ptr<mariadb::connection> sql_connection = mariadb::connection::create(sql_account);

            std::shared_ptr<mariadb::statement> query_node_types;
            std::shared_ptr<mariadb::statement> query_config_i2c;
            std::shared_ptr<mariadb::statement> query_topics_i2c;
};

int main(int argc, char* argv[])
{
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when exectued simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<OrchestratorConfigNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
