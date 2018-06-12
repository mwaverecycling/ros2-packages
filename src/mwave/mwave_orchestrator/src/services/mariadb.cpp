#include <memory>
#include <string>
#include <sstream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

// Source: https://github.com/viaduck/mariadbpp
#include <mysql/mysql.h>
#include <mariadb++/connection.hpp>

#include "mwave_config/msg/i2_c_device.hpp"
#include "mwave_config/srv/fetch_config_type.hpp"
#include "mwave_config/srv/fetch_io_config.hpp"
#include "mwave_config/srv/fetch_hmi_config.hpp"
#include "mwave_config/srv/fetch_logic_config.hpp"

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
            this->query_nodes      = sql_connection->create_statement("SELECT * FROM nodes WHERE slug=?;");
            this->query_config_i2c = sql_connection->create_statement("SELECT * FROM i2c_config WHERE node=?;");
            this->query_topics_i2c = sql_connection->create_statement("SELECT pin, topic FROM i2c_topics WHERE node=? AND bus=? AND address=?;");

            //A callback function for when nodes initiate their configuration. (Message Type: FetchConfigType)
            auto handle_fetch_config_type =
                [this](const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<mwave_config::srv::FetchConfigType::Request> request,
                       std::shared_ptr<mwave_config::srv::FetchConfigType::Response> response) -> void
            {
                (void)request_header;
                RCLCPP_INFO(this->get_logger(), "Request of type for node: %s", request->node_name.c_str());
                response->error = "";
		        std::vector<std::string> ret_nodes = std::vector<std::string>();
		
        		// TODO: Change the database to provide a list of nodes (comma seperated) given a node name.
		        // 	 This change can be in the ros2config:nodes table.

                query_nodes->set_string(0, request->node_name);
                auto sql_result = query_nodes->query();

                if (sql_result->next()) {
			        // TODO: See above 'todo'
               		//response->nodes = split( sql_result->get_string("type") );
                } else {
                    RCLCPP_WARN(this->get_logger(), "No SQL Results");
                    response->error = "Node name is not recognized";
                }
		
		        response->error = "This has not been implemented!";
            };
            //A callback function for when nodes configure thier input and output devices. (Message Type: FetchIOConfig)
            auto handle_fetch_io_config = 
                [this](const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<mwave_config::srv::FetchIOConfig::Request> request,
                       std::shared_ptr<mwave_config::srv::FetchIOConfig::Response> response) -> void
            {
                (void)request_header;
                RCLCPP_INFO(this->get_logger(), "Request of input/output for node: %s", request->node.c_str());

                query_config_i2c->set_string(0, request->node);
                auto sql_result = query_config_i2c->query();
                
                std::vector<mwave_config::msg::I2CDevice> ret_devices;
                
                while(sql_result->next()) {
                    auto device = this->parseDevice(sql_result);
                    ret_devices.push_back(*device);
                }

                std::vector<mwave_config::msg::I2CDevice>::iterator itr;
                for(itr = ret_devices.begin(); itr != ret_devices.end(); itr++) {
                    query_topics_i2c->set_string(0, request->node);
                    query_topics_i2c->set_unsigned8(1, itr->bus);
                    query_topics_i2c->set_unsigned8(2, itr->address);
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
                [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<mwave_config::srv::FetchHMIConfig::Request> request, std::shared_ptr<mwave_config::srv::FetchHMIConfig::Response> response) -> void
                {
                    (void)request_header;
                    (void)request;
                    response->dummy_response = "Dummy Response.";
                };            
            //A callback function for when nodes configure their logic modules (Message Type: FetchLogicConfig)
            auto handle_fetch_logic_config = 
                [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<mwave_config::srv::FetchLogicConfig::Request> request, std::shared_ptr<mwave_config::srv::FetchLogicConfig::Response> response) -> void
                {
                    (void)request_header;
                    (void)request;
                    response->dummy_response = "Dummy Response.";
                };            
            
            srv_fetch_config_type = create_service<mwave_config::srv::FetchConfigType>("FetchConfigType", handle_fetch_config_type);
            srv_fetch_io_config = create_service<mwave_config::srv::FetchIOConfig>("FetchIOConfig", handle_fetch_io_config);
            srv_fetch_hmi_config = create_service<mwave_config::srv::FetchHMIConfig>("FetchHMIConfig", handle_fetch_hmi_config);
            srv_fetch_logic_config = create_service<mwave_config::srv::FetchLogicConfig>("FetchLogicConfig", handle_fetch_logic_config);

            RCLCPP_INFO(this->get_logger(), "Listening!");
        };

        std::shared_ptr<mwave_config::msg::I2CDevice> parseDevice(const std::shared_ptr<mariadb::result_set>& sql_result)
        {
            mwave_config::msg::I2CDevice config;

            config.bus = sql_result->get_unsigned8("bus");
            config.address = sql_result->get_unsigned8("address");
            config.device = sql_result->get_string("device");

            std::string option_keys_raw = sql_result->get_string("option_keys");
            std::string option_values_raw = sql_result->get_string("option_values");
            config.option_keys = split(option_keys_raw);
            config.option_values = split(option_values_raw);

            config.topics = std::vector<std::string>();
            return std::make_shared<mwave_config::msg::I2CDevice>(config);
        };


    private:
            rclcpp::Service<mwave_config::srv::FetchConfigType>::SharedPtr srv_fetch_config_type;
            rclcpp::Service<mwave_config::srv::FetchIOConfig>::SharedPtr srv_fetch_io_config;
            rclcpp::Service<mwave_config::srv::FetchHMIConfig>::SharedPtr srv_fetch_hmi_config;
            rclcpp::Service<mwave_config::srv::FetchLogicConfig>::SharedPtr srv_fetch_logic_config;

            //TODO: Load in configs from mwave_orchestrator/private/Orchestrator.db.json
            std::shared_ptr<mariadb::account> sql_account = mariadb::account::create("127.0.0.1", "ros2", "oCXxFFUmBbVbV3gTM35DaTYveA4Ahh2P", "ros2config");
            std::shared_ptr<mariadb::connection> sql_connection = mariadb::connection::create(sql_account);

            std::shared_ptr<mariadb::statement> query_nodes;
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
