// TODO: Rewrite as a component.
// Guide: https://github.com/ros2/demos/blob/master/composition/include/composition/listener_component.hpp

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/u_int16.hpp"

#include "mwave_modules/io_component.hpp"
#include "i2cbridge/i2cbridge.hpp"

using namespace std::chrono_literals;

// Lifecycle_I2C_Input class inheriting from mwave_util::HandledLifecycleNode
/**
 * Every lifecycle node has a set of services attached to it
 * which make it controllable from the outside and invoke state
 * changes.
 * 
 * Available Service:
 * - <node_name>__get_state
 * - <node_name>__change_state
 * - <node_name>__get_available_states
 * - <node_name>__get_available_transitions
 * 
 * Additionally, a publisher for state changes notifications is
 * created:
 * - <node_name>__transition_event
 */
class Lifecycle_I2C_Input : public mwave_util::HandledLifecycleNode, public std::enable_shared_from_this<Lifecycle_I2C_Input>
{
    public:
        explicit Lifecycle_I2C_Input(const std::string & node_name, bool intra_proccess_comms = false)
            : mwave_util::HandledLifecycleNode(node_name, "", intra_proccess_comms)
        {  }

        // Callback for walltimer to publish I2C values
        /**
         * Callback for walltime. This function gets involked by the time and 
         * executes the publishing.If the lifecycle publisher is not active, 
         * we still invoke publish, but the communication is blocked so that
         * no messages is actually transferred.
         */    
        void publish()
        {
            // TODO: Make data be useful read from i2c
            // msg_->data = 0;
            
            if (pub_->is_activated()) {
                RCUTILS_LOG_INFO_NAMED(get_name(), "Lifecycle publisher is active and publishing.");
                //RCLCPP_INFO(get_logger(), "Lifecycle publisher is active and publishing.");
            } else {
                RCUTILS_LOG_INFO_NAMED(get_name(), "Lifecycle publisher is currently inactive.");
                //RCLCPP_INFO(get_logger(), "Lifecycle publisher is currently inactive.");
            }

            pub_->publish(msg_);
        }
        
        // Transition callback for state configuring
        /**
         * on_configure callback is being called when the lifecycle node
         * enters the "configuring" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays 
         * in "unconfigured".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        //TODO: Can we pass more parameters to this function?
        rcl_lifecycle_transition_key_t on_configure(const rclcpp_lifecycle::State & state)
        {
            (void)state;
            // Initialize and configure messages, publishers, and timers.
            // The lifecycle node API does return lifecycle components such as
            // lifecycle publishers. These entities obey the lifecycle and 
            // can comply to the current state of the node.
            // As of the beta version, there is only a lifecycle publisher
            // available.

            /* TODO:

            std::vector<mwave_messages::msg::I2CDevices> configs = GET_YOURSELF_SOME_CONFIGURATION();
            std::vector<mwave_messages::msg::I2CDevices>::const_iterator cfg_itr;
            for(cfg_itr = configs.begin(); cfg_itr != configs.end(); cfg_itr++)
            {
                this->i2cbridge->configureDevice(*cfg_itr);
            }

            */

            //RCLCPP_INFO(get_logger(), "on_configure() is called.");
            RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
              
            // We return a success and hence invoke the transition to the next
            // step: "inactive".
            // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
            // would stay in the "unconfigured" state.
            // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
            // this callback, the state machine transitions to state "errorprocessing".
            return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
        }

        // Transition callback for state activating
        /**
         * on_activate callback is being called when the lifecycle node
         * enters the "activating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "active" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transition to "active"
         * TRANSITION_CALLBACK_FAILURE transition to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rcl_lifecycle_transition_key_t on_activate(const rclcpp_lifecycle::State & state)
        {
            (void)state;

            // We explicitly activate the lifecycle publisher.
            // Starting from this point, all message are no longer
            // ignored but sent into the network.
            pub_->on_activate();

            RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
            
            // TODO: Add activation code.
            std::this_thread::sleep_for(2s);

            // We return a success and hence invoke the transition to the next
            // step: "active".
            // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
            // would stay in the "inactive" state.
            // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
            // this callback, the state machine transitions to state "errorprocessing".
            return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
        }

        // Transition callback for state deactivating
        /**
         * on_deactivate callback is being called when the lifecycle node 
         * enters the "deactivating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" stte or stays
         * in "active"
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "active"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rcl_lifecycle_transition_key_t on_deactivate(const rclcpp_lifecycle::State & state)
        {
            (void)state;
            // We explicitly deactivate the lifecycle publisher.
            // Starting from this point, all messages are no longer
            // sent into the network.
            pub_->on_deactivate();

            RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

            // We return a success and hence invoke the transition to the next
            // step: "inactive".
            // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
            // would stay in the "active" state.
            // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
            // this callback, the state machine transitions to state "errorprocessing".
            return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
        }

        // Transition callback for state cleaningup
        /**
         * on_cleanup callback is being called when the lifecycle node
         * enters the "cleaningup" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "uncofigured" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rcl_lifecycle_transition_key_t
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            // In our cleanup phase, we release the shared pointers to the
            // timer and publisher. These entities are no longer available
            // and our node is "clean".
            timer_.reset();
            pub_.reset();

            RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup is called.");

            // We return a success and hence invoke the transition to the next
            // step: "unconfigured".
            // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
            // would stay in the "inactive" state.
            // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
            // this callback, the state machine transitions to state "errorprocessing".
            return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
        }
        
    private:
        I2CROSBridge::I2CBridge i2cbridge;

        std::shared_ptr<std_msgs::msg::UInt16> msg_;

        // We hold an instance of a lifecycle publisher. This lifecycle publisher
        // can be activated or deactivated regarding on which state the lifecycle node
        // is in.
        // By default, a lifecycle publisher is inactive by creation and has to be
        // activated to publsh messages in the ROS world.
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt16>> pub_;

        // We hold instance of a timer which periodically triggers the publish function.
        // As for the beta version, this is a regular timer. IN a future version, a
        // lifecycle timer will be created which obeys the same lifecycle management as the
        // lifecycle publisher.
        std::shared_ptr<rclcpp::TimerBase> timer_;
};

/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char * argv[])
{
    // TODO: See https://github.com/ros2/demos/blob/master/composition/src/server_component.cpp for implementation changes
    //       needed for allowing other nodes to start this one.

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<Lifecycle_I2C_Input> lc_node =
        std::make_shared<Lifecycle_I2C_Input>("lc_input", false);

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
