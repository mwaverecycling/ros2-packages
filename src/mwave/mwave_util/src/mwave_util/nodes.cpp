#include "mwave_util/nodes.hpp"

namespace mwave_util
{
    HandledNode::HandledNode(const std::string & node_name) : Node(node_name) {  }
    void HandledNode::start()
    {
        RCLCPP_INFO(this->get_logger(), "Started node '%s'", this->get_name());
    }

    template<typename MessageT, typename Alloc>
    void HandledNode::add_publisher(const std::string & topic,
        const rmw_qos_profile_t & qos_profile,
        std::shared_ptr<Alloc> allocator)
    {
    	auto ret = this->create_publisher<MessageT>(topic, qos_profile, allocator);
    	this->publishers[topic] = ret;
    }
    template<typename MessageT, typename Alloc, typename PublisherT>
    std::shared_ptr<PublisherT> HandledNode::get_publisher(const std::string & topic)
    {
    	return this->publishers.at(topic);
    }


    template<typename MessageT, typename CallbackT, typename Alloc, typename SubscriptionT>
    std::shared_ptr<SubscriptionT> HandledNode::add_subscription(
        const std::string & topic_name,
        CallbackT && callback,
        const rmw_qos_profile_t & qos_profile,
        rclcpp::callback_group::CallbackGroup::SharedPtr group,
        bool ignore_local_publications,
        typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr msg_mem_strat,
        std::shared_ptr<Alloc> allocator)
    {
    	auto ret = this->create_subscription(topic_name, callback, qos_profile, group, ignore_local_publications, msg_mem_strat, allocator);
    	this->subscriptions.push_back(ret);
    	return ret;
    }

    template<typename DurationT, typename CallbackT>
    typename rclcpp::WallTimer<CallbackT>::SharedPtr
    HandledNode::add_wall_timer(std::chrono::duration<int64_t, DurationT> period, CallbackT callback,
        rclcpp::callback_group::CallbackGroup::SharedPtr group)
    {
    	auto ret = this->create_wall_timer(period, callback, group);
    	this->timers.push_back(ret);
    	return ret;
    }





    HandledLifecycleNode::HandledLifecycleNode(const std::string & node_name) : LifecycleNode(node_name) {  }
    void HandledLifecycleNode::start()
    {
        RCLCPP_INFO(this->get_logger(), "Started node '%s'", this->get_name());
    }

    template<typename MessageT, typename Alloc>
    void HandledLifecycleNode::add_publisher(const std::string & topic,
        const rmw_qos_profile_t & qos_profile,
        std::shared_ptr<Alloc> allocator)
    {
    	auto ret = this->create_publisher<MessageT>(topic, qos_profile, allocator);
    	this->publishers[topic] = ret;
    }
    template<typename MessageT, typename Alloc, typename PublisherT>
    std::shared_ptr<PublisherT> HandledLifecycleNode::get_publisher(const std::string & topic)
    {
    	return this->publishers.at(topic);
    }


    template<typename MessageT, typename CallbackT, typename Alloc, typename SubscriptionT>
    std::shared_ptr<SubscriptionT> HandledLifecycleNode::add_subscription(
        const std::string & topic_name,
        CallbackT && callback,
        const rmw_qos_profile_t & qos_profile,
        rclcpp::callback_group::CallbackGroup::SharedPtr group,
        bool ignore_local_publications,
        typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr msg_mem_strat,
        std::shared_ptr<Alloc> allocator)
    {
    	auto ret = this->create_subscription(topic_name, callback, qos_profile, group, ignore_local_publications, msg_mem_strat, allocator);
    	this->subscriptions.push_back(ret);
    	return ret;
    }

    template<typename DurationT, typename CallbackT>
    typename rclcpp::WallTimer<CallbackT>::SharedPtr
    HandledLifecycleNode::add_wall_timer(std::chrono::duration<int64_t, DurationT> period, CallbackT callback,
        rclcpp::callback_group::CallbackGroup::SharedPtr group)
    {
    	auto ret = this->create_wall_timer(period, callback, group);
    	this->timers.push_back(ret);
    	return ret;
    }
}