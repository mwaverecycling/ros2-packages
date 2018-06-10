#ifndef MWAVEUTIL_HANDLEDNODE_HPP
#define MWAVEUTIL_HANDLEDNODE_HPP

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <vector>
#include <map>

namespace mwave_util
{
    class HandledNode : public rclcpp::Node
    {
        public:
            using SharedPtr = std::shared_ptr<HandledNode>;
            
            explicit HandledNode(const std::string & node_name);
            virtual void start();

            template<typename MessageT, typename Alloc = std::allocator<void>>
            void add_publisher(const std::string & topic,
                const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
                std::shared_ptr<Alloc> allocator = nullptr);

            template<
                typename MessageT,
                typename Alloc = std::allocator<void>,
                typename PublisherT = ::rclcpp::Publisher<MessageT, Alloc>>
            std::shared_ptr<PublisherT> get_publisher(const std::string & topic);

            template<
                typename MessageT,
                typename CallbackT,
                typename Alloc = std::allocator<void>,
                typename SubscriptionT = rclcpp::Subscription<MessageT, Alloc>>
            std::shared_ptr<SubscriptionT> add_subscription(
                const std::string & topic_name,
                CallbackT && callback,
                const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
                rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
                bool ignore_local_publications = false,
                typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
                msg_mem_strat = nullptr,
                std::shared_ptr<Alloc> allocator = nullptr);

            template<typename DurationT = std::milli, typename CallbackT>
            typename rclcpp::WallTimer<CallbackT>::SharedPtr
            add_wall_timer(std::chrono::duration<int64_t, DurationT> period, CallbackT callback,
                rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

        private:
            std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
            std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
            std::vector<rclcpp::TimerBase::SharedPtr> timers;
    };
}

#endif