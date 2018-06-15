#ifndef MWAVE_MODULE__MODULE_COMPONENT_HPP_
#define MWAVE_MODULE__MODULE_COMPONENT_HPP_

#include "mwave_util/nodes.hpp"
#include "mwave_config/msg/broadcast.msg"

namespace mwave_module
{
	class Module : public mwave_util::HandledLifecycleNode 
	{
	public: 
		using SharedPtr = std::shared_ptr<Module>;

		explicit Module(const std::string & node_name, const std::string & namespace = "", bool use_intra_process_comms = false)
			: mwave_util::HandledLifecycleNode(node_name, "", intra_proccess_comms);

	protected:
		void broadcast(std::string type, std::string message);

	private:
		rclcpp::Publisher<mwave_config::msg::broadcast, std::allocator<void>> pub_broadcast_;
	};
} //namespace mwave_module
#endif //MWAVE_MODULE__MODULE_COMPONENT_HPP_