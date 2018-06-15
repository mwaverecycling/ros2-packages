#include "mwave_util/module_component.hpp"
#include "mwave_util/nodes.hpp"

namespace mwave_util
{
	LifecycleModuleComponent::LifecycleModuleComponent() : mwave_util::HandledLifecycleNode(
		const std::string & node_name,
		const std::string & namespace_,
		bool use_intra_process_comms) : mwave_utils::HandledLifecyleNode(node_name, namespace_, use_intra_process_comms) { }

	void LifecycleModuleComponent::broadcast() {
		
	}

	// Create subscription and publisher to global broadcast
}