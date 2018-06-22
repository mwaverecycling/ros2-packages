// TODO 

#include "mwave_modules/hmi_component.hpp"

namespace mwave_modules
{
	HMIComponent::HMIComponent(const std::string & node_name, const std::string & namespace_)
		: mwave_util::BroadcastNode(node_name, namespace_)
	{
		
	}
}