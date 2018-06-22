// TODO 

#include "mwave_modules/logic_component.hpp"

namespace mwave_modules
{
	LogicComponent::LogicComponent(const std::string & node_name, const std::string & namespace_)
		: mwave_util::BroadcastNode(node_name, namespace_)
	{
		
	}
}