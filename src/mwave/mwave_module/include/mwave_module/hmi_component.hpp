#ifndef MWAVE_MODULE__IO_COMPONENT_HPP_
#define MWAVE_MODULE__IO_COMPONENT_HPP_

#include "mwave_module/module_component.hpp"
#include "mwave_config/srv/fetch_hmi_config.hpp"

namespace mwave_module
{
	class HMI : public mwave_module::Module
	{
	public: 
		using SharedPtr = std::shared_ptr<HMI>;

		explicit HMI(const std::string & node_name, const std::string & namespace = "", bool use_intra_process_comms = false)
			: mwave_module::Module(node_name, "", intra_proccess_comms);

	private:
		rclcpp::Client<mwave_config::srv::FetchHMIConfig>::SharedPtr client_;
	};
} //namespace mwave_module
#endif //MWAVE_MODULE__IO_COMPONENT_HPP_