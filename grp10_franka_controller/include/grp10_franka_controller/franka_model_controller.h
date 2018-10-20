#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace grp10_franka_controller {

	class FrankaModelController
		: public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
		franka_hw::FrankaStateInterface> {
			public:
				bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
				void update(const ros::Time&, const ros::Duration&) override;

			private:
				franka_hw::FrankaStateInterface* franka_state_interface_;
				std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
				franka_hw::FrankaModelInterface* model_interface_;
				std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
				franka_hw::TriggerRate rate_trigger_{1.0};
		};

} // namespace grp10_franka_controller
