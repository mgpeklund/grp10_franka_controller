#include <grp10_franka_controller/franka_controller.h>


#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace grp10_franka_controller {
	bool FrankaController::init(hardware_interface::RobotHW* robot_hardware,
			ros::NodeHandle& node_handle) {
		cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
		if (cartesian_pose_interface_ == nullptr) {
			ROS_ERROR(
					"FrankaController: Could not get Cartesian Pose "
					"interface from hardware");
			return false;
		}

		std::string arm_id;
		if (!node_handle.getParam("arm_id", arm_id)) {
			ROS_ERROR("FrankaController: Could not get parameter arm_id");
			return false;
		}

		try {
			cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
					cartesian_pose_interface_->getHandle(arm_id + "_robot"));
		} catch (const hardware_interface::HardwareInterfaceException& e) {
			ROS_ERROR_STREAM(
					"FrankaController: Exception getting Cartesian handle: " << e.what());
			return false;
		}

		auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
		if (state_interface == nullptr) {
			ROS_ERROR("FrankaController: Could not get state interface from hardware");
			return false;
		}
/*
		try {
    		velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        	velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  		} catch (const hardware_interface::HardwareInterfaceException& e) {
    		ROS_ERROR_STREAM(
        		"CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    		return false;
  		}
*/
		try {
			auto state_handle = state_interface->getHandle(arm_id + "_robot");

			std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
			for (size_t i = 0; i < q_start.size(); i++) {
				if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
					ROS_ERROR_STREAM(
							"FrankaController: Robot is not in the expected starting position for "
							"running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
							"robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
					return false;
				}
			}
		} catch (const hardware_interface::HardwareInterfaceException& e) {
			ROS_ERROR_STREAM(
					"FrankaController: Exception getting state handle: " << e.what());
			return false;
		}

		return true;
	}

	void FrankaController::starting(const ros::Time& /* time */) {
		initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
		elapsed_time_ = ros::Duration(0.0);
	}

	void FrankaController::update(const ros::Time& /* time */,
								  const ros::Duration& period) {
		elapsed_time_ += period;
/*
		double time_max = 4.0;
  		double v_max = 0.05;
		double angle = M_PI / 4.0;
		double cycle = std::floor(
		pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
		double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
		double v_x = std::cos(angle) * v;
		double v_z = -std::sin(angle) * v;
		std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
		velocity_cartesian_handle_->setCommand(command);
*/
		

		double radius = 0.3;
		double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
		double delta_x = radius * std::sin(angle);
		double delta_z = radius * (std::cos(angle) - 1);
		std::array<double, 16> new_pose = initial_pose_;
		new_pose[12] -= delta_x;
		new_pose[14] -= delta_z;
		cartesian_pose_handle_->setCommand(new_pose);

		
	}

}

PLUGINLIB_EXPORT_CLASS(grp10_franka_controller::FrankaController,
						controller_interface::ControllerBase)