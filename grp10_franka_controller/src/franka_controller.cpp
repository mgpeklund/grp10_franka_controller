#include <grp10_franka_controller/franka_controller.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <memory>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace grp10_franka_controller {
	bool FrankaController::init(hardware_interface::RobotHW* robot_hardware,
			ros::NodeHandle& node_handle) {

		cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
		if (cartesian_pose_interface_ == nullptr) {
			ROS_ERROR(
        		"CartesianPoseExampleController: Could not get Cartesian Pose "
        		"interface from hardware");
    		return false;
  		}

  		std::string arm_id;
  		if (!node_handle.getParam("arm_id", arm_id)) {
    		ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    		return false;
  		}

		try {
    		cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        		cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  		} catch (const hardware_interface::HardwareInterfaceException& e) {
    		ROS_ERROR_STREAM(
        		"CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    			return false;
		}

		auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
		if (state_interface == nullptr) {
    		ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    		return false;
		}

		try {
    		auto state_handle = state_interface->getHandle(arm_id + "_robot");

			std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
			for (size_t i = 0; i < q_start.size(); i++) {
    			if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        			ROS_ERROR_STREAM(
            			"CartesianPoseExampleController: Robot is not in the expected starting position for "
            			"running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            			"robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        			return false;
      			}
    		}
		} catch (const hardware_interface::HardwareInterfaceException& e) {
			ROS_ERROR_STREAM(
    	    	"CartesianPoseExampleController: Exception getting state handle: " << e.what());
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

		double angle = M_PI / 4 * (1 - std::cos( (M_PI / 5.0) * elapsed_time_.toSec() ));

/*
		double phi = 0;
		double theta = angle;
		double psi = 0;

		double c_phi = std::cos(phi);
		double c_theta = std::cos(theta);
		double c_psi = std::cos(psi);

		double s_phi = std::sin(phi);
		double s_theta = std::sin(theta);
		double s_psi = std::sin(psi);

		double n_x = c_phi * c_theta;
		double n_y = s_phi * c_theta;
		double n_z = -s_theta;

		double o_x = -s_phi * c_psi + c_phi * s_theta * s_psi;
		double o_y = c_phi * c_psi + s_phi * s_theta * s_psi;
		double o_z = c_theta * s_psi;

		double a_x = s_phi * s_psi + c_phi * s_theta * c_psi;
		double a_y = -c_phi * s_psi + s_phi * s_theta * c_psi;
		double a_z = c_theta * c_psi;
*/
		double radius = 0.3;
		
		double delta_x = radius * std::sin(angle);
		double delta_z = radius * (std::cos(angle) - 1);
		std::array<double, 16> new_pose = initial_pose_;
/*
		// n
		new_pose[0] += n_x;
		new_pose[1] += n_y;
		new_pose[2] += n_z;

		// o
		new_pose[4] += o_x;
		new_pose[5] += o_y;
		new_pose[6] += o_z;

		//a
		new_pose[8] += a_x;
		new_pose[9] += a_y;
		new_pose[10] += a_z;
*/

		new_pose[12] -= delta_x;
		new_pose[14] -= delta_z;

		cartesian_pose_handle_->setCommand(new_pose);
/*
		std::array<double, 16> pose = model_handle_->getPose(franka::Frame::kEndEffector);
		
		for (int i = 0; i < 4; ++i)
		{
			std::cout << pose[i] << "\t" << pose[i+4] << "\t" << pose[i+8] << "\t" << pose[i+12] << std::endl;
			
		}
*/
		

		

	}

}

PLUGINLIB_EXPORT_CLASS(grp10_franka_controller::FrankaController,
						controller_interface::ControllerBase)