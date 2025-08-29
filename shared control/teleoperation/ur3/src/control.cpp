#include "manipulator.hpp"

int main (int argc, char **argv) {
	
	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(125);
	manipulator arm("ur3");
	
	// make the last joint velocity message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);
	
	while (rclcpp::ok() && !flag_stop) {
		arm.get_pose_jacobian();
		arm.set_joint_velocity();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	arm.set_joint_velocity();
	rclcpp::shutdown();
	
	return 0;
}
