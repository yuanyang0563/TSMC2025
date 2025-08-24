#include "manipulator.hpp"

int main (int argc, char **argv) {

	if (argc<2) {
		std::cout << "set control mode: home | work" << std::endl;
		return 1;
	} else {
		mode = argv[1];
		if (mode!="home" && mode!="work")
			return 1;
	}

	ros::init(argc,argv,"gen3");
	ros::start();
	ros::Rate loop_rate(100);
	
	manipulator arm("gen3");
	
	//make the last twist message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);
	
	while (ros::ok() && !flag_stop) {
		if (flag_js) {
			arm.send_pose();
			arm.send_twist();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	arm.send_twist();
	
	return 0;

}
