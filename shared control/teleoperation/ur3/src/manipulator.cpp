#include "manipulator.hpp"

manipulator::manipulator (const std::string& name) {
  	node = std::make_shared<rclcpp::Node>(name);
  	pub_jv = node->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
  	sub_js = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&manipulator::joint_state_callback, this, std::placeholders::_1));
  	a = Eigen::VectorXf(6);
  	alpha = Eigen::VectorXf(6);
  	d = Eigen::VectorXf(6);
  	theta = Eigen::VectorXf(6);
  	q = Eigen::VectorXf(6);
  	dq = Eigen::VectorXf(6);
  	J = Eigen::MatrixXf(6, 6);
	xd <<  0.0, -0.4, 0.4;
  	Rd << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
}

void manipulator::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
	q << msg->position[5], msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4];
  	flag_js = true;
}
  	
void manipulator::get_pose_jacobian () {
  	a << 0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0;
	alpha << M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0, 0.0;
	d << 0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819;
	theta << q;
  	Eigen::Matrix4f T;
  	T << cos(3.0*M_PI/4.0), -sin(3.0*M_PI/4.0), 0.0,  0.0,
  	     sin(3.0*M_PI/4.0),  cos(3.0*M_PI/4.0), 0.0, -0.66,
  	     0.0,                0.0,               1.0,  0.0,
  	     0.0,                0.0,               0.0,  1.0;
  	Eigen::Matrix<float, 3, 7> o, z;
  	o.col(0) << T.block(0,3,3,1);
  	z.col(0) << T.block(0,2,3,1);
  	Eigen::Matrix4f A;
  	for (size_t i=0; i<6; ++i) {
  		A << cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i)),
  		     sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i)),
  		     0.0,            sin(alpha(i)),                cos(alpha(i)),               d(i),
  		     0.0,            0.0,                          0.0,                         1.0;
  		T *=A;
  		o.col(i+1) = T.block(0,3,3,1);
  		z.col(i+1) = T.block(0,2,3,1);
  	}
  	x = T.block(0,3,3,1);
  	R = T.block(0,0,3,3);
  	for (size_t i=0; i<6; ++i)
  		J.col(i) << z.col(i).cross(o.col(6)-o.col(i)), z.col(i);
}
  	
void manipulator::set_joint_velocity () {
	dq.setZero();
  	if (flag_js && !flag_stop) {
  		dq.head(3) = limitVelocity(lambda_u*(xd-x), upsilon_lim);
  		dq.tail(3) = limitVelocity(lambda_o*skewVec(Rd*R.transpose()), omega_lim);
  		dq = J.transpose()*(J*J.transpose()).inverse()*dq;
  	}
  	auto msg = std_msgs::msg::Float64MultiArray();
  	msg.data.resize(6);
  	for (size_t i=0; i<6; ++i)
  		msg.data[i] = dq(i);
  	pub_jv->publish(msg);
}

