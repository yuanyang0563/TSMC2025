#include "manipulator.hpp"

manipulator::manipulator (std::string name) {
  	pub_jv = nh.advertise<std_msgs::Float64MultiArray>(name+"/joint_group_vel_controller/command", 10);
  	pub_pp = nh.advertise<geometry_msgs::Pose>(name+"/partial_pose", 10);
  	sub_js = nh.subscribe(name+"/joint_states", 10, &manipulator::joint_state_callback, this, ros::TransportHints().tcpNoDelay());
  	sub_ft = nh.subscribe(name+"/wrench", 10, &manipulator::force_torque_callback, this, ros::TransportHints().tcpNoDelay());
  	sub_gen= nh.subscribe("/gen3/pose", 10, &manipulator::gen_pose_callback, this, ros::TransportHints().tcpNoDelay());
  	sub_ur = nh.subscribe("/ur3/partial_pose", 10, &manipulator::ur_pose_callback, this, ros::TransportHints().tcpNoDelay());
  	xd << 0.15, -0.3, 0.4;
  	Rd << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
  	a = Eigen::VectorXf(6);
  	alpha = Eigen::VectorXf(6);
  	d = Eigen::VectorXf(6);
  	theta = Eigen::VectorXf(6);
  	w = Eigen::VectorXf(6);
  	q = Eigen::VectorXf(6);
  	dq = Eigen::VectorXf(6);
  	J = Eigen::MatrixXf(6,6);
  	wh = Eigen::MatrixXf(6,25);
}

void manipulator::joint_state_callback (const sensor_msgs::JointState::ConstPtr& msg) {
	q << msg->position[2], msg->position[1], msg->position[0], msg->position[3], msg->position[4], msg->position[5];
	get_pose_jacobian();
  	if (!flag_js) {
  		x0 = x;
  		R0 = R;
  		Rw0 = Rw;
  		flag_js = !flag_js;
  	}
}

void manipulator::force_torque_callback (const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  	for (size_t i=0; i<24; ++i)
  		wh.col(i) = wh.col(i+1);
  	wh.col(24) << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  	if (!flag_ft) {
  		wh.setZero();
  		flag_ft = !flag_ft;
  	}
  	Eigen::VectorXf we = Eigen::VectorXf::Zero(6);
  	for (size_t i=0; i<25; ++i)
  		we += 0.04*wh.col(i);
  	w = filter_db(we);
  	w.head(3) = kf*R*w.head(3);
  	w.tail(3) = kt*R*w.tail(3);
}

void manipulator::gen_pose_callback (const geometry_msgs::Pose::ConstPtr& msg) {
  	x_gen << msg->position.x, msg->position.y, msg->position.z;
  	Eigen::Quaternionf quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  	R_gen = quat.toRotationMatrix();
  	Rw_gen = R2Rw(R_gen);
  	if (!flag_gen) {
  		x0_gen = x_gen;
  		R0_gen = R_gen;
  		Rw0_gen = Rw_gen;
  		flag_gen = !flag_gen;
  	}
}

void manipulator::ur_pose_callback (const geometry_msgs::Pose::ConstPtr& msg) {
  	x_ur << msg->position.x, msg->position.y, msg->position.z;
  	Eigen::Quaternionf quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  	Rw_ur = quat.toRotationMatrix();
  	if (!flag_ur) {
  		x0_ur = x_ur;
  		Rw0_ur = Rw_ur;
  		flag_ur = !flag_ur;
  	}
}

void manipulator::get_pose_jacobian() {
  	a << 0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0;
	alpha << M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0, 0.0;
	d << 0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921+0.01;
	theta << q;
  	Eigen::Matrix4f T;
  	T << cos(3.0*M_PI/4.0), -sin(3.0*M_PI/4.0), 0.0,  0.22,
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
  	Rw = R2Rw(R);
  	Tw = R2Jw(R);
}

void manipulator::send_partial_pose() {
	auto msg = geometry_msgs::Pose();
	msg.position.x = x(0);
	msg.position.y = x(1);
	msg.position.z = x(2);
	Eigen::Quaternionf quat(Rw);
	msg.orientation.w = quat.w();
	msg.orientation.x = quat.x();
	msg.orientation.y = quat.y();
	msg.orientation.z = quat.z();
	pub_pp.publish(msg);
}

void manipulator::send_joint_velocity() {
	dq.setZero();
	if (mode=="home" && !flag_stop) {
		dq.head(3) = xd-x;
		dq.tail(3) = skewVec(Rd*R.transpose());
	} else if (mode=="work" && !flag_stop) {
		if (flag_gen) {
			dq.head(3) += x0_gen-x-R_gen*R0_gen.transpose()*(x0_gen-x0);
			dq.tail(3) += Tw.transpose()*skewVec(Rw_gen*Rw0_gen.transpose()*Rw0*Rw.transpose());
		}
		if (flag_ur) {
			dq.head(3) += 0.5*(Rw*Rw0.transpose()+Rw_ur*Rw0_ur.transpose())*(x0-x0_ur)-(x-x_ur);
			dq.tail(3) += 0.5*Tw.transpose()*skewMat(Rw*Rw0.transpose()*(x0-x0_ur))*(x-x_ur)-Tw.transpose()*skewVec(Rw*Rw0.transpose()*Rw0_ur*Rw_ur.transpose());
		}
		dq = kp*dq+kf*w;
	}
	dq = J.transpose()*(J*J.transpose()).inverse()*dq;
	auto msg = std_msgs::Float64MultiArray();
	msg.data.resize(6);
	for (size_t i=0; i<6; ++i)
		msg.data[i] = dq(i);
	pub_jv.publish(msg);
}

