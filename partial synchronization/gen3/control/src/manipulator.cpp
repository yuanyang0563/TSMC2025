#include "manipulator.hpp"

manipulator::manipulator (std::string name) {
  	pub_ev = nh.advertise<control::TwistCommand>(name+"/in/cartesian_velocity", 10);
  	pub_ep = nh.advertise<geometry_msgs::Pose>(name+"/pose", 10);
  	sub_js = nh.subscribe(name+"/joint_states", 10, &manipulator::joint_state_callback, this, ros::TransportHints().tcpNoDelay());
  	sub_ur3 = nh.subscribe("/ur3/partial_pose", 10, &manipulator::ur3_pose_callback, this, ros::TransportHints().tcpNoDelay());
  	sub_ur3e= nh.subscribe("/ur3e/partial_pose", 10, &manipulator::ur3e_pose_callback, this, ros::TransportHints().tcpNoDelay());
  	xb << 0.0, 0.55, 0.02;
  	Rb << cos(-M_PI/4.0), -sin(-M_PI/4.0), 0.0, sin(-M_PI/4.0),  cos(-M_PI/4.0), 0.0, 0.0, 0.0, 1.0;
  	xd << 0.0, 0.0, 0.35;
  	Rd << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
  	xec << 0.0, 0.055, 0.0;
  	Rec << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
  	Tec = Eigen::MatrixXf(6,6);
  	Tec.block(0,0,3,3) = Rec;
  	Tec.block(0,3,3,3) = skewMat(xec)*Rec;
  	Tec.block(3,0,3,3) = Eigen::Matrix3f::Zero();
  	Tec.block(3,3,3,3) = Rec;
  	a = Eigen::VectorXf(7);
  	alpha = Eigen::VectorXf(7);
  	d = Eigen::VectorXf(7);
  	theta = Eigen::VectorXf(7);
  	q = Eigen::VectorXf(7);
  	twist = Eigen::VectorXf(6);
}

void manipulator::joint_state_callback (const sensor_msgs::JointState::ConstPtr& msg) {
  	q << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6]+0.15;
  	get_pose();
  	if (!flag_js) {
  		x0 = xc;
  		R0 = Rc;
  		Rw0 = Rw;
  		flag_js = !flag_js;
  	}
}

void manipulator::ur3_pose_callback (const geometry_msgs::Pose::ConstPtr& msg) {
  	x_ur3 << msg->position.x, msg->position.y, msg->position.z;
  	Eigen::Quaternionf quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  	Rw_ur3 = quat.toRotationMatrix();
  	if (!flag_ur3) {
  		x0_ur3 = x_ur3;
  		Rw0_ur3 = Rw_ur3;
  		flag_ur3 = !flag_ur3;
  	}
}
  	
void manipulator::ur3e_pose_callback (const geometry_msgs::Pose::ConstPtr& msg) {
  	x_ur3e << msg->position.x, msg->position.y, msg->position.z;
  	Eigen::Quaternionf quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  	Rw_ur3e = quat.toRotationMatrix();
  	if (!flag_ur3e) {
  		x0_ur3e = x_ur3e;
  		Rw0_ur3e = Rw_ur3e;
  		flag_ur3e = !flag_ur3e;
  	}
}

void manipulator::get_pose() {
  	a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  	alpha << -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, 0.0;
    	d << 0.1564+0.1284, -0.0054-0.0064, 0.2104+0.2104, -0.0064-0.0064, 0.2084+0.1059, 0.0, 0.1059+0.0615;
    	theta << -q(0), q(1), -q(2), q(3), -q(4), q(5), -q(6);
    	Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    	T.block(0,0,3,3) = Rb;
    	T.block(0,3,3,1) = xb;
    	Eigen::Matrix4f A;
    	for (size_t i=0; i<7; ++i) {
    		A << cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i)),
    		     sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i)),
    		     0.0,            sin(alpha(i)),                cos(alpha(i)),               d(i),
    		     0.0,            0.0,                          0.0,                         1.0;
    		T *= A;
    	}
    	xe = T.block(0,3,3,1);
    	Re = T.block(0,0,3,3);
    	xc = Re*xec+xe;
    	Rc = Re*Rec;
    	Rw = R2Rw(Rc);
    	Tw = R2Jw(Rc);
}

void manipulator::send_pose() {
  	auto msg = geometry_msgs::Pose();
  	msg.position.x = xc(0);
  	msg.position.y = xc(1);
  	msg.position.z = xc(2);
  	Eigen::Quaternionf quat(Rc);
  	msg.orientation.w = quat.w();
  	msg.orientation.x = quat.x();
  	msg.orientation.y = quat.y();
  	msg.orientation.z = quat.z();
  	pub_ep.publish(msg);
}

void manipulator::send_twist() {
  	twist.setZero();
  	if (mode=="home" && !flag_stop) {
  		twist.head(3) = xd-xe;
  		twist.tail(3) = skewVec(Rd*Re.transpose());
  	} else if (mode=="work" && !flag_stop) {
  		if (flag_ur3 && flag_ur3e) {
  			twist.head(3) += (x_ur3-xc)-Rc*R0.transpose()*(x0_ur3-x0);
  			twist.tail(3) += Tw.transpose()*skewVec(Rw_ur3*Rw0_ur3.transpose()*Rw0*Rw.transpose())+skewMat(Rc*R0.transpose()*(x0_ur3-x0))*(x_ur3-xc);
  			twist.head(3) += (x_ur3e-xc)-Rc*R0.transpose()*(x0_ur3e-x0);
  			twist.tail(3) += Tw.transpose()*skewVec(Rw_ur3e*Rw0_ur3e.transpose()*Rw0*Rw.transpose())+skewMat(Rc*R0.transpose()*(x0_ur3e-x0))*(x_ur3e-xc);
  		}
  	}
  	twist.head(3) = 0.5*Rb.transpose()*twist.head(3);
  	twist.tail(3) = 0.5*Re.transpose()*twist.tail(3);
  	auto msg = control::TwistCommand();
  	msg.twist.linear_x = twist(0);
  	msg.twist.linear_y = twist(1);
  	msg.twist.linear_z = twist(2);
  	msg.twist.angular_x = twist(3);
  	msg.twist.angular_y = twist(4);
  	msg.twist.angular_z = twist(5);
  	pub_ev.publish(msg);
}


  	
