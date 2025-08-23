#include "common.hpp"

class manipulator {

  public:
  
  	ros::NodeHandle nh;
  	ros::Publisher pub_jv;
  	ros::Subscriber sub_js;
  	
  	Eigen::Vector3f x;
  	Eigen::Matrix3f R;
  	Eigen::VectorXf a, alpha, d, theta, q, dq;
  	Eigen::MatrixXf J; 	
  	
  	manipulator (std::string name) {
  		pub_jv = nh.advertise<std_msgs::Float64MultiArray>(name+"/joint_group_vel_controller/command", 10);
  		sub_js = nh.subscribe(name+"/joint_states", 10, &manipulator::joint_state_callback, this, ros::TransportHints().tcpNoDelay());
  		a = Eigen::VectorXf(6);
  		alpha = Eigen::VectorXf(6);
  		d = Eigen::VectorXf(6);
  		theta = Eigen::VectorXf(6);
  		q = Eigen::VectorXf(6);
  		dq = Eigen::VectorXf(6);
  		J = Eigen::MatrixXf(6,6);
  	}
  	
  	void joint_state_callback (const sensor_msgs::JointState::ConstPtr& msg) {
  		q << msg->position[2], msg->position[1], msg->position[0], msg->position[3], msg->position[4], msg->position[5];
  		if (!flag_js) {
  			flag_js = !flag_js;
  		}
  	}
  	
  	void get_pose_jacobian() {
  		a << 0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0;
		alpha << M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0, 0.0;
		d << 0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819+0.005;
		theta << q;
  		Eigen::Matrix4f T;
  		T << cos(3.0*M_PI/4.0), -sin(3.0*M_PI/4.0), 0.0, -0.22,
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
  	
  	void set_joint_velocity() {
  		dq.setZero();
  		/*
  		if (flag_js && !flag_stop) {
  		
  		}
  		*/
  		auto msg = std_msgs::Float64MultiArray();
  		msg.data.resize(6);
  		for (size_t i=0; i<6; ++i)
  			msg.data[i] = dq(i);
  		pub_jv.publish(msg);
  	}

};



int main (int argc, char **argv) {

	ros::init(argc,argv,"ur3");
	ros::start();
	ros::Rate loop_rate(125);
	manipulator arm("ur3");
    
	// make the last joint velocity message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);

	while (ros::ok() && !flag_stop) {
    		arm.set_joint_velocity();
		ros::spinOnce();
		loop_rate.sleep();
    
	}
	arm.set_joint_velocity();

	return 0;
    
}
