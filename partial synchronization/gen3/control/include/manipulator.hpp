#include "common.hpp"

class manipulator {

  public:
  	
  	ros::NodeHandle nh;
  	ros::Publisher pub_ev, pub_ep;
  	ros::Subscriber sub_js, sub_ur3, sub_ur3e;
  	
  	Eigen::Vector3f xb, xe, xc, xec, xd, x0, x_ur3, x0_ur3, x_ur3e, x0_ur3e;
  	Eigen::Matrix3f Rb, Re, Rc, Rec, Rd, R0, Tw, Rw, Rw0, Rw_ur3, Rw0_ur3, Rw_ur3e, Rw0_ur3e;
  	Eigen::VectorXf a, alpha, d, theta, q, twist;
  	Eigen::MatrixXf Tec;
  	
  	manipulator (std::string name);
  	
  	void joint_state_callback (const sensor_msgs::JointState::ConstPtr& msg);
  	void ur3_pose_callback (const geometry_msgs::Pose::ConstPtr& msg);	
  	void ur3e_pose_callback (const geometry_msgs::Pose::ConstPtr& msg);
  	
  	void get_pose();	
  	void send_pose();	
  	void send_twist();

};
