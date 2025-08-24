#include "common.hpp"

class manipulator {

  public:
  
  	ros::NodeHandle nh;
  	ros::Publisher pub_jv, pub_pp;
  	ros::Subscriber sub_js, sub_gen, sub_ur;
  	
  	Eigen::Vector3f x, xd, x0, x_ur, x0_ur, x_gen, x0_gen;
  	Eigen::Matrix3f R, Rd, R0, Tw, Rw, Rw0, Rw_ur, Rw0_ur, R_gen, R0_gen, Rw_gen, Rw0_gen; 
  	Eigen::VectorXf a, alpha, d, theta, q, dq;
  	Eigen::MatrixXf J;
  	
  	manipulator (std::string name);
  	
  	void joint_state_callback (const sensor_msgs::JointState::ConstPtr& msg);
  	void gen_pose_callback (const geometry_msgs::Pose::ConstPtr& msg);
  	void ur_pose_callback (const geometry_msgs::Pose::ConstPtr& msg);
  	
  	void get_pose_jacobian();
  	void send_partial_pose();
  	void send_joint_velocity();

};
