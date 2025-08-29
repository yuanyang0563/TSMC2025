#include "common.hpp"

class manipulator {

  public:
  
  	rclcpp::Node::SharedPtr node;  	
  	explicit manipulator (const std::string& name);  	
  	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);  	
  	void get_pose_jacobian();  	
  	void set_joint_velocity();
  	
  private:
  
  	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_jv;
  	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
  	Eigen::VectorXf a, alpha, d, theta, q, dq;
  	Eigen::MatrixXf J;
  	Eigen::Vector3f x, xd;
	Eigen::Matrix3f R, Rd;

};
