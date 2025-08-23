#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>

extern bool flag_js;
extern std::atomic_bool flag_stop;

void signalHandler (int signum);

Eigen::Vector3f skewVec(const Eigen::Matrix3f& R);

Eigen::Matrix3f skewMat(const Eigen::Vector3f& v);

Eigen::Matrix3f R2Rw(const Eigen::Matrix3f& R);

Eigen::Matrix3f R2Jw(const Eigen::Matrix3f& R);


