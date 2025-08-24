#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include "control/Twist.h"
#include "control/TwistCommand.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>

extern bool flag_js;
extern bool flag_ur3;
extern bool flag_ur3e;
extern std::string mode;
extern std::atomic_bool flag_stop;

void signalHandler (int signum);

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R);

Eigen::Matrix3f skewMat (const Eigen::Vector3f& v);

Eigen::Matrix3f R2Rw(const Eigen::Matrix3f& R);

Eigen::Matrix3f R2Jw(const Eigen::Matrix3f& R);
