#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <csignal>

extern float kp;
extern float kf;
extern float kt;
extern bool flag_js;
extern bool flag_ft;
extern bool flag_gen;
extern bool flag_ur;
extern std::string mode;
extern std::atomic_bool flag_stop;

void signalHandler (int signum);

Eigen::Vector3f skewVec(const Eigen::Matrix3f& R);

Eigen::Matrix3f skewMat(const Eigen::Vector3f& v);

Eigen::Matrix3f R2Rw(const Eigen::Matrix3f& R);

Eigen::Matrix3f R2Jw(const Eigen::Matrix3f& R);

Eigen::VectorXf filter_db (const Eigen::VectorXf& w);
