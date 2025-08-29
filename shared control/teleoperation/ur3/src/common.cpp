#include "common.hpp"

float lambda_u = 0.50;
float lambda_o = 0.50;
float upsilon_lim = 0.05;
float omega_lim = 0.10;
bool flag_js = false;
std::atomic_bool flag_stop = false;

void signalHandler (int signum) {
    	flag_stop = true;
}

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R) {
	Eigen::Matrix3f S = 0.5*(R-R.transpose());
	Eigen::Vector3f v(S(2,1),S(0,2),S(1,0));
	return v;
}


Eigen::Vector3f limitVelocity (Eigen::Vector3f velocity, float vel_lim) {
  Eigen::Vector3f velocity_abs;
  for (int i=0; i<3; i++) {
  	velocity_abs(i) = abs(velocity(i));
  }
  Eigen::Vector3f velocity_lim = velocity;
  double vel_max = velocity_abs.maxCoeff();
  if (vel_max>vel_lim)
  	velocity_lim = vel_lim/vel_max*velocity;
  return velocity_lim;
}
