#include "common.hpp"

float kp = 5.0;
float kf = 0.1;
float kt = 1.0;
bool flag_js = false;
bool flag_ft = false;
bool flag_gen= false;
bool flag_ur = false;
std::string mode;
std::atomic_bool flag_stop = false;

void signalHandler (int signum) {
    	flag_stop = true;
}

Eigen::Vector3f skewVec(const Eigen::Matrix3f& R) {
  Eigen::Matrix3f S = 0.5*(R-R.transpose());
  Eigen::Vector3f v(S(2,1),S(0,2),S(1,0));
  return v;
}

Eigen::Matrix3f skewMat(const Eigen::Vector3f& v) {
  Eigen::Matrix3f S;
  S << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return S;
}

Eigen::Matrix3f R2Rw(const Eigen::Matrix3f& R) {

  float a1(R(0,2)), a2(R(1,2)), a3(R(2,2));
  float w1 = a2/(1.0+a3);
  float w2 =-a1/(1.0+a3);
  Eigen::Matrix3f Rw;
  Rw << 1.0+pow(w1,2.0)-pow(w2,2.0), 2.0*w1*w2, -2.0*w2, 2.0*w1*w2, 1.0-pow(w1,2.0)+pow(w2,2.0), 2.0*w1, 2.0*w2, -2.0*w1, 1.0-pow(w1,2.0)-pow(w2,2.0);
  Rw /= 1.0+pow(w1,2.0)+pow(w2,2.0);
  return Rw;

}

Eigen::Matrix3f R2Jw(const Eigen::Matrix3f& R) {

  float a1(R(0,2)), a2(R(1,2)), a3(R(2,2));
  float w1 = a2/(1.0+a3);
  float w2 =-a1/(1.0+a3);
  Eigen::Matrix3f Jw;
  Jw(0,0) = (a3/(1.0+a3)+pow(a2,2.0)/pow(1.0+a3,2.0))/(1.0+pow(w1,2.0)+pow(w2,2.0))-8.0*(1.0+pow(w1,2.0)-pow(w2,2.0))*w1*w2/pow(1.0+pow(w1,2.0)+pow(w2,2.0),3.0)*a1*a2/pow(1.0+a3,2.0);
  Jw(0,1) = -a1*a2/pow(1.0+a3,2.0)*2.0/(1.0+pow(w1,2.0)+pow(w2,2.0))+(a3/(1.0+a3)+pow(a1,2.0)/pow(1.0+a3,2.0))*8.0*(1.0+pow(w1,2.0)-pow(w2,2.0))*w1*w2/pow(1.0+pow(w1,2.0)+pow(w2,2.0),3.0);
  Jw(0,2) = -a1/(1.0+a3)*2.0/(1.0+pow(w1,2.0)+pow(w2,2.0))-a2/(1.0+a3)*8.0*(1+pow(w1,2.0)-pow(w2,2.0))*w1*w2/pow(1.0+pow(w1,2.0)+pow(w2,2.0),3.0);
  Jw(1,0) = -a1*a2/pow(1.0+a3,2.0)*2.0/(1.0+pow(w1,2.0)+pow(w2,2.0));
  Jw(1,1) = (a3/(1.0+a3)+pow(a1,2.0)/pow(1.0+a3,2.0))*2.0/(1.0+pow(w1,2.0)+pow(w2,2.0));
  Jw(1,2) = -a2/(1.0+a3)*2.0/(1.0+pow(w1,2.0)+pow(w2,2.0));
  Jw(2,0) = (a3/(1.0+a3)+pow(a2,2.0)/pow(1.0+a3,2.0))*2.0*w2/(1.0+pow(w1,2.0)+pow(w2,2.0))+a1*a2/pow(1.0+a3,2.0)*2.0*w1/(1.0+pow(w1,2.0)+pow(w2,2.0));
  Jw(2,1) = -a1*a2/pow(1.0+a3,2.0)*2.0*w2/(1.0+pow(w1,2.0)+pow(w2,2.0))-(a3/(1.0+a3)+pow(a1,2.0)/pow(1.0+a3,2.0))*2.0*w1/(1.0+pow(w1,2.0)+pow(w2,2.0));
  Jw(2,2) = -a1/(1.0+a3)*2.0*w2/(1.0+pow(w1,2.0)+pow(w2,2.0))+a2/(1.0+a3)*2.0*w1/(1.0+pow(w1,2.0)+pow(w2,2.0));
  return Jw;

}

Eigen::VectorXf filter_db (const Eigen::VectorXf& w) {
	Eigen::VectorXf w_e(6);
	Eigen::VectorXf db(6);
	db << 2.0,2.0,2.0,0.01,0.01,0.01;
	for (int i=0; i<6; i++) {
		if (w(i)>0) {
			w_e(i) =  std::max(abs(w(i))-db(i),0.0f);
		} else {
			w_e(i) = -std::max(abs(w(i))-db(i),0.0f);
		}
	}
	return w_e;
}
