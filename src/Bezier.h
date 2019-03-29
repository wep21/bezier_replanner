#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <cmath>

class CubicBezier {

public:
  CubicBezier(Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, Eigen::Vector3d &p4, int num)
    : start_point(p1), start_control_point(p2), end_control_point(p3),
      end_point(p4), resampling_num(num){};
  ~CubicBezier(){};
  Eigen::MatrixXd getPath();
  double getX(double t);
  double getY(double t);
  double getZ(double t);
  double getYaw(double t);
private:
  Eigen::Vector3d start_point;
  Eigen::Vector3d start_control_point;
  Eigen::Vector3d end_control_point;
  Eigen::Vector3d end_point;
  unsigned int resampling_num;
};

double CubicBezier::getX(double t) {
  return (1 - t) * (1 - t) * (1 - t) * start_point[0] +
         3 * (1 - t) * (1 - t) * t * start_control_point[0] +
         3 * (1 - t) * t * t * end_control_point[0] +
         t * t * t * end_control_point[0];
}

double CubicBezier::getY(double t) {
  return (1 - t) * (1 - t) * (1 - t) * start_point[1] +
         3 * (1 - t) * (1 - t) * t * start_control_point[1] +
         3 * (1 - t) * t * t * end_control_point[1] +
         t * t * t * end_control_point[1];
}

double CubicBezier::getZ(double t) {
  return (1 - t) * (1 - t) * (1 - t) * start_point[2] +
         3 * (1 - t) * (1 - t) * t * start_control_point[2] +
         3 * (1 - t) * t * t * end_control_point[2] +
         t * t * t * end_control_point[2];
}

double CubicBezier::getYaw(double t) {
  double dy = -3 * (1 - t) * (1 - t) * start_point[1] +
       3 * (1 - t) * (1 - 3 * t) * start_control_point[1] +
       3 * t * (2 - 3 * t) * end_control_point[1] + 3 * t * t * end_point[1];
  double dx = -3 * (1 - t) * (1 - t) * start_point[0] +
       3 * (1 - t) * (1 - 3 * t) * start_control_point[0] +
       3 * t * (2 - 3 * t) * end_control_point[0] + 3 * t * t * end_point[0];
  return std::atan2(dy, dx);
}

Eigen::MatrixXd CubicBezier::getPath() {
  Eigen::VectorXd t_list = Eigen::VectorXd::LinSpaced(resampling_num + 1, 0, 1);
  Eigen::MatrixXd path = Eigen::MatrixXd::Zero(resampling_num, 4);
  for (int i = 0; i < resampling_num + 1; i++) {
    path(i, 0) = getX(t_list[i]);
    path(i, 1) = getY(t_list[i]);
    path(i, 2) = getZ(t_list[i]);
    path(i, 3) = getYaw(t_list[i]);
  }

  return path;
}