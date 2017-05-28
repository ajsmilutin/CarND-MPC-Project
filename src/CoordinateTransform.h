//
// Created by milutin on 23.5.17..
//

#ifndef MPC_COORDINATETRANSFORM_H
#define MPC_COORDINATETRANSFORM_H


#include <vector>

class CoordinateTransform {
  double phi;
  double x;
  double y;

public:
  CoordinateTransform(double x_=0, double y_=0, double phi_=0):x(x_), y(y_), phi(phi_){};
  void setPos(double x, double y, double phi);

  void Transform(const double global_x, const double global_y, double &local_x, double &local_y);
  void Transform(const std::vector<double> global_x, const std::vector<double> global_y, std::vector<double>& local_x, std::vector<double>& local_y);

  void InverseTransform(const double local_x, const double local_y, double& global_x, double& global_y);
  void InverseTransform(const std::vector<double> local_x, const std::vector<double> local_y, std::vector<double>& global_x, std::vector<double>& global_y);

};


#endif //MPC_COORDINATETRANSFORM_H
