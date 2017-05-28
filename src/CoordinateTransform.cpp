//
// Created by milutin on 23.5.17..
//

#include "CoordinateTransform.h"
#include <math.h>
void CoordinateTransform::setPos(double x_pos, double y_pos, double phi_ang) {
  x = x_pos;
  y = y_pos;
  phi = phi_ang;
}

void CoordinateTransform::Transform(const double global_x, const double global_y, double &local_x, double &local_y) {
  local_x = cos(phi) * (global_x - x) + sin(phi) * (global_y - y);
  local_y = -sin(phi) * (global_x - x) + cos(phi) * (global_y - y);

}

void
CoordinateTransform::Transform(const std::vector<double> global_x, const std::vector<double> global_y, std::vector<double> &local_x,
                               std::vector<double>& local_y) {
  local_x.resize(global_x.size());
  local_y.resize(global_y.size());
  for (int i=0; i<global_x.size(); i++) {
    Transform(global_x[i], global_y[i], local_x[i], local_y[i]);
  }
}


void CoordinateTransform::InverseTransform(const double local_x, const double local_y, double &global_x, double &global_y) {
  global_x = x + cos(phi)*local_x - sin(phi)*local_y;
  global_y = y + sin(phi)*local_x + cos(phi)*local_y;
}

void CoordinateTransform::InverseTransform(const std::vector<double> local_x, const std::vector<double> local_y,
                                           std::vector<double> &global_x, std::vector<double>& global_y) {
  global_x.resize(local_x.size());
  global_y.resize(local_y.size());
  for (int i=0; i<local_x.size(); i++){
    InverseTransform(local_x[i], local_y[i], global_x[i], global_y[i]);
  }
}
