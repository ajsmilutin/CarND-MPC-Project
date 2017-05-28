//
// Created by milutin on 24.5.17..
//

#ifndef MPC_HELPERS_H
#define MPC_HELPERS_H


#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

constexpr double pi() { return M_PI; }
double deg2rad(double x);

double rad2deg(double x) ;

template <class T> T polyeval(Eigen::VectorXd coeffs, T x)
{
  T result = 0.0;
  T tmpPow = 1;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * tmpPow;
    tmpPow *= x;
  }
  return result;
};

template <class T>  void applyControl(T& px, T& py, T& phi, T& ve, const T steer, const T throt, double dt, double Lf){
  px  += ve*cos(phi)*dt;
  py  += ve*sin(phi)*dt;
  phi += ve/Lf * steer*dt;
  ve  += throt*dt;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif //MPC_HELPERS_H
