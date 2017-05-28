#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 8;
extern const double dt = 0.5;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
extern const double Lf = 2.67;

class FG_eval {
 private:
    double alpha;
    double beta;
    double lambda;
    double nu;
    double vref;
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs, double v, double a, double b, double l, double n):
      vref(v), alpha(a), beta(b), lambda(l), nu(n) { this->coeffs = coeffs;  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Fg

    Eigen::VectorXd state;
    AD<double> px  = vars[0];
    AD<double> py  = vars[1];
    AD<double> phi = vars[2];
    AD<double> ve  = vars[3];
    fg[0] =0;

    for (int i=0; i<N-1; i++){
      AD<double> steer = vars[4+i];
      AD<double> throt = vars[4+(N-1)+i];
      applyControl<AD<double>>(px, py, phi, ve, steer, throt, dt, Lf);
      fg[0] += alpha*CppAD::pow(polyeval<AD<double>>(coeffs, px) - py, 2)
            +  beta*CppAD::pow(ve - vref,2)
            +  lambda*CppAD::pow(steer,2)
            +  nu*CppAD::pow(throt, 2);
      fg[i+1] = phi; // total angle
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  previousSteer.resize(N-1);
  previousThrotle.resize(N-1);
  for (int i=0; i<N-1; i++){
    previousSteer[i]=0;
    previousThrotle[i]=0;
  }
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).

  // I will just use the initial state, and inputs

  size_t n_vars = 4 + (N-1)*2;
  // TODO: Set the number of constraints
  size_t n_constraints = N-1;

  // Initial value of the independent variables.

  Dvector vars(n_vars);



  for (int i =0; i<4; i++){
    vars[i]=state(i);
  }

  // initialize to previous solutions, that produces more stable solution
  // Hot-start optimization
  for (int i=0; i<(N-1)-1 ; i++){
    vars[4+i] = previousSteer[i+1];
  }
  vars[4+(N-1)-1] =0;

  for (int i=0; i<(N-1)-1 ; i++){
    vars[4+i+N-1] = previousThrotle[i+1];
  }
  vars[4+2*(N-1)-1] =0;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // fix the initial state
  for (int i =0; i<4; i++){
    vars_lowerbound[i]=state(i);
    vars_upperbound[i]=state(i);
  }

  // steering angle bound -25 degrees to 25 degrees, must be converted to radians
  for (int i =4; i<N-1+4; i++){
    vars_lowerbound[i]=-deg2rad(17.5);
    vars_upperbound[i]=deg2rad(17.5);
  }

  for (int i =N-1+4; i<2*(N-1)+4; i++){
    vars_lowerbound[i]=0;
    vars_upperbound[i]=2;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = -5*pi()/8;
    constraints_upperbound[i] = 5*pi()/8;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, 27, 4, 2, 90, 2);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;


  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  // there are cases when the solution imediately jumps to some suboptimal
  // value. That solution is identified by higgh cost. So in that case, the
  // solution calculated in previous timestamp is used
  vector<double> controll;
  double alpha = 0.25;
  for (int i=0; i<N-1-1; i++){
    previousSteer[i]= alpha * previousSteer[i+1] + (1-alpha)*solution.x[4+i];
    previousThrotle[i]=alpha * previousThrotle[i+1] + (1-alpha)*solution.x[4+i + N-1];
    controll.push_back(previousSteer[i]);
    controll.push_back(previousThrotle[i]);
  }
  previousSteer[N-1-1]=solution.x[4 + N-1-1];
  previousThrotle[N-1-1]=solution.x[4 + 2*(N-1)-1];
  controll.push_back(previousSteer[N-1-1]);
  controll.push_back(previousThrotle[N-1-1]);

  return controll;
}
