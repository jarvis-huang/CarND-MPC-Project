#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

extern AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x);
extern AD<double> polyeval_dot(Eigen::VectorXd coeffs, AD<double> x);

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.05;
int x_start = 0;
int y_start = x_start + N;
int psi_start = y_start + N;
int v_start = psi_start + N;
int delta_start = v_start + N;
int a_start = delta_start + N-1;
int a_end = a_start + N-1;

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
const double Lf = 2.67;
const double v_ref = 10;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;
    for (unsigned int t = 1; t < N; t++) {
      AD<double> x = vars[x_start + t];
      AD<double> y = vars[y_start + t];
      AD<double> psi = vars[psi_start + t];
      AD<double> v = vars[v_start + t];
      AD<double> y_ref = polyeval(coeffs, x);
      AD<double> cte = y - y_ref;
      fg[0] += CppAD::pow(cte, 2);
      AD<double> psi_ref = CppAD::atan(polyeval_dot(coeffs, x));
      fg[0] += CppAD::pow(psi-psi_ref, 2);
      fg[0] += CppAD::pow(v-v_ref, 2);
    }
    for (unsigned int t = 0; t < N-1; t++) {
      AD<double> delta = vars[delta_start + t];
      AD<double> a = vars[a_start + t];
      fg[0] += CppAD::pow(delta, 2);
      fg[0] += CppAD::pow(a, 2);
    }
    for (unsigned int t = 1; t < N-1; t++) {
      AD<double> delta = vars[delta_start + t];
      AD<double> a = vars[a_start + t];
      AD<double> delta_prev = vars[delta_start + t - 1];
      AD<double> a_prev = vars[a_start + t - 1];
      fg[0] += CppAD::pow(delta-delta_prev, 2);
      fg[0] += CppAD::pow(a-a_prev, 2);
    }
    // Initial states
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    // JHUANG: why don't need these two??
    //fg[1 + delta_start] = vars[delta_start];
    //fg[1 + a_start] = vars[a_start];
    
    for (unsigned int t = 1; t < N; t++) {
      AD<double> x = vars[x_start + t];
      AD<double> y = vars[y_start + t];
      AD<double> psi = vars[psi_start + t];
      AD<double> v = vars[v_start + t];
      
      AD<double> x0 = vars[x_start + t -1];
      AD<double> y0 = vars[y_start + t -1];
      AD<double> psi0 = vars[psi_start + t -1];
      AD<double> v0 = vars[v_start + t -1];
      AD<double> delta0 = vars[delta_start + t -1];
      AD<double> a0 = vars[a_start + t -1];
      
      fg[x_start + t + 1] = x - (x0 + v0*CppAD::cos(psi0)*dt);
      fg[y_start + t + 1] = y - (y0 + v0*CppAD::sin(psi0)*dt);
      fg[psi_start + t + 1] = psi - (psi0 + v0 / Lf * delta0 * dt);
      fg[v_start + t +1] = v - (v0 + a0*dt);
    }
          
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  std::cout << "inside solve" << std::endl;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 4*N + 2*(N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = 4*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  
  
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (int i = 0; i < delta_start; i++) {
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1.0e19;
  }
  for (int i=delta_start; i<a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
  }
  for (int i=a_start; i<a_end; i++) {
      vars_lowerbound[i] = -1;
      vars_upperbound[i] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;  

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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
  vector<double> ret;
  ret.push_back(solution.x[delta_start]);
  ret.push_back(solution.x[a_start]);
  for (unsigned int t = 1; t < N; t++) {
      ret.push_back(solution.x[x_start+t]);
  }
  for (unsigned int t = 1; t < N; t++) {
      ret.push_back(solution.x[y_start+t]);
  }  
  return ret;
}
