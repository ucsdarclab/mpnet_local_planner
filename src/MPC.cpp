#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  std::vector<double> tgx, tgy;
  FG_eval(std::vector<double> ptsx, std::vector<double> ptsy) { 
    this -> tgx = ptsx;
    this -> tgy = ptsy; 
  }
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;
    for (int t = 0; t < N; t++) {
      // fg[0] += 100.0*CppAD::pow(vars[cte_start + t], 2);
      // fg[0] += 100.0*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 500 *CppAD::pow(vars[x_start + t]-tgx[t], 2);
      fg[0] += 500 *CppAD::pow(vars[y_start + t]-tgy[t], 2);
      fg[0] += 100 *CppAD::pow(vars[v_start + t]- ref_v, 2);
      // fg[0] += 10*(CppAD::pow(vars[x_start + t] - goal[0], 2)+CppAD::pow(vars[y_start + t] - goal[1], 2));
    }

    fg[0] += 500 * CppAD::pow(vars[x_start + N-1]-tgx[N-1], 2);
    fg[0] += 500 * CppAD::pow(vars[y_start + N-1]-tgy[N-1], 2);

    for (int t = 0; t < N - 1; t++) {
      fg[0] += 1*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 1*CppAD::pow(vars[a_start + t], 2);
    }

    for (int t = 0; t < N - 2; t++) {
      fg[0] += 50*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    // terminal loss
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    for (int t = 1; t < N; t++) {
      if(t >= tgx.size()){
        fg[1 + x_start + t] = 0;
        fg[1 + y_start + t] = 0;
        fg[1 + psi_start + t] = 0;
        fg[1 + v_start + t] = 0;
      }
      else{
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        // AD<double> cte1 = vars[cte_start + t];
        // AD<double> epsi1 = vars[epsi_start + t];
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        // AD<double> cte0 = vars[cte_start + t - 1];
        // AD<double> epsi0 = vars[epsi_start + t - 1];
        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> a0 = vars[a_start + t - 1];
        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
        fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      }
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(std::vector<double> state, std::vector<double> ptsx, std::vector<double> ptsy) {
  bool ok = true;
  
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  // Number of model variables (includes both states and inputs).
  size_t n_vars = N * 4 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 4;
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Lower and upper limits for variables.
  for (i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (i = v_start; i < delta_start; i++) {
    vars_lowerbound[i] =  0; 
    vars_upperbound[i] =  0.2; 
  }

  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] =  -0.5; 
    vars_upperbound[i] =  0.5; 
  }

  // Acceleration/decceleration upper and lower limits.
  for (i = a_start; i < n_vars; i++) {
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
  FG_eval fg_eval(ptsx, ptsy);

  // options for IPOPT solver
  std::string options;
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
  options += "Numeric max_cpu_time          0.1\n";

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
 
  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  // result.push_back(solution.x[v_start]);

  for(i=0; i< N-1; i++){
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  
  return result;
}
