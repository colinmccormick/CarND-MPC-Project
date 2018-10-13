#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// TODO: Set N and dt
size_t N = 10;
double dt = 0.1;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Set some reference values
const double ref_v = 85;
const double ref_cte = 0;
const double ref_epsi = 0;

// Set some convenient indices
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {

    // fg[0] is the cost; initialize at 0
    fg[0] = 0;

    // *************************************
    // Add up costs
    // *************************************

    // Add cost from state variables
    for (int t = 0; t < N; t++) {                     
      fg[0] += 2500.0 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);       
      fg[0] += 2500.0 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);      
      fg[0] += 1.0 * CppAD::pow(vars[v_start + t] - ref_v, 2);   
    }

    // Add cost from actuators
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 10.0 * CppAD::pow(vars[delta_start + t], 2);     
      fg[0] += 10.0 * CppAD::pow(vars[a_start + t], 2);        
    }
 
    // Add cost from actuator changes
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 1000.0 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 100.0 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
 
    // *************************************
    // Calculate constraints
    // *************************************

    fg[1 + x_start]     = vars[x_start];
    fg[1 + y_start]     = vars[y_start];
    fg[1 + psi_start]   = vars[psi_start];
    fg[1 + v_start]     = vars[v_start];
    fg[1 + cte_start]   = vars[cte_start];
    fg[1 + epsi_start]  = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {

      // state variables at previous timestep
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // state variables at current timestep
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // actuator values at last timestep
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> x02 = x0 * x0;
      AD<double> lane1 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x02 + coeffs[3]*x02*x0;
      AD<double> psides1 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x02);

      // calculate constraints
      // difference between t+1 value and model prediction from t value
      AD<double> vdt = v0 * dt;
      AD<double> dLf = delta0 / Lf;
      fg[1 + x_start + t] = x1 - (x0 + CppAD::cos(psi0) * vdt);
      fg[1 + y_start + t] = y1 - (y0 + CppAD::sin(psi0) * vdt);
      fg[1 + psi_start + t] = psi1 - (psi0 + dLf * vdt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ( lane1 - (y0 - CppAD::sin(epsi0) * vdt ));
      fg[1 + epsi_start + t] = epsi1 - ( (psi0 + dLf * vdt) - psides1 );
    }
  }
};

//
// MPC class definition
//

MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // *************************************
  // Set number of variables and constraints
  // *************************************

  size_t n_vars = N * 6 + (N - 1) * 2;
  size_t n_constraints = N * 6;

  double x          = x0[0];
  double y          = x0[1];
  double psi        = x0[2];
  double v          = x0[3];
  double cte        = x0[4];
  double epsi       = x0[5];

  // *************************************
  // Set upper and lower bounds 
  // *************************************

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  vars[x_start]     = x;
  vars[y_start]     = y;
  vars[psi_start]   = psi;
  vars[v_start]     = v;
  vars[cte_start]   = cte;
  vars[epsi_start]  = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuator values to max and min number
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Set steering angle bounds (+/-25 degrees in radians)
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332 * Lf;
    vars_upperbound[i] = 0.436332 * Lf;
  }

  // Set acceleration max/min
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // *************************************
  // Create object that computes objective and constraints
  // *************************************

  FG_eval fg_eval(coeffs);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // *************************************
  // Solve!
  // *************************************

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Print out the cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // *************************************
  // Assemble the result vector and return
  // *************************************

  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int i=0; i < N-1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;

}
