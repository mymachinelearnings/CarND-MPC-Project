#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 10;
double dt = 0.1;

const double Lf = 2.67;

//Objectives for MPC
//Have the car on the line, align with the line and at max velocity
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 70;

/**
 * vars is a very long vector, and if you have N timesteps
 * x's will be from 0 to N-1,
 * y's will be from N to 2N-1,
 * and so on...
 * these values defined are nothing but constants to access the elements from vars vector
 *
 */
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

		//fg[0] is the cost function
		//fg[1 to N] is the constraints vector
		fg[0] = 0;

		/**
		 * The part of the cost based on the reference state.
		 * coeffs below are the weights applied to a particular type of error
		 * how much weight you want to give for that particular error
		 * you might want to give importance to cte & epsi, but not much to velocity or delta or a
		 * since cte & error in orientation are super important as they are the primary factors for error, have their values as higher
		 *
		 */

	    const int cte_cost_weight = 2000;
	    const int epsi_cost_weight = 2000;
	    const int v_cost_weight = 1;
	    const int delta_cost_weight = 5;
	    const int a_cost_weight = 5;
//	    const int delta_vel_weight = 700;
	    const int delta_change_cost_weight = 200;
	    const int a_change_cost_weight = 10;

		for (unsigned int t = 0; t < N; t++) {
			fg[0] += cte_cost_weight  * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
			fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
			fg[0] += v_cost_weight    * CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Minimize the use of actuators.
		for (unsigned int t = 0; t < N - 1; t++) {
			fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
			fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);

//			fg[0] += delta_vel_weight * CppAD::pow(vars[delta_start + t] * vars[a_start + t], 2);
		}

		// Minimize the value gap between sequential actuations.
		for (unsigned int t = 0; t < N - 2; t++) {
			fg[0] += delta_change_cost_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);  //TODO alter this if the jerks are more
			fg[0] += a_change_cost_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		/**
		 * Vars initialization
		 * vars[0] is the cost function, so you need to start from 1
		 *
		 */

	    fg[1 + x_start] = vars[x_start];
	    fg[1 + y_start] = vars[y_start];
	    fg[1 + psi_start] = vars[psi_start];
	    fg[1 + v_start] = vars[v_start];
	    fg[1 + cte_start] = vars[cte_start];
	    fg[1 + epsi_start] = vars[epsi_start];

	    // The rest of the constraints
	    for (unsigned int t = 0; t < N-1; t++) {
	      // The state at time t+1 .
	      AD<double> x1 = vars[x_start + t + 1];
	      AD<double> y1 = vars[y_start + t + 1];
	      AD<double> psi1 = vars[psi_start + t + 1];
	      AD<double> v1 = vars[v_start + t + 1];
	      AD<double> cte1 = vars[cte_start + t + 1];
	      AD<double> epsi1 = vars[epsi_start + t + 1];

	      // The state at time t.
	      AD<double> x0 = vars[x_start + t];
	      AD<double> y0 = vars[y_start + t];
	      AD<double> psi0 = vars[psi_start + t];
	      AD<double> v0 = vars[v_start + t];
	      AD<double> cte0 = vars[cte_start + t];
	      AD<double> epsi0 = vars[epsi_start + t];

	      // Only consider the actuation at time t.
	      AD<double> delta0 = vars[delta_start + t];
	      AD<double> a0 = vars[a_start + t];

	      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
	      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);

	      fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	      fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	      fg[2 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta0 * dt);
	      fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
	      fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
	      fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt); //TODO is this minus
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
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  //Total length of var is n times each of the 6 state variables + (N-1) for each of delta & a
  //Since delta & a doens't make sense for hte last timestep as it dones't have next timestep to calculate delta & a, the lenght is N-1
  size_t n_vars = N * 6 + (N-1) * 2;
  size_t n_constraints = N * 6;

  /**
   * Initializing all vars to 0
   * The first vars values for each of the parameters should be current state
   *
   */
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  /**
   * Setting lower and upper bounds
   * for x, y, psi, v, cte, psi - values could be anything
   * so just set them to very low and very high values as boundaries
   * For delta, it should be -25 degree to + 25 degree
   * For throttle, it should be -1 to +1
   */

  for(unsigned int i=0; i<delta_start; i++) {
	  vars_lowerbound[i] = -1.0e19; //TODO chk with 12
	  vars_upperbound[i] =  1.0e19;
  }

  for(unsigned int i=delta_start; i<a_start; i++) {
	  vars_lowerbound[i] = -0.436332 * Lf;
	  vars_upperbound[i] =  0.436332 * Lf;
  }

  for(unsigned int i=a_start; i<n_vars; i++) {
	  vars_lowerbound[i] = -1.0;
	  vars_upperbound[i] =  1.0;
  }



  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  //Initial state should be same as current state, otherwise the solver might suddenly apply the values from 0 which is a sudden jerk
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

  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for(unsigned int i=0; i<N-1; i++) {
	  result.push_back(solution.x[x_start + i + 1]);
	  result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
