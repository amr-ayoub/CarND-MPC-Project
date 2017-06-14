#include "MPC.h"

using CppAD::AD;
using namespace std;

class FG_eval {

  public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    
    // Fitted polynomial coefficients
    Eigen::VectorXd coeff; 

    FG_eval(Eigen::VectorXd coeffin) : coeff(coeffin) {}

    void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
    ////////////////////////////////////////////////////////////////
    /////////**** Cost **********//////////////////////////////

      fg[0] = 0.0;

    
      // minimizing cte and epsi
      for (int i = 0; i < N; ++i) 
      {

        const auto cte = vars[cte_start + i];
        const auto epsi = vars[epsi_start + i];
        const auto v = vars[v_start + i] - max_velocity;

        fg[0] += (1000.0 * cte * cte + 1000.0 * epsi * epsi + v * v);
      }

      // Avoiding erratic driving

      for (int i = 0; i < N - 1; ++i) 
      {

        const auto delta = vars[delta_start + i];
        const auto a = vars[a_start + i];

        fg[0] += (10.0 * delta * delta + 10.0 * a * a);
      }

    
      // make it more smooth
      for (int i = 0; i < N - 2; ++i) 
      {

        const auto d_delta = vars[delta_start + i + 1] - vars[delta_start + i];
        const auto d_a = vars[a_start + i + 1] - vars[a_start + i];

        fg[0] += (100.0 * d_delta * d_delta + 15.0 * d_a * d_a);
      }


      ///////////////////////////////////////////////////////////////////////////////
      ///////////***** Constraints *****//////////////////////////////////////
    
      //don't change current state
      fg[px_start + 1] = vars[px_start];
      fg[py_start + 1] = vars[py_start];
      fg[psi_start + 1] = vars[psi_start];
      fg[v_start + 1] = vars[v_start];
      fg[cte_start + 1] = vars[cte_start];
      fg[epsi_start + 1] = vars[epsi_start];

      
      for (int i = 0; i < N - 1; ++i) 
      {


        // Current state
        const auto px0 = vars[px_start + i];
        const auto py0 = vars[py_start + i];
        const auto psi0 = vars[psi_start + i];
        const auto v0 = vars[v_start + i];
        const auto cte0 = vars[cte_start + i];
        const auto epsi0 = vars[epsi_start + i];
        const auto delta0 = vars[delta_start + i];
        const auto a0 = vars[a_start + i];

        //Next state
        const auto px1 = vars[px_start + i + 1];
        const auto py1 = vars[py_start + i + 1];
        const auto psi1 = vars[psi_start + i + 1];
        const auto v1 = vars[v_start + i + 1];
        const auto cte1 = vars[cte_start + i + 1];
        const auto epsi1 = vars[epsi_start + i + 1];

        const auto py_desired = coeff[3] * px0 * px0 * px0 + coeff[2] * px0 * px0 + coeff[1] * px0 + coeff[0];
        const auto psi_desired = CppAD::atan(3.0 * coeff[3] * px0 * px0 + 2.0 * coeff[2] * px0 + coeff[1]);


        // Using our kinematic model
        const auto px1_f = px0 + v0 * CppAD::cos(psi0) * dt;
        const auto py1_f = py0 + v0 * CppAD::sin(psi0) * dt;
        const auto psi1_f = psi0 + v0 * (-delta0) / Lf * dt;
        const auto v1_f = v0 + a0 * dt;
        const auto cte1_f = py_desired - py0 + v0 * CppAD::sin(epsi0) * dt;
        const auto epsi1_f = psi0 - psi_desired + v0 * (-delta0) / Lf * dt;

        // set the constraints
        fg[px_start + i + 2] = px1 - px1_f;
        fg[py_start + i + 2] = py1 - py1_f;
        fg[psi_start + i + 2] = psi1 - psi1_f;
        fg[v_start + i + 2] = v1 - v1_f;
        fg[cte_start + i + 2] = cte1 - cte1_f;
        fg[epsi_start + i + 2] = epsi1 - epsi1_f;
      }
    }
};

//
// MPC class definition implementation.
//

MPC::MPC() {


  this->vars.resize(n_vars);

  // Initial value of the independent variables.

  for (int i = 0; i < n_vars; ++i) 
  {
    this->vars[i] = 0.0;
  }


  //Set lower and upper limits for variables.
  this->vars_lowerbound.resize(n_vars);
  this->vars_upperbound.resize(n_vars);

  
  for (int i = 0; i < delta_start; ++i) 
  {
    this->vars_lowerbound[i] = -1.0e10;
    this->vars_upperbound[i] = 1.0e10;
  }

  
  // making sure steering and acceleration between [-1, 1]

  for (int i = delta_start; i < a_start; ++i)
  {
    this->vars_lowerbound[i] = -1.0;
    this->vars_upperbound[i] = 1.0;
  }

  for (int i = a_start; i < n_vars; ++i) 
  {
    this->vars_lowerbound[i] = -1.0;
    this->vars_upperbound[i] = 1.0;
  }


  
  // Lower and upper limits for the constraints

  this->constraints_lowerbound.resize(n_constraints);
  this->constraints_upperbound.resize(n_constraints);


  for (int i = 0; i < n_constraints; ++i) 
  {
    this->constraints_lowerbound[i] = 0.0;
    this->constraints_upperbound[i] = 0.0;
  }
  
}

MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeff) {

  const double px = state[0];
  const double py = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  this->vars[px_start] = px;
  this->vars[py_start] = py;
  this->vars[psi_start] = psi;
  this->vars[v_start] = v;
  this->vars[cte_start] = cte;
  this->vars[epsi_start] = epsi;

  this->constraints_lowerbound[px_start] = px;
  this->constraints_lowerbound[py_start] = py;
  this->constraints_lowerbound[psi_start] = psi;
  this->constraints_lowerbound[v_start] = v;
  this->constraints_lowerbound[cte_start] = cte;
  this->constraints_lowerbound[epsi_start] = epsi;

  this->constraints_upperbound[px_start] = px;
  this->constraints_upperbound[py_start] = py;
  this->constraints_upperbound[psi_start] = psi;
  this->constraints_upperbound[v_start] = v;
  this->constraints_upperbound[cte_start] = cte;
  this->constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeff);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options,
      vars,
      vars_lowerbound,
      vars_upperbound,
      constraints_lowerbound,
      constraints_upperbound,
      fg_eval,
      solution);

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  
  this->steering = solution.x[delta_start];
  this->throttle = solution.x[a_start];

  this->next_xs = {};
  this->next_ys = {};

  for (int i = 0; i < N; ++i) 
  {

    const double px = solution.x[px_start + i];
    const double py = solution.x[py_start + i];

    this->next_xs.emplace_back(px);
    this->next_ys.emplace_back(py);
  }
}
