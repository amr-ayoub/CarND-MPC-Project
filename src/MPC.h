#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <vector>

typedef CPPAD_TESTVECTOR(double) Dvector;

// TODO: Set the timestep length and duration
const int N = 10; 
const double dt = 0.1; 

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

// Maximum target velocity to reach
const double max_velocity = 50.0;

// number of state and actuation variables
const int n_vars =  N * 6 + (N - 1) * 2;

// number of constraints
const int n_constraints = N * 6; 


const int px_start = 0;
const int py_start = px_start + N;
const int psi_start = py_start + N;
const int v_start = psi_start + N;
const int cte_start = v_start + N;
const int epsi_start = cte_start + N;
const int delta_start = epsi_start + N;
const int a_start = delta_start + N - 1;

class MPC {

 public:

  double steering;
  double throttle;

  //contains all the state and actuation variables
  Dvector vars; 
  //lower limit for variables in vars
  Dvector vars_lowerbound; 
  //upper limit for variables in vars
  Dvector vars_upperbound; 
  // lower limit for constraints
  Dvector constraints_lowerbound;  
  // upper limit for constraints
  Dvector constraints_upperbound; 

  std::vector<double> next_xs;
  std::vector<double> next_ys;

  MPC();
  virtual ~MPC();

  // Solving the model given an initial state and polynomial coefficients.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd K);
};

#endif /* MPC_H */
