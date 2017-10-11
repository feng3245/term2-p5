#ifndef MPC_H
#define MPC_H
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cppad/ipopt/solve_result.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>

using namespace std;
using CppAD::AD;
using namespace Ipopt;
typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  Dvector GetDefault(int size);
};

#endif /* MPC_H */
