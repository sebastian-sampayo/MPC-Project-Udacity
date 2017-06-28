/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 10: MPC
 * Date: 27th June 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: MPC.h
 * Description: Declaration of the MPC class.
\****************************************************************************/

#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  /**
   * Solve the model given an initial state and polynomial coefficients.
   * Return the first actuations + predicted path
   * @param state State vector containing position x, position y, orientation psi,
                  cross-track error cte, and orientation error epsi.
     @param coeffs Coefficients of the polynomial 
     @output A vector of 3 vectors: 
              actuators, 
              x-values of the predicted path, 
              y-values of the predicted path.
   */
  vector<vector<double>> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
