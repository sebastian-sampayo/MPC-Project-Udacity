/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 10: MPC
 * Date: 27th June 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: utils.cpp
 * Description: Implementation of utils.h (Read header for more description)
\****************************************************************************/

#include "utils.h"

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "globals.h"

using namespace std;

// ------------------------------------------------------------------------------------------------
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// ------------------------------------------------------------------------------------------------
inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// ------------------------------------------------------------------------------------------------
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// ------------------------------------------------------------------------------------------------
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

// ------------------------------------------------------------------------------------------------
double dpolyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * i * pow(x, i-1);
  }
  return result;
}

// ------------------------------------------------------------------------------------------------
CppAD::AD<double> dpolyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * i * CppAD::pow(x, i-1);
  }
  return result;
}

// ------------------------------------------------------------------------------------------------
double ddpolyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 2; i < coeffs.size(); i++) {
    result += coeffs[i] * i * (i-1) * pow(x, i-2);
  }
  return result;
}

// ------------------------------------------------------------------------------------------------
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// ------------------------------------------------------------------------------------------------
double calculate_ref_v(Eigen::VectorXd coeffs, double x, CppAD::AD<double> cte, CppAD::AD<double> epsi) {
  const double df = dpolyeval(coeffs, x); // first derivative
  const double ddf = ddpolyeval(coeffs, x); // second derivative

  double num = 1 + df*df;
  num *= num;
  num *= num; // num^3
  const double radius = sqrt(num) / abs(ddf);

  #ifdef DEBUG
    cout << "Radius of curvature: " << radius << endl;
  #endif

  double v;
  
  // Proportional
  if (radius > R_max) {
    v = ref_v_max;
  } else if (radius > R_min) {
    // Linear interpolation between ref_v_min and ref_v_max
    v = ref_v_min + (radius - R_min) / (R_max - R_min) * (ref_v_max - ref_v_min);
  } else { // radius < R_min
    v = ref_v_min;
  }
  
  // If the radius is small but we are doing good, speed up.
  const double tol_cte = 1;
  const double tol_epsi = 10 * M_PI/180.0;
  if (radius < 4*R_min && CppAD::abs(cte) < tol_cte && CppAD::abs(epsi) < tol_epsi) {
    v *= 1.25;
    #ifdef DEBUG
      cout << "Small R as well as small error!" << endl;
    #endif
  }

  return v;
}

//------------------------------------------------------------------------------------------------
vector<Eigen::VectorXd> transformCoordinates(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi) {
  const size_t waypoints_size = ptsx.size();
  assert(waypoints_size == ptsy.size());
  Eigen::VectorXd waypoints_x(waypoints_size);
  Eigen::VectorXd waypoints_y(waypoints_size);
    // Transform waypoints from global's coordinates to vehicle's coordinates
    // Rotation coord from global to vehicle: ([T]^{VG}(psi))
      // [ cos(psi) sin(psi)
      //   -sin(psi) cos(psi)]
    // Translation:
    // waypoint_v = waypoint_g - carpos_g
    // [x]^V = [T]^{VG}(psi) * ([x]^G - [p]^G), where x is the waypoint and p the vehicle position
  for (size_t i = 0; i < waypoints_size; ++i) {
    const double dx = ptsx[i] - px;
    const double dy = ptsy[i] - py;
    waypoints_x(i) = cos(psi) * dx + sin(psi) * dy;
    waypoints_y(i) = -sin(psi) * dx + cos(psi) * dy;
  }

  return {waypoints_x, waypoints_y};
}