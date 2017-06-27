#ifndef UTILS_H
#define UTILS_H

#include <cppad/cppad.hpp>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


// ------------------------------------------------------------------------------------------------
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

// ------------------------------------------------------------------------------------------------
double deg2rad(double x);
double rad2deg(double x);

// ------------------------------------------------------------------------------------------------
/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2);

// ------------------------------------------------------------------------------------------------
// Evaluate a polynomial.
// f(x) = sum_{i=0}^{P} coeffs(i) * x^i
// with P = coeffs.size() - 1 = order of the polynomial
double polyeval(Eigen::VectorXd coeffs, double x);

// ------------------------------------------------------------------------------------------------
// Overload of polyeval
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

// ------------------------------------------------------------------------------------------------
// TODO: helper function to calculate slope in radians given 2 points.
// Remember that the slope of a polynomial at a given point is the derivative at that point.
// And we know that we are using polynomials, ans we know its order, so it's easy to calculate.
// f'(x) = sum_{i=1}^{P} coeffs(i) * i * x^{i-1}
// with P = coeffs.size() - 1 = order of the polynomial
double dpolyeval(Eigen::VectorXd coeffs, double x);

// ------------------------------------------------------------------------------------------------
// Overload of dpolyeval
CppAD::AD<double> dpolyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

// ------------------------------------------------------------------------------------------------
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

// ------------------------------------------------------------------------------------------------
// Calculates reference speed taking into account the radius of curvature of the reference path
double calculate_ref_v(Eigen::VectorXd coeffs, double x, CppAD::AD<double> cte, CppAD::AD<double> epsi);


#endif /* UTILS_H */