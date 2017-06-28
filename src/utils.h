/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 10: MPC
 * Date: 27th June 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: utils.h
 * Description: Some utilitarian functions to calculate polynomial coefficients,
 *    evaluate them, derive them, etc.
\****************************************************************************/

#ifndef UTILS_H
#define UTILS_H

#include <cppad/cppad.hpp>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

/*** NOT USED *** - but could be useful in a future version
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2);

/**
 * polyeval Evaluate a polynomial.
 * f(x) = sum_{i=0}^{P} coeffs(i) * x^i
 * with P = coeffs.size() - 1 = order of the polynomial
 * @param coeffs Polynomial coefficients.
 * @param x Point where to evaluate
 * @output f(x)
 */
double polyeval(Eigen::VectorXd coeffs, double x);

// Evaluate a polynomial.
// Overload
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

/**
 * dpolyeval Calculates the first derivative of a polynomial at a point x
 * f'(x) = sum_{i=1}^{P} coeffs(i) * i * x^{i-1}
 * with P = coeffs.size() - 1 = order of the polynomial
 * @param coeffs Polynomial coefficients.
 * @param x Point where to evaluate
 * @output f'(x)
 */
double dpolyeval(Eigen::VectorXd coeffs, double x);

// Calculates the first derivative of a polynomial at a point x
// Overload
CppAD::AD<double> dpolyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

/**
 * ddpolyeval Calculates the second derivative of a polynomial at a point x
 * f''(x) = sum_{i=2}^{P} coeffs(i) * i (i-1) * x^{i-2}
 * with P = coeffs.size() - 1 = order of the polynomial
 * @param coeffs Polynomial coefficients.
 * @param x Point where to evaluate
 * @output f''(x)
 */
double ddpolyeval(Eigen::VectorXd coeffs, double x);

/**
 * polyfit Fit a polynomial.
 * Adapted from
 * https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 * @param xvals Vector with x values of the input points.
 * @param yvals Vector with y values of the input points.
 * @param order Order of the fitted polynomial.
 * @output Coefficient of the polynomial fitted
 */
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

/**
 * calculate_ref_v Calculates the reference velocity of the algorithm taking into account the radius of curvature of the reference path (waypoints path) at a given point and the current errors (cte and epsi).
 * @param coeffs Reference path polynomial coefficients.
 * @param x Point where to evaluate the radius of curvature
 * @param cte Current cross-track error
 * @param epsi Current orientation error
 * @output Current reference velocity
 */
double calculate_ref_v(Eigen::VectorXd coeffs, double x, CppAD::AD<double> cte, CppAD::AD<double> epsi);


#endif /* UTILS_H */