/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 10: MPC
 * Date: 27th June 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: globals.h
 * Description: Global constants.
\****************************************************************************/

#ifndef GLOBALS_H
#define GLOBALS_H

// DEBUG
// #define DEBUG

// Set the timestep length (dt) and duration (N, number of timesteps)
// After trying several combinations of N and dt, I arrived at the conclusion that 
// this values provide enough time to look ahead in time (1.6 seconds) for the curbs, as well as 
// reduced computation time which is critical in a control application. In other words,
// it is the maximum dt and minimum N that provides optimal precision and performance.
const size_t N = 8;
const double dt = 0.2;
// => T = 1.6 s

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

// Both the reference cross track and orientation errors are 0.
// The reference velocity is a variable in the range 60-100 mph. 
// If the radius of curvature is lower than R_min, the reference velocity will be ref_v_min
// If the radius of curvature is higher then R_max, the reference velocity will be ref_v_max
// In the middle we extrapolate the velocity.
const double ref_v_max = 120 *0.447; // m/s
const double ref_v_min = 70 *0.447; // m/s
const double R_max = 300; // m
const double R_min = 25; // m

// Actuator latency
// TODO: This latency actually depends on the resolution of the simulator and may be is platform dependent. A great improvement would be to measure in some way exactly this latency, may be dynamically for each frame.
const double actuator_lag = 0.1; // 0.1 seconds = 100 ms

// Scale factor to convert from mph to m/s
const double MPH2MS = 0.447;

#endif /* GLOBALS_H */