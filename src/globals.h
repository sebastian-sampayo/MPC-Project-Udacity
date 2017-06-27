#ifndef GLOBALS_H
#define GLOBALS_H

// DEBUG
// #define DEBUG

// TODO: Set the timestep length and duration
const size_t N = 20;
const double dt = 0.05;
// => T = 1.25 s

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
// The reference velocity is set to 40 mph.
const double ref_v_max = 100 *0.447; // m/s
const double ref_v_min = 60 *0.447; // m/s
const double R_max = 300; // m
const double R_min = 25; // m

// Actuator latency
const double actuator_lag = 0.1; // seconds

#endif /* GLOBALS_H */