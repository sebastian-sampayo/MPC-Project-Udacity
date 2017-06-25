#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// DEBUG
#define DEBUG

// for convenience
using json = nlohmann::json;
using namespace std;

// ------------------------------------------------------------------------------------------------
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// ------------------------------------------------------------------------------------------------
/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// ------------------------------------------------------------------------------------------------
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// ------------------------------------------------------------------------------------------------
// Evaluate a polynomial.
// f(x) = sum_{i=0}^{P} coeffs(i) * x^i
// with P = coeffs.size() - 1 = order of the polynomial
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// ------------------------------------------------------------------------------------------------
// TODO: helper function to calculate slope in radians given 2 points.
// Remember that the slope of a polynomial at a given point is the derivative at that point.
// And we know that we are using polynomials, ans we know its order, so it's easy to calculate.
// f'(x) = sum_{i=1}^{P} coeffs(i) * i * x^{i-1}
// with P = coeffs.size() - 1 = order of the polynomial
double dpolyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * i * pow(x, i-1);
  }
  return result;
}

// ------------------------------------------------------------------------------------------------
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
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
// void transformWaypoints(Eigen::VectorXd& waypoints_car, const std::vector<double> waypoints_global, 
                        // double px_global, double py_global, double psi_global) {
  // // Transform waypoints from global coordinates to car coordinates
  // // Rotation coord from global to car: ([T]^{CG}(psi))
    // // [ cos(psi) sin(psi)
    // //   -sin(psi) cos(psi)]
  // // Translation:
  // // waypoint_car = waypoint_global - carpos_global
  // // [x]^C = [T]^{CG}(psi) * ([x]^G - [p]^G), where x is the waypoint and p the car position
  
  // const size_t waypoints_size = waypoints_global.size();
// }

// ------------------------------------------------------------------------------------------------
int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // State in global coordinates
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          
          // Waypoints in global coordinates
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          // Fit a polynomial with order 3 to the waypoints.
          const size_t waypoints_size = ptsx.size();
          assert(waypoints_size == ptsy.size());
          Eigen::VectorXd waypoints_x(waypoints_size);
          Eigen::VectorXd waypoints_y(waypoints_size);
            // Transform waypoints from global's coordinates to vehicle's coordinates
            // Rotation coord from global to vehicle: ([T]^{CG}(psi))
              // [ cos(psi) sin(psi)
              //   -sin(psi) cos(psi)]
            // Translation:
            // waypoint_v = waypoint_g - carpos_g
            // [x]^C = [T]^{CG}(psi) * ([x]^G - [p]^G), where x is the waypoint and p the car position
          for (size_t i = 0; i < waypoints_size; ++i) {
            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;
            waypoints_x(i) = cos(psi) * dx + sin(psi) * dy;
            waypoints_y(i) = -sin(psi) * dx + cos(psi) * dy;
          }
          
          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

          
          // CTE: 
          // The minimum distance between the current position and the reference trajectory.
          
          
          // Orientation error
          // Calculate predicted path orientation, as the angle of the slope of the polynomial at that point. The slope is calculated with the derivative of the polynomial.
          double dpy = dpolyeval(coeffs, px);
          

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;
          

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          // Must be in vehicle's coordinate system
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(waypoints_size);
          vector<double> next_y_vals(waypoints_size);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (size_t i = 0; i < waypoints_size; ++i) {
            next_x_vals[i] = waypoints_x(i);
            next_y_vals[i] = waypoints_y(i);
          }

          // Must be in vehicle's coordinate system
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
