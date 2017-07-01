/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 10: MPC
 * Date: 27th June 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: main.cpp
 * Description: Main function of the MPC project. Implements the interface 
 *    between the simulator and the MPC algorithm.
\****************************************************************************/

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <cppad/cppad.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "globals.h"
#include "utils.h"

// for convenience
using json = nlohmann::json;
using namespace std;

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
    #ifdef DEBUG
      cout << sdata << endl;
    #endif
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
          v = v * MPH2MS; // convert from mph to m/s

          // Waypoints in global coordinates
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          // Transform waypoints from global to vehicle coorinates and 
          // Fit a polynomial with order 3 to the waypoints.
          // Note: Here we are assuming 2 things:
          //  1) The first waypoint is the closest point in the reference trajectory to the current position
          //  2) The current orientation of the vehicle is approximately the same as the orientation of the reference trajectory at the current position.
          // This may lead to inestability issues. 
          // TODO: To overcome this we could find the exact point in the reference trajectory closest to the current position, and calculate the orientation of the trajectory at that point. That way, the yellow line would be absolutely stable (as it should be, because it's a static reference).
          auto waypoints = transformCoordinates(ptsx, ptsy, px, py, psi);
          Eigen::VectorXd& waypoints_x = waypoints[0];
          Eigen::VectorXd& waypoints_y = waypoints[1];
          const size_t waypoints_size = waypoints_y.size();

          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

          // CTE: 
          // The minimum distance between the reference trajectory and the current position .
          // Note: Read the assumption I mentioned above.
          double cte = polyeval(coeffs, 0);
          
          // Orientation error
          // Calculate predicted path orientation, as the angle of the slope of the polynomial at that point. The slope is calculated with the derivative of the polynomial.
          // Note: Read the assumption I mentioned above.
          const double dpy = dpolyeval(coeffs, 0);
          double epsi = - atan(dpy);
          
          double steer_angle = j[1]["steering_angle"]; // last actuator value
          double throttle = j[1]["throttle"]; // last actuator value
          steer_angle = -steer_angle; // Simulator stuff
          // Position and orientation are now null in vehicle's coordinate system, but will change after the model propagation.
          px = py = psi = 0;
          // Propagate the model for an interval of time equal to the latency before calling the control algorithm.
          px += v * cos(psi) * actuator_lag;
          py += v * sin(psi) * actuator_lag;
          psi += v * steer_angle * actuator_lag / Lf;
          cte += v * sin(epsi) * actuator_lag;
          epsi += v * steer_angle / Lf * actuator_lag;
          v += throttle * actuator_lag;
          
          // Call the MPC algorithm to calculate steering angle and throttle.
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          auto solution = mpc.Solve(state, coeffs);

          const double steer_value = solution[0][0];
          const double throttle_value = solution[0][1];

          json msgJson;
          // Steering command in the simulator whould be between [-1, 1], so we divide by the max possible value: 25 degrees.
          msgJson["steering_angle"] = -steer_value / deg2rad(25); // need to multiply by -1 for the simulator.
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          // This points are in reference to the vehicle's coordinate system
          // and are connected by a Green line
          const vector<double>& mpc_x_vals = solution[1];
          const vector<double>& mpc_y_vals = solution[2];

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(waypoints_size);
          vector<double> next_y_vals(waypoints_size);

          // This points are in reference to the vehicle's coordinate system
          // and are connected by a Yellow line
          for (size_t i = 0; i < waypoints_size; ++i) {
            next_x_vals[i] = waypoints_x(i);
            next_y_vals[i] = waypoints_y(i);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          #ifdef DEBUG
            std::cout << msg << std::endl;
          #endif
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          this_thread::sleep_for(chrono::milliseconds(int(actuator_lag*1000)));
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
