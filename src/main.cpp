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

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          /**
           * Changing the reference co-ordinates to match the car's current position and orientation
           * Involves 2 steps
           * 1. Shift the axes to the current position
           * 2. Rotate the axes to align to car's current orientation (psi)
           */
          vector<double> ptsx_car;
          vector<double> ptsy_car;


          for(unsigned int i=0; i<ptsx.size(); i++) {
        	  //Moving the frame of reference
        	  double diffx = ptsx[i] - px;
        	  double diffy = ptsy[i] - py;
        	  //Turning the frame of reference
        	  ptsx_car.push_back(diffx * cos(0-psi) - diffy * sin(0-psi));
        	  ptsy_car.push_back(diffx * sin(0-psi) + diffy * cos(0-psi));
          }

          double *ptrx = &ptsx_car[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

          double *ptry = &ptsy_car[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          //cte here is the y value at the very initial point x = 0
          double cte = polyeval(coeffs, 0);
          //epsi is actually arctan(3*coeffs[3].x^2 + 2.coeffs[2]*x^1 + 1.coeffs[1]*x^0)
          //since x is 0 (we made the axis shift, we are left with only coeffs[1]
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);

          /**
           * It is given that the latency is 0.1 seconds
           * In order to account for latency, instead of using the current state
           * we'll use the future state at t = 0.1 sec and use that as the current state
           * This accounts for any changes during this 0.1 second at which the actuator commands actually take place
           *
           */
//          state << 0, 0, 0, v, cte, epsi;

          const double Lf = 2.67;
          const double dt = 0.1;

          double future_x = 0 + v*dt;
          double future_y = 0; //since the vehicle direction is in the direction of x axis
          double future_psi = 0 - v * steer_value / Lf * dt;
          double future_v = v + throttle_value * dt;
          double future_cte = cte + v * sin(epsi) * dt;
          double future_epsi = epsi - v * steer_value / Lf * dt;

          state << future_x, future_y, future_psi, future_v, future_cte, future_epsi;


          //coeffs are passed since we need to calculate the future cte and epsi(error in orientation)
          auto vars = mpc.Solve(state, coeffs);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //distance of each x point is 2.5 units
          double poly_inc = 2.5;
          int num_points = 25;

          for(int i=1; i<num_points; i++) {
        	  next_x_vals.push_back(poly_inc * i);
        	  next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }

          /**
           * MPC predicts next values which are returned by mpc.Solve function
           * vars = [steering_angle, throttle, x1, y1, x2, y2, ...]
           * x's are in even, y's are in odd positions starting from 3nd element which is vars[2]
           */

          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for(unsigned int i=2; i<vars.size(); i++) {
        	  if(i%2 == 0) {
        		  mpc_x_vals.push_back(vars[i]);
        	  } else {
        		  mpc_y_vals.push_back(vars[i]);
        	  }
          }


          /**
           * This block passes the info to the simulator
           * steering angle & throttle deteremined by MPC
           * what are my next x,y values based on the polynomial
           * what are MPC's x,y values
           *
           */
          json msgJson;
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);
          msgJson["throttle"] = vars[1];

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

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
