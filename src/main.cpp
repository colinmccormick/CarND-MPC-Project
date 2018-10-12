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

// Set latency here
const int LATENCY_MS = 100;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

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
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          // *************************************
          // Get current state of vehicle; j[1] is the data JSON object
          // *************************************

          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px     = j[1]["x"];
          double py     = j[1]["y"];
          double psi    = j[1]["psi"];
          double v      = j[1]["speed"];
          double delta  = j[1]["steering_angle"];
          double a      = j[1]["throttle"];

          // *************************************
          // Convert ptsx/ptsy to vehicle frame of reference
          // Rotate so that psi = 0 (helps with polynomial fit)
          // *************************************

          double sp = sin(0-psi);
          double cp = cos(0-psi);

          for (int i=0; i < ptsx.size(); i++) {
            double delta_x = ptsx[i] - px;
            double delta_y = ptsy[i] - py;
            ptsx[i] = delta_x * cp - delta_y * sp;
            ptsy[i] = delta_x * sp + delta_y * cp;
          }

          double* ptrx = &ptsx[0];
          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transformed(ptrx,6);
          Eigen::Map<Eigen::VectorXd> ptsy_transformed(ptry,6);

          // *************************************
          // Do polynomial fit to waypoints
          // *************************************

          auto coeffs = polyfit(ptsx_transformed,ptsy_transformed,3);  
          double cte  = polyeval(coeffs,0);
          double epsi = -atan(coeffs[1]);

          // *************************************
          // Predict where car will be after latency interval
          // Use fact that px, py, psi = 0 (simplifies equations)
          // *************************************

          double dt = LATENCY_MS / 1000.0;

          double px_new     = v * dt;
          double py_new     = 0;
          double psi_new    = v * (-delta) / Lf * dt;
          double v_new      = v + a * dt;
          double cte_new    = cte + sin(psi) * v * dt;
          double epsi_new   = epsi + v * (-delta) / Lf * dt;

          // *************************************
          // Prepare state vector (after latency) and call solver
          // *************************************
          Eigen::VectorXd state(6);
          state << px_new, py_new, psi_new, v_new, cte_new, epsi_new;
          auto vars = mpc.Solve(state, coeffs);

          // *************************************
          // Get points for forward polynomial and predicted trajectory
          // *************************************

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_step = 2.5;
          int N_points = 25;
          for (int i = 1; i < N_points; i++) {
            next_x_vals.push_back(poly_step*i);
            next_y_vals.push_back(polyeval(coeffs,poly_step*i));
          }

          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (int i = 2; i < vars.size(); i++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            } else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          // *************************************
          // Prepare to send steering, throttle, forward poly and predicted trajectory to simulator
          // *************************************

          double Lf = 2.67;

          json msgJson;
          msgJson["steering_angle"] = (-1.0)*vars[0]/(deg2rad(25)*Lf);
          msgJson["throttle"] = vars[1];

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          // *************************************
          // Add latency
          // *************************************
          this_thread::sleep_for(chrono::milliseconds(LATENCY_MS));

          // *************************************
          // Send message
          // *************************************
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
