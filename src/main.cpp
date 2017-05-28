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
#include "CoordinateTransform.h"
#include "helpers.h"
// for convenience
using json = nlohmann::json;

string hasData(string s);
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


          vector<double> next_x_vals;
          vector<double> next_y_vals;

          CoordinateTransform ct(px, py, psi);

          // I will solve the MPC in local coordinate frame,
          // so I'll first transform the reference points from global to local coordiane frame
          ct.Transform(ptsx, ptsy, next_x_vals, next_y_vals);

          // fit the future reference points,
          // using second order polynomial, since it can have at most one 'hump', so it is good if we predict
          // steering for only one turn;
          Eigen::VectorXd coeffs;
          Eigen::VectorXd reference_x=Eigen::Map<Eigen::VectorXd>(next_x_vals.data(), next_x_vals.size());
          Eigen::VectorXd reference_y=Eigen::Map<Eigen::VectorXd>(next_y_vals.data(), next_y_vals.size());

          coeffs = polyfit(reference_x, reference_y, 4);

          Eigen::VectorXd state(4);

          // since I am working in local coordinate frame, the coordinates are 0, 0, 0
          double tmpPx=0;
          double tmpPy=0;
          double tmpPhi=0;
          double tmpVe= v * 0.44704;
          // to meters per second
          state<<tmpPx, tmpPy, tmpPhi, tmpVe;
          vector<double> result;
          result = mpc.Solve(state, coeffs);
          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */



          // steer value of 1 corresponds to the 25 degrees to the right
          // meaning negative -25
          double steer_value=-result[0]/deg2rad(25);
          double throttle_value=result[1] / 0.44704;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          mpc_x_vals.push_back(tmpPx);
          mpc_y_vals.push_back(tmpPy);
          for (int i=0; i<result.size(); i+=2){
            applyControl<double>(tmpPx, tmpPy, tmpPhi, tmpVe, result[i], result[i+1], dt, Lf);
            mpc_x_vals.push_back(tmpPx);
            mpc_y_vals.push_back(tmpPy);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line


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
