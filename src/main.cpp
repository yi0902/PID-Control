#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.1, 0.01, 4.96);
  
  static int steps = 0;
  static int reset_count = 0;
  static int case_count = 0;
  static double throttle_value = 0.5;
  static double max_cte = 4;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double mean_sq_error;
          
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          steps += 1;
          
          // implement twiddle to tune parameters
          // if cross tracking error is too big
          // -> modify the coefficients
          // -> reset the simulator
          if(fabs(cte) > max_cte){
            
            mean_sq_error = pid.sq_error / steps ;
            
            // if after first round
            if(reset_count == 0){
              pid.best_sq_error = mean_sq_error;
            }
            
            if(mean_sq_error < pid.best_sq_error){
              pid.best_sq_error = mean_sq_error;
              // update dp
              if ((case_count == 0) || (case_count == 1) || (case_count == 2)) pid.dp *= 1.1;
              // update di
              if ((case_count == 3) || (case_count == 4) || (case_count == 5)) pid.di *= 1.1;
              // update dd
              if ((case_count == 6) || (case_count == 7) || (case_count == 8)) pid.dd *= 1.1;
            }
            else{
              // update Kp
              if(case_count == 0){
                pid.Kp += pid.dp;
              }
              if (case_count == 1) {
                pid.Kp -= 2 * pid.dp;
              }
              if (case_count == 2) {
                pid.Kp += pid.dp;
                pid.dp *= 0.9;
              }
              // update Ki
              if(case_count == 3){
                pid.Ki += pid.di;
              }
              if (case_count == 4) {
                pid.Ki -= 2 * pid.di;
              }
              if (case_count == 5) {
                pid.Ki += pid.di;
                pid.di *= 0.9;
              }
              // update Kd
              if(case_count == 6){
                pid.Kd += pid.dd;
              }
              if (case_count == 7) {
                pid.Kd -= 2 * pid.dd;
              }
              if (case_count == 8) {
                pid.Kd += pid.dd;
                pid.dd *= 0.9;
              }
              // update case counter
              case_count += 1;
              if(case_count % 9 == 0) case_count = 0;
            }
            
            // reset the simulator
            reset_count += 1;
            steps = 0;
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          
          pid.UpdateError(cte);
          steer_value = - pid.TotalError();
          
          // limit the steering angle
          if (steer_value < -1) steer_value = -1;
          if (steer_value > 1) steer_value = 1;
          
          // update the throttle according to steering angle and speed
          if (fabs(steer_value) > 0.5){
            throttle_value = 0.5 * (1 - 0.8 * fabs(steer_value)) ;
            if (speed >= 30) throttle_value = 0;
          }
          if (fabs(steer_value) < 0.2){
            throttle_value = 1 - fabs(steer_value);
            if(speed >= 50) throttle_value = 0.5;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Kp: " << pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << " Steering : " << steer_value << " Throttle: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
