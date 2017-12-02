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

double clamp(double value, double lower, double upper)
{
  return std::min(upper, std::max(value, lower));
}


int main()
{
  //drives but not very stable
  //double steer_Kp = 0.1;
  //double steer_Ki = 0.005;
  //double steer_Kd = 3;

  // removing integral part doesn't help a lot
  //double steer_Kp = 0.1;
  //double steer_Ki = 0.0;
  //double steer_Kd = 3;

  // adding sum of 10 last element so integral enabled again, up to 20 km speed
  //double steer_Kp = 0.07;
  //double steer_Ki = 0.005;
  //double steer_Kd = 3.0;


  double steer_Kp = 0.07;
  double steer_Ki = 0.005;
  double steer_Kd = 3.0;

  double speed_Kp = 1.0;
  double speed_Ki = 0.05;
  double speed_Kd = 4.0;

  uWS::Hub h;

  PID pid_for_steer, pid_for_speed;

  pid_for_steer.Init(steer_Kp, steer_Ki, steer_Kd);
  pid_for_speed.Init(speed_Kp, speed_Ki, speed_Kd);

  h.onMessage([&pid_for_steer,&pid_for_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
         
          std::cout << "SPEED: " << speed << " ANGLE: " << angle << std::endl;

          pid_for_steer.UpdateError(cte);
          double steer_value =  pid_for_steer.TotalError();
          double clamped_steer_value = clamp(steer_value, -1, 1);
          std::cout << "CLAMPED: " << clamped_steer_value << std::endl;
         
          pid_for_speed.UpdateError(clamped_steer_value);
          // PID controller for spped
          double max_throttle = 0.5;
          double speed_correction_value = max_throttle + pid_for_speed.TotalError();     

       // //// Throttle thresholds
          //double throttle = 0.0;
          //double inverse_steer = (1.0-fabs(clamped_steer_value));
          //std::cout << "INVERSE: " << inverse_steer << std::endl;
          //if (inverse_steer > 0.9)
          //  throttle = inverse_steer;
          //else if (inverse_steer > 0.1)
          //  throttle = inverse_steer * 0.1;
          //else
          //  throttle = inverse_steer * 0.05;
          //std::cout << "THROTTLE: " << throttle << std::endl;
          //          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "Speed CTE: " << speed - desired_speed << " Speed Correction Value " << speed_correction_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = clamped_steer_value;
          msgJson["throttle"] = speed_correction_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
