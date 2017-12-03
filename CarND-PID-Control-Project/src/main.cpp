#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <time.h>


// for convenience
using json = nlohmann::json;
using namespace std;

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

  // Propotional gain presents the ratio of output response to the error signal. 
  // is the error term has a magnitude of 10, a propotional gain of 5 would produce propotional response of 50. 
  // In general increasing proportional speed will increase the speed of the control system response. 
  // However if hte gain is too big (like here ~ 0.5) , the process value will begin oscillate nad then system become unstable. 
  // If the gain is too small (like here ~0.01), the process value will not react fast enough and system will fail. 
  // here proportional gain of 0.7 with constant throttle of 0.1 is very stable, and with 0.2 is stable enough.
  double steer_Kp = 0.07;
  // In general, this component takes advantages of longer run's errors summing them up over time and increasing gain slowly as a concequence. 
  // Given that we can expect Ki gain helps mostly on longer turn when sum of error incresing. 
  // For this particular lab I'm using window version of that sum to makes calculations to be more local.
  // Low number is because we are dealing here with kind of a big number (here 10x bigger than Kp because we are summing up 10 last Kp gains)
  double steer_Ki = 0.005;
  // Kd gain is a derivative component which causes the output to decrease if the process varaible is increasing rapidly.
  // Too less number won't help to stabilize the system, too big will provide too much additional unstability. 
  double steer_Kd = 2.0;

  double speed_Kp = 20.0;
  double speed_Ki = 0.0;
  double speed_Kd = 0.0;


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
         
          std::cout << " CTE: " << cte  << " SPEED: " << speed << " ANGLE: " << angle << std::endl;

          std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
          std::cout << "PID FOR STEERing" << std::endl;
          pid_for_steer.UpdateError(cte);
          double steer_value =  pid_for_steer.TotalError();
          double clamped_steer_value = clamp(steer_value, -1, 1);
          std::cout << "Steer correction: " << clamped_steer_value << std::endl;
          std::cout << "PID FOR STEERing END" << std::endl;

          std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
          std::cout << "PID FOR SPEED" << std::endl;
          std::cout << "angle " << angle << "deg2rad " << deg2rad(angle) << std::endl;

          pid_for_speed.UpdateError(deg2rad(angle));

          double throttle = 0.1; //pid_for_speed.TotalError();

          std::cout << "Speed correction:  " << throttle << std::endl;
          std::cout << "PID FOR SPEED END" << std::endl;
          std::cout << "------------------------------" << std::endl;

          json msgJson;
          msgJson["steering_angle"] = clamped_steer_value;
          msgJson["throttle"] = throttle;
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
