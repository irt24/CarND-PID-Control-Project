#include <cstdlib>
#include <assert.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "pid_wrapper.h"
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

// Command-line arguments (all optional):
//     Kp, Ki, Kd, run_id, num_noneval_steps, num_eval_steps, stop_criterium.
int main(int argc, char **argv)
{
  // Coefficients the Twiddle algorithm converged to.
  double Kp =  0.281000;
  double Ki = 0.00051;
  double Kd = 5.364032;
  std::string run_id = "";

  // By setting these values to 0, Twiddle is not run.
  int num_noneval_steps = 0; 
  int num_eval_steps = 0;
  double stop_criterium = 0.0;

  // Overwrite default values when command-line arguments are provided.
  if (argc > 1) Kp = atof(argv[1]);
  if (argc > 2) Ki = atof(argv[2]);
  if (argc > 3) Kd = atof(argv[3]);
  if (argc > 4) run_id = argv[4];
  if (argc > 5) num_noneval_steps = atoi(argv[5]); 
  if (argc > 6) num_eval_steps = atoi(argv[6]); 
  if (argc > 7) stop_criterium = atof(argv[7]); 

  // Print the flags just to double-check they're assigned to the right variables.
  std::cout << "Initial Kp: " << Kp << std::endl;
  std::cout << "Initial Ki: " << Ki << std::endl;
  std::cout << "Initial Kd: " << Kd << std::endl;
  std::cout << "Run ID: " << run_id << std::endl;
  std::cout << "Num noneval steps for twiddle: " << num_noneval_steps << std::endl;
  std::cout << "Num eval steps for twiddle: " << num_eval_steps << std::endl;
  std::cout << "Stop criterium for twiddle: " << stop_criterium << std::endl;

  PIDWrapper pid_wrapper(Kp, Ki, Kd,
                         num_noneval_steps, num_eval_steps, stop_criterium,
                         run_id);
  int step = 0;

  uWS::Hub h;
  h.onMessage([&pid_wrapper, &step](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        step += 1;

        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());

          // Calculate steering value.
          pid_wrapper.UpdateError(cte);
          double steer_value = -pid_wrapper.TotalError();

          if (steer_value < -1) {
            steer_value = -1;
          } else if (steer_value > 1) {
            steer_value = 1;
          }

          if (cte > 5) {
            // We're probably off-track by now. Start over.
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            std::cout << "Restarting at step " << step << std::endl;
          }
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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
