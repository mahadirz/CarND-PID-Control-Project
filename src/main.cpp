#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <vector>
#include <numeric>
// for convenience
using nlohmann::json;
using std::cout;
using std::endl;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/**
 * https://stackoverflow.com/a/868894/11189869
 * @param begin
 * @param end
 * @param option
 * @return
 */
char *getCmdOption(char **begin, char **end, const std::string &option)
{
  char **itr = std::find(begin, end, option);
  if (itr != end && ++itr != end)
  {
    return *itr;
  }
  return 0;
}

/**
 * https://stackoverflow.com/a/868894/11189869
 * @param begin
 * @param end
 * @param option
 * @return
 */
bool cmdOptionExists(char **begin, char **end, const std::string &option)
{
  return std::find(begin, end, option) != end;
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  double p = 0.2;
  double i = 0.00009;
  double d = 6;

  double dp = 0.1;
  double di = 0.0001;
  double dd = 1;
  bool is_twidle = false;

  if (cmdOptionExists(argv, argv + argc, "-t") == 1)
  {
    is_twidle = true;
    cout << "twiddle enabled" << endl;
  }
  if (cmdOptionExists(argv, argv + argc, "-p") == 1)
  {
    string _p = getCmdOption(argv, argv + argc, "-p");
    p = std::stod(_p);
  }
  if (cmdOptionExists(argv, argv + argc, "-i") == 1)
  {
    string _i = getCmdOption(argv, argv + argc, "-i");
    i = std::stod(_i);
  }
  if (cmdOptionExists(argv, argv + argc, "-d") == 1)
  {
    string _d = getCmdOption(argv, argv + argc, "-d");
    d = std::stod(_d);
  }
  if (cmdOptionExists(argv, argv + argc, "--dp") == 1)
  {
    string _dp = getCmdOption(argv, argv + argc, "--dp");
    dp = std::stod(_dp);
  }
  if (cmdOptionExists(argv, argv + argc, "--di") == 1)
  {
    string _di = getCmdOption(argv, argv + argc, "--di");
    di = std::stod(_di);
  }
  if (cmdOptionExists(argv, argv + argc, "--dd") == 1)
  {
    string _dd = getCmdOption(argv, argv + argc, "--dd");
    dd = std::stod(_dd);
  }

  cout << "init P: " << p << " I: " << i << " D: " << d << endl;
  cout << "init dP: " << dp << " dI: " << di << " dD: " << dd << endl;
  pid.Init(p, i, d);
  int it_count = 0;
  int dp_pointer = 0;
  vector<double> params = {p, d, i};
  vector<double> d_params = {dp, dd, di};
  double best_err = -1;
  double sum_cte = 0;
  bool is_decrement_flag = false;
  int total_reset = 0;

  // pid" -p 0.2 -i 0.00009 -d 6

  h.onMessage([&pid, &it_count, &is_twidle, &dp_pointer, &params, &d_params, &best_err, &sum_cte, &is_decrement_flag, &total_reset](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                                    uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket messaged
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          if (is_twidle)
          {

            double sum_dp = d_params[0] + d_params[1] + d_params[2];
            double tolerance = 0.1;
            if (sum_dp < tolerance)
            {
              cout << "sum_dp:" << sum_dp << endl;
              exit(0);
            }

            if (it_count >= 2000)
            {
              cout << "best error: " << best_err << " sum_cte:" << sum_cte << " p: " << params[0]
                   << " i: " << params[2] << " d:" << params[1] << endl;

              if (best_err == -1)
              {
                best_err = sum_cte;
              }
              else
              {
                // reset
                if (sum_cte < best_err)
                {
                  best_err = sum_cte;
                  d_params[dp_pointer] *= 1.1;

                  //go to next param
                  dp_pointer = (dp_pointer + 1) % params.size();
                  is_decrement_flag = false;
                }
                else if (is_decrement_flag)
                {
                  params[dp_pointer] += d_params[dp_pointer];
                  d_params[dp_pointer] *= 0.9;
                  is_decrement_flag = false;
                }
                else
                {
                  // decrement the value
                  is_decrement_flag = true;
                }
              }

              // reset iteration to 0
              it_count = 0;
              if (is_decrement_flag)
              {
                params[dp_pointer] -= 2 * d_params[dp_pointer];
              }
              else
              {
                params[dp_pointer] += d_params[dp_pointer];
              }

              pid.Init(params[0], params[2], params[1]);
              // reset simulator
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

              total_reset++;
              cout << "total_reset:" << total_reset << endl;
              cout << "New p: " << params[0]
                   << " i: " << params[2] << " d:" << params[1] << endl;
              cout << " dp:" << d_params[0] << " di:" << d_params[2] << " dd:" << d_params[1] << endl;
              sum_cte = 0;
            }
            else if (it_count > 100)
            {
              // start recording the error after x epoch
              sum_cte += (cte * cte);
            }

            it_count++;
          }

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (steer_value > 1)
            steer_value = 1;
          else if (steer_value < -1)
            steer_value = -1;

          if (!is_twidle)
          {
            // DEBUG
            cout << "CTE: " << cte << " Steering Value: " << steer_value
                 << " Speed: " << speed << " Angle: " << angle
                 << endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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