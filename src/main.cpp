#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h" // Don't forget to modify the CMakeList if use additional files

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  // Init some variables
  int lane = 1;
  double ref_vel = 0.0;
  // Init the vehicle object
  Vehicle vehicle = Vehicle(lane, ref_vel);

  h.onMessage([&lane, &ref_vel, &vehicle, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;
          // previous list of points #
          // use it to smooth/optimaze the heading direction,
          // so that we dont have to create a new starting ref every single loop
          int previous_size = previous_path_x.size();
          if (previous_size > 0) car_s = end_path_s;


          //--------------------------
          /* Prediction */
          //--------------------------
          // pass the current known info into the vehicle object for the state prediction
          vehicle.Update(car_x, car_y, car_s, car_d, car_yaw, car_speed, lane, ref_vel, previous_size*.02);
          vehicle.choose_next_state(sensor_fusion);
          //------------------------------
          /* Behavior Planner */
          //------------------------------
          // Update the vehicle with the Behavior plan dicision
          lane = vehicle.next_update.lane;
          ref_vel = vehicle.next_update.ref_v;
          //-------------------------------
          /* Path Generation*/
          //-------------------------------
          vector<double> ptsx;
          vector<double> ptsy;

          // create the command ref of the current state
          //points 01
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if the previous list of path point is almost empty, we will create a new start ref
          if(previous_size < 2) {
              // create a prev car location that is tan to the cur car location
              // points 02
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              // load those points for path prepare
              // those points represent the beginning of the spline
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);


          }
          else {
              // use the prev path end point as the starting ref
              // points 01
              ref_x = previous_path_x.back();
              ref_y = previous_path_y.back();
              // points 02
              double prev_ref_x = previous_path_x[previous_size-2];
              double prev_ref_y = previous_path_y[previous_size-2];

              // load those points for path prepare
              // those points represent the beginning of the spline
              ptsx.push_back(prev_ref_x);
              ptsx.push_back(ref_x);

              ptsy.push_back(prev_ref_y);
              ptsy.push_back(ref_y);
          }

          // adding more way points ahead of the starting ref (int fernet coord)
          vector<double> next_wp0 = getXY(car_s + 30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          // load those points for path prepare
          // those way point represent the end of the spline
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // since different heading angle will generate different trajectory, and we need to update that every loop
          for(int i = 0; i < ptsx.size();i++){ // this loop will reset the heading angle to 0 deg (local car coord)
            //generate new angle based on the new path prepare
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            // this is just cur location * rotation matrix to get the new coord based on the heading
            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // create the spline
          tk::spline s;
          // feed the spline with the path prepare we just created
          s.set_points(ptsx,ptsy);

          // then next_x_vals and next_y_vals will be the actual XY points use in the global map (planner)
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start building the future path
          for(int i = 0; i<previous_path_x.size();i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Fill in the points inbetween the beginning and end of the spline
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          for (int i = 1; i<= 50-previous_path_x.size();i++){ // we use 50 points in total to build the path
            double N = (target_dist/(0.02*ref_vel/2.24)); //0.02 is the sampling freq
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to XY coord
            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x; // update back to the real path, add on to the end of the car.x/.y
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          //


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
