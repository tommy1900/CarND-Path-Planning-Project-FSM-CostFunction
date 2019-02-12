#ifndef VEHICLE_H
#define VEHICLE_H


#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

enum States { KL, LCL, LCR, PLCL, PLCR }; // have to create a enum, if we want to use switch case for the FSM

class Vehicle {
 public:
 /**
  * Constructor
  */
  Vehicle(int lane, double target_speed);

  // Init some global variables:

  States state = KL; // init the state
  double ref_speed = 0;
  int ref_lane = 0;

  double x = 0;
  double y = 0;
  double s = 0;
  double d = 0;
  double yaw = 0;
  double speed = 0;
  double deltaT = 0;


  // struct to track the potential trajectory
  struct trajectory_plan{
   int lane_start = 1;
   int lane_end = 1;
   double target_speed = 0;
  } trajectory_plan;

  // struct to track the potential collider
  struct collider{
    vector<double> dist_to_colli_front = {999,999,999};
    vector<double> dist_to_colli_back = {999,999,999};
    vector<double> spd_of_colli = {49.5,49.5,49.5};
  } collider;

  // struct to store the action for next loop
  struct next_update{
    double ref_v = 0;
    int lane = 0;
  } next_update;

  // Function to update the vehicle info for this loop
  void Update(double ax, double ay, double as, double ad, double ayaw, double aspeed, int lane, double target_speed, double delta);
  // Function to select the next best state based on FSM
  void choose_next_state(vector<vector<double>> sensor);

private:
  // In case some of the info may not be updated, we should manually reset it every loop
  void _init_vehicle_info();
  // List the potential states based on FSM
  vector<States> _successor_states();
  // Check surrunding vehicles use the sensor info
  void _check_collision_targets(vector<vector<double>> sensor_fusion);
  // Find the best solution using cost functions
  States _find_best_state(vector<States> states, vector<vector<double>> sensor_fusion);
  // Update the state info based on the best solution
  void _realize_next_state(States state);

};


#endif  // VEHICLE_H
