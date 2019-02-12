#ifndef COST_H
#define COST_H
#include <iostream>
#include <vector>
#include <math.h>
#include "vehicle.h"

using namespace std;

class Cost {
public:
  // The weighting of different vehicle situations
  const double DANGER     = pow(10,7);
  const double REACH_GOAL = pow(10,5);
  const double EFFICIENCY = pow(10,5);
  const double COMFORT    = pow(10,4);

  // Init some global variables
  vector<vector<double>> sensor_fusion;
  Vehicle *vehicle; // all info from the Vehicle object pointer

  /**
   * Constructor
   */
  Cost(Vehicle *v, vector<vector<double>> s);

  /*
   * Cost functions
   */
  double calculate_cost(); // command to check the overall cost
  double reach_goal(); // cost of (distance to the goal location), if vehicle front is close (cost high)
  double inefficiency_cost(); // cost of (lane speed), if vehicle front is in slower speed (cost high)
  double bufferLen_cost(); // cost of (free space to make a lane change), if vehicle on the L/R is close (cost high)
  double comfort_cost(); // cost of (change lane), if we have to change a lane (cost high)

};

#endif
