#include "cost.h"
#include <iostream>
#include <iostream>
#include <math.h>
#include "vehicle.h"

Cost::Cost(Vehicle *v, vector<vector<double>> s){
  vehicle = v;
  sensor_fusion = s;
}


double Cost::calculate_cost() {
  /*std::cout << "reach_goal" << reach_goal() << "|"
  << "inefficiency" << inefficiency_cost() << "|"
  << "Buffer" <<  bufferLen_cost() << "|"
  << "comfort" << comfort_cost() << "\n";
*/
 return reach_goal() + inefficiency_cost() + bufferLen_cost() + comfort_cost();
}

double Cost::reach_goal(){
  int end_lane = vehicle->trajectory_plan.lane_end;
  return exp(-vehicle->collider.dist_to_colli_front[end_lane]/50)*REACH_GOAL;
}

double Cost::inefficiency_cost(){
  int end_lane = vehicle->trajectory_plan.lane_end;
  return ((49.5 - vehicle->collider.spd_of_colli[end_lane])/49.5)*((49.5 - vehicle->collider.spd_of_colli[end_lane])/49.5)*EFFICIENCY;
}

double Cost::bufferLen_cost(){
  int end_lane = vehicle->trajectory_plan.lane_end;
  int start_lane = vehicle->trajectory_plan.lane_start;
  if (end_lane != start_lane){ // if about to change a lane
    return  (exp(-vehicle->collider.dist_to_colli_back[end_lane]/100)+exp(-vehicle->collider.dist_to_colli_front[end_lane]/100))*DANGER;
  }
  return 0; // if stay in the lane 
}

double Cost::comfort_cost(){
  int start_lane = vehicle->trajectory_plan.lane_start;
  int end_lane = vehicle->trajectory_plan.lane_end;
  return (start_lane != end_lane)*COMFORT;
}
