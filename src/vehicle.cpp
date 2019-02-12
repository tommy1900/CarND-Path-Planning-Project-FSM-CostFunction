#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"

using std::string;
using std::vector;


Vehicle::Vehicle(int lane, double target_speed){
    ref_speed = target_speed;
    ref_lane = lane;
}


//-----------------------------------------------------
/* Public Functions*/
//-----------------------------------------------------

void Vehicle::Update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, int lane, double target_speed, double delta){
  //update the current info
  x = car_x;
  y = car_y;
  s = car_s;
  d = car_d;
  yaw = car_yaw;
  speed = car_speed;
  deltaT = delta;

  ref_speed = target_speed;
  ref_lane = lane;

  //clean data
  _init_vehicle_info(); // some of the info may not be updated, we should manually reset it every loop
}



void Vehicle:: choose_next_state(vector<vector<double>> sensor_fusion){
  vector<States> states = _successor_states();
  _check_collision_targets(sensor_fusion); // update the surrunding vehicles
  // DEBUG
  /*
  cout << "left_lane_v" << collider.dist_to_colli_front[0] << ";"
  << "mid_lane_v" << collider.dist_to_colli_front[1] << ";"
  << "right_lane_v" << collider.dist_to_colli_front[2] << "\n";

  cout << "left_lane_v_back" << collider.dist_to_colli_back[0] << ";"
  << "mid_lane_v_back" << collider.dist_to_colli_back[1] << ";"
  << "right_lane_v_back" << collider.dist_to_colli_back[2] << "\n";
  */
  //DEBUG
  States best_state = _find_best_state(states,sensor_fusion);
  //update state for current loop
  state = best_state;
  //_init_vehicle_info();
  _realize_next_state(state);

  // DEBUG
  /*
  if(state == KL){
    std::cout << "KL " << "\n";
  }
  else if(state == LCL){
    std::cout << "LCL " << "\n";
  }
  else if(state == LCR){
    std::cout << "LCR " << "\n";
  }
  else if(state == PLCL){
    std::cout << "PLCL " << "\n";
  }
  else if(state == PLCR){
    std::cout << "PLCR " << "\n";
  }
  else{

  }
  */
  // DEBUG
  //------------------------------
  /* Behavior Planner */
  //------------------------------
  //update speed based on the potential collider
  double collider_spd = collider.spd_of_colli[ref_lane];
  double collider_dist = collider.dist_to_colli_front[ref_lane];
  //std::cout << "the collider dist" << collider_dist << "|" << "the collider spd" << collider_spd << "\n";
  if( collider_dist > 30 && collider_spd > ref_speed && ref_speed < 49.5){
    next_update.ref_v += 0.224;// std::cout << "speed up" << "\n";
  } else if(collider_dist < 30 && ref_speed > collider_spd && ref_speed > 0){
    next_update.ref_v -= 0.15;// std::cout << "slow down" << "\n"; // drive faster !!!
  }
}




//------------------------------------------------------
/* Private Functions */
//------------------------------------------------------

void Vehicle::_init_vehicle_info(){
   //reset trajectory_plan
   trajectory_plan.lane_start = ref_lane;
   trajectory_plan.lane_end = ref_lane;
   trajectory_plan.target_speed = ref_speed;

   //reset the next update info
   next_update.ref_v = ref_speed;
   next_update.lane = ref_lane;
   collider.dist_to_colli_front = {999,999,999};
   collider.dist_to_colli_back = {999,999,999};
   collider.spd_of_colli = {50,50,50};
}

void Vehicle::_check_collision_targets(vector<vector<double>> sensor_fusion){

  for(int i = 0; i < sensor_fusion.size(); i++){
      float d = sensor_fusion[i][6];

      if(d < (2+4*ref_lane+2) && d>(2+4*ref_lane-2)) {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];
        check_car_s += (double)deltaT*check_speed;
        // if in front of us && within 30 meters
        if((check_car_s > s) && (check_car_s-s < 30)){
          collider.dist_to_colli_front[ref_lane] = (check_car_s-s);
          collider.spd_of_colli[ref_lane] = check_speed;
        }
        else if((check_car_s < s) && (s-check_car_s < 30)){
          collider.dist_to_colli_back[ref_lane] = (s-check_car_s);
        }
      }
      // if on the left lane (we need to consider situation if we at the most right lane)
      else if(d<(2+4*ref_lane-2)) {
        int lane_num;
        if (d < ((ref_lane-1)*4)){ // that vehicle is at lane 0
          lane_num = ref_lane - 2;}
        else{ // else is at lane 1
          lane_num = ref_lane - 1;}

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];
        check_car_s += (double)deltaT*check_speed;
        if((check_car_s - s) < 0){ // car at left lane behind us
          if((s - check_car_s) > 15) continue; // collision will not happen,safe to chenge lane
          // reduce it for more aggressive driving
          else collider.dist_to_colli_back[lane_num] = (s - check_car_s);
        }
        else if((check_car_s - s) < 30){ // car at left lane is ahead us
          collider.dist_to_colli_front[lane_num] = (check_car_s - s);
          collider.spd_of_colli[lane_num] = check_speed;
        }
      }
      // if on the right lane
      else if(d>(2+4*ref_lane+2)) {
        int lane_num;
        if (d > ((ref_lane+2)*4)){ // that vehicle is at lane 0
          lane_num = ref_lane + 2;}
        else{ // else is at lane 1
          lane_num = ref_lane + 1;}

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];
        check_car_s += (double)deltaT*check_speed;
        if((check_car_s - s) < 0){ // car at right lane behind us
          if((s - check_car_s) > 15) continue; // collision will not happen,safe to chenge lane
          // reduce it here for more aggressive driving
          else collider.dist_to_colli_back[lane_num] = (s - check_car_s);
        }
        else if ((check_car_s - s) < 30){ // car at right lane is ahead us
          collider.dist_to_colli_front[lane_num] = (check_car_s - s);
          collider.spd_of_colli[lane_num] = check_speed;
        }
      }
    }
}


States Vehicle::_find_best_state(vector<States> states, vector<vector<double>> sensor_fusion) {
  double cost;
  States best_state = KL;
  double best_cost = 99999999;

  for (int i = 0; i <states.size();i++) {
      //DEBUG

      States potential_state = states[i];
      if(potential_state == KL){
        std::cout<< "potential states: " << "KL " << "\n";
      }
      else if(potential_state == LCL){
        std::cout<< "potential states: " << "LCL " << "\n";
      }
      else if(potential_state == LCR){
        std::cout<< "potential states: " << "LCR " << "\n";
      }
      else if(potential_state == PLCL){
        std::cout<< "potential states: " << "PLCL " << "\n";
      }
      else if(potential_state == PLCR){
        std::cout<< "potential states: " << "PLCR " << "\n";
      }
      else{

      }

      // DEBUG
      _realize_next_state(states[i]);
      // the trajectory_plan should then change based on the state
      Cost CF = Cost(this, sensor_fusion); // create the CostFunction object
      cost = CF.calculate_cost(); std::cout << cost << "\n";
      if(cost<best_cost){
        best_cost = cost;
        best_state = states[i];
      }
  }
  //std::cout << " with cost " << best_cost << "\n";
  return best_state;
}

void Vehicle:: _realize_next_state(States state){
  switch(state){
  case KL: {
    //same lane
    trajectory_plan.lane_start = ref_lane;
    trajectory_plan.lane_end = ref_lane;
    next_update.lane = ref_lane;
    break;
  }
  case PLCL:{
    //same lane
    trajectory_plan.lane_start = ref_lane;
    trajectory_plan.lane_end = ref_lane - 1;
    next_update.lane = ref_lane;
    break;
  }
  case LCL:{
    //same lane
    trajectory_plan.lane_start = ref_lane;
    trajectory_plan.lane_end = ref_lane - 1;
    next_update.lane = ref_lane - 1;
    break;
  }
  case PLCR:{
    //same lane
    trajectory_plan.lane_start = ref_lane;
    trajectory_plan.lane_end = ref_lane + 1;
    next_update.lane = ref_lane;
    break;
  }
  case LCR:{
    //same lane
    trajectory_plan.lane_start = ref_lane;
    trajectory_plan.lane_end = ref_lane + 1;
    next_update.lane = ref_lane + 1;
    break;
  }
  default:
    std::cout << "STATE ERROR\n";
  }
}

vector<States> Vehicle::_successor_states() {
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.

  States current_state = state;
  vector<States> states;
  //select reachable states
  states.push_back(KL);
  if(state == PLCL){
    states.push_back(LCL);
    states.push_back(PLCL);
  } else if(state == PLCR){
    states.push_back(LCR);
    states.push_back(PLCR);
  } else { // when state is KL, LCL and LCR, the potential states are the same
    //check if the LCL/LCR state is already finished
    if(d<(2+4*(ref_lane)+2) && d>(2+4*(ref_lane)-2) && speed > 20){
      if(ref_lane != 0){ // 0 is the most left lane already
        states.push_back(PLCL);
      }
      if(ref_lane != 2){
        states.push_back(PLCR);
      }
    }
  }
  return states;
}
