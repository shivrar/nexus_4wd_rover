//
// Created by shivan on 4/14/22.
//

#ifndef NEXUS_4WD_ROVER_COMMON_ITEMS_H
#define NEXUS_4WD_ROVER_COMMON_ITEMS_H

#define MAX_LR_VEL 200
#define MAX_FORWARD_VEL 200
#define MAX_ANG_VEL PI/4

float MapCommand(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max){
  return static_cast<float>(x - in_min) * (out_max - out_min) / static_cast<float>(in_max - in_min) + out_min;
}

/**
 *
 * Storage container to store commands for the rover;
 *
 * */
struct VelCommand{
  int fwd;
  int lr;
  float omega;
  uint32_t time;

  inline explicit VelCommand():fwd(0),lr(0), omega(0), time(0){}

  inline explicit VelCommand(int f, int lr, float omega, uint32_t t):fwd(f),lr(lr), omega(omega), time(t){}
};



#endif //NEXUS_4WD_ROVER_COMMON_ITEMS_H
