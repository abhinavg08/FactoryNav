// SPDX-License-Identifier: MIT
#include <iostream>
#include "factorynav_mppi_critics/temporal_cost.hpp"
int main() {
  auto risk=[](float x,float y,float t) {
    return factorynav::temporal_penalty(x,y,0,-1,0,1,.28f,t,.08f,.30f,.025f,180,10);
  };
  const auto simultaneous=risk(0,0,1),early=risk(0,0,.1f),lateral=risk(0,-1,1);
  if(!(simultaneous>early*5 && simultaneous>lateral*5)) return 1;
  float straight=0,detour=0;
  for(int i=0;i<=20;++i) {
    float t=i*.1f; straight+=risk(-1+t,0,t); detour+=risk(-1+t,-1,t);
  }
  if(!(straight>detour*2)) return 2;
  const auto stopped=factorynav::terminal_viability_penalty(0,0,3,0,-.24f,0,.78f,4.8f,8,80);
  const auto escaped=factorynav::terminal_viability_penalty(0,1,3,0,-.24f,0,.78f,4.8f,8,80);
  if(!(stopped>40 && escaped==0)) return 3;
  if(std::abs(factorynav::obstacle_distance(1,0,0,0,0,.5f,.35f,0)-.5f)>.001f) return 4;
  if(factorynav::obstacle_distance(0,0,0,0,0,.5f,.35f,0)>=0) return 5;
  const auto crossing_stopped=factorynav::terminal_viability_penalty(-.2f,0,0,2,0,-.18f,1.1f,4.8f,8,80,0,0,.5f,.35f,1.5707963f,.61f);
  const auto crossing_moving=factorynav::terminal_viability_penalty(-.2f,0,0,2,0,-.18f,1.1f,4.8f,8,80,.35f,0,.5f,.35f,1.5707963f,.61f);
  if(!(crossing_moving<crossing_stopped*.3f)) return 6;
  std::cout<<"Temporal collision and lateral avoidance checks passed\n";
  return 0;
}
