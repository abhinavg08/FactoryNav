// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <cmath>

namespace factorynav {
inline float obstacle_distance(float rx,float ry,float ox,float oy,float radius,
    float half_length=0,float half_width=0,float angle=0) {
  const float dx=rx-ox,dy=ry-oy;
  if(half_length<=0||half_width<=0) return std::hypot(dx,dy)-radius;
  const float c=std::cos(angle),s=std::sin(angle);
  const float x=std::abs(c*dx+s*dy)-half_length,y=std::abs(-s*dx+c*dy)-half_width;
  return std::hypot(std::max(x,0.f),std::max(y,0.f))+std::min(std::max(x,y),0.f);
}
inline float terminal_viability_penalty(float rx,float ry,float ox,float oy,float vx,float vy,
    float radius,float horizon,float lookahead,float weight,float ego_vx=0,float ego_vy=0,
    float half_length=0,float half_width=0,float angle=0,float shape_radius=0) {
  ox+=vx*horizon; oy+=vy*horizon;
  vx-=ego_vx; vy-=ego_vy;
  const float speed2=vx*vx+vy*vy;
  if(speed2<.0064f) return 0;
  const float time=std::clamp(((rx-ox)*vx+(ry-oy)*vy)/speed2,0.0f,lookahead);
  float clearance=1e9f;
  for(float sample:{0.f,time,std::max(0.f,time-1),std::min(lookahead,time+1),lookahead}) {
    const float distance=obstacle_distance(rx,ry,ox+vx*sample,oy+vy*sample,shape_radius,half_length,half_width,angle);
    clearance=std::min(clearance,distance-(radius-shape_radius));
  }
  const float gap=-clearance;
  return weight*std::max(gap,0.0f)*std::max(gap,0.0f);
}
inline float temporal_penalty(float rx,float ry,float ox,float oy,float vx,float vy,
    float obstacle_radius,float time,float margin,float robot_radius,
    float uncertainty_rate,float critical_weight,float repulsion_weight,float half_length=0,float half_width=0,float angle=0) {
  const float distance=obstacle_distance(rx,ry,ox+vx*time,oy+vy*time,obstacle_radius,half_length,half_width,angle);
  const float clearance=distance-(robot_radius+margin+uncertainty_rate*time);
  const float collision=clearance<0 ? critical_weight*(1.0f+std::min(-clearance,1.0f)) : 0.0f;
  return (collision+repulsion_weight*std::exp(-std::max(clearance,0.0f)/.35f))*std::exp(-time/4.0f);
}
}
