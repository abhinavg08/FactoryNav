// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <cmath>

namespace factorynav {
inline float terminal_viability_penalty(float rx,float ry,float ox,float oy,float vx,float vy,
    float radius,float horizon,float lookahead,float weight) {
  ox+=vx*horizon; oy+=vy*horizon;
  const float speed2=vx*vx+vy*vy;
  if(speed2<.0064f) return 0;
  const float time=std::clamp(((rx-ox)*vx+(ry-oy)*vy)/speed2,0.0f,lookahead);
  const float gap=radius-std::hypot(rx-ox-vx*time,ry-oy-vy*time);
  return weight*std::max(gap,0.0f)*std::max(gap,0.0f);
}
inline float temporal_penalty(float rx,float ry,float ox,float oy,float vx,float vy,
    float obstacle_radius,float time,float margin,float robot_radius,
    float uncertainty_rate,float critical_weight,float repulsion_weight) {
  const float dx=rx-(ox+vx*time),dy=ry-(oy+vy*time);
  const float safe=obstacle_radius+robot_radius+margin+uncertainty_rate*time;
  const float clearance=std::hypot(dx,dy)-safe;
  const float collision=clearance<0 ? critical_weight*(1.0f+std::min(-clearance,1.0f)) : 0.0f;
  return (collision+repulsion_weight*std::exp(-std::max(clearance,0.0f)/.35f))*std::exp(-time/4.0f);
}
}
