// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "factorynav_mppi_critics/temporal_cost.hpp"

namespace mppi::critics {
class DynamicObstacleCritic : public CriticFunction {
  struct Obstacle {float x,y,vx,vy,radius;};
  std::mutex mutex_;
  std::vector<Obstacle> obstacles_;
  double stamp_{-100};
  float margin_{.08f},robot_radius_{.30f},uncertainty_{.025f};
  float critical_{180.f},repulsion_{10.f},weight_{1.f};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
public:
  void initialize() override {
    auto get=parameters_handler_->getParamGetter(name_);
    get(margin_,"collision_margin",.08f); get(robot_radius_,"robot_radius",.30f);
    get(uncertainty_,"uncertainty_rate",.025f); get(critical_,"critical_weight",180.f);
    get(repulsion_,"repulsion_weight",10.f); get(weight_,"cost_weight",1.f);
    auto node=parent_.lock();
    subscription_=node->create_subscription<sensor_msgs::msg::PointCloud>("/dynamic/tracked_obstacles",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud::ConstSharedPtr message) {
        if(message->header.frame_id!=costmap_ros_->getGlobalFrameID()) return;
        const std::vector<float> *vx=nullptr,*vy=nullptr,*radius=nullptr;
        for(const auto &channel:message->channels) {
          if(channel.name=="vx") vx=&channel.values;
          if(channel.name=="vy") vy=&channel.values;
          if(channel.name=="radius") radius=&channel.values;
        }
        if(!vx||!vy||!radius) return;
        const auto count=std::min({message->points.size(),vx->size(),vy->size(),radius->size(),size_t(16)});
        std::vector<Obstacle> next;
        for(size_t i=0;i<count;++i) {
          const auto &p=message->points[i];
          if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite((*vx)[i])&&std::isfinite((*vy)[i])&&(*radius)[i]>.05f&&(*radius)[i]<1.5f)
            next.push_back({p.x,p.y,(*vx)[i],(*vy)[i],(*radius)[i]});
        }
        std::lock_guard<std::mutex> guard(mutex_); obstacles_=std::move(next);
        stamp_=rclcpp::Time(message->header.stamp).seconds();
      });
    RCLCPP_INFO(logger_,"Loaded time-indexed LiDAR DynamicObstacleCritic (no ground-truth input)");
  }

  void score(CriticData &data) override {
    if(!enabled_) return;
    auto node=parent_.lock(); if(!node) return;
    std::vector<Obstacle> tracks; double age;
    {std::lock_guard<std::mutex> guard(mutex_); tracks=obstacles_; age=node->now().seconds()-stamp_;}
    if(age<-.1||age>.4||tracks.empty()) return;
    const size_t batches=data.trajectories.x.shape()[0],steps=data.trajectories.x.shape()[1];
    for(size_t i=0;i<batches;++i) {
      float total=0;
      for(size_t j=0;j<steps;j+=2) {
        const float time=(j+1)*data.model_dt+std::max(0.0,age);
        for(const auto &o:tracks) total+=factorynav::temporal_penalty(
          data.trajectories.x(i,j),data.trajectories.y(i,j),o.x,o.y,o.vx,o.vy,o.radius,
          time,margin_,robot_radius_,uncertainty_,critical_,repulsion_);
      }
      data.costs(i)+=weight_*total/std::max(size_t(1),(steps+1)/2);
    }
    RCLCPP_INFO_THROTTLE(logger_,*node->get_clock(),5000,"DynamicObstacleCritic scored %zu trajectories against %zu live LiDAR tracks",batches,tracks.size());
  }
};
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicObstacleCritic,mppi::critics::CriticFunction)
