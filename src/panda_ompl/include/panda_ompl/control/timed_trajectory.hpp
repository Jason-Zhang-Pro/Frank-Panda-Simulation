#pragma once
#include "panda_ompl/control/gravity_model.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <algorithm>
#include <optional>

namespace panda_ompl::control {
class TimedTrajectory {
  struct Segment { double begin, duration; std::array<Joints,6> c; };
  std::vector<Segment> segments_;
  Joints endpoint_{};
  std::optional<double> start_;
  static int choose(int n,int k) {
    int v=1;for(int i=1;i<=k;++i)v=v*(n-i+1)/i;return v;
  }
 public:
  double duration=0;
  // Same normalized quintic Hermite coefficients as the B-spline collision checker.
  TimedTrajectory(const trajectory_msgs::msg::JointTrajectory &msg,const Joints &current,
                  const GravityModel &model) {
    if(msg.joint_names.size()!=7||msg.points.size()<2||msg.header.stamp.sec||msg.header.stamp.nanosec)
      throw std::runtime_error("轨迹需七个关节、至少两点、零 header 时间");
    std::array<int,7> order{};
    for(int j=0;j<7;++j) {
      if(std::count(msg.joint_names.begin(),msg.joint_names.end(),names[j])!=1)
        throw std::runtime_error("关节名称缺失或重复");
      order[j]=std::find(msg.joint_names.begin(),msg.joint_names.end(),names[j])-msg.joint_names.begin();
    }
    bool derivatives=!msg.points.front().velocities.empty()||!msg.points.front().accelerations.empty();
    std::vector<double> times;
    std::vector<Joints> q,v,a;
    for(const auto &p:msg.points) {
      if(p.positions.size()!=7||!p.effort.empty()||
         (derivatives?(p.velocities.size()!=7||p.accelerations.size()!=7):(!p.velocities.empty()||!p.accelerations.empty())))
        throw std::runtime_error("轨迹点需一致使用仅位置或完整位置/速度/加速度");
      if(p.time_from_start.sec<0||p.time_from_start.nanosec>=1000000000u)
        throw std::runtime_error("无效轨迹时间");
      double t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
      if((times.empty()&&t!=0)||(!times.empty()&&t<=times.back()))throw std::runtime_error("轨迹时间必须从零严格递增");
      Joints pq{},pv{},pa{};
      for(int j=0;j<7;++j) {
        int k=order[j];pq[j]=p.positions[k];
        if(derivatives){pv[j]=p.velocities[k];pa[j]=p.accelerations[k];}
        if(!std::isfinite(pq[j])||!std::isfinite(pv[j])||!std::isfinite(pa[j])||pq[j]<model.lower[j]||pq[j]>model.upper[j])
          throw std::runtime_error("轨迹含非有限值或超出关节限位");
      }
      times.push_back(t);q.push_back(pq);v.push_back(pv);a.push_back(pa);
    }
    for(int j=0;j<7;++j) {
      if(std::abs(q.front()[j]-current[j])>.05)throw std::runtime_error("轨迹起点误差超过 0.05 rad");
      if(std::abs(v.front()[j])>1e-6||std::abs(v.back()[j])>1e-6||std::abs(a.front()[j])>1e-6||std::abs(a.back()[j])>1e-6)
        throw std::runtime_error("整段轨迹起终点导数必须为零");
    }
    for(std::size_t i=1;i<q.size();++i) {
      Segment s{};s.begin=times[i-1];s.duration=times[i]-times[i-1];double dt=s.duration;
      for(int j=0;j<7;++j) {
        auto &c=s.c;c[0][j]=q[i-1][j];c[1][j]=dt*v[i-1][j];c[2][j]=.5*dt*dt*a[i-1][j];
        double r0=q[i][j]-c[0][j]-c[1][j]-c[2][j];
        double r1=dt*v[i][j]-c[1][j]-2*c[2][j],r2=dt*dt*a[i][j]-2*c[2][j];
        c[3][j]=10*r0-4*r1+.5*r2;c[4][j]=-15*r0+7*r1-r2;c[5][j]=6*r0-3*r1+.5*r2;
        double peak=1.875*std::abs(q[i][j]-q[i-1][j])/dt;
        if(derivatives) {
          std::array<double,6> b{};peak=0;
          for(int k=0;k<6;++k) {
            for(int d=0;d<=k;++d)b[k]+=c[d][j]*double(choose(k,d))/choose(5,d);
            if(b[k]<model.lower[j]-1e-9||b[k]>model.upper[j]+1e-9)throw std::runtime_error("插值超出关节限位");
            if(k)peak=std::max(peak,5*std::abs(b[k]-b[k-1])/dt);
          }
        }
        if(peak>model.speed[j]*(1+1e-6))throw std::runtime_error("插值速度超过 URDF 上限");
      }
      segments_.push_back(s);
    }
    duration=times.back();endpoint_=q.back();
  }
  bool sample(double stamp,Joints &q,Joints &v) {
    if(!start_)start_=stamp;
    double elapsed=stamp-*start_;
    if(elapsed>=duration){q=endpoint_;v.fill(0);return true;}
    auto it=std::upper_bound(segments_.begin(),segments_.end(),elapsed,
      [](double t,const Segment &s){return t<s.begin;});
    if(it!=segments_.begin())--it;
    double u=std::clamp((elapsed-it->begin)/it->duration,0.,1.);
    for(int j=0;j<7;++j) {
      q[j]=it->c[5][j];v[j]=5*it->c[5][j];
      for(int k=4;k>=0;--k)q[j]=q[j]*u+it->c[k][j];
      for(int k=4;k>=1;--k)v[j]=v[j]*u+k*it->c[k][j];
      v[j]/=it->duration;
    }
    return false;
  }
};
}
