#include "panda_ompl/planning_engine.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>

namespace panda_ompl {
namespace {
struct Curve {
  int degree;
  std::vector<Q> controls;
  std::vector<double> knots;

  Q evaluate(double u) const {
    u=std::clamp(u,0.,1.);
    const int n=static_cast<int>(controls.size());
    int span=u>=1.?n-1:int(std::upper_bound(knots.begin(),knots.end(),u)-knots.begin())-1;
    span=std::clamp(span,degree,n-1);
    std::vector<Q> d;
    for(int i=0;i<=degree;++i)d.push_back(controls[span-degree+i]);
    for(int r=1;r<=degree;++r)for(int j=degree;j>=r;--j) {
      const int index=span-degree+j;
      const double denominator=knots[index+degree-r+1]-knots[index];
      const double alpha=denominator>0?(u-knots[index])/denominator:0.;
      for(int k=0;k<7;++k)d[j][k]=(1-alpha)*d[j-1][k]+alpha*d[j][k];
    }
    return d[degree];
  }
  Curve derivative() const {
    if(degree==0)return {0,{Q(7,0.)},{0.,1.}};
    Curve result{degree-1,{},std::vector<double>(knots.begin()+1,knots.end()-1)};
    for(std::size_t i=0;i+1<controls.size();++i) {
      Q q(7);const double denominator=knots[i+degree+1]-knots[i+1];
      for(int j=0;j<7;++j)q[j]=denominator>0?degree*(controls[i+1][j]-controls[i][j])/denominator:0.;
      result.controls.push_back(q);
    }
    return result;
  }
  Q absolute_bound() const {
    Q bound(7,0.);for(const auto &q:controls)for(int j=0;j<7;++j)bound[j]=std::max(bound[j],std::abs(q[j]));
    return bound;
  }
};
Curve make_curve(const std::vector<Q>&points,int refinement) {
  const int segments=static_cast<int>(points.size())-1;
  const int divisions=std::max(1,(3+segments-1)/segments)*refinement;
  Curve result{3,{points.front()},{}};
  for(std::size_t i=1;i<points.size();++i)for(int k=1;k<=divisions;++k) {
    Q q(7);const double u=double(k)/divisions;
    for(int j=0;j<7;++j)q[j]=points[i-1][j]+u*(points[i][j]-points[i-1][j]);
    result.controls.push_back(q);
  }
  const int n=static_cast<int>(result.controls.size());
  for(int i=0;i<n+4;++i)result.knots.push_back(i<=3?0.:(i>=n?1.:double(i-3)/(n-3)));
  return result;
}
int choose(int n,int k) {
  if(k<0||k>n)return 0;int value=1;
  for(int i=1;i<=k;++i)value=value*(n-i+1)/i;
  return value;
}
// Same normalized-time quintic Hermite polynomial used by TimedTrajectory.
std::array<Q,6> coefficients(const trajectory_msgs::msg::JointTrajectoryPoint &a,
                            const trajectory_msgs::msg::JointTrajectoryPoint &b,double dt) {
  std::array<Q,6> c;for(auto &q:c)q=Q(7);
  for(int j=0;j<7;++j) {
    c[0][j]=a.positions[j];c[1][j]=dt*a.velocities[j];c[2][j]=.5*dt*dt*a.accelerations[j];
    const double r0=b.positions[j]-c[0][j]-c[1][j]-c[2][j];
    const double r1=dt*b.velocities[j]-c[1][j]-2*c[2][j];
    const double r2=dt*dt*b.accelerations[j]-2*c[2][j];
    c[3][j]=10*r0-4*r1+.5*r2;c[4][j]=-15*r0+7*r1-r2;c[5][j]=6*r0-3*r1+.5*r2;
  }
  return c;
}
Q sample(const std::array<Q,6>&c,double u) {
  Q q=c[5];for(int i=4;i>=0;--i)for(int j=0;j<7;++j)q[j]=q[j]*u+c[i][j];return q;
}
int64_t stamp(const trajectory_msgs::msg::JointTrajectoryPoint &p) {
  return int64_t(p.time_from_start.sec)*1000000000LL+p.time_from_start.nanosec;
}
}
PlanResult PlanningEngine::bspline_path(const std::vector<Q>&input,PlanResult initial,
                                      double scale,double acceleration,
                                      const std::optional<CartesianConstraint> &constraint) {
  std::vector<Q> points{input.front()};
  for(std::size_t i=1;i<input.size();++i) {
    double d=0;for(int j=0;j<7;++j)d+=std::pow(input[i][j]-points.back()[j],2);
    if(d>1e-20)points.push_back(input[i]);
  }
  if(points.size()==1)return time_path(input,initial,scale,acceleration);
  std::string rejection="B 样条未通过运行时检查";
  for(int refinement:{1,2,4,8,16}) {
    const auto curve=make_curve(points,refinement);
    if(curve.controls.size()>5000){rejection="B 样条控制点过多";break;}
    const auto first=curve.derivative(),second=first.derivative();
    const auto d1=first.absolute_bound(),d2=second.absolute_bound();
    double duration=.3;
    // Convex-hull bounds on spline derivatives, composed with a quintic
    // global time law. Only the whole motion's start/end have zero speed.
    for(int j=0;j<7;++j)duration=std::max({duration,1.875*d1[j]/(speed_[j]*scale),
      std::sqrt((1.875*1.875*d2[j]+(10*std::sqrt(3.)/3)*d1[j])/acceleration)});
    duration*=1.10;
    if(!std::isfinite(duration)||duration>400){rejection="B 样条时间缩放过大";continue;}
    const double conservative_duration=duration;
    if(constraint && tight_cartesian_timing_) {
      // Estimate the composed curve derivatives at their actual time phase.
      // Unlike the old bound, this does not combine unrelated global maxima.
      // Sampling is ONLY an initial estimate, not the final limit certificate.
      double estimated=.3;
      const int samples=std::max(400,int(curve.controls.size())*8);
      for(int k=0;k<=samples;++k) {
        const double r=double(k)/samples;
        const double u=std::clamp(10*std::pow(r,3)-15*std::pow(r,4)+6*std::pow(r,5),0.,1.);
        const double du=30*r*r-60*r*r*r+30*r*r*r*r;
        const double ddu=60*r-180*r*r+120*r*r*r;
        const auto v=first.evaluate(u),a=second.evaluate(u);
        for(int j=0;j<7;++j)estimated=std::max({estimated,
          std::abs(v[j]*du)/(speed_[j]*scale),
          std::sqrt(std::abs(a[j]*du*du+v[j]*ddu)/acceleration)});
      }
      // Limit this first optimization to at most 2x speed-up. Retain 10% margin.
      duration=std::min(conservative_duration,std::max(.5*conservative_duration,1.10*estimated));
    }
    auto build_trajectory=[&](double requested_duration) {
    double duration=requested_duration;
    const int64_t end_ns=static_cast<int64_t>(std::ceil(duration*1e9));
    duration=double(end_ns)*1e-9;
    const int intervals=std::max(2,int(std::ceil(duration/.02)));
    std::set<int64_t> times{0,end_ns};
    for(int i=1;i<intervals;++i)times.insert(static_cast<int64_t>(std::llround(double(end_ns)*i/intervals)));
    // Include crossings of the geometric spline knots in the time grid.
    for(double knot:curve.knots)if(knot>0&&knot<1) {
      double low=0,high=1;
      for(int k=0;k<45;++k){const double u=(low+high)/2,s=10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5);if(s<knot)low=u;else high=u;}
      times.insert(static_cast<int64_t>(std::llround(end_ns*(low+high)/2)));
    }
    PlanResult result=initial;result.checked_samples=0;result.trajectory.points.clear();
    result.trajectory.joint_names=names_;result.duration=duration;
    for(int64_t ns:times) {
      const double r=double(ns)/end_ns;
      const double s=std::clamp(10*std::pow(r,3)-15*std::pow(r,4)+6*std::pow(r,5),0.,1.);
      const double ds=(30*r*r-60*r*r*r+30*r*r*r*r)/duration;
      const double dds=(60*r-180*r*r+120*r*r*r)/(duration*duration);
      const auto q=curve.evaluate(s),v=first.evaluate(s),a=second.evaluate(s);
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions=q;p.velocities=Q(7);p.accelerations=Q(7);
      for(int j=0;j<7;++j){p.velocities[j]=v[j]*ds;p.accelerations[j]=a[j]*ds*ds+v[j]*dds;}
      p.time_from_start.sec=ns/1000000000LL;p.time_from_start.nanosec=ns%1000000000LL;
      // Merge almost coincident times after integer-nanosecond quantization.
      if(!result.trajectory.points.empty()&&ns-stamp(result.trajectory.points.back())<1000&&ns!=end_ns)continue;
      if(ns==end_ns&&!result.trajectory.points.empty()&&ns-stamp(result.trajectory.points.back())<1000)
        result.trajectory.points.pop_back();
      result.trajectory.points.push_back(p);
    }
    result.trajectory.points.front().positions=points.front();
    result.trajectory.points.back().positions=points.back();
    return result;
    };
    PlanResult result;
    bool timing_passed=false;
    for(int attempt=0;attempt<6;++attempt) {
      // Last attempt restores the original conservative duration.
      if(attempt==5)duration=conservative_duration;
      result=build_trajectory(duration);
      double stretch=1.;
      for(std::size_t i=1;i<result.trajectory.points.size();++i) {
        const auto &a=result.trajectory.points[i-1],&b=result.trajectory.points[i];
        const double dt=double(stamp(b)-stamp(a))*1e-9;
        const auto c=coefficients(a,b,dt);
        std::array<Q,6> bernstein;for(auto &q:bernstein)q=Q(7,0.);
        for(int k=0;k<6;++k)for(int d=0;d<=k;++d)for(int j=0;j<7;++j)
          bernstein[k][j]+=c[d][j]*double(choose(k,d))/choose(5,d);
        for(int j=0;j<7;++j) {
          for(int k=0;k<5;++k)stretch=std::max(stretch,
            5*std::abs(bernstein[k+1][j]-bernstein[k][j])/(dt*speed_[j]*scale));
          for(int k=0;k<4;++k)stretch=std::max(stretch,std::sqrt(
            20*std::abs(bernstein[k+2][j]-2*bernstein[k+1][j]+bernstein[k][j])/(dt*dt*acceleration)));
        }
      }
      if(std::isfinite(stretch)&&stretch<=1+1e-6){timing_passed=true;break;}
      if(!std::isfinite(stretch))break;
      duration=std::min(conservative_duration,duration*stretch*1.05);
    }
    if(!timing_passed){rejection="Hermite 时间缩放复检未通过";continue;}
    bool acceptable=true;
    for(std::size_t i=1;i<result.trajectory.points.size()&&acceptable;++i) {
      const auto &a=result.trajectory.points[i-1],&b=result.trajectory.points[i];
      const double dt=double(stamp(b)-stamp(a))*1e-9;
      if(dt<=0){acceptable=false;rejection="B 样条时间量化无效";break;}
      const auto c=coefficients(a,b,dt);
      std::array<Q,6> bernstein;for(auto &q:bernstein)q=Q(7,0.);
      for(int k=0;k<=5;++k)for(int d=0;d<=k;++d)for(int j=0;j<7;++j)
        bernstein[k][j]+=c[d][j]*double(choose(k,d))/choose(5,d);
      double squared_speed_bound=0;
      for(int j=0;j<7&&acceptable;++j) {
        double vmax=0,amax=0;
        for(int k=0;k<6;++k)if(bernstein[k][j]<lower_[j]-1e-9||bernstein[k][j]>upper_[j]+1e-9){acceptable=false;rejection="Hermite 插值超出关节限位";}
        for(int k=0;k<5;++k)vmax=std::max(vmax,5*std::abs(bernstein[k+1][j]-bernstein[k][j])/dt);
        for(int k=0;k<4;++k)amax=std::max(amax,20*std::abs(bernstein[k+2][j]-2*bernstein[k+1][j]+bernstein[k][j])/(dt*dt));
        if(vmax>speed_[j]*scale*(1+1e-6)||amax>acceleration*(1+1e-6)){acceptable=false;rejection="Hermite 插值未满足速度/加速度界限";}
        squared_speed_bound+=vmax*vmax;
      }
      if(!acceptable)break;
      const int count=std::max({1,int(std::ceil(dt/.01)),int(std::ceil(std::sqrt(squared_speed_bound)*dt/.005))});
      for(int k=0;k<=count;++k) {
        std::string reason;
        if(!valid(sample(c,double(k)/count),&reason)){acceptable=false;rejection="B 样条实际插值碰撞："+reason;break;}
        if(constraint) {
          // valid() has updated state_ to the actual controller interpolation.
          const auto tcp=state_->getGlobalLinkTransform("panda_hand_tcp");
          const Eigen::Vector3d direction=constraint->end-constraint->begin;
          const double length_squared=direction.squaredNorm();
          const double fraction=length_squared>1e-16?
            std::clamp((tcp.translation()-constraint->begin).dot(direction)/length_squared,0.,1.):0.;
          const Eigen::Vector3d nearest=constraint->begin+fraction*direction;
          const double angle=Eigen::AngleAxisd(constraint->orientation*tcp.linear().transpose()).angle();
          if((tcp.translation()-nearest).norm()>.002||angle>.025) {
            acceptable=false;rejection="抓放平滑超出直线 2 mm 或姿态 0.025 rad 约束";break;
          }
        }
        ++result.checked_samples;
      }
    }
    if(acceptable) {
      result.trajectory_type=constraint?"cartesian_bspline":"bspline";
      result.smoothing_note="三次 B 样条＋全段时间缩放，保留非零中间速度；控制点细分倍数 "+std::to_string(refinement);
      if(constraint)result.smoothing_note+="；抓放实际插值已离散复检直线 2 mm / 姿态 0.025 rad 约束";
      if(constraint && tight_cartesian_timing_)result.smoothing_note+=
        "；Hermite 包络限速时间优化 "+std::to_string(conservative_duration)+" -> "+std::to_string(result.duration)+" 秒";
      return result;
    }
  }
  auto fallback=time_path(points,initial,scale,acceleration);
  fallback.trajectory_type="stop_fallback";
  fallback.smoothing_note=rejection+"；已回退到原无碰撞的分段停车轨迹";
  return fallback;
}
}
