#pragma once
#include <urdf/model.h>
#include <Eigen/Geometry>
#include <array>
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

namespace panda_ompl::control {
using Joints = std::array<double, 7>;
inline const std::array<std::string, 7> names{
  "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
  "panda_joint5", "panda_joint6", "panda_joint7"};
inline Eigen::Isometry3d transform(const urdf::Pose &p) {
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  t.linear()=Eigen::Quaterniond(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z).normalized().toRotationMatrix();
  return t;
}
// Sum gravitational moments about arm joint axes. Includes moving fingers.
// This compensates gravity only, not inertia, Coriolis or carried-object mass.
class GravityModel {
 public:
  urdf::Model model;
  Joints lower{},upper{},speed{};
  explicit GravityModel(const std::string &xml) {
    if(!model.initString(xml))throw std::runtime_error("URDF 解析失败");
    for(int i=0;i<7;++i) {
      auto j=model.getJoint(names[i]);
      if(!j||j->type!=urdf::Joint::REVOLUTE||!j->limits)throw std::runtime_error("缺少 Panda 关节限位");
      lower[i]=j->limits->lower;upper[i]=j->limits->upper;speed[i]=j->limits->velocity;
      if(!std::isfinite(lower[i])||!std::isfinite(upper[i])||!std::isfinite(speed[i])||lower[i]>=upper[i]||speed[i]<=0)
        throw std::runtime_error("URDF 限位无效");
    }
    for(const auto &n:{"panda_hand","panda_leftfinger","panda_rightfinger"}) {
      auto l=model.getLink(n);
      if(!l||!l->inertial||l->inertial->mass<=0)throw std::runtime_error("缺少夹爪质量");
    }
  }
  Joints evaluate(const Joints &q,const std::array<double,2> &fingers) const {
    struct Axis { int index; Eigen::Vector3d point,axis; };
    Joints g{};
    std::function<void(urdf::LinkConstSharedPtr,const Eigen::Isometry3d&,std::vector<Axis>)> visit;
    visit=[&](urdf::LinkConstSharedPtr link,const Eigen::Isometry3d &t,std::vector<Axis> ancestors) {
      if(link->inertial) {
        const auto &in=*link->inertial;
        if(!std::isfinite(in.mass)||in.mass<0)throw std::runtime_error("URDF 质量无效");
        Eigen::Vector3d com=t*Eigen::Vector3d(in.origin.position.x,in.origin.position.y,in.origin.position.z);
        Eigen::Vector3d force(0,0,-9.81*in.mass);
        for(const auto &a:ancestors)g[a.index]-=a.axis.dot((com-a.point).cross(force));
      }
      for(const auto &joint:link->child_joints) {
        auto frame=t*transform(joint->parent_to_joint_origin_transform);
        auto next=ancestors;
        if(joint->type!=urdf::Joint::FIXED) {
          Eigen::Vector3d axis(joint->axis.x,joint->axis.y,joint->axis.z);
          if(axis.norm()<1e-12)throw std::runtime_error("零关节轴");
          axis.normalize();
          auto it=std::find(names.begin(),names.end(),joint->name);
          if(it!=names.end()&&joint->type==urdf::Joint::REVOLUTE) {
            int index=std::distance(names.begin(),it);
            next.push_back({index,frame.translation(),frame.linear()*axis});
            frame.rotate(Eigen::AngleAxisd(q[index],axis));
          } else if(joint->type==urdf::Joint::PRISMATIC&&
                    (joint->name=="panda_finger_joint1"||joint->name=="panda_finger_joint2")) {
            frame.translate(axis*fingers[joint->name=="panda_finger_joint1"?0:1]);
          } else if(joint->type==urdf::Joint::PRISMATIC&&joint->name.find("finger_pad_")!=std::string::npos) {
            // Passive tile displacement is not in controller feedback. Include
            // tile mass at its zero-compression location (<=1.5 mm error).
          } else throw std::runtime_error("不支持的活动关节："+joint->name);
        }
        visit(model.getLink(joint->child_link_name),frame,next);
      }
    };
    visit(model.getRoot(),Eigen::Isometry3d::Identity(),{});
    for(double v:g)if(!std::isfinite(v))throw std::runtime_error("重力计算非有限值");
    return g;
  }
};
}
