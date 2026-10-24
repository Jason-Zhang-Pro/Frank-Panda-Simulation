#include "panda_ompl/planning_engine.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <random>
#include <Eigen/Cholesky>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace panda_ompl {
namespace {
Eigen::Isometry3d transform(const urdf::Pose &pose) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  t.linear() = Eigen::Quaterniond(pose.rotation.w, pose.rotation.x,
                                pose.rotation.y, pose.rotation.z).normalized().toRotationMatrix();
  return t;
}
geometry_msgs::msg::Pose message(const Eigen::Isometry3d &t) {
  geometry_msgs::msg::Pose p;
  p.position.x=t.translation().x(); p.position.y=t.translation().y(); p.position.z=t.translation().z();
  const Eigen::Quaterniond q(t.linear());
  p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
  return p;
}
double distance(const Q &a, const Q &b) {
  double sum=0;
  for(std::size_t j=0;j<a.size();++j) sum += std::pow(a[j]-b[j],2);
  return std::sqrt(sum);
}
Q interpolate(const Q &a, const Q &b, double u) {
  Q q(a.size());
  for(std::size_t j=0;j<a.size();++j) q[j]=a[j]+u*(b[j]-a[j]);
  return q;
}
}
PlanningEngine::PlanningEngine(const std::string &xml, const std::string &world,double clearance) {
  if(!std::isfinite(clearance)||clearance<0)throw std::runtime_error("碰撞裕量非法");
  clearance_=clearance;
  auto u=urdf::parseURDF(xml);
  if(!u) throw std::runtime_error("机器人 URDF 解析失败");
  auto s=std::make_shared<srdf::Model>();
  if(!s->initString(*u,"<robot name=\""+u->getName()+"\"><group name=\"panda_arm\"><chain base_link=\"panda_link0\" tip_link=\"panda_hand_tcp\"/></group></robot>"))
    throw std::runtime_error("SRDF 解析失败");
  model_=std::make_shared<moveit::core::RobotModel>(u,s);
  scene_=std::make_shared<planning_scene::PlanningScene>(model_);
  scene_->getCollisionEnvNonConst()->setPadding(clearance);
  state_=std::make_shared<moveit::core::RobotState>(model_);
  state_->setToDefaultValues();
  for(int i=1;i<=7;++i) {
    auto name="panda_joint"+std::to_string(i);
    auto joint=u->getJoint(name);
    if(!joint || !joint->limits) throw std::runtime_error("缺少七关节限位");
    names_.push_back(name); lower_.push_back(joint->limits->lower);
    upper_.push_back(joint->limits->upper); speed_.push_back(joint->limits->velocity);
  }
  // Same policy as the existing checker: fixed assemblies and directly
  // adjacent assemblies only. No posture-dependent automatic suppression.
  auto &acm=scene_->getAllowedCollisionMatrixNonConst();
  std::map<std::string,std::string> groups;
  for(const auto &item:u->links_) groups[item.first]=item.first;
  std::function<std::string(const std::string&)> group=[&](const std::string &n) {
    return groups[n]==n?n:groups[n]=group(groups[n]);
  };
  for(const auto &item:u->joints_) {
    const auto &j=item.second;
    if(j->type==urdf::Joint::FIXED) groups[group(j->child_link_name)]=group(j->parent_link_name);
  }
  std::set<std::pair<std::string,std::string>> adjacent;
  for(const auto &item:u->joints_) {
    auto a=group(item.second->parent_link_name), b=group(item.second->child_link_name);
    if(a>b) std::swap(a,b); adjacent.emplace(a,b);
  }
  for(const auto &a:u->links_) for(const auto &b:u->links_) if(a.first<b.first) {
    auto ga=group(a.first),gb=group(b.first); if(ga>gb) std::swap(ga,gb);
    if(ga==gb || adjacent.count({ga,gb})) acm.setEntry(a.first,b.first,true);
  }
  load_world(world);
  // Only rigid base assembly is allowed to touch its mounting floor.
  const auto base_group=group(u->getRoot()->name);
  for(const auto &item:u->links_) if(group(item.first)==base_group)
    acm.setEntry("floor",item.first,true);
}
void PlanningEngine::load_world(const std::string &path) {
  auto world=urdf::parseURDFFile(path);
  if(!world) throw std::runtime_error("世界 URDF 读取失败："+path);
  // Existing Gazebo spawns Panda at identity. Refuse silently mismatched frames.
  if(model_->getModelFrame()!="world")
    throw std::runtime_error("当前实现要求机器人根坐标为 world，且 Gazebo 生成位姿为零");
  std::function<void(urdf::LinkConstSharedPtr,const Eigen::Isometry3d&)> visit;
  visit=[&](urdf::LinkConstSharedPtr link,const Eigen::Isometry3d &base) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id=model_->getModelFrame(); object.id=link->name;
    object.operation=moveit_msgs::msg::CollisionObject::ADD;
    for(const auto &collision:link->collision_array) {
      if(!collision || !collision->geometry) continue;
      shape_msgs::msg::SolidPrimitive primitive;
      const auto &g=collision->geometry;
      if(g->type==urdf::Geometry::BOX) {
        const auto *box=static_cast<const urdf::Box*>(g.get());
        primitive.type=primitive.BOX;
        primitive.dimensions={box->dim.x,box->dim.y,box->dim.z};
      } else if(g->type==urdf::Geometry::CYLINDER) {
        const auto *c=static_cast<const urdf::Cylinder*>(g.get());
        primitive.type=primitive.CYLINDER; primitive.dimensions={c->length,c->radius};
      } else if(g->type==urdf::Geometry::SPHERE) {
        primitive.type=primitive.SPHERE;
        primitive.dimensions={static_cast<const urdf::Sphere*>(g.get())->radius};
      } else throw std::runtime_error("不支持的世界碰撞形状："+link->name);
      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(message(base*transform(collision->origin)));
    }
    if(!object.primitives.empty()) {
      if(!scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("世界碰撞体加载失败："+link->name);
      ++objects_;
      if(link->name=="target_cube") {
        if(object.primitives.size()!=1 || object.primitives[0].type!=shape_msgs::msg::SolidPrimitive::BOX)
          throw std::runtime_error("目标方块必须是一个 box");
        const auto &d=object.primitives[0].dimensions;cube_size_=Eigen::Vector3d(d[0],d[1],d[2]);
      }
      if(link->name=="target_pedestal" || link->name=="place_pedestal") {
        const auto &d=object.primitives.at(0).dimensions;
        if(object.primitives[0].type!=shape_msgs::msg::SolidPrimitive::BOX ||
           !base.linear().isApprox(Eigen::Matrix3d::Identity(),1e-8))
          throw std::runtime_error("支撑台必须是轴对齐 box");
        supports_[link->name]={base.translation(),Eigen::Vector3d(d[0],d[1],d[2])};
      }
    }
    for(const auto &child:link->child_links) {
      if(!child->parent_joint || child->parent_joint->type!=urdf::Joint::FIXED)
        throw std::runtime_error("规划世界必须是固定关节树");
      visit(child,base*transform(child->parent_joint->parent_to_joint_origin_transform));
    }
  };
  visit(world->getRoot(),Eigen::Isometry3d::Identity());
  if(!supports_.count("place_pedestal") || !supports_.count("target_pedestal"))
    throw std::runtime_error("场景缺少抓放支撑台");
  place_=supports_.at("place_pedestal").center;
  place_.z()+=supports_.at("place_pedestal").size.z()/2+cube_size_.z()/2;
}
Q PlanningEngine::arm(const Positions &positions) const {
  Q q; for(const auto &name:names_) q.push_back(positions.at(name)); return q;
}
void PlanningEngine::set_positions(const Positions &positions) {
  for(const auto &name:variables()) {
    const auto it=positions.find(name);
    if(it==positions.end() || !std::isfinite(it->second))
      throw std::runtime_error("缺少实际关节状态："+name);
    state_->setVariablePosition(name,it->second);
  }
  state_->update();
}
bool PlanningEngine::valid(const Q &q,std::string *reason) {
  if(q.size()!=7) { if(reason)*reason="必须是七关节状态"; return false; }
  for(std::size_t j=0;j<7;++j) {
    if(!std::isfinite(q[j]) || q[j]<lower_[j] || q[j]>upper_[j]) {
      if(reason)*reason="关节限位："+names_[j]; return false;
    }
    state_->setVariablePosition(names_[j],q[j]);
  }
  state_->update();
  // Gazebo's prismatic hard stops can report tiny solver penetration.
  // Arm limits remain strict above; tolerate at most 0.1 mm for fingers.
  for(const auto &name:variables()) {
    const auto &bounds=model_->getVariableBounds(name);
    const double value=state_->getVariablePosition(name);
    const double tolerance=name.find("finger")!=std::string::npos?1e-4:0.;
    if(bounds.position_bounded_ && (value<bounds.min_position_-tolerance || value>bounds.max_position_+tolerance)) {
      if(reason)*reason="关节限位："+name+"="+std::to_string(value);
      return false;
    }
  }
  if(payload_ && !support_.empty()) {
    const auto t=state_->getGlobalLinkTransform("panda_hand_tcp")*payload_->tcp_to_cube;
    const auto half=t.linear().cwiseAbs()*cube_size_/2;
    const auto &table=supports_.at(support_);
    const double top=table.center.z()+table.size.z()/2;
    const bool overlap=std::abs(t.translation().x()-table.center.x())<half.x()+table.size.x()/2 &&
                       std::abs(t.translation().y()-table.center.y())<half.y()+table.size.y()/2;
    if(overlap && t.translation().z()-half.z()<top-.002) {
      if(reason)*reason="携物穿入支撑台超过 2mm";return false;
    }
  }
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.contacts=reason!=nullptr; req.max_contacts=1;
  scene_->checkCollision(req,res,*state_);
  if(res.collision && reason) {
    *reason="碰撞";
    if(!res.contacts.empty()) *reason += ": "+res.contacts.begin()->first.first+" <-> "+res.contacts.begin()->first.second;
  }
  return !res.collision;
}
bool PlanningEngine::valid_edge(const Q &a,const Q &b,double resolution) {
  if(a.size()!=7 || b.size()!=7 || !std::isfinite(resolution) || resolution<=0) return false;
  const auto count=std::max(1,static_cast<int>(std::ceil(distance(a,b)/resolution)));
  for(int i=0;i<=count;++i) if(!valid(interpolate(a,b,double(i)/count))) return false;
  return true;
}
Eigen::Isometry3d PlanningEngine::link_pose(const std::string &name) const {
  if(!model_->hasLinkModel(name)) throw std::runtime_error("未知连杆："+name);
  return state_->getGlobalLinkTransform(name);
}
void PlanningEngine::update_target(const Eigen::Isometry3d &pose) {
  moveit_msgs::msg::CollisionObject target;
  target.id="target_cube";target.header.frame_id=model_->getModelFrame();
  shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;
  box.dimensions={cube_size_.x(),cube_size_.y(),cube_size_.z()};
  target.primitives.push_back(box);target.primitive_poses.push_back(message(pose));
  target.operation=moveit_msgs::msg::CollisionObject::ADD;
  if(!scene_->processCollisionObjectMsg(target)) throw std::runtime_error("同步方块失败");
}
void PlanningEngine::set_algorithm(const std::string &algorithm) {
  if(algorithm!="rrt"&&algorithm!="rrt_connect"&&algorithm!="rrt_star")
    throw std::runtime_error("planner_algorithm 必须为 rrt / rrt_connect / rrt_star");
  algorithm_=algorithm;
}
PlanResult PlanningEngine::plan(const Positions &positions,const Q &goal,double timeout,
                               bool shortcut,double velocity_scale,double acceleration) {
  if(!std::isfinite(timeout)||timeout<=0||!std::isfinite(velocity_scale)||velocity_scale<=0||velocity_scale>1||
     !std::isfinite(acceleration)||acceleration<=0) throw std::runtime_error("规划参数非法");
  set_positions(positions);
  const auto start=arm(positions);
  std::string reason;
  if(!valid(start,&reason)) throw std::runtime_error("起点无效："+reason);
  if(!valid(goal,&reason)) throw std::runtime_error("终点无效："+reason);
  auto space=std::make_shared<ob::RealVectorStateSpace>(7);
  ob::RealVectorBounds bounds(7);
  for(std::size_t j=0;j<7;++j) {bounds.setLow(j,lower_[j]); bounds.setHigh(j,upper_[j]);}
  space->setBounds(bounds);
  og::SimpleSetup setup(space);
  std::size_t queries=0;
  setup.setStateValidityChecker([this,&queries](const ob::State *state) {
    ++queries;
    const auto *q=state->as<ob::RealVectorStateSpace::StateType>();
    return valid(Q(q->values,q->values+7));
  });
  setup.getSpaceInformation()->setStateValidityCheckingResolution(.02/space->getMaximumExtent());
  ob::ScopedState<> begin(space),end(space);
  for(std::size_t j=0;j<7;++j) {begin[j]=start[j];end[j]=goal[j];}
  setup.setStartAndGoalStates(begin,end);
  if(algorithm_=="rrt") {
    auto planner=std::make_shared<og::RRT>(setup.getSpaceInformation());
    planner->setRange(.15);planner->setGoalBias(.05);setup.setPlanner(planner);
  } else if(algorithm_=="rrt_star") {
    auto planner=std::make_shared<og::RRTstar>(setup.getSpaceInformation());
    planner->setRange(.15);planner->setGoalBias(.05);planner->setDelayCC(true);
    setup.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    setup.setPlanner(planner);
  } else {
    auto planner=std::make_shared<og::RRTConnect>(setup.getSpaceInformation());
    planner->setRange(.15);setup.setPlanner(planner);
  }
  const auto began=std::chrono::steady_clock::now();
  double first_solution=-1;
  setup.getProblemDefinition()->setIntermediateSolutionCallback(
    [&](const ob::Planner*,const std::vector<const ob::State*>&,const ob::Cost) {
      if(first_solution<0)first_solution=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    });
  const auto status=setup.solve(timeout);
  PlanResult result;result.planner=algorithm_;result.validity_queries=queries;
  result.search_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
  if(!setup.haveExactSolutionPath())throw std::runtime_error(algorithm_+" 未找到精确路径："+status.asString());
  result.first_solution_seconds=first_solution<0?result.search_seconds:first_solution;
  const auto &path=setup.getSolutionPath(); result.raw_points=path.getStateCount();
  std::vector<Q> points;
  for(std::size_t i=0;i<path.getStateCount();++i) {
    const auto *q=path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    points.emplace_back(q->values,q->values+7);
  }
  auto length=[](const std::vector<Q>&path) {
    double total=0;for(std::size_t i=1;i<path.size();++i)total+=distance(path[i-1],path[i]);return total;
  };
  result.raw_length=length(points);
  if(shortcut)points=shortcut_path(points);
  result.shortcut_length=length(points);
  if(bspline_enabled_)return bspline_path(points,result,velocity_scale,acceleration);
  return time_path(points,result,velocity_scale,acceleration);
}
std::vector<Q> PlanningEngine::shortcut_path(const std::vector<Q> &points,double timeout) {
  if(points.empty() || !std::isfinite(timeout) || timeout<=0)throw std::runtime_error("shortcut 输入非法");
  std::vector<Q> reduced{points.front()};
  std::size_t i=0;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
  while(i+1<points.size()) {
    std::size_t next=i+1;
    for(std::size_t j=points.size()-1;j>i+1;--j) {
      if(std::chrono::steady_clock::now()>deadline)break;
      if(valid_edge(points[i],points[j])){next=j;break;}
    }
    reduced.push_back(points[next]);i=next;
  }
  return reduced;
}
PlanResult PlanningEngine::time_path(const std::vector<Q> &points,PlanResult result,
                                     double velocity_scale,double acceleration) {
  std::string reason;
  std::vector<Q> unique{points.front()};
  for(std::size_t i=1;i<points.size();++i) if(distance(unique.back(),points[i])>1e-10) unique.push_back(points[i]);
  if(unique.size()==1) unique.push_back(unique.front());
  result.trajectory.joint_names=names_;
  double elapsed=0;
  auto append=[&](const Q &q,double seconds) {
    trajectory_msgs::msg::JointTrajectoryPoint p;p.positions=q;
    const auto ns=static_cast<int64_t>(std::llround(seconds*1e9));
    p.time_from_start.sec=ns/1000000000LL;p.time_from_start.nanosec=ns%1000000000LL;
    result.trajectory.points.push_back(p);
  };
  append(unique[0],0);
  for(std::size_t i=1;i<unique.size();++i) {
    const auto &a=unique[i-1],&b=unique[i]; double duration=.3;
    for(std::size_t j=0;j<7;++j) {
      const double delta=std::abs(b[j]-a[j]);
      duration=std::max({duration,1.875*delta/(speed_[j]*velocity_scale),
                         std::sqrt((10*std::sqrt(3.)/3)*delta/acceleration)});
    }
    duration*=1.02;
    const int count=std::max({1,int(std::ceil(duration/.02)),int(std::ceil(1.875*distance(a,b)/.01))});
    for(int k=0;k<=count;++k) {
      const double u=double(k)/count, blend=10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5);
      if(!valid(interpolate(a,b,blend),&reason)) throw std::runtime_error("轨迹碰撞复检失败："+reason);
      ++result.checked_samples;
    }
    elapsed+=duration;append(b,elapsed);
  }
  result.duration=elapsed; return result;
}
void PlanningEngine::configure_task(const Positions &positions,const Eigen::Isometry3d &cube,
                                   const std::optional<Payload> &payload,bool finger_contact,
                                   const std::string &support) {
  if(scene_->getCurrentState().hasAttachedBody("target_cube")) {
    moveit_msgs::msg::AttachedCollisionObject remove;
    remove.object.id="target_cube";remove.object.operation=moveit_msgs::msg::CollisionObject::REMOVE;
    if(!scene_->processAttachedCollisionObjectMsg(remove)) throw std::runtime_error("解除规划附着失败");
  }
  update_target(cube);
  payload_=payload;support_=support;
  auto &acm=scene_->getAllowedCollisionMatrixNonConst();
  for(const auto &finger:{"panda_leftfinger","panda_rightfinger"})acm.setEntry("target_cube",finger,finger_contact);
  for(const auto &name:{"target_pedestal","place_pedestal"})acm.setEntry("target_cube",name,support==name);
  if(payload) {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name="panda_hand_tcp";
    attached.touch_links={"panda_leftfinger","panda_rightfinger"};
    attached.object.id="target_cube";
    attached.object.header.frame_id="panda_hand_tcp";
    attached.object.operation=moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;
    // MoveIt link padding does not expand attached bodies. Expand the carried
    // cube explicitly so it retains the same configured environment clearance.
    box.dimensions={cube_size_.x()+2*clearance_,cube_size_.y()+2*clearance_,cube_size_.z()+2*clearance_};
    attached.object.primitives.push_back(box);
    attached.object.primitive_poses.push_back(message(payload->tcp_to_cube));
    if(!scene_->processAttachedCollisionObjectMsg(attached))throw std::runtime_error("建立携物碰撞体失败");
  }
  state_=std::make_shared<moveit::core::RobotState>(scene_->getCurrentState());
  set_positions(positions);
}
Payload PlanningEngine::capture(const Positions &positions,const Eigen::Isometry3d &cube) {
  set_positions(positions);Payload p;
  p.tcp_to_cube=state_->getGlobalLinkTransform("panda_hand_tcp").inverse()*cube;
  return p;
}
Q PlanningEngine::solve_ik(const Positions &positions,const Eigen::Vector3d &xyz,bool local,
                           const std::vector<double> &xyzw) {
  if(!xyz.allFinite())throw std::runtime_error("IK 目标必须有限");
  set_positions(positions);
  Eigen::Quaterniond rotation(0,1,0,0);
  if(!xyzw.empty()) {
    if(xyzw.size()!=4)throw std::runtime_error("IK 四元数需要四个数");
    rotation=Eigen::Quaterniond(xyzw[3],xyzw[0],xyzw[1],xyzw[2]);
    if(!rotation.coeffs().allFinite()||rotation.norm()<1e-8)throw std::runtime_error("IK 四元数非法");
  }
  return inverse_kinematics(xyz,arm(positions),local,local?1.:5.,rotation,xyzw.empty());
}
Q PlanningEngine::inverse_kinematics(const Eigen::Vector3d &xyz,const Q &seed,bool local,double seconds,
                                   const Eigen::Quaterniond &orientation,bool position_only) {
  const auto *group=model_->getJointModelGroup("panda_arm");
  if(!group || group->getVariableCount()!=7)throw std::runtime_error("缺少七轴 IK 关节组");
  const Eigen::Matrix3d rotation=orientation.normalized().toRotationMatrix();
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
  std::mt19937 rng(3);std::normal_distribution<double> noise(0,.7);
  for(int attempt=0;attempt<(local?1:32);++attempt) {
    Q q=seed;
    if(attempt)for(int j=0;j<7;++j)q[j]=std::clamp(seed[j]+noise(rng),lower_[j]+.001,upper_[j]-.001);
    for(int iteration=0;iteration<300;++iteration) {
      if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("IK 超时");
      for(int j=0;j<7;++j)state_->setVariablePosition(names_[j],q[j]);state_->update();
      const auto tcp=state_->getGlobalLinkTransform("panda_hand_tcp");
      const Eigen::AngleAxisd aa(rotation*tcp.linear().transpose());
      Eigen::Matrix<double,6,1> error;
      error.head<3>()=xyz-tcp.translation();error.tail<3>()=aa.axis()*aa.angle();
      if(error.head<3>().norm()<.001 && (position_only || error.tail<3>().norm()<.015)) {
        if(valid(q))return q;
        break;
      }
      Eigen::MatrixXd jacobian;
      if(!state_->getJacobian(group,model_->getLinkModel("panda_hand_tcp"),Eigen::Vector3d::Zero(),jacobian))
        throw std::runtime_error("计算末端 Jacobian 失败");
      const double weight=position_only?0.:.3;
      error.tail<3>()*=weight;jacobian.bottomRows(3)*=weight;
      Eigen::VectorXd delta=jacobian.transpose()*(jacobian*jacobian.transpose()+
                          .0004*Eigen::Matrix<double,6,6>::Identity()).ldlt().solve(error);
      if(!delta.allFinite())break;
      if(delta.norm()>.15)delta*=.15/delta.norm();
      for(int j=0;j<7;++j)q[j]=std::clamp(q[j]+delta[j],lower_[j]+1e-6,upper_[j]-1e-6);
    }
  }
  throw std::runtime_error("未找到满足末端误差和碰撞约束的 IK");
}
PlanResult PlanningEngine::plan_pose(const Positions &positions,const Eigen::Vector3d &xyz,bool cartesian,
                                    double timeout,bool shortcut,double scale,double acceleration) {
  if(!std::isfinite(timeout)||timeout<=0||!std::isfinite(scale)||scale<=0||scale>1||
     !std::isfinite(acceleration)||acceleration<=0)throw std::runtime_error("规划参数非法");
  set_positions(positions);const Q start=arm(positions);
  if(!xyz.allFinite())throw std::runtime_error("末端坐标非法");
  if(!cartesian) {
    const auto goal=inverse_kinematics(xyz,start,false,std::min(timeout,5.));
    return plan(positions,goal,timeout,shortcut,scale,acceleration);
  }
  const auto begin=state_->getGlobalLinkTransform("panda_hand_tcp").translation().eval();
  const int count=std::max(1,int(std::ceil((xyz-begin).norm()/.003)));
  std::vector<Q> path{start};
  for(int i=1;i<=count;++i) {
    auto q=inverse_kinematics(begin+(xyz-begin)*(double(i)/count),path.back(),true,.5);
    for(int j=0;j<7;++j)if(std::abs(q[j]-path.back()[j])>.25)throw std::runtime_error("直线抓放发生 IK 跳变");
    path.push_back(q);
  }
  PlanResult result;result.planner="cartesian_ik";result.raw_points=path.size();
  if(bspline_enabled_) {
    // Keep the Cartesian line within 2 mm after joint-space smoothing.
    // This is a sampled constraint, not an exact Cartesian spline guarantee.
    CartesianConstraint constraint{begin,xyz,Eigen::Quaterniond(0,1,0,0).toRotationMatrix()};
    return bspline_path(path,result,scale,acceleration,constraint);
  }
  return time_path(path,result,scale,acceleration);
}

}
