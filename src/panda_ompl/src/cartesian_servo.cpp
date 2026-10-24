#include "panda_ompl/planning_engine.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace panda_ompl {
PlanningEngine::ServoResult PlanningEngine::servo_step(
    const Positions &positions,const Q &reference,const Q &previous_velocity,
    const Eigen::Vector3d &desired,const Eigen::Vector3d &feedforward,
    const Eigen::Matrix3d &rotation,double dt,double gain,double damping,
    double scale,double acceleration) {
  if(dt<=0||dt>.1||reference.size()!=7||previous_velocity.size()!=7)
    throw std::runtime_error("雅可比控制周期或参考无效");
  set_positions(positions);const auto actual=arm(positions);
  const auto tcp=state_->getGlobalLinkTransform("panda_hand_tcp");
  const Eigen::Vector3d error=desired-tcp.translation();
  const Eigen::AngleAxisd angular(rotation*tcp.linear().transpose());
  if(error.norm()>.025||angular.angle()>.15)
    throw std::runtime_error("笛卡尔跟踪偏差过大，已停止");
  const auto group=model_->getJointModelGroup("panda_arm");
  Eigen::MatrixXd jacobian;
  if(!state_->getJacobian(group,model_->getLinkModel("panda_hand_tcp"),Eigen::Vector3d::Zero(),jacobian))
    throw std::runtime_error("计算 Jacobian 失败");
  // MoveIt Jacobian is expressed in the group root frame. Rotate world errors
  // into the parent frame of the group's root joint (normally base_link).
  const auto parent=group->getJointModels().front()->getParentLinkModel();
  Eigen::Matrix3d world_to_root=Eigen::Matrix3d::Identity();
  if(parent)world_to_root=state_->getGlobalLinkTransform(parent).linear().transpose();
  Eigen::Matrix<double,6,1> command;
  command.head<3>()=world_to_root*(feedforward+gain*error);
  command.tail<3>()=world_to_root*(gain*angular.angle()*angular.axis());
  // Scale angular rows by a characteristic length to balance meters/radians.
  jacobian.bottomRows(3)*=.2;command.tail<3>()*=.2;
  Eigen::Matrix<double,6,6> normal=jacobian*jacobian.transpose();
  normal.diagonal().array()+=damping*damping;
  Eigen::VectorXd velocity=jacobian.transpose()*normal.ldlt().solve(command);
  if(!velocity.allFinite())throw std::runtime_error("雅可比求解非有限值");
  double factor=1;
  for(int j=0;j<7;++j)factor=std::max(factor,std::abs(velocity[j])/(speed_[j]*scale));
  ServoResult result;result.q=reference;result.dq=Q(7);
  result.position_error=error.norm();result.rotation_error=angular.angle();
  for(int j=0;j<7;++j) {
    result.dq[j]=std::clamp(velocity[j]/factor,previous_velocity[j]-acceleration*dt,previous_velocity[j]+acceleration*dt);
    result.q[j]+=result.dq[j]*dt;
    if(std::abs(result.q[j]-actual[j])>.08||result.q[j]<lower_[j]+.001||result.q[j]>upper_[j]-.001)
      throw std::runtime_error("关节参考偏差或限位检查失败");
  }
  // Discrete prediction checks, including configured carried-object geometry.
  if(!valid_edge(actual,result.q,.003)||!valid_edge(reference,result.q,.003))
    throw std::runtime_error("雅可比预测运动碰撞，已停止");
  return result;
}
}
