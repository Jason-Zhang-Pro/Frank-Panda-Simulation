#include "panda_ompl/control/timed_trajectory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <sstream>

namespace panda_ompl::control {
using Steady=std::chrono::steady_clock;
class ControlNode : public rclcpp::Node {
 public:
  ControlNode():Node("panda_pd_gravity_control") {
    kp_=gains("kp",{64.0, 80.0, 64.0, 80.0, 24.0, 16.0, 8.0});
    kd_=gains("kd",{2.2,11,2.75,13.2,1.65,3.3,.55});
    limits_=gains("torque_limits",{40,70,35,35,10,10,8});
    gravity_scale_=declare_parameter("gravity_scale",1.);
    gripper_close_kp_=declare_parameter("gripper_close_kp",400.);
    gripper_close_force_limit_=declare_parameter("gripper_close_force_limit",10.);
    if(!std::isfinite(gripper_close_force_limit_)||gripper_close_force_limit_<=0||gripper_close_force_limit_>10.)
      throw std::runtime_error("gripper_close_force_limit 必须在 (0,10] N 内，与模型上限一致");
    gripper_hold_force_=declare_parameter("gripper_hold_force",10.);
    if(!std::isfinite(gripper_hold_force_)||gripper_hold_force_<=0||gripper_hold_force_>gripper_close_force_limit_)
      throw std::runtime_error("gripper_hold_force 必须大于零且不超过闭合力上限");
    gripper_center_kp_=declare_parameter("gripper_center_kp",200.);
    gripper_center_kd_=declare_parameter("gripper_center_kd",4.);
    if(!std::isfinite(gripper_center_kp_)||gripper_center_kp_<0||
       !std::isfinite(gripper_center_kd_)||gripper_center_kd_<0)
      throw std::runtime_error("双指中心控制增益必须为有限非负数");
    if(!std::isfinite(gripper_close_kp_)||gripper_close_kp_<=0)
      throw std::runtime_error("gripper_close_kp 必须为有限正数");
    if(gravity_scale_!=0&&gravity_scale_!=1)throw std::runtime_error("gravity_scale 只支持 0 或 1");
    torque_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/panda_effort_controller/commands",1);
    finger_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/panda_gripper_controller/commands",1);
    gripper_status_pub_=create_publisher<std_msgs::msg::String>("/gripper_status",rclcpp::QoS(1).transient_local());
    contact_sub_=create_subscription<ros_gz_interfaces::msg::Contacts>("/target_contacts",rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg){contacts(*msg);});
    status_pub_=create_publisher<std_msgs::msg::String>("/trajectory_status",10);
    description_sub_=create_subscription<std_msgs::msg::String>("/robot_description",rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr m){description(m->data);});
    state_sub_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){feedback(*m);});
    trajectory_sub_=create_subscription<trajectory_msgs::msg::JointTrajectory>("/planned_trajectory",10,
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr m){trajectory(*m);});
    servo_sub_=create_subscription<sensor_msgs::msg::JointState>("/cartesian_joint_reference",rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){servo_reference(*m);});
    goal_sub_=create_subscription<std_msgs::msg::Float64>("/gripper_goal",10,
      [this](std_msgs::msg::Float64::ConstSharedPtr m){
        if(std::isfinite(m->data)&&m->data>=0&&m->data<=.04) {
          if(m->data>0 || gripper_goal_>0)reset_grip();
          gripper_goal_=m->data;
        }
        else RCLCPP_WARN(get_logger(),"夹爪半开口必须在 [0,0.04] m");});
    clock_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr m){sim_time_=m->clock.sec+m->clock.nanosec*1e-9;});
    watchdog_=create_wall_timer(std::chrono::milliseconds(50),[this]{watchdog();});
    RCLCPP_INFO(get_logger(),"C++ PD＋重力补偿：等待模型和关节状态；只能运行一个力矩发布节点");
  }
 private:
  void grip_status(const std::string &mode) {
    if(mode==grip_mode_)return;
    grip_mode_=mode;std_msgs::msg::String msg;msg.data=mode;gripper_status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),"夹爪状态: %s",mode.c_str());
  }
  void reset_grip() {
    close_epoch_=feedback_stamp_.value_or(0.);
    grip_hold_=false;contact_since_.reset();left_contact_.reset();right_contact_.reset();
  }
  void contacts(const ros_gz_interfaces::msg::Contacts &msg) {
    if(gripper_goal_!=0||!feedback_stamp_)return;
    const double stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;
    // Require simulation-stamped contacts; do not turn delayed pre-close
    // messages into new contacts by assigning them the reception time.
    if(!std::isfinite(stamp)||stamp<=close_epoch_||std::abs(stamp-*feedback_stamp_)>.25)return;
    for(const auto &c:msg.contacts) {
      const auto pair=c.collision1.name+" "+c.collision2.name;
      if(pair.find("target_cube")==std::string::npos)continue;
      if(pair.find("panda_leftfinger")!=std::string::npos)left_contact_=stamp;
      if(pair.find("panda_rightfinger")!=std::string::npos)right_contact_=stamp;
    }
  }
  Joints gains(const std::string &name,const std::vector<double> &defaults) {
    auto values=declare_parameter(name,defaults);
    if(values.size()!=7)throw std::runtime_error(name+" 必须包含七项");
    Joints result{};for(int j=0;j<7;++j) {
      if(!std::isfinite(values[j])||values[j]<0||(name=="torque_limits"&&values[j]==0))throw std::runtime_error(name+" 参数无效");
      result[j]=values[j];
    }return result;
  }
  template<class Container> void publish(const Container &data,
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub) {
    std_msgs::msg::Float64MultiArray msg;msg.data.assign(data.begin(),data.end());pub->publish(msg);
  }
  void status(const std::string &s){std_msgs::msg::String msg;msg.data=s;status_pub_->publish(msg);}
  void description(const std::string &xml) {
    if(model_||model_failed_)return;
    try {
      auto model=std::make_unique<GravityModel>(xml);
      model->evaluate(Joints{0,-.785398,0,-2.356194,0,1.570796,.785398},{.04,.04});
      model_=std::move(model);RCLCPP_INFO(get_logger(),"机器人重力模型已加载，包含活动夹爪质量");
    }catch(const std::exception &e){model_failed_=true;RCLCPP_ERROR(get_logger(),"模型加载失败：%s",e.what());}
  }
  void trajectory(const trajectory_msgs::msg::JointTrajectory &msg) {
    try {
      if(!model_||!current_)throw std::runtime_error("等待模型和反馈");
      if(!last_feedback_||std::chrono::duration<double>(Steady::now()-*last_feedback_).count()>.2)
        throw std::runtime_error("关节状态过期，请恢复仿真后重发");
      if(trajectory_||servo_q_)throw std::runtime_error("已有轨迹或笛卡尔控制正在执行");
      auto next=std::make_unique<TimedTrajectory>(msg,*current_,*model_);
      RCLCPP_INFO(get_logger(),"接收 %zu 个轨迹点，总时长 %.2f 秒",msg.points.size(),next->duration);
      trajectory_=std::move(next);status("accepted");
    }catch(const std::exception &e){status(std::string("rejected:")+e.what());RCLCPP_WARN(get_logger(),"拒绝轨迹：%s",e.what());}
  }
  void servo_reference(const sensor_msgs::msg::JointState &msg) {
    if(msg.name.empty()) {
      servo_q_.reset();if(current_)hold_=current_;return;
    }
    try {
      if(!model_||!current_||trajectory_)throw std::runtime_error("控制器未就绪或正在播放轨迹");
      if(msg.name.size()!=7||msg.position.size()!=7||msg.velocity.size()!=7)
        throw std::runtime_error("笛卡尔参考需要七个位置与速度");
      const double stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;
      if(!feedback_stamp_||std::abs(stamp-*feedback_stamp_)>.1)
        throw std::runtime_error("笛卡尔参考时间过期");
      Joints q{},v{};
      for(int j=0;j<7;++j) {
        if(msg.name[j]!=names[j])throw std::runtime_error("笛卡尔参考关节顺序错误");
        q[j]=msg.position[j];v[j]=msg.velocity[j];
        if(!std::isfinite(q[j])||!std::isfinite(v[j])||q[j]<model_->lower[j]||q[j]>model_->upper[j]||
           std::abs(v[j])>model_->speed[j]||std::abs(q[j]-(*current_)[j])>.08)
          throw std::runtime_error("笛卡尔参考限位或跟踪偏差过大");
      }
      const bool first=!servo_q_;servo_q_=q;servo_v_=v;servo_stamp_=stamp;
      if(first)status("servo_accepted");
    }catch(const std::exception &e){servo_q_.reset();if(current_)hold_=current_;status(std::string("rejected:")+e.what());}
  }
  void feedback(const sensor_msgs::msg::JointState &msg) {
    if(!model_)return;
    Joints q{},dq{};std::array<double,2> fingers{},finger_speed{};
    auto read=[&](const std::string &name,double &p,double &v) {
      auto it=std::find(msg.name.begin(),msg.name.end(),name);
      auto i=std::distance(msg.name.begin(),it);
      if(it==msg.name.end()||static_cast<std::size_t>(i)>=msg.position.size()||static_cast<std::size_t>(i)>=msg.velocity.size())
        throw std::runtime_error("缺少关节反馈");
      p=msg.position[i];v=msg.velocity[i];
      if(!std::isfinite(p)||!std::isfinite(v))throw std::runtime_error("反馈含非有限值");
    };
    try {
      for(int j=0;j<7;++j)read(names[j],q[j],dq[j]);
      read("panda_finger_joint1",fingers[0],finger_speed[0]);
      read("panda_finger_joint2",fingers[1],finger_speed[1]);
    }catch(const std::exception &){reset_grip();last_finger_force_={};publish(Joints{},torque_pub_);publish(std::array<double,2>{},finger_pub_);return;}
    auto now=Steady::now();last_feedback_=now;warned_=false;
    double stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;feedback_stamp_=stamp;
    double dt=gripper_stamp_?std::clamp(stamp-*gripper_stamp_,0.,.02):0.;
    if(gripper_stamp_&&stamp<*gripper_stamp_) {
      reset_grip();last_finger_force_={};gripper_ref_=(fingers[0]+fingers[1])/2;
    }
    gripper_stamp_=stamp;gripper_ref_+=std::clamp(gripper_goal_-gripper_ref_,-.02*dt,.02*dt);
    std::array<double,2> force{};
    const bool closing=gripper_goal_==0;
    const bool both=left_contact_&&right_contact_&&stamp-*left_contact_>=-.05&&stamp-*right_contact_>=-.05&&
      stamp-*left_contact_<.15&&stamp-*right_contact_<.15;
    if(closing&&!grip_hold_) {
      if(both) {
        if(!contact_since_)contact_since_=stamp;
        if(stamp-*contact_since_>=.08) {
          grip_hold_=true;
          RCLCPP_INFO(get_logger(),"双指接触确认：切换恒力保持，每指目标 %.2f N",gripper_hold_force_);
        }
      } else contact_since_.reset();
    }
    if(!closing)grip_status("position: 张开或位置控制");
    else if(grip_hold_)grip_status("force_hold: 双指接触已确认，夹持力与双指中心保持");
    else grip_status("closing: 缓慢闭合，等待双指接触");
    const double finger_limit=closing?gripper_close_force_limit_:5.;
    // qL and qR both increase outwards: physical aperture midpoint is
    // (qL-qR)/2. Regulate only this finger coordinate, not arm joint damping.
    const double center=.5*(fingers[0]-fingers[1]);
    const double center_speed=.5*(finger_speed[0]-finger_speed[1]);
    const double correction=std::clamp(-gripper_center_kp_*center-gripper_center_kd_*center_speed,
                                      -.4*gripper_hold_force_,.4*gripper_hold_force_);
    // At the effort ceiling, reserve differential authority by reducing
    // common squeeze instead of clipping away the centering action.
    // Both final targets remain inward, between 0.2*F and F.
    const double squeeze=gripper_hold_force_-std::abs(correction);
    for(int j=0;j<2;++j) {
      if(closing&&grip_hold_) {
        // Positive joint effort opens both prismatic joints; negative closes.
        // Left finger farther out -> more inward left effort, less right.
        const double target=-squeeze+(j==0?correction:-correction);
        force[j]=last_finger_force_[j]+std::clamp(target-last_finger_force_[j],-25.*dt,25.*dt);
      } else force[j]=(closing?gripper_close_kp_:100.)*(gripper_ref_-fingers[j])-finger_speed[j];
      force[j]=std::clamp(force[j],-finger_limit,finger_limit);
    }
    if(closing&&grip_hold_)RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
      "夹爪中心偏差=%.2f mm | 差动力=%.2f N | 双指指令=[%.2f, %.2f] N",
      center*1000,correction,force[0],force[1]);
    last_finger_force_=force;
    publish(force,finger_pub_);
    if(last_stamp_&&stamp>=*last_stamp_&&stamp-*last_stamp_<.01)return;
    if(last_stamp_&&stamp<*last_stamp_) {
      trajectory_.reset();servo_q_.reset();hold_=q;status("rejected:仿真时间回退，轨迹已取消");
    }
    last_stamp_=stamp;current_=q;if(!hold_)hold_=q;
    Joints reference=*hold_,velocity{};
    if(servo_q_) {
      if(stamp-servo_stamp_>.1||stamp<servo_stamp_-.1) {
        servo_q_.reset();hold_=q;reference=q;status("rejected:笛卡尔参考超时，保持当前位置");
      } else {reference=*servo_q_;velocity=servo_v_;}
    }
    if(trajectory_&&trajectory_->sample(stamp,reference,velocity)) {
      hold_=reference;trajectory_.reset();status("reference_complete");
    }
    try {
      auto gravity=model_->evaluate(q,fingers);Joints torque{};bool saturated=false;double max_error=0;
      for(int j=0;j<7;++j) {
        double error=reference[j]-q[j];max_error=std::max(max_error,std::abs(error));
        double raw=kp_[j]*error+kd_[j]*(velocity[j]-dq[j])+gravity_scale_*gravity[j];
        if(!std::isfinite(raw))throw std::runtime_error("控制计算非有限值");
        saturated|=std::abs(raw)>limits_[j];torque[j]=std::clamp(raw,-limits_[j],limits_[j]);
      }
      publish(torque,torque_pub_);
      if(!last_log_||std::chrono::duration<double>(now-*last_log_).count()>=1) {
        RCLCPP_INFO(get_logger(),"max_error=%.4f rad | saturated=%s | tracking=%s",max_error,saturated?"true":"false",trajectory_?"true":"false");last_log_=now;
      }
    }catch(const std::exception &e){publish(Joints{},torque_pub_);RCLCPP_ERROR(get_logger(),"控制失败：%s",e.what());}
  }
  void watchdog() {
    if(!last_feedback_||std::chrono::duration<double>(Steady::now()-*last_feedback_).count()<=.2)return;
    // While paused, preserve the last torque for the next simulation step.
    if(sim_time_&&feedback_stamp_&&*sim_time_<=*feedback_stamp_+.02)return;
    reset_grip();last_finger_force_={};
    publish(Joints{},torque_pub_);publish(std::array<double,2>{},finger_pub_);
    if(!warned_){RCLCPP_WARN(get_logger(),"反馈超时，已发零力矩；零力矩不支撑重力");warned_=true;}
  }
  Joints kp_{},kd_{},limits_{};double gripper_close_kp_=400.,gripper_close_force_limit_=10.,gripper_hold_force_=10.;
  double gripper_center_kp_=200.,gripper_center_kd_=4.;
  double gravity_scale_=1,gripper_goal_=.04,gripper_ref_=.04;
  bool grip_hold_=false;double close_epoch_=0;
  std::string grip_mode_;
  std::array<double,2> last_finger_force_{};
  std::optional<double> left_contact_,right_contact_,contact_since_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_status_pub_;
  bool model_failed_=false,warned_=false;
  std::unique_ptr<GravityModel> model_;std::unique_ptr<TimedTrajectory> trajectory_;
  std::optional<Joints> current_,hold_,servo_q_;
  Joints servo_v_{};double servo_stamp_=0;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_sub_;
  std::optional<double> last_stamp_,feedback_stamp_,sim_time_,gripper_stamp_;
  std::optional<Steady::time_point> last_feedback_,last_log_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_,finger_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr goal_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<panda_ompl::control::ControlNode>());}
  catch(const std::exception &e){RCLCPP_FATAL(rclcpp::get_logger("control_node"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
