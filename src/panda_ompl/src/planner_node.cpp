#include "panda_ompl/planning_engine.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ompl/util/RandomNumbers.h>

using Clock=std::chrono::steady_clock;
using Trigger=std_srvs::srv::Trigger;
using namespace std::chrono_literals;
class PandaPlannerNode : public rclcpp::Node {
public:
  PandaPlannerNode():Node("panda_ompl_planner") {
    declare_parameter("world_urdf",std::string(""));
    declare_parameter("goal_joint_positions",std::vector<double>{.3,-.785398,0.,-2.356194,0.,1.570796,.785398});
    declare_parameter("planner_algorithm",std::string("rrt_connect"));
    declare_parameter("trajectory_smoothing",std::string("bspline"));
    declare_parameter("planning_timeout",15.);declare_parameter("shortcut_enabled",true);
    declare_parameter("velocity_scale",.3);declare_parameter("acceleration_limit",.9);
    declare_parameter("grasp_wait",1.4);declare_parameter("release_wait",.5);
    declare_parameter("cartesian_speed",.01);
    declare_parameter("cartesian_gain",2.);
    declare_parameter("cartesian_damping",.03);
    declare_parameter("seed",3);
    declare_parameter("home_joint_positions",std::vector<double>{0.,-.785398,0.,-2.356194,0.,1.570796,.785398});
    const auto seed=get_parameter("seed").as_int();
    if(seed<0) throw std::runtime_error("seed 必须非负");
    ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(seed));
    auto latched=rclcpp::QoS(1).reliable().transient_local();
    status_pub_=create_publisher<std_msgs::msg::String>("/planner_status",latched);
    preview_pub_=create_publisher<trajectory_msgs::msg::JointTrajectory>("/ompl_planned_trajectory",latched);
    servo_pub_=create_publisher<sensor_msgs::msg::JointState>("/cartesian_joint_reference",rclcpp::QoS(1).reliable());
    command_pub_=create_publisher<trajectory_msgs::msg::JointTrajectory>("/planned_trajectory",10);
    description_sub_=create_subscription<std_msgs::msg::String>("/robot_description",latched,
      [this](std_msgs::msg::String::SharedPtr m){
        if(engine_)return;
        try {
          if(get_parameter("world_urdf").as_string().empty())throw std::runtime_error("请通过 start_cpp.sh 启动，或指定 world_urdf 参数");
          engine_=std::make_shared<panda_ompl::PlanningEngine>(m->data,get_parameter("world_urdf").as_string());
          status("idle: C++ OMPL 规划器就绪，已加载 "+std::to_string(engine_->object_count())+" 个世界碰撞体；等待实际状态及目标");
        } catch(const std::exception &e){engine_.reset();status(std::string("model_failed: ")+e.what());}
      });
    state_sub_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr m){
        if(!engine_)return;
        panda_ompl::Positions p;
        for(std::size_t i=0;i<m->name.size()&&i<m->position.size();++i)
          if(std::isfinite(m->position[i]))p[m->name[i]]=m->position[i];
        for(const auto &name:engine_->variables())if(!p.count(name))return;
        measured_velocity_.clear();
        for(std::size_t i=0;i<m->name.size()&&i<m->velocity.size();++i)
          if(std::isfinite(m->velocity[i]))measured_velocity_[m->name[i]]=m->velocity[i];
        positions_=std::move(p);feedback_=Clock::now();
        sim_seconds_=m->header.stamp.sec+m->header.stamp.nanosec*1e-9;
      });
    pose_sub_=create_subscription<tf2_msgs::msg::TFMessage>("/world/empty/dynamic_pose/info",rclcpp::SensorDataQoS(),
      [this](tf2_msgs::msg::TFMessage::SharedPtr m){
        for(const auto &t:m->transforms)if(t.child_frame_id=="target_cube") {
          const auto &v=t.transform.translation;const auto &r=t.transform.rotation;
          Eigen::Vector3d xyz(v.x,v.y,v.z);Eigen::Quaterniond q(r.w,r.x,r.y,r.z);
          if(!xyz.allFinite()||!q.coeffs().allFinite()||q.norm()<1e-8)return;
          Eigen::Isometry3d p=Eigen::Isometry3d::Identity();p.translation()=xyz;p.linear()=q.normalized().toRotationMatrix();
          target_=p;target_time_=Clock::now();
        }
      });
    control_sub_=create_subscription<std_msgs::msg::String>("/trajectory_status",10,
      [this](std_msgs::msg::String::SharedPtr m){
        if(servo_active_) {
          if(m->data=="servo_accepted")servo_accepted_=true;
          if(m->data.rfind("rejected:",0)==0){stop_servo();status("failed: "+m->data);}
          return;
        }
        if(!executing_)return;
        if(m->data=="accepted"){accepted_=true;status("executing: 控制节点已接受 C++ 轨迹");}
        else if(m->data=="reference_complete")reference_done_=true;
        else if(m->data.rfind("rejected:",0)==0){executing_=false;status("failed: "+m->data);}
      });
    plan_service_=create_service<Trigger>("/ompl_plan",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){request(false,*r);});
    move_service_=create_service<Trigger>("/ompl_move",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){request(true,*r);});
    execute_service_=create_service<Trigger>("/ompl_execute",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){
      try{execute();r->success=true;r->message="已发送轨迹，查看 /planner_status 确认实际完成";}
      catch(const std::exception&e){r->success=false;r->message=e.what();}
    });
    gripper_pub_=create_publisher<std_msgs::msg::Float64>("/gripper_goal",10);
    contacts_sub_=create_subscription<ros_gz_interfaces::msg::Contacts>("/target_contacts",rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::SharedPtr msg){
        for(const auto &c:msg->contacts) {
          const auto names=c.collision1.name+" "+c.collision2.name;
          if(names.find("target_cube")==std::string::npos)continue;
          if(names.find("panda_leftfinger")!=std::string::npos)left_contact_=sim_seconds_;
          if(names.find("panda_rightfinger")!=std::string::npos)right_contact_=sim_seconds_;
        }
      });
    pick_service_=create_service<Trigger>("/pick_and_place",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){start_pick(*r);});
    open_service_=create_service<Trigger>("/open_gripper",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){manual_gripper(.04,*r);});
    close_service_=create_service<Trigger>("/close_gripper",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){manual_gripper(0.,*r);});
    timer_=create_wall_timer(10ms,[this]{poll();});
    status("idle: 等待机器人模型；/ompl_plan 仅规划，/ompl_move 规划并执行");
  }
private:
  void status(const std::string &text) {
    if(task_active_) {
      if(text.rfind("succeeded:",0)==0)arm_done_=true;
      if(text.rfind("failed:",0)==0)task_failure_=text;
    }
    std_msgs::msg::String m;m.data=text;status_pub_->publish(m);RCLCPP_INFO(get_logger(),"%s",text.c_str());
  }
  void fresh() const {
    if(!engine_||positions_.empty())throw std::runtime_error("等待机器人模型和九个实际关节状态");
    if(Clock::now()-feedback_>1s)throw std::runtime_error("关节反馈过期，请检查仿真播放状态");
    if(!target_||Clock::now()-target_time_>2s)throw std::runtime_error("等待实际方块位姿，不能用初始场景代替");
  }
  bool target_matches(const Eigen::Isometry3d &old) const {
    return target_ && (target_->translation()-old.translation()).norm()<.005 &&
      Eigen::AngleAxisd(old.linear().transpose()*target_->linear()).angle()<.05;
  }
  void request(bool execute_after,Trigger::Response &response,bool internal=false,
               std::optional<Eigen::Vector3d> xyz={},bool cartesian=false,
               std::optional<panda_ompl::Q> joint_goal={}) {
    try {
      if(!internal&&(task_active_||grasp_locked_))throw std::runtime_error("抓放任务或携物状态下不允许独立运动");
      if(future_.valid()||executing_||servo_active_)throw std::runtime_error("已有规划或执行任务");
      fresh();
      // The original Python planner must be disabled so it cannot overwrite
      // references on the same teaching controller interface.
      const auto nodes=get_node_names();
      if(std::find(nodes.begin(),nodes.end(),"/panda_planner")!=nodes.end())
        throw std::runtime_error("请用 ./start.sh planner_backend:=ompl_cpp 重启，避免同时运行两个规划器");
      const auto goal=joint_goal?*joint_goal:get_parameter("goal_joint_positions").as_double_array();
      if(goal.size()!=7)throw std::runtime_error("goal_joint_positions 必须为七个弧度值");
      const double timeout=get_parameter("planning_timeout").as_double();
      const double scale=get_parameter("velocity_scale").as_double();
      const double acceleration=get_parameter("acceleration_limit").as_double();
      const bool shortcut=get_parameter("shortcut_enabled").as_bool();
      const auto smoothing=get_parameter("trajectory_smoothing").as_string();
      if(smoothing!="bspline"&&smoothing!="stop")throw std::runtime_error("trajectory_smoothing 必须为 bspline 或 stop");
      const bool bspline=smoothing=="bspline";
      const auto algorithm=get_parameter("planner_algorithm").as_string();
      job_aborted_=false;cached_.reset();snapshot_=positions_;planned_target_=*target_;auto_execute_=execute_after;
      const auto positions=snapshot_;const auto cube=planned_target_;const auto engine=engine_;
      const auto payload=payload_;
      const bool finger_contact=internal && stage_!="homing";
      const std::string support=stage_=="lifting"?"target_pedestal":(stage_=="lowering"?"place_pedestal":"");
      future_=std::async(std::launch::async,[engine,positions,cube,goal,timeout,scale,acceleration,shortcut,xyz,cartesian,payload,finger_contact,support,bspline,algorithm]{
        engine->set_algorithm(algorithm);
        engine->enable_bspline(bspline);
        engine->configure_task(positions,cube,payload,finger_contact,support);
        if(xyz)return engine->plan_pose(positions,*xyz,cartesian,timeout,shortcut,scale,acceleration);
        return engine->plan(positions,goal,timeout,shortcut,scale,acceleration);
      });
      status("planning: C++ OMPL "+algorithm+"；七关节、静态环境及自碰撞检查");
      response.success=true;response.message="任务已接受；查看 /planner_status，不代表执行完成";
    }catch(const std::exception&e){response.success=false;response.message=e.what();}
  }
  void execute() {
    if(grasp_locked_ && !task_active_)throw std::runtime_error("携物失败状态禁止执行旧轨迹，请重置场景");
    if(task_active_ && !auto_execute_)throw std::runtime_error("抓放任务正在执行");
    if(future_.valid()||executing_||servo_active_)throw std::runtime_error("已有规划或执行任务");
    if(!cached_)throw std::runtime_error("没有可执行轨迹，请先调用 /ompl_plan");
    fresh();
    if(!target_matches(planned_target_))throw std::runtime_error("方块已移动，请重新规划");
    for(const auto &item:snapshot_) {
      const double tolerance=item.first.find("finger")!=std::string::npos?.003:.03;
      if(std::abs(positions_.at(item.first)-item.second)>tolerance)
        throw std::runtime_error("机器人已偏离规划起点，请重新规划");
    }
    std::size_t controllers=0;
    for(const auto &endpoint:get_subscriptions_info_by_topic("/planned_trajectory"))
      if(endpoint.node_name()=="panda_pd_gravity_control")++controllers;
    if(controllers!=1)
      throw std::runtime_error("需要且只允许一个 panda_pd_gravity_control 控制订阅者");
    final_=cached_->trajectory.points.back().positions;
    duration_=cached_->duration;started_sim_=sim_seconds_;sent_=Clock::now();
    accepted_=false;reference_done_=false;executing_=true;
    command_pub_->publish(cached_->trajectory);cached_.reset();
    status("sent: 已发送轨迹，等待控制器接受");
  }
  void poll() {
    poll_servo();
    poll_task();
    if(future_.valid() && future_.wait_for(0ms)==std::future_status::ready) {
      try {
        cached_=future_.get();
        if(job_aborted_){cached_.reset();job_aborted_=false;return;}
        const auto count=cached_->trajectory.points.size();
        RCLCPP_INFO(get_logger(),"OMPL: %.3fs，原始路径 %zu 点，输出轨迹 %zu 点，复检 %zu 样本，轨迹 %.2fs，模式 %s",
          cached_->search_seconds,cached_->raw_points,count,cached_->checked_samples,cached_->duration,cached_->trajectory_type.c_str());
        RCLCPP_INFO(get_logger(),"planner=%s | first_solution=%.3fs | raw_length=%.4f | shortcut_length=%.4f | validity_queries=%zu",
          cached_->planner.c_str(),cached_->first_solution_seconds,cached_->raw_length,cached_->shortcut_length,cached_->validity_queries);
        if(!cached_->smoothing_note.empty())RCLCPP_INFO(get_logger(),"%s",cached_->smoothing_note.c_str());
        preview_pub_->publish(cached_->trajectory);
        status("planned: 路径、实际插值碰撞及限速检查通过；/ompl_execute 执行");
        if(auto_execute_)execute();
      }catch(const std::exception&e){cached_.reset();status(std::string("failed: ")+e.what());}
    }
    if(!executing_)return;
    if(!accepted_&&Clock::now()-sent_>3s){executing_=false;status("failed: 控制器没有确认轨迹");return;}
    if(accepted_&&reference_done_&&Clock::now()-feedback_<1s) {
      const auto q=engine_->arm(positions_);double error=0;
      for(std::size_t j=0;j<7;++j)error=std::max(error,std::abs(q[j]-final_[j]));
      if(error<.03){executing_=false;status("succeeded: 实际关节到位，最大误差 "+std::to_string(error)+" rad");return;}
    }
    if(sim_seconds_<started_sim_-.1 || sim_seconds_-started_sim_>duration_+15) {
      executing_=false;status("failed: 执行超时或仿真时间重置，检查控制器状态");
    }
  }
  void gripper(double half_width) {
    std_msgs::msg::Float64 m;m.data=half_width;gripper_pub_->publish(m);
  }
  bool fingers_open() const {
    return positions_.count("panda_finger_joint1") && positions_.count("panda_finger_joint2") &&
      positions_.at("panda_finger_joint1")>=.038 && positions_.at("panda_finger_joint2")>=.038;
  }
  bool both_contacts() const {
    return sim_seconds_-left_contact_<.25 && sim_seconds_-right_contact_<.25;
  }
  void manual_gripper(double width,Trigger::Response &r) {
    try {
      if(task_active_||grasp_locked_||future_.valid()||executing_||servo_active_)throw std::runtime_error("任务或携物状态禁止手动夹爪");
      fresh();if(gripper_pub_->get_subscription_count()!=1)throw std::runtime_error("夹爪控制未连接");
      gripper(width);r.success=true;r.message="已发送夹爪命令，不代表抓取成功";
    }catch(const std::exception&e){r.success=false;r.message=e.what();}
  }
  void start_pick(Trigger::Response &r) {
    try {
      if(task_active_||grasp_locked_||future_.valid()||executing_||servo_active_)throw std::runtime_error("已有任务或仍处于携物状态");
      fresh();if(gripper_pub_->get_subscription_count()!=1)throw std::runtime_error("夹爪控制未连接");
      grasp_wait_=get_parameter("grasp_wait").as_double();
      release_wait_=get_parameter("release_wait").as_double();
      if(!std::isfinite(grasp_wait_)||grasp_wait_<.5||grasp_wait_>4||
         !std::isfinite(release_wait_)||release_wait_<.2||release_wait_>4)
        throw std::runtime_error("grasp_wait 需在 [0.5,4] 秒，release_wait 需在 [0.2,4] 秒");
      pick_=target_->translation();place_=engine_->place_position();
      if((pick_.head<2>()-place_.head<2>()).norm()<.06)throw std::runtime_error("目标已在放置台附近，请重置场景");
      cached_.reset();task_active_=true;task_failure_.clear();stage_="opening";
      task_started_=Clock::now();stage_started_=sim_seconds_;arm_done_=false;
      left_contact_=right_contact_=-1000;gripper(.04);
      status("pick_opening: C++ 抓放任务开始，张开夹爪");
      r.success=true;r.message="C++ 物理抓放任务已接受，查看 /planner_status";
    }catch(const std::exception&e){r.success=false;r.message=e.what();}
  }
  void stop_servo() {
    servo_active_=false;sensor_msgs::msg::JointState stop;servo_pub_->publish(stop);
  }
  void start_servo(const Eigen::Vector3d &goal) {
    fresh();
    if(future_.valid()||executing_||servo_active_)throw std::runtime_error("已有运动任务");
    if(servo_pub_->get_subscription_count()!=1)throw std::runtime_error("需要一个 C++ 笛卡尔控制订阅者");
    servo_speed_=get_parameter("cartesian_speed").as_double();
    servo_gain_=get_parameter("cartesian_gain").as_double();
    servo_damping_=get_parameter("cartesian_damping").as_double();
    servo_scale_=get_parameter("velocity_scale").as_double();
    servo_acceleration_=get_parameter("acceleration_limit").as_double();
    if(!std::isfinite(servo_speed_)||servo_speed_<=0||servo_speed_>.05||
       !std::isfinite(servo_gain_)||servo_gain_<=0||!std::isfinite(servo_damping_)||servo_damping_<=0||
       !std::isfinite(servo_scale_)||servo_scale_<=0||servo_scale_>1||
       !std::isfinite(servo_acceleration_)||servo_acceleration_<=0)
      throw std::runtime_error("笛卡尔控制参数非法");
    engine_->set_positions(positions_);
    const auto pose=engine_->link_pose("panda_hand_tcp");
    servo_begin_=pose.translation();servo_goal_=goal;servo_rotation_=pose.linear();
    servo_q_=engine_->arm(positions_);servo_v_=panda_ompl::Q(7,0.);
    // Quintic time law has maximum normalized speed 1.875.
    servo_duration_=std::max(1.,1.875*(goal-servo_begin_).norm()/servo_speed_);
    servo_started_=servo_last_=sim_seconds_;servo_settled_=-1;
    servo_sent_=Clock::now();servo_accepted_=false;servo_active_=true;
    status("executing: 抓放采用末端 P＋阻尼雅可比速度外环＋关节 PD；预测碰撞检查开启");
  }
  void poll_servo() {
    if(!servo_active_)return;
    try {
      fresh();
      if(!servo_accepted_&&Clock::now()-servo_sent_>3s)throw std::runtime_error("控制器未确认雅可比参考");
      const double dt=sim_seconds_-servo_last_;
      if(dt<0||dt>.1)throw std::runtime_error("雅可比反馈时间回退或间隔超过 0.1 秒");
      if(dt<.009)return;
      servo_last_=sim_seconds_;
      const double elapsed=sim_seconds_-servo_started_;
      if(elapsed>servo_duration_+10)throw std::runtime_error("雅可比抓放动作超时");
      const double u=std::clamp(elapsed/servo_duration_,0.,1.);
      const double blend=10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5);
      const double rate=(30*u*u-60*u*u*u+30*u*u*u*u)/servo_duration_;
      const auto desired=(servo_begin_+blend*(servo_goal_-servo_begin_)).eval();
      const auto feedforward=(rate*(servo_goal_-servo_begin_)).eval();
      const std::string support=stage_=="lifting"?"target_pedestal":(stage_=="lowering"?"place_pedestal":"");
      engine_->configure_task(positions_,*target_,payload_,true,support);
      auto result=engine_->servo_step(positions_,servo_q_,servo_v_,desired,feedforward,
        servo_rotation_,dt,servo_gain_,servo_damping_,servo_scale_,servo_acceleration_);
      double speed=0;for(double v:result.dq)speed=std::max(speed,std::abs(v));
      for(const auto &name:engine_->names()) {
        if(!measured_velocity_.count(name))throw std::runtime_error("缺少实际关节速度");
        speed=std::max(speed,std::abs(measured_velocity_.at(name)));
      }
      if(elapsed>=servo_duration_&&result.position_error<.002&&result.rotation_error<.025&&speed<.02) {
        if(servo_settled_<0)servo_settled_=sim_seconds_;
        if(servo_accepted_&&sim_seconds_-servo_settled_>.25) {
          stop_servo();status("succeeded: 雅可比抓放段末端到位并稳定");return;
        }
      }else servo_settled_=-1;
      servo_q_=result.q;servo_v_=result.dq;
      sensor_msgs::msg::JointState msg;msg.name=engine_->names();msg.position=servo_q_;msg.velocity=servo_v_;
      const auto ns=static_cast<int64_t>(std::llround(sim_seconds_*1e9));
      msg.header.stamp.sec=ns/1000000000LL;msg.header.stamp.nanosec=ns%1000000000LL;
      servo_pub_->publish(msg);
    }catch(const std::exception &e){stop_servo();status(std::string("failed: ")+e.what());}
  }
  void move_stage(const std::string &stage,const Eigen::Vector3d &xyz,bool cartesian=false) {
    stage_=stage;arm_done_=false;Trigger::Response r;
    // Cartesian pick/place uses preplanned IK + constrained B-spline tracking.
    // The experimental online Jacobian servo is no longer started by the task.
    request(true,r,true,xyz,cartesian);
    if(!r.success)throw std::runtime_error(r.message);
  }
  void poll_task() {
    if(!task_active_)return;
    try {
      if(!task_failure_.empty())throw std::runtime_error(task_failure_);
      if(Clock::now()-task_started_>240s)throw std::runtime_error("抓放任务超过 240 秒");
      fresh();
      const double elapsed=sim_seconds_-stage_started_;
      if(stage_=="opening") {
        if(fingers_open())move_stage("approaching",pick_+Eigen::Vector3d(0,0,.13));
        else if(elapsed>5)throw std::runtime_error("夹爪张开失败");
      } else if(stage_=="approaching"&&arm_done_) {
        move_stage("descending",pick_+Eigen::Vector3d(0,0,.005),true);
      } else if(stage_=="descending"&&arm_done_) {
        stage_="closing";stage_started_=sim_seconds_;left_contact_=right_contact_=-1000;
        grasp_locked_=true;gripper(0.);status("pick_closing: 等待真实双指接触");
      } else if(stage_=="closing") {
        const double a=positions_.at("panda_finger_joint1"),b=positions_.at("panda_finger_joint2");
        if(elapsed>grasp_wait_&&both_contacts()&&std::min(a,b)>.015&&std::max(a,b)<.038) {
          payload_=engine_->capture(positions_,*target_);
          move_stage("lifting",pick_+Eigen::Vector3d(0,0,.105),true);
        } else if(elapsed>5)throw std::runtime_error("未确认双指夹住方块");
      } else if(stage_=="lifting"&&arm_done_) {
        if(target_->translation().z()<pick_.z()+.055||!both_contacts())throw std::runtime_error("实际方块未抬起或失去接触");
        status("pick_lifted: 真实抬升通过，C++ OMPL 携物转运");
        move_stage("transferring",place_+Eigen::Vector3d(0,0,.13));
      } else if(stage_=="transferring"&&arm_done_) {
        if(!both_contacts())throw std::runtime_error("转运后失去双指接触");
        move_stage("lowering",place_+Eigen::Vector3d(0,0,.005),true);
      } else if(stage_=="lowering"&&arm_done_) {
        if((target_->translation().head<2>()-place_.head<2>()).norm()>.04)throw std::runtime_error("方块未到放置位置");
        stage_="releasing";stage_started_=sim_seconds_;gripper(.04);
        status("place_releasing: 下降完成，张开夹爪放下方块");
      } else if(stage_=="releasing") {
        if(fingers_open()&&elapsed>release_wait_) {
          if((target_->translation()-place_).norm()>.025)throw std::runtime_error("方块实际放置位置不正确");
          payload_.reset();grasp_locked_=false;
          move_stage("retreating",place_+Eigen::Vector3d(0,0,.15),true);
        } else if(elapsed>5)throw std::runtime_error("夹爪释放失败");
      } else if(stage_=="retreating"&&arm_done_) {
        if((target_->translation()-place_).norm()>.025)throw std::runtime_error("放置后方块未稳定");
        stage_="homing";arm_done_=false;Trigger::Response r;
        status("returning_home: C++ OMPL 规划归位");
        request(true,r,true,{},false,get_parameter("home_joint_positions").as_double_array());
        if(!r.success)throw std::runtime_error(r.message);
      } else if(stage_=="homing"&&arm_done_) {
        if((target_->translation()-place_).norm()>.025)throw std::runtime_error("归位后方块位置偏离");
        task_active_=false;status("pick_place_succeeded: 真实抓取、放置、松开和避障归位全部完成");
      }
    }catch(const std::exception&e) {
      if(servo_active_)stop_servo();
      task_active_=false;auto_execute_=false;job_aborted_=future_.valid();cached_.reset();
      status(std::string(stage_=="homing"?"pick_place_home_failed: ":"pick_place_failed: ")+e.what()+"；保持夹爪指令");
    }
  }
  bool servo_active_=false,servo_accepted_=false;
  double servo_speed_=0,servo_gain_=0,servo_damping_=0,servo_scale_=0,servo_acceleration_=0;
  double servo_started_=0,servo_last_=0,servo_duration_=0,servo_settled_=-1;
  Clock::time_point servo_sent_{};
  Eigen::Vector3d servo_begin_,servo_goal_;
  Eigen::Matrix3d servo_rotation_;
  panda_ompl::Q servo_q_,servo_v_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_pub_;
  bool task_active_=false,grasp_locked_=false,arm_done_=false,job_aborted_=false;
  std::string stage_,task_failure_;
  double grasp_wait_=1.4,release_wait_=.5;
  double stage_started_=0,left_contact_=-1000,right_contact_=-1000;
  Clock::time_point task_started_{};
  Eigen::Vector3d pick_,place_;
  std::optional<panda_ompl::Payload> payload_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_sub_;
  rclcpp::Service<Trigger>::SharedPtr pick_service_,open_service_,close_service_;
  std::shared_ptr<panda_ompl::PlanningEngine> engine_;
  panda_ompl::Positions positions_,snapshot_,measured_velocity_;
  std::future<panda_ompl::PlanResult> future_;
  std::optional<panda_ompl::PlanResult> cached_;
  std::optional<Eigen::Isometry3d> target_;
  Eigen::Isometry3d planned_target_=Eigen::Isometry3d::Identity();
  Clock::time_point feedback_{},target_time_{},sent_{};
  bool auto_execute_=false,executing_=false,accepted_=false,reference_done_=false;
  double sim_seconds_=0,started_sim_=0,duration_=0;
  panda_ompl::Q final_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr preview_pub_,command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_,control_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Service<Trigger>::SharedPtr plan_service_,move_service_,execute_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PandaPlannerNode>());
  rclcpp::shutdown();return 0;
}
