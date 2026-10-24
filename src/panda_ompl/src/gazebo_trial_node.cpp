// Observe and execute ONE complete Gazebo task. No simulated success inference.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <iostream>
#include <algorithm>
using Steady=std::chrono::steady_clock;
static std::string csv(const std::string &s){std::string out="\"";for(char c:s){if(c=='\"')out+='\"';out+=(c=='\n'?' ':c);}return out+'\"';}
class Trial : public rclcpp::Node {
  using Trigger=std_srvs::srv::Trigger;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr poses_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr preview_;
  rclcpp::Client<Trigger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream events_;
  std::optional<Steady::time_point> began_;
  Steady::time_point startup_=Steady::now();
  bool submitted_=false,done_=false,success_=false;
  bool have_cube_=false;
  std::optional<std::array<double,7>> previous_;
  double sim_=0,initial_sim_=0,actual_length_=0,planned_length_=0,reference_seconds_=0;
  std::array<double,3> cube_{};
  int segments_=0,arrivals_=0;
  std::string terminal_,prefix_;
 public:
  Trial():Node("panda_gazebo_trial") {
    prefix_=declare_parameter("output_prefix",std::string("/tmp/panda_trial"));
    declare_parameter("algorithm",std::string("rrt_connect"));declare_parameter("seed",1);
    auto parent=std::filesystem::path(prefix_).parent_path();if(!parent.empty())std::filesystem::create_directories(parent);
    events_.open(prefix_+"_events.csv");events_<<"wall_elapsed_s,sim_time_s,status\n";
    status_=create_subscription<std_msgs::msg::String>("/planner_status",rclcpp::QoS(100).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m){
        if(!submitted_||done_)return;
        double elapsed=std::chrono::duration<double>(Steady::now()-*began_).count();
        events_<<std::setprecision(12)<<elapsed<<','<<sim_<<','<<csv(m->data)<<'\n';events_.flush();
        if(m->data.rfind("succeeded:",0)==0)++arrivals_;
        if(m->data.rfind("pick_place_succeeded:",0)==0){success_=true;finish(m->data);}
        else if(m->data.rfind("pick_place_failed:",0)==0||m->data.rfind("pick_place_home_failed:",0)==0)finish(m->data);
      });
    state_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){
        std::array<double,7> q{};
        for(int j=0;j<7;++j){auto it=std::find(m->name.begin(),m->name.end(),"panda_joint"+std::to_string(j+1));auto i=std::distance(m->name.begin(),it);
          if(it==m->name.end()||static_cast<std::size_t>(i)>=m->position.size()||!std::isfinite(m->position[i]))return;q[j]=m->position[i];}
        sim_=m->header.stamp.sec+m->header.stamp.nanosec*1e-9;
        if(submitted_&&!done_&&previous_){double d=0;for(int j=0;j<7;++j)d+=std::pow(q[j]-(*previous_)[j],2);actual_length_+=std::sqrt(d);}
        previous_=q;
      });
    poses_=create_subscription<tf2_msgs::msg::TFMessage>("/world/empty/dynamic_pose/info",rclcpp::SensorDataQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){for(const auto &t:m->transforms)if(t.child_frame_id=="target_cube"){
        cube_={t.transform.translation.x,t.transform.translation.y,t.transform.translation.z};have_cube_=true;}});
    preview_=create_subscription<trajectory_msgs::msg::JointTrajectory>("/ompl_planned_trajectory",rclcpp::QoS(10).reliable().transient_local(),
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr m){
        if(!submitted_||done_||m->points.empty())return;++segments_;
        const auto &last=m->points.back();reference_seconds_+=last.time_from_start.sec+last.time_from_start.nanosec*1e-9;
        for(std::size_t i=1;i<m->points.size();++i){double d=0;for(int j=0;j<7;++j)d+=std::pow(m->points[i].positions[j]-m->points[i-1].positions[j],2);planned_length_+=std::sqrt(d);}
      });
    client_=create_client<Trigger>("/pick_and_place");
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{
      if(done_)return;
      if(!submitted_){
        if(Steady::now()-startup_>std::chrono::seconds(30)){finish("runner_failed: 等待反馈或任务服务超时");return;}
        if(!previous_||!have_cube_||!client_->service_is_ready())return;
        submitted_=true;began_=Steady::now();initial_sim_=sim_;
        client_->async_send_request(std::make_shared<Trigger::Request>(),[this](rclcpp::Client<Trigger>::SharedFuture f){
          auto r=f.get();if(!r->success)finish("request_rejected: "+r->message);});
      }else if(Steady::now()-*began_>std::chrono::seconds(270))finish("runner_failed: 整体墙钟超时");
    });
  }
  void finish(const std::string &terminal) {
    if(done_)return;done_=true;terminal_=terminal;
    const double wall=began_?std::chrono::duration<double>(Steady::now()-*began_).count():0;
    std::ofstream out(prefix_+"_task.csv");
    out<<"algorithm,seed,success,task_wall_s,task_sim_s,planned_segments,arrived_segments,planned_length_rad,actual_sampled_length_rad,reference_s,cube_x,cube_y,cube_z,terminal\n";
    out<<std::setprecision(12)<<get_parameter("algorithm").as_string()<<','<<get_parameter("seed").as_int()<<','<<success_<<','<<wall<<','<<(submitted_?sim_-initial_sim_:0)<<','<<segments_<<','<<arrivals_<<','<<planned_length_<<','<<actual_length_<<','<<reference_seconds_<<','<<cube_[0]<<','<<cube_[1]<<','<<cube_[2]<<','<<csv(terminal)<<'\n';out.close();
    std::cout<<"TRIAL_DONE success="<<success_<<" wall_s="<<wall<<" "<<terminal<<std::endl;
  }
  bool done()const{return done_;}
};
int main(int argc,char **argv){rclcpp::init(argc,argv);auto node=std::make_shared<Trial>();rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);while(rclcpp::ok()&&!node->done())executor.spin_once(std::chrono::milliseconds(10));rclcpp::shutdown();return 0;}
