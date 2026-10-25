// Activate controllers with paused simulation steps before allowing free play.
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>
#include <thread>
using namespace std::chrono_literals;
using Steady=std::chrono::steady_clock;
class Startup : public rclcpp::Node {
  using List=controller_manager_msgs::srv::ListControllers;
  using Switch=controller_manager_msgs::srv::SwitchController;
  using World=ros_gz_interfaces::srv::ControlWorld;
  std::array<std::string,3> controllers_{"joint_state_broadcaster","panda_effort_controller","panda_gripper_controller"};
  rclcpp::Client<List>::SharedPtr list_;
  rclcpp::Client<Switch>::SharedPtr switch_;
  rclcpp::Client<World>::SharedPtr world_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_;
  bool feedback_=false,torque_=false;
  double max_error_=0,max_speed_=0;
  Steady::time_point torque_stamp_{};
 public:
  Startup():Node("panda_startup") {
    declare_parameter("auto_play",true);
    list_=create_client<List>("/controller_manager/list_controllers");
    switch_=create_client<Switch>("/controller_manager/switch_controller");
    world_=create_client<World>("/world/empty/control");
    state_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){
        const std::array<double,7> home{0,-.785398,0,-2.356194,0,1.570796,.785398};
        double error=0,speed=0;
        for(int j=0;j<7;++j) {
          auto it=std::find(m->name.begin(),m->name.end(),"panda_joint"+std::to_string(j+1));
          auto i=std::distance(m->name.begin(),it);
          if(it==m->name.end()||static_cast<std::size_t>(i)>=m->position.size()||static_cast<std::size_t>(i)>=m->velocity.size())return;
          if(!std::isfinite(m->position[i])||!std::isfinite(m->velocity[i]))return;
          error=std::max(error,std::abs(m->position[i]-home[j]));speed=std::max(speed,std::abs(m->velocity[i]));
        }
        feedback_=true;max_error_=error;max_speed_=speed;
      });
    command_=create_subscription<std_msgs::msg::Float64MultiArray>("/panda_effort_controller/commands",10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m){
        if(m->data.size()!=7)return;
        double peak=0;for(double t:m->data){if(!std::isfinite(t))return;peak=std::max(peak,std::abs(t));}
        torque_=peak>.01;torque_stamp_=Steady::now();
      });
  }
  void spin_for(std::chrono::milliseconds duration) {
    auto end=Steady::now()+duration;
    while(rclcpp::ok()&&Steady::now()<end){rclcpp::spin_some(shared_from_this());std::this_thread::sleep_for(1ms);}
  }
  template<class Future> auto wait(Future &future) {
    if(rclcpp::spin_until_future_complete(shared_from_this(),future,5s)!=rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("启动服务响应超时");
    return future.get();
  }
  void world(bool pause,unsigned steps=0) {
    auto request=std::make_shared<World::Request>();request->world_control.pause=pause;request->world_control.multi_step=steps;
    auto future=world_->async_send_request(request);
    if(!wait(future)->success)throw std::runtime_error("Gazebo 世界控制失败");
  }
  bool inactive() {
    List::Response::SharedPtr response;
    // Discovery may announce the service before its reply endpoint is matched.
    // Retry this read-only query while the world remains paused; never retry
    // activation or simulation stepping implicitly.
    for(int attempt=0;attempt<3;++attempt) {
      auto future=list_->async_send_request(std::make_shared<List::Request>());
      if(rclcpp::spin_until_future_complete(shared_from_this(),future,5s)==rclcpp::FutureReturnCode::SUCCESS) {
        response=future.get();break;
      }
      list_->remove_pending_request(future);
      RCLCPP_WARN(get_logger(),"控制器列表查询响应超时，保持暂停并重试（%d/3）",attempt+1);
      spin_for(200ms);
    }
    if(!response)throw std::runtime_error("控制器列表查询连续三次超时");
    std::map<std::string,std::string> states;
    for(const auto &c:response->controller)states[c.name]=c.state;
    for(const auto &name:controllers_)if(states[name]!="inactive")return false;
    return true;
  }
  void run() {
    if(!world_->wait_for_service(60s)||!list_->wait_for_service(60s)||!switch_->wait_for_service(60s))
      throw std::runtime_error("等待仿真或 controller_manager 超时");
    world(true);
    const auto deadline=Steady::now()+60s;
    while(!inactive()){if(Steady::now()>deadline)throw std::runtime_error("控制器未全部配置为 inactive");spin_for(100ms);}
    while(count_publishers("/panda_effort_controller/commands")==0&&Steady::now()<deadline)spin_for(50ms);
    if(count_publishers("/panda_effort_controller/commands")!=1)throw std::runtime_error("只能有一个力矩控制节点");
    auto request=std::make_shared<Switch::Request>();
    request->activate_controllers.assign(controllers_.begin(),controllers_.end());
    request->strictness=Switch::Request::STRICT;request->timeout.sec=30;
    auto future=switch_->async_send_request(request);
    for(int k=0;k<100&&future.wait_for(0s)!=std::future_status::ready;++k){world(true,1);spin_for(20ms);}
    if(future.wait_for(0s)!=std::future_status::ready||!future.get()->ok)throw std::runtime_error("控制器激活失败");
    torque_=false;bool ready=false;
    for(int k=0;k<100;++k) {
      world(true,1);spin_for(40ms);
      if(feedback_&&max_error_>.05)throw std::runtime_error("启动阶段偏离初始姿态");
      if(feedback_&&torque_&&max_speed_<.25&&Steady::now()-torque_stamp_<150ms){ready=true;break;}
    }
    if(!ready)throw std::runtime_error("未收到稳定关节反馈和保持力矩");
    world(!get_parameter("auto_play").as_bool());
    RCLCPP_INFO(get_logger(),"启动完成：三个控制器已激活，反馈与保持力矩就绪");
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);auto node=std::make_shared<Startup>();int result=0;
  try{node->run();rclcpp::spin(node);}catch(const std::exception &e){RCLCPP_ERROR(node->get_logger(),"%s",e.what());try{node->world(true);}catch(...){}result=1;}
  rclcpp::shutdown();return result;
}
