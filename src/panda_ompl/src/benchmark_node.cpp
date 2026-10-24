// One offline query per process: no Gazebo and no robot motion.
#include "panda_ompl/planning_engine.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ompl/util/RandomNumbers.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
static std::string read_file(const std::string &path) {
  std::ifstream in(path);if(!in)throw std::runtime_error("无法读取 "+path);
  return std::string(std::istreambuf_iterator<char>(in),{});
}
static std::string csv(const std::string &value) {
  std::string out="\"";for(char c:value){if(c=='\"')out+='\"';out+=c=='\n'?' ':c;}return out+'\"';
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("panda_benchmark");
  node->declare_parameter("urdf_file",std::string(""));node->declare_parameter("world_file",std::string(""));
  node->declare_parameter("planner_algorithm",std::string("rrt_connect"));
  node->declare_parameter("seed",3);node->declare_parameter("timeout",5.);
  node->declare_parameter("shortcut",true);node->declare_parameter("bspline",true);
  node->declare_parameter("output",std::string("results.csv"));
  node->declare_parameter("start",std::vector<double>{0,-.785398,0,-2.356194,0,1.570796,.785398});
  node->declare_parameter("goal",std::vector<double>{.3,-.785398,0,-2.356194,0,1.570796,.785398});
  const auto algorithm=node->get_parameter("planner_algorithm").as_string();
  const auto seed=node->get_parameter("seed").as_int();
  const auto timeout=node->get_parameter("timeout").as_double();
  const bool shortcut=node->get_parameter("shortcut").as_bool(),smooth=node->get_parameter("bspline").as_bool();
  const auto start=node->get_parameter("start").as_double_array(),goal=node->get_parameter("goal").as_double_array();
  const auto began=std::chrono::steady_clock::now();panda_ompl::PlanResult result;
  std::string error;bool success=false;
  try {
    if(seed<0||start.size()!=7||goal.size()!=7)throw std::runtime_error("无效随机种子或七轴起终点");
    ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(seed));
    panda_ompl::PlanningEngine engine(read_file(node->get_parameter("urdf_file").as_string()),node->get_parameter("world_file").as_string());
    engine.set_algorithm(algorithm);engine.enable_bspline(smooth);
    panda_ompl::Positions positions;
    // Panda URDF includes two actuated fingers; keep both open in benchmarks.
    positions["panda_finger_joint1"]=positions["panda_finger_joint2"]=.04;
    for(int j=0;j<7;++j)positions["panda_joint"+std::to_string(j+1)]=start[j];
    result=engine.plan(positions,goal,timeout,shortcut,.3,.9);success=true;
  }catch(const std::exception &e){error=e.what();}
  const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
  const auto path=node->get_parameter("output").as_string();
  const bool header=!std::filesystem::exists(path)||std::filesystem::file_size(path)==0;
  std::ofstream out(path,std::ios::app);
  if(!out){std::cerr<<"无法写入 CSV: "<<path<<'\n';rclcpp::shutdown();return 1;}
  if(header)out<<"planner,seed,budget_s,shortcut,bspline,success,search_s,first_solution_s,total_wall_s,raw_points,raw_length_rad,shortcut_length_rad,trajectory_length_rad,duration_s,checked_samples,validity_queries,trajectory_type,error\n";
  double trajectory_length=0;
  for(std::size_t i=1;i<result.trajectory.points.size();++i) {
    double squared=0;for(int j=0;j<7;++j)squared+=std::pow(result.trajectory.points[i].positions[j]-result.trajectory.points[i-1].positions[j],2);
    trajectory_length+=std::sqrt(squared);
  }
  out<<std::setprecision(9)<<algorithm<<','<<seed<<','<<timeout<<','<<shortcut<<','<<smooth<<','<<success<<',';
  if(success)out<<result.search_seconds<<','<<result.first_solution_seconds;else out<<',';
  out<<','<<wall<<',';
  if(success)out<<result.raw_points<<','<<result.raw_length<<','<<result.shortcut_length<<','<<trajectory_length<<','<<result.duration<<','<<result.checked_samples<<','<<result.validity_queries<<','<<result.trajectory_type;
  else out<<",,,,,,,";
  out<<','<<csv(error)<<'\n';out.close();
  std::cout<<algorithm<<" seed="<<seed<<" success="<<success<<" total_wall_s="<<wall<<' '<<error<<'\n';
  rclcpp::shutdown();return 0; // Planning failures are recorded, not hidden by early batch exit.
}
