#pragma once
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace panda_ompl {
using Q = std::vector<double>;
using Positions = std::map<std::string, double>;
struct PlanResult {
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::size_t raw_points = 0, checked_samples = 0;
  double search_seconds = 0, duration = 0;
  std::string planner = "rrt_connect";
  double first_solution_seconds = -1;
  double raw_length = 0, shortcut_length = 0;
  std::size_t validity_queries = 0;
  std::string trajectory_type = "stop";
  std::string smoothing_note;
};
struct Payload { Eigen::Isometry3d tcp_to_cube = Eigen::Isometry3d::Identity(); };
class PlanningEngine {
public:
  PlanningEngine(const std::string &xml, const std::string &world, double clearance = .008);
  void set_algorithm(const std::string &algorithm);
  void enable_bspline(bool enabled) { bspline_enabled_ = enabled; }
  void enable_tight_cartesian_timing(bool enabled) { tight_cartesian_timing_ = enabled; }
  const std::vector<std::string> &variables() const { return model_->getVariableNames(); }
  const std::vector<std::string> &names() const { return names_; }
  std::size_t object_count() const { return objects_; }
  Q arm(const Positions &positions) const;
  void set_positions(const Positions &positions);
  bool valid(const Q &q, std::string *reason = nullptr);
  std::vector<Q> shortcut_path(const std::vector<Q> &points, double timeout = 2.);
  bool valid_edge(const Q &a, const Q &b, double resolution = .01);
  Eigen::Isometry3d link_pose(const std::string &name) const;
  void update_target(const Eigen::Isometry3d &pose);
  void configure_task(const Positions &positions, const Eigen::Isometry3d &cube,
                      const std::optional<Payload> &payload, bool finger_contact,
                      const std::string &support);
  struct ServoResult { Q q, dq; double position_error=0, rotation_error=0; };
  ServoResult servo_step(const Positions &positions, const Q &reference, const Q &previous_velocity,
                        const Eigen::Vector3d &desired, const Eigen::Vector3d &feedforward,
                        const Eigen::Matrix3d &rotation, double dt, double gain, double damping,
                        double scale, double acceleration);
  Q solve_ik(const Positions &positions, const Eigen::Vector3d &xyz, bool local,
             const std::vector<double> &xyzw);
  Payload capture(const Positions &positions, const Eigen::Isometry3d &cube);
  Eigen::Vector3d place_position() const { return place_; }
  PlanResult plan_pose(const Positions &positions, const Eigen::Vector3d &xyz,
                       bool cartesian, double timeout, bool shortcut,
                       double velocity_scale, double acceleration);
  PlanResult plan(const Positions &positions, const Q &goal, double timeout,
                  bool shortcut = true, double velocity_scale = .2,
                  double acceleration = .6);
private:
  std::vector<Q> astar_path(const Q &start, const Q &goal, double timeout, PlanResult &result);
  std::string algorithm_ = "rrt_connect";
  bool bspline_enabled_ = false;
  bool tight_cartesian_timing_ = true;
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  moveit::core::RobotStatePtr state_;
  std::vector<std::string> names_;
  Q lower_, upper_, speed_;
  std::size_t objects_ = 0;
  Eigen::Vector3d place_ = Eigen::Vector3d(.32,-.55,.35);
  double clearance_ = .008;
  Eigen::Vector3d cube_size_ = Eigen::Vector3d(.06,.06,.06);
  std::optional<Payload> payload_;
  std::string support_;
  struct Support { Eigen::Vector3d center, size; };
  std::map<std::string,Support> supports_;
  Q inverse_kinematics(const Eigen::Vector3d &xyz, const Q &seed, bool local, double seconds,
                       const Eigen::Quaterniond &orientation = Eigen::Quaterniond(0,1,0,0),
                       bool position_only = false);
  struct CartesianConstraint {
    Eigen::Vector3d begin, end;
    Eigen::Matrix3d orientation;
  };
  PlanResult bspline_path(const std::vector<Q> &points, PlanResult result,
                          double velocity_scale, double acceleration,
                          const std::optional<CartesianConstraint> &constraint = std::nullopt);
  PlanResult time_path(const std::vector<Q> &points, PlanResult result,
                       double velocity_scale, double acceleration);
  void load_world(const std::string &path);
};
}
