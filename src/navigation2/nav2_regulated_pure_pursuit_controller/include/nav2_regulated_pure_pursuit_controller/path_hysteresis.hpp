// Copyright 2026 outdoor_nav contributors
// Licensed under the Apache License, Version 2.0.
#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HYSTERESIS_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HYSTERESIS_HPP_

#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_regulated_pure_pursuit_controller
{
// Selects between an unchanged planner proposal and the still-valid previous route.
// It neither alters a planner's optimization objective nor creates a ROS interface.
class PathHysteresis
{
public:
  struct Decision
  {
    uint64_t current_id{0}, candidate_id{0};
    double current_cost{std::numeric_limits<double>::infinity()};
    double candidate_cost{std::numeric_limits<double>::infinity()};
    bool current_valid{false}, candidate_valid{false}, switched{false};
    std::string reason;
  };
  using SafetyCheck = std::function<bool(double, double, double)>;
  bool enabled{true};
  // Weighted metres: 1.0 is approximately half a metre in free space (w_euc=2).
  double switch_penalty{1.0};
  double min_switch_interval{2.0};
  // Keep the selected route while it is collision-free. A completed local
  // horizon may still be extended so the controller can keep moving.
  bool switch_only_on_obstacle{false};

  void reset();
  void setGoal(const nav_msgs::msg::Path & guide);
  const Decision & decision() const {return decision_;}
  uint64_t currentId() const {return current_id_;}

  nav_msgs::msg::Path select(
    const nav_msgs::msg::Path & candidate,
    const geometry_msgs::msg::PoseStamped & robot,
    nav2_costmap_2d::Costmap2D & map, double now,
    double euclidean_weight, double traversal_weight,
    const SafetyCheck & retained_safety = {});

  // Shared scoring for both paths; independent of waypoint density and map resolution.
  double score(
    const nav_msgs::msg::Path & path, nav2_costmap_2d::Costmap2D & map,
    double euclidean_weight, double traversal_weight,
    const SafetyCheck & safety = {}) const;
  bool sameRoute(
    const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b,
    nav2_costmap_2d::Costmap2D & map) const;

private:
  nav_msgs::msg::Path remaining(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & robot,
    const geometry_msgs::msg::PoseStamped & goal,
    nav2_costmap_2d::Costmap2D & map) const;
  nav_msgs::msg::Path selected_;
  geometry_msgs::msg::PoseStamped goal_;
  std::string frame_;
  bool have_goal_{false};
  uint64_t current_id_{0}, next_id_{1};
  double last_switch_{0.0};
  Decision decision_;
};
}  // namespace nav2_regulated_pure_pursuit_controller
#endif
