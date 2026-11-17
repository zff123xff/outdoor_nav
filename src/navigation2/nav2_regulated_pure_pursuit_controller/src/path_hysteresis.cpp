// Copyright 2026 outdoor_nav contributors
// Licensed under the Apache License, Version 2.0.
#include "nav2_regulated_pure_pursuit_controller/path_hysteresis.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_regulated_pure_pursuit_controller
{
namespace
{
using Pose = geometry_msgs::msg::PoseStamped;
double distance(const Pose & a, const Pose & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}
double yaw(const Pose & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
}
bool finite(const Pose & p)
{
  return std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y);
}
// Reuse the controller's lethal-obstacle boundary. Its existing footprint checker
// distinguishes physical collision from merely inscribed/inflated cost (253).
bool segmentSafe(const Pose & a, const Pose & b, nav2_costmap_2d::Costmap2D & map)
{
  if (!finite(a) || !finite(b)) {return false;}
  const int count = std::max(1, static_cast<int>(std::ceil(distance(a, b)/(map.getResolution()*.5))));
  unsigned int px = 0, py = 0;
  bool previous = false;
  for (int i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i)/count;
    unsigned int x, y;
    if (!map.worldToMap(a.pose.position.x + t*(b.pose.position.x-a.pose.position.x),
      a.pose.position.y + t*(b.pose.position.y-a.pose.position.y), x, y) ||
      map.getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      return false;
    }
    if (previous && x != px && y != py &&
      (map.getCost(px, y) >= nav2_costmap_2d::LETHAL_OBSTACLE ||
      map.getCost(x, py) >= nav2_costmap_2d::LETHAL_OBSTACLE))
    {
      return false;
    }
    px = x; py = y; previous = true;
  }
  return true;
}
struct Projection {Pose pose; size_t segment{0}; double error{INFINITY}; double progress{0.0};};
Projection project(const Pose & point, const nav_msgs::msg::Path & path)
{
  Projection result;
  double length = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i];
    const auto & b = path.poses[std::min(i+1, path.poses.size()-1)];
    const double dx = b.pose.position.x-a.pose.position.x;
    const double dy = b.pose.position.y-a.pose.position.y;
    const double squared = dx*dx+dy*dy;
    const double t = squared > 1e-12 ? std::clamp(
      ((point.pose.position.x-a.pose.position.x)*dx +
      (point.pose.position.y-a.pose.position.y)*dy)/squared, 0.0, 1.0) : 0.0;
    Pose p = a;
    p.pose.position.x += t*dx; p.pose.position.y += t*dy;
    const double error = distance(p, point);
    if (error < result.error) {result = {p, i, error, length + t*std::sqrt(squared)};}
    length += std::sqrt(squared);
  }
  return result;
}
}  // namespace

void PathHysteresis::reset()
{
  selected_.poses.clear(); current_id_ = 0; last_switch_ = 0.0;
}

void PathHysteresis::setGoal(const nav_msgs::msg::Path & guide)
{
  if (guide.poses.empty()) {reset(); have_goal_ = false; return;}
  const auto & end = guide.poses.back();
  const double angle = yaw(end)-yaw(goal_);
  if (!have_goal_ || frame_ != guide.header.frame_id || distance(goal_, end) > 1e-3 ||
    std::abs(goal_.pose.position.z-end.pose.position.z) > 1e-3 ||
    std::abs(std::atan2(std::sin(angle), std::cos(angle))) > 1e-3)
  {
    reset();
  }
  have_goal_ = true; goal_ = end; frame_ = guide.header.frame_id;
}

nav_msgs::msg::Path PathHysteresis::remaining(
  const nav_msgs::msg::Path & path, const Pose & robot, const Pose & goal,
  nav2_costmap_2d::Costmap2D & map) const
{
  nav_msgs::msg::Path result;
  if (path.poses.empty() || !finite(robot) || !finite(goal)) {return result;}
  const auto start = project(robot, path);
  const auto finish = project(goal, path);
  // A robot far away from the retained route must not be pulled back onto it.
  // In sticky mode a collision-free connector can rejoin the same route.
  if (!switch_only_on_obstacle && start.error > std::max(.5, 3.0*map.getResolution())) {
    return result;
  }
  // A shrinking rolling horizon must crop the old route, never double back from its tail.
  const bool crop = finish.error < (switch_only_on_obstacle ?
    std::max(.3, 3.0*map.getResolution()) : 1e-6);
  if (crop && finish.progress + 1e-6 < start.progress) {return result;}
  result.header = path.header;
  result.poses.push_back(robot);
  if (start.error > 1e-9) {result.poses.push_back(start.pose);}
  const size_t end = crop ? std::min(finish.segment+1, path.poses.size()) : path.poses.size();
  for (size_t i = start.segment+1; i < end; ++i) {result.poses.push_back(path.poses[i]);}
  // A rolling local goal can advance. Connect to the exact same endpoint before scoring.
  // The connector is subject to all validation below; no unchecked shortcut is retained.
  if (!switch_only_on_obstacle || crop) {
    if (distance(result.poses.back(), goal) > 1e-9) {result.poses.push_back(goal);}
    else {result.poses.back() = goal;}
  }
  if (!switch_only_on_obstacle) {return result;}

  // The rolling costmap can move past the old local horizon. Keep the safe
  // prefix of the same route instead of treating its out-of-map tail as a
  // reason to replace the entire path.
  nav_msgs::msg::Path clipped;
  clipped.header = result.header;
  unsigned int mx, my;
  if (!map.worldToMap(result.poses.front().pose.position.x,
    result.poses.front().pose.position.y, mx, my)) {return clipped;}
  clipped.poses.push_back(result.poses.front());
  for (size_t i = 1; i < result.poses.size(); ++i) {
    const auto & a = result.poses[i-1];
    const auto & b = result.poses[i];
    Pose last_inside = a;
    const int count = std::max(1, static_cast<int>(std::ceil(
      distance(a, b) / (map.getResolution() * .5))));
    for (int j = 1; j <= count; ++j) {
      const double t = static_cast<double>(j) / count;
      Pose sample = a;
      sample.pose.position.x += t * (b.pose.position.x - a.pose.position.x);
      sample.pose.position.y += t * (b.pose.position.y - a.pose.position.y);
      if (!map.worldToMap(sample.pose.position.x, sample.pose.position.y, mx, my)) {
        if (distance(clipped.poses.back(), last_inside) > 1e-9) {
          clipped.poses.push_back(last_inside);
        }
        return clipped;
      }
      last_inside = sample;
    }
    clipped.poses.push_back(b);
  }
  return clipped;
}

double PathHysteresis::score(
  const nav_msgs::msg::Path & path, nav2_costmap_2d::Costmap2D & map,
  double euclidean_weight, double traversal_weight, const SafetyCheck & safety) const
{
  if (path.poses.empty()) {return INFINITY;}
  double total = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i];
    const auto & b = path.poses[std::min(i+1, path.poses.size()-1)];
    if (!segmentSafe(a, b, map)) {return INFINITY;}
    const double length = distance(a, b);
    const int count = std::max(1, static_cast<int>(std::ceil(length/(map.getResolution()*.5))));
    const double heading = length > 1e-9 ?
      std::atan2(b.pose.position.y-a.pose.position.y, b.pose.position.x-a.pose.position.x) : yaw(a);
    double previous_density = 0.0;
    for (int j = 0; j <= count; ++j) {
      const double t = static_cast<double>(j)/count;
      const double wx = a.pose.position.x + t*(b.pose.position.x-a.pose.position.x);
      const double wy = a.pose.position.y + t*(b.pose.position.y-a.pose.position.y);
      unsigned int x, y;
      if (!map.worldToMap(wx, wy, x, y) || (safety && !safety(wx, wy, heading))) {return INFINITY;}
      // Theta*'s exact cost transform/weights, integrated over metres, not waypoint count.
      const double normalized = (26.0 + .9*map.getCost(x, y))/252.0;
      const double density = euclidean_weight + traversal_weight*normalized*normalized;
      if (j > 0) {total += .5*(previous_density+density)*length/count;}
      previous_density = density;
    }
  }
  return total;
}

bool PathHysteresis::sameRoute(
  const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b,
  nav2_costmap_2d::Costmap2D & map) const
{
  if (a.poses.empty() || b.poses.empty()) {return false;}
  const double tolerance = std::max(.3, 3.0*map.getResolution());
  auto matches = [&](const nav_msgs::msg::Path & from, const nav_msgs::msg::Path & to) {
      double last_progress = 0.0;
      for (size_t i = 0; i < from.poses.size(); ++i) {
        const auto & begin = from.poses[i];
        const auto & end = from.poses[std::min(i+1, from.poses.size()-1)];
        const int count = std::max(1, static_cast<int>(std::ceil(
          distance(begin, end)/std::max(.1, map.getResolution()))));
        for (int j = 0; j <= count; ++j) {
          Pose p = begin;
          const double t = static_cast<double>(j)/count;
          p.pose.position.x += t*(end.pose.position.x-begin.pose.position.x);
          p.pose.position.y += t*(end.pose.position.y-begin.pose.position.y);
          const auto match = project(p, to);
          if (match.error > tolerance || match.progress + tolerance < last_progress ||
            !segmentSafe(p, match.pose, map)) {return false;}
          last_progress = std::max(last_progress, match.progress);
        }
      }
      return true;
    };
  return matches(a, b) && matches(b, a);
}

nav_msgs::msg::Path PathHysteresis::select(
  const nav_msgs::msg::Path & candidate, const Pose & robot,
  nav2_costmap_2d::Costmap2D & map, double now,
  double euclidean_weight, double traversal_weight, const SafetyCheck & retained_safety)
{
  decision_ = Decision{};
  if (!enabled || candidate.poses.empty()) {
    reset(); decision_.reason = enabled ? "planner_failed" : "disabled";
    return candidate;  // In particular, planner failure never resurrects a cached path.
  }
  decision_.current_id = current_id_;
  auto previous = remaining(selected_, robot, candidate.poses.back(), map);
  decision_.current_cost = score(previous, map, euclidean_weight, traversal_weight, retained_safety);
  const bool horizon_complete = switch_only_on_obstacle && current_id_ &&
    !selected_.poses.empty() && distance(robot, selected_.poses.back()) < .5;
  // The candidate needs a footprint scan only when it can actually be used.
  decision_.candidate_cost = score(candidate, map, euclidean_weight, traversal_weight,
    switch_only_on_obstacle && (!std::isfinite(decision_.current_cost) || horizon_complete) ?
    retained_safety : SafetyCheck{});
  decision_.current_valid = std::isfinite(decision_.current_cost);
  decision_.candidate_valid = std::isfinite(decision_.candidate_cost);
  const bool same = !switch_only_on_obstacle && decision_.current_valid &&
    decision_.candidate_valid && sameRoute(previous, candidate, map);
  decision_.candidate_id = same ? current_id_ : next_id_;
  bool take_candidate = true;
  if (!decision_.current_valid) {
    if (switch_only_on_obstacle && current_id_ && !horizon_complete && previous.poses.empty()) {
      decision_.reason = "retained_unavailable";
      take_candidate = false;
    } else {
      decision_.reason = current_id_ ?
        (horizon_complete ? "horizon_continue" :
        (switch_only_on_obstacle ? "current_blocked" : "current_invalid")) : "initial";
    }
  } else if (!decision_.candidate_valid) {
    decision_.reason = "candidate_invalid"; take_candidate = false;
  } else if (horizon_complete) {
    decision_.reason = "horizon_continue";
  } else if (same) {
    decision_.reason = "same_route";
    if (switch_only_on_obstacle) {take_candidate = false;}
  } else if (switch_only_on_obstacle) {
    decision_.reason = "route_locked"; take_candidate = false;
  } else if (now-last_switch_ < min_switch_interval) {
    decision_.reason = "min_switch_interval"; take_candidate = false;
  } else if (!(decision_.candidate_cost+switch_penalty < decision_.current_cost)) {
    decision_.reason = "switch_penalty"; take_candidate = false;
  } else {
    decision_.reason = "cost_improved";
  }
  decision_.switched = take_candidate && current_id_ != 0 && !same && !horizon_complete;
  if (take_candidate) {
    if (!decision_.candidate_valid) {
      reset();
      return switch_only_on_obstacle ? nav_msgs::msg::Path{} : candidate;
    }
    if (!same && !horizon_complete) {current_id_ = next_id_++; last_switch_ = now;}
    selected_ = candidate;
  } else {selected_ = std::move(previous);}
  return selected_;
}
}  // namespace nav2_regulated_pure_pursuit_controller
