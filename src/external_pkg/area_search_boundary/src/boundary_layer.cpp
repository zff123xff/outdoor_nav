#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace area_search_boundary
{
class BoundaryLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("boundary layer lifecycle node unavailable");}
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("robot_namespace", rclcpp::ParameterValue(std::string("")));
    node->get_parameter(name_ + ".enabled", enabled_);
    std::string ns;
    node->get_parameter(name_ + ".robot_namespace", ns);
    if (ns.empty()) {
      ns = node->get_namespace();
      const auto suffix = ns.rfind("/local_costmap");
      if (suffix != std::string::npos && suffix + 14 == ns.size()) {ns.resize(suffix);}
    }
    if (ns.empty() || ns == "/") {throw std::runtime_error("boundary layer requires a robot namespace");}
    if (ns.front() != '/') {ns = "/" + ns;}
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    ack_ = node->create_publisher<std_msgs::msg::String>(ns + "/area_search/boundary_ready", qos);
    region_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      ns + "/area_search/region", qos,
      [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->header.frame_id != layered_costmap_->getGlobalFrameID() || msg->polygon.points.size() != 4) {return;}
        for (const auto & p : msg->polygon.points) {
          if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return;}
        }
        points_ = msg->polygon.points;
        token_ = std::to_string(msg->header.stamp.sec) + ":" + std::to_string(msg->header.stamp.nanosec);
      });
    active_sub_ = node->create_subscription<std_msgs::msg::Bool>(ns + "/area_search/active", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); active_ = msg->data;
      });
    current_ = true;
  }

  void updateBounds(double, double, double, double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_) {return;}
    // Full bounds also clear previous boundary costs after a task is cancelled.
    auto * map = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, map->getOriginX());
    *min_y = std::min(*min_y, map->getOriginY());
    *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInMetersX());
    *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInMetersY());
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & map, int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || !active_) {return;}
    const bool valid = points_.size() == 4;
    for (int y = min_j; y < max_j; ++y) {
      for (int x = min_i; x < max_i; ++x) {
        double wx, wy; map.mapToWorld(x, y, wx, wy);
        bool inside = valid;
        // CCW convex rectangle; treating boundary cells as obstacles keeps the
        // local planner from cutting outside. Footprint collision checks remain.
        if (valid) {
          for (size_t i = 0; i < 4; ++i) {
            const auto & a = points_[i]; const auto & b = points_[(i + 1) % 4];
            if ((b.x-a.x)*(wy-a.y) - (b.y-a.y)*(wx-a.x) < 0) {inside = false; break;}
          }
        }
        if (!inside) {map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);}
      }
    }
    // Ack means the active master costmap has actually applied this geometry.
    if (valid && token_ != last_ack_) {
      std_msgs::msg::String msg; msg.data = token_; ack_->publish(msg); last_ack_ = token_;
    }
    current_ = true;
  }
  void reset() override {current_ = true;}
  bool isClearable() override {return false;}
private:
  std::mutex mutex_;
  bool active_{false};
  std::vector<geometry_msgs::msg::Point32> points_;
  std::string token_, last_ack_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr region_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_;
};
}
PLUGINLIB_EXPORT_CLASS(area_search_boundary::BoundaryLayer, nav2_costmap_2d::Layer)
