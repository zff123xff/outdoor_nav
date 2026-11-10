// Native co-search algorithms; this node adapts live ROS input and spline output.
//
// Threading contract (root fix for "insufficient accumulated cloud" during planning):
//   * Sensor callbacks (odom/cloud) live in their own MutuallyExclusive group and only
//     convert/filter/transform/buffer data. They never touch SDFMap or the native manager.
//   * Every SDFMap access -- cloud integration, ESDF update, frontier/SNN planning and
//     entry validation -- stays in the node default group together with all callbacks the
//     native manager creates, so mapping and planning remain serialized as upstream
//     assumes. A MultiThreadedExecutor then lets reception continue while a plan runs.
//   * Freshness of the point cloud is tracked by *reception* time (atomic), not by the
//     integration time, so a long LKH/optimization cycle can no longer make the node
//     report its own input as stale.
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/expl_data.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <active_perception/frontier_finder.h>
#include <bspline/msg/bspline.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <algorithm>
#include <vector>
#include <deque>

using namespace fast_planner;
class SearchEngine {
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FastExplorationManager> manager_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr entry_failure_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr spline_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr integrate_timer_;
  rclcpp::CallbackGroup::SharedPtr odom_cbg_, cloud_cbg_;

  // --- shared with the sensor thread (guarded) ---
  std::mutex pose_mutex_, cloud_queue_mutex_, error_mutex_;
  Eigen::Vector3d position_{0,0,.3}, velocity_{0,0,0};
  double yaw_=0, odom_time_=0;
  std::string pose_error_, frame_error_;
  struct PendingCloud {
    pcl::PointCloud<pcl::PointXYZ> points;
    Eigen::Vector3d origin;
    double stamp=0.;
  };
  std::deque<PendingCloud> pending_clouds_;
  std::atomic<double> cloud_receive_time_{0.};
  std::atomic<unsigned long> cloud_received_{0};
  std::atomic<unsigned long> cloud_dropped_{0};
  std::atomic<double> odom_receive_stamp_{0.};

  // --- planning thread only (node default callback group) ---
  Eigen::Vector3d center_;
  double cloud_tf_lookup_timeout_;
  Eigen::Matrix3d rotation_;
  std::string frame_, base_frame_, odom_frame_, last_cloud_error_;
  bool cloud_x_filter_;
  double cloud_min_x_, cloud_max_x_, cloud_voxel_size_, cloud_future_tolerance_;
  double ground_height_, min_obstacle_height_, max_obstacle_height_;
  double cloud_time_=0, last_cloud_process_=0, timeout_, spacing_;
  std::string search_strategy_="co_search";
  double parallel_lane_spacing_=2., parallel_margin_=.8, parallel_waypoint_tolerance_=.5;
  std::vector<Eigen::Vector3d> parallel_points_;
  size_t parallel_cursor_=0;
  Eigen::Vector3d parallel_projection_{0,0,.3};
  bool parallel_initialized_=false, parallel_projection_pending_=false;
  struct BufferedCloud {
    pcl::PointCloud<pcl::PointXYZ> points;
    Eigen::Vector3d origin;
    double stamp;
    bool integrated=false;
  };
  std::deque<BufferedCloud> cloud_buffer_;
  double cloud_window_s_;
  int cloud_min_points_;
  size_t cloud_window_points_=0;
  bool enabled_=false;
  unsigned clouds_=0;
  bool start_pos_sent_=false;
  Eigen::Vector3d entry_target_{0,0,.3};
  bool entry_active_=false, entry_failed_=false;
  double entry_inset_, entry_spacing_, entry_clearance_, entry_cooldown_, entry_timeout_;
  double entry_blocked_since_=-1.;
  int entry_max_attempts_, entry_max_failures_, entry_execution_failures_=0;
  std::vector<std::pair<Eigen::Vector3d,double>> rejected_entries_;
  template<class T> T param(const std::string &name, const T &value) {
    if (!node_->has_parameter(name)) node_->declare_parameter<T>(name,value);
    return node_->get_parameter(name).get_value<T>();
  }
  Eigen::Isometry3d transformTo(const std::string &target, const std::string &source,
                                const builtin_interfaces::msg::Time &stamp,
                                double timeout_sec=0.1) {
    auto msg=tf_->lookupTransform(target,source,stamp,rclcpp::Duration::from_seconds(timeout_sec));
    auto &q=msg.transform.rotation; auto &t=msg.transform.translation;
    Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
    out.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
    out.translation()<<t.x,t.y,t.z; return out;
  }
  Eigen::Isometry3d transform(const std::string &source, const builtin_interfaces::msg::Time &stamp,
                              double timeout_sec=0.1) {
    return transformTo(frame_,source,stamp,timeout_sec);
  }
  size_t reachableFrontiers() const {
    return std::count_if(manager_->ed_->points_.begin(), manager_->ed_->points_.end(),
      [this](const Eigen::Vector3d &p) { return manager_->isReachableForAdapter(p); });
  }
  void state(const std::string &phase, const std::string &reason, int result=-1) {
    std_msgs::msg::String msg;
    msg.data=nlohmann::json{{"phase",phase},{"reason",reason},{"native_result",result},
      {"clouds",clouds_},{"odom_stamp",odom_receive_stamp_.load()},{"cloud_stamp",cloud_time_},
      {"cloud_receive_stamp",cloud_receive_time_.load()},
      {"cloud_frames",cloud_received_.load()},{"cloud_dropped",cloud_dropped_.load()},
      {"cloud_window_frames",cloud_buffer_.size()},{"cloud_window_points",cloud_window_points_},
      {"reachable_frontiers",reachableFrontiers()},{"frontier_clusters",manager_->ed_->points_.size()},
      {"entry_target_map",entry_active_ ? nlohmann::json::array({worldPoint(entry_target_).x(),worldPoint(entry_target_).y()}) : nlohmann::json(nullptr)},
      {"entry_execution_failures",entry_execution_failures_}}.dump();
    state_pub_->publish(msg);
  }
  // Sensor thread: pose only, no manager access.
  void odom(const nav_msgs::msg::Odometry &msg) {
    try {
      const auto &source=msg.header.frame_id.empty() ? odom_frame_ : msg.header.frame_id;
      if(msg.header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
          "Odometry header.frame_id is empty; using configured adapter/odom_frame '%s'",
          odom_frame_.c_str());
      }
      auto world=transform(source,msg.header.stamp);
      auto &p=msg.pose.pose.position; auto &q=msg.pose.pose.orientation;
      Eigen::Matrix3d orientation=world.linear()*Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
      Eigen::Vector3d local=rotation_*(world*Eigen::Vector3d(p.x,p.y,p.z)-center_);
      // Native co-search is a 2D ground-vehicle planner at its configured slice.
      local.z()=.3;
      const double yaw=std::atan2((rotation_*orientation)(1,0),(rotation_*orientation)(0,0));
      auto &v=msg.twist.twist.linear;
      Eigen::Vector3d vel=rotation_*orientation*Eigen::Vector3d(v.x,v.y,v.z); vel.z()=0;
      const double stamp=rclcpp::Time(msg.header.stamp).seconds();
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        position_=local; velocity_=vel; yaw_=yaw; odom_time_=stamp;
      }
      odom_receive_stamp_.store(stamp);
      std::lock_guard<std::mutex> lock(error_mutex_);
      pose_error_.clear();
    } catch(const std::exception &e) {
      std::lock_guard<std::mutex> lock(error_mutex_);
      pose_error_=e.what();
    }
  }
  void setFrameError(const std::string &text) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    frame_error_=text;
  }
  // Planning thread: absorb every buffered frame, then apply the rolling window.
  void drainPending(double now) {
    std::deque<PendingCloud> incoming;
    {
      std::lock_guard<std::mutex> lock(cloud_queue_mutex_);
      incoming.swap(pending_clouds_);
    }
    for(auto &frame:incoming) {
      cloud_buffer_.push_back({std::move(frame.points),frame.origin,frame.stamp,false});
    }
    cloud_buffer_.erase(std::remove_if(cloud_buffer_.begin(),cloud_buffer_.end(),
      [&](const auto &frame){return now-frame.stamp>cloud_window_s_;}),cloud_buffer_.end());
    // Bound memory even if a sensor publishes at an unexpected rate.
    while(cloud_buffer_.size()>256)cloud_buffer_.pop_front();
  }
  void integrateCloudWindow(double now) {
    drainPending(now);
    if(now-last_cloud_process_<param<double>("adapter/map_period_s",.2))return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined(new pcl::PointCloud<pcl::PointXYZ>);
    bool pending=false;
    for(const auto &frame:cloud_buffer_) {
      *combined+=frame.points;pending=pending || !frame.integrated;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(combined);
    voxel.setLeafSize(cloud_voxel_size_,cloud_voxel_size_,cloud_voxel_size_);
    pcl::PointCloud<pcl::PointXYZ> distinct;voxel.filter(distinct);
    cloud_window_points_=distinct.size();
    if(cloud_window_points_<static_cast<size_t>(cloud_min_points_)) {
      std::string frame_error;
      { std::lock_guard<std::mutex> lock(error_mutex_); frame_error=frame_error_; }
      std::ostringstream reason;
      reason<<"insufficient accumulated cloud: window_s="<<cloud_window_s_
            <<" frames="<<cloud_buffer_.size()<<" distinct_points="<<cloud_window_points_
            <<" required="<<cloud_min_points_;
      if(!frame_error.empty())reason<<"; last_frame="<<frame_error;
      last_cloud_error_=reason.str();state("cloud_rejected",last_cloud_error_);return;
    }
    if(!pending)return;
    std::vector<std::pair<const pcl::PointCloud<pcl::PointXYZ>*,Eigen::Vector3d>> observations;
    for(const auto &frame:cloud_buffer_)if(!frame.integrated)observations.emplace_back(&frame.points,frame.origin);
    manager_->sdf_map_->integrateExternalCloudBatch(observations);
    for(auto &frame:cloud_buffer_) {
      if(frame.integrated)continue;
      // Preserve each observation's ray origin and integrate it only once.
      frame.integrated=true;cloud_time_=std::max(cloud_time_,frame.stamp);
    }
    last_cloud_process_=node_->now().seconds();
    last_cloud_error_.clear();++clouds_;
  }
  // Sensor thread: convert, filter and buffer only. No SDFMap, no state publishing.
  void cloud(const sensor_msgs::msg::PointCloud2 &msg) {
    const double now=node_->now().seconds();
    const double stamp=rclcpp::Time(msg.header.stamp).seconds();
    if(now-stamp>timeout_ || stamp-now>cloud_future_tolerance_) {
      setFrameError("stale or future point cloud");
      return;
    }
    try {
      pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>), filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(msg,*input);
      input->erase(std::remove_if(input->begin(),input->end(),[](const auto &p){
        return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z);}),input->end());
      // Defaults match upstream MapROS, with explicit task configuration.
      if(cloud_x_filter_) {
        pcl::PassThrough<pcl::PointXYZ> pass; pass.setInputCloud(input);
        pass.setFilterFieldName("x");pass.setFilterLimits(cloud_min_x_,cloud_max_x_);pass.filter(*filtered);
      } else { *filtered=*input; }
      pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(filtered);
      voxel.setLeafSize(cloud_voxel_size_,cloud_voxel_size_,cloud_voxel_size_);
      pcl::PointCloud<pcl::PointXYZ> points;voxel.filter(points);
      if(points.empty()) {
        double min_x=std::numeric_limits<double>::infinity(), max_x=-min_x;
        size_t finite=0;
        for(const auto &p:*input) {
          if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
          ++finite;min_x=std::min(min_x,double(p.x));max_x=std::max(max_x,double(p.x));
        }
        std::ostringstream detail;
        detail<<"empty upstream-filtered cloud: input="<<input->size()<<" finite_xyz="<<finite
              <<" sensor_frame='"<<msg.header.frame_id<<"'";
        if(finite)detail<<" x_range=["<<min_x<<","<<max_x<<"]";
        detail<<" x_filter="<<cloud_x_filter_<<" required_x=["<<cloud_min_x_<<","<<cloud_max_x_
              <<"] voxel_size="<<cloud_voxel_size_;
        setFrameError(detail.str());
        return;
      }
      auto world=transform(msg.header.frame_id,msg.header.stamp,cloud_tf_lookup_timeout_);
      auto base=transformTo(base_frame_,msg.header.frame_id,msg.header.stamp,cloud_tf_lookup_timeout_);
      pcl::PointCloud<pcl::PointXYZ> height_filtered;
      height_filtered.reserve(points.size());
      for(auto &p:points) {
        const Eigen::Vector3d sensor_point(p.x,p.y,p.z);
        const Eigen::Vector3d base_point=base*sensor_point;
        const double height_above_ground=base_point.z()-ground_height_;
        if(height_above_ground<min_obstacle_height_ || height_above_ground>max_obstacle_height_)continue;
        const Eigen::Vector3d local=rotation_*(world*sensor_point-center_);
        pcl::PointXYZ accepted;
        accepted.x=local.x();accepted.y=local.y();
        // SDFMap consumes this coordinate as height above the base_link plane.
        accepted.z=base_point.z();
        height_filtered.push_back(accepted);
      }
      if(height_filtered.empty()) {
        std::ostringstream detail;
        detail<<"empty cloud in base_link obstacle height band ["<<min_obstacle_height_<<","<<max_obstacle_height_<<"] m";
        setFrameError(detail.str());
        return;
      }
      const Eigen::Vector3d map_origin=rotation_*(world.translation()-center_);
      const Eigen::Vector3d base_origin=base.translation();
      const Eigen::Vector3d origin(map_origin.x(),map_origin.y(),base_origin.z());
      {
        std::lock_guard<std::mutex> lock(cloud_queue_mutex_);
        if(pending_clouds_.size()>=512) { pending_clouds_.pop_front(); ++cloud_dropped_; }
        pending_clouds_.push_back({std::move(height_filtered),origin,stamp});
      }
      cloud_receive_time_.store(stamp);
      ++cloud_received_;
      setFrameError("");
    } catch(const std::exception &e) {
      setFrameError(std::string("tf: ")+e.what());
    }
  }
  Eigen::Vector3d worldPoint(const Eigen::Vector3d &p) {return rotation_.transpose()*p+center_;}
  double steadySeconds() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  bool entryPoseSafe(const Eigen::Vector3d &point) {
    if(!point.allFinite())return false;
    auto &map=*manager_->sdf_map_;
    // Use the native inflated occupancy columns; unknown-space handling stays
    // with the existing optimistic path planner and final Nav2 collision checks.
    const double step=std::max(.02,param<double>("sdf_map/resolution",.1));
    const int cells=std::max(1,int(std::ceil(entry_clearance_/step)));
    for(int x=-cells;x<=cells;++x)for(int y=-cells;y<=cells;++y) {
      if(std::hypot(x*step,y*step)>entry_clearance_+step*.5)continue;
      Eigen::Vector3d sample=point+Eigen::Vector3d(x*step,y*step,0);
      Eigen::Vector3i index;
      if(!map.isInMap(sample))return false;
      map.posToIndex(sample,index);
      if(map.isColumnOccupied(index.x(),index.y()))return false;
    }
    return true;
  }
  void rejectEntry() {
    if(entry_active_)rejected_entries_.emplace_back(entry_target_,steadySeconds()+entry_cooldown_);
    entry_active_=false;
  }
  bool planEntry(double half_x,double half_y,const Eigen::Vector3d &pos,const Eigen::Vector3d &vel,double yaw) {
    const double now=steadySeconds();
    rejected_entries_.erase(std::remove_if(rejected_entries_.begin(),rejected_entries_.end(),
      [now](const auto &item){return item.second<=now;}),rejected_entries_.end());
    std::vector<Eigen::Vector3d> candidates;
    const double x=half_x-entry_inset_, y=half_y-entry_inset_;
    if(x<=0 || y<=0) {
      entry_failed_=true;state("entry_failed","search rectangle too small for configured entry inset");return false;
    }
    auto add=[&](double px,double py) {
      Eigen::Vector3d p(px,py,.3);
      for(const auto &old:candidates)if((p-old).head<2>().norm()<.01)return;
      candidates.push_back(p);
    };
    add(std::clamp(pos.x(),-x,x),std::clamp(pos.y(),-y,y));
    // Bounded samples on all four inset edges, including corners.
    const int nx=std::min(128,std::max(1,int(std::ceil(2*x/entry_spacing_))));
    const int ny=std::min(128,std::max(1,int(std::ceil(2*y/entry_spacing_))));
    for(int i=0;i<=nx;++i){const double px=-x+2*x*i/nx;add(px,-y);add(px,y);}
    for(int i=0;i<=ny;++i){const double py=-y+2*y*i/ny;add(-x,py);add(x,py);}
    std::stable_sort(candidates.begin(),candidates.end(),[&](const auto &a,const auto &b){
      return (a-pos).squaredNorm()<(b-pos).squaredNorm();});
    if(entry_active_)candidates.insert(candidates.begin(),entry_target_);
    int attempted=0;
    for(const auto &entry:candidates) {
      const bool quarantined=std::any_of(rejected_entries_.begin(),rejected_entries_.end(),
        [&](const std::pair<Eigen::Vector3d,double> &item){return (entry-item.first).head<2>().norm()<entry_spacing_*.5;});
      if(quarantined || !entryPoseSafe(entry))continue;
      if(attempted++>=entry_max_attempts_)break;
      const double heading=std::atan2(entry.y()-pos.y(),entry.x()-pos.x());
      bool safe=false;
      try {
        safe=manager_->planTrajToView(pos,vel,Eigen::Vector3d::Zero(),
          Eigen::Vector3d(yaw,0,0),entry,heading)==SUCCEED;
        if(safe) {
        auto &curve=manager_->planner_manager_->local_data_.position_traj_;
        const double duration=curve.getTimeSum();
        safe=std::isfinite(duration) && duration>0;
        // Validate the optimized segment too, including native near-goal shortcuts.
        const double velocity=std::max(.1,param<double>("manager/max_vel",1.));
        const double dt=std::min(.05,param<double>("sdf_map/resolution",.1)/(2*velocity));
        if(safe)for(double t=0;t<=duration+dt;t+=dt) {
          if(!entryPoseSafe(curve.evaluateDeBoorT(std::min(t,duration)))){safe=false;break;}
        }
        }
      } catch(const std::exception &e) {
        safe=false;
        RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
          "Area entry candidate planning failed: %s",e.what());
      }
      if(safe) {
        if(!entry_active_ || (entry_target_-entry).head<2>().norm()>.01) {
          const auto world=worldPoint(entry);
          RCLCPP_INFO(node_->get_logger(),"Area entry selected: map=(%.3f, %.3f), clearance=%.2fm, attempt=%d",
            world.x(),world.y(),entry_clearance_,attempted);
        }
        entry_target_=entry;entry_active_=true;entry_blocked_since_=-1.;return true;
      }
      rejected_entries_.emplace_back(entry,now+entry_cooldown_);
      if(entry_active_ && (entry_target_-entry).head<2>().norm()<.01)entry_active_=false;
    }
    entry_active_=false;
    if(entry_blocked_since_<0)entry_blocked_since_=now;
    entry_failed_=now-entry_blocked_since_>=entry_timeout_;
    state(entry_failed_ ? "entry_failed":"entry_waiting",
      entry_failed_ ? "no collision-free plannable area entry before timeout" :
      "no area entry planned this cycle; waiting for map update or candidate cooldown");
    return false;
  }
  bool makeParallelSweep(double half_x,double half_y,const Eigen::Vector3d &pos) {
    if (parallel_initialized_) return true;
    const bool along_x=half_x>=half_y;
    const double along_half=(along_x?half_x:half_y)-parallel_margin_;
    const double across_half=(along_x?half_y:half_x)-parallel_margin_;
    if (along_half<=0. || across_half<=0.)
      throw std::runtime_error("parallel-line boundary margin leaves no drivable area");
    const int lanes=std::max(1,static_cast<int>(std::ceil(2.*across_half/parallel_lane_spacing_))+1);
    std::vector<Eigen::Vector3d> best;
    double best_distance=std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_projection=Eigen::Vector3d::Zero();
    size_t best_segment=0;
    for(int across_sign : {-1,1}) for(int along_sign : {-1,1}) {
      std::vector<Eigen::Vector3d> route;
      route.reserve(static_cast<size_t>(lanes)*2);
      for(int lane=0;lane<lanes;++lane) {
        const int order=across_sign>0 ? lane : lanes-1-lane;
        const double across=lanes==1 ? 0. : -across_half+2.*across_half*order/(lanes-1);
        const bool forward=(lane%2==0);
        const int sign=(forward ? along_sign : -along_sign);
        const double first=-sign*along_half, last=sign*along_half;
        Eigen::Vector3d a,b;
        if(along_x) { a<<first,across,.3; b<<last,across,.3; }
        else { a<<across,first,.3; b<<across,last,.3; }
        route.push_back(a); route.push_back(b);
      }
      for(size_t i=0;i+1<route.size();++i) {
        const Eigen::Vector2d a=route[i].head<2>(), d=route[i+1].head<2>()-a;
        const double denom=d.squaredNorm();
        const double t=denom>1e-9 ? std::clamp((pos.head<2>()-a).dot(d)/denom,0.,1.) : 0.;
        const Eigen::Vector3d projection=route[i]+t*(route[i+1]-route[i]);
        const double distance=(projection-pos).head<2>().squaredNorm();
        if(distance<best_distance) {
          best_distance=distance; best=route; best_projection=projection; best_segment=i;
        }
      }
    }
    if(best.size()<2) throw std::runtime_error("could not generate parallel-line route");
    parallel_points_=std::move(best);
    parallel_projection_=best_projection;
    parallel_cursor_=best_segment+1;
    parallel_projection_pending_=(best_projection-pos).head<2>().norm()>parallel_waypoint_tolerance_;
    parallel_initialized_=true;
    RCLCPP_INFO(node_->get_logger(),
      "Parallel-line sweep initialized: lanes=%zu lane_spacing=%.2fm first_segment=%zu",
      parallel_points_.size()/2,parallel_lane_spacing_,best_segment/2);
    return true;
  }
  bool publishParallelSweep(const Eigen::Vector3d &pos,double half_x,double half_y) {
    makeParallelSweep(half_x,half_y,pos);
    if(parallel_projection_pending_ &&
       (pos-parallel_projection_).head<2>().norm()<=parallel_waypoint_tolerance_)
      parallel_projection_pending_=false;
    while(parallel_cursor_<parallel_points_.size()) {
      const Eigen::Vector2d target=parallel_points_[parallel_cursor_].head<2>();
      const Eigen::Vector2d previous=parallel_cursor_>0 ? parallel_points_[parallel_cursor_-1].head<2>() :
        parallel_projection_.head<2>();
      const Eigen::Vector2d direction=target-previous;
      const bool reached=(pos.head<2>()-target).norm()<=parallel_waypoint_tolerance_;
      const bool passed=direction.squaredNorm()>1e-8 &&
        (pos.head<2>()-target).dot(direction)>=0. &&
        std::abs((pos.head<2>()-target).x()*direction.y() -
                 (pos.head<2>()-target).y()*direction.x())/direction.norm()<=
          std::max(parallel_lane_spacing_,2.*parallel_waypoint_tolerance_);
      if(!reached && !passed) break;
      ++parallel_cursor_;
    }
    if(parallel_cursor_>=parallel_points_.size()) {
      state("exhausted","parallel-line sweep complete",NO_FRONTIER);
      return false;
    }
    std::vector<Eigen::Vector3d> targets;
    if(parallel_projection_pending_) targets.push_back(parallel_projection_);
    for(size_t i=parallel_cursor_;i<parallel_points_.size();++i) targets.push_back(parallel_points_[i]);
    if(targets.empty()) { state("exhausted","parallel-line sweep complete",NO_FRONTIER); return false; }
    nav_msgs::msg::Path path; path.header.frame_id=frame_; path.header.stamp=node_->now();
    std::vector<Eigen::Vector3d> local_path{pos};
    Eigen::Vector3d previous=pos;
    double total=0.;
    for(const auto &target:targets) {
      total+=(target-previous).head<2>().norm();
      previous=target;
    }
    const double step=std::max(spacing_,total/20000.);
    previous=pos;
    for(const auto &target:targets) {
      const int count=std::max(1,static_cast<int>(std::ceil((target-previous).head<2>().norm()/step)));
      for(int i=1;i<=count;++i) local_path.push_back(previous+(target-previous)*(double(i)/count));
      previous=target;
    }
    for(size_t i=0;i<local_path.size();++i) {
      const Eigen::Vector3d world=worldPoint(local_path[i]);
      geometry_msgs::msg::PoseStamped pose; pose.header=path.header;
      pose.pose.position.x=world.x(); pose.pose.position.y=world.y(); pose.pose.position.z=world.z();
      const size_t next=std::min(i+1,local_path.size()-1);
      const Eigen::Vector3d heading_delta=local_path[next]-local_path[i];
      const double yaw=heading_delta.head<2>().norm()>1e-6 ?
        std::atan2(heading_delta.y(),heading_delta.x())+param<double>("adapter/region_yaw",0.) : yaw_;
      pose.pose.orientation.z=std::sin(yaw/2.); pose.pose.orientation.w=std::cos(yaw/2.);
      path.poses.push_back(std::move(pose));
    }
    if(path.poses.size()<2) { state("planning","parallel-line path has fewer than two points"); return false; }
    path_pub_->publish(path);
    state("executing","parallel-line sweep -> existing FollowPath controller",SUCCEED);
    return true;
  }
  void plan() {
    static unsigned long plan_cycle = 0;
    const auto cycle = ++plan_cycle;
    const auto plan_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=cycle_begin enabled=%d", cycle, enabled_);
    const double now=node_->now().seconds();
    // Absorb buffered frames first: mapping stays serialized with planning.
    auto stage_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=cloud_integration begin", cycle);
    integrateCloudWindow(now);
    RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=cloud_integration done elapsed_ms=%.1f",
      cycle, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-stage_start).count());
    Eigen::Vector3d pos, vel;
    double yaw=0, odom_time=0;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pos=position_; vel=velocity_; yaw=yaw_; odom_time=odom_time_;
    }
    auto &self=manager_->ed_->swarm_state_.at(0);
    self.pos_=pos; self.vel_=vel; self.yaw_=yaw; self.stamp_=odom_time;
    if(!start_pos_sent_ && odom_time>0) {
      self.start_pos_=pos; self.start_pos_valid_=true; start_pos_sent_=true;
    }
    const double cloud_recv=cloud_receive_time_.load();
    const bool cloud_fresh=cloud_recv>0 && now-cloud_recv<=timeout_;
    if(odom_time<=0 || !clouds_ || now-odom_time>timeout_ || !cloud_fresh) {
      const bool cloud_missing=!clouds_ || !cloud_fresh;
      std::string detail="fresh real odometry and point cloud required";
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if(!pose_error_.empty()) detail=pose_error_;
        else if(!frame_error_.empty()) detail=frame_error_;
      }
      state("waiting_sensor",cloud_missing && !last_cloud_error_.empty() ? last_cloud_error_ : detail);
      return;
    }
    if(!enabled_) {state("ready","native map ready; waiting for task enable");return;}
    if(entry_failed_){state("entry_failed","area entry attempts exhausted");return;}
    try {
      const double half_x=param<double>("adapter/task_length_m",10.)/2;
      const double half_y=param<double>("adapter/task_width_m",10.)/2;
      const bool outside=std::abs(pos.x())>half_x || std::abs(pos.y())>half_y;
      if(!outside && entry_active_ && (pos-entry_target_).head<2>().norm()<=.25) {
        entry_active_=false;rejected_entries_.clear();entry_execution_failures_=0;
      }
      const bool entering=outside || entry_active_;
      int result;
      if(entering) {
        if(!planEntry(half_x,half_y,pos,vel,yaw))return;
        result=SUCCEED;
      } else if(search_strategy_=="parallel_lines") {
        publishParallelSweep(pos,half_x,half_y);
        return;
      } else {
        stage_start = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=frontier_update begin", cycle);
        manager_->updateFrontierStruct(pos);
        RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=frontier_update done elapsed_ms=%.1f clusters=%zu viewpoints=%zu",
          cycle, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-stage_start).count(),
          manager_->ed_->frontiers_.size(), manager_->ed_->points_.size());
        stage_start = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=route_planning begin", cycle);
        result=manager_->planExploreMotion(pos,vel,Eigen::Vector3d::Zero(),Eigen::Vector3d(yaw,0,0));
        RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=route_planning done elapsed_ms=%.1f result=%d",
          cycle, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-stage_start).count(), result);
      }
      if(result!=SUCCEED) {
        state(!entering && (result==NO_FRONTIER || result==NO_GRID) && reachableFrontiers()==0 ? "exhausted":"planning", "native planner result",result);return;
      }
      auto &data=manager_->planner_manager_->local_data_;
      auto &curve=data.position_traj_;
      const double duration=curve.getTimeSum();
      if(!std::isfinite(duration)||duration<=0) {state("planning","invalid native spline duration");return;}
      bspline::msg::Bspline spline;
      spline.drone_id=1;spline.order=manager_->planner_manager_->pp_.bspline_degree_;
      spline.traj_id=data.traj_id_;spline.start_time=node_->now();
      const auto control=curve.getControlPoint();
      for(int i=0;i<control.rows();++i) {geometry_msgs::msg::Point p;p.x=control(i,0);p.y=control(i,1);p.z=control(i,2);spline.pos_pts.push_back(p);}
      auto knots=curve.getKnot();for(int i=0;i<knots.size();++i)spline.knots.push_back(knots[i]);
      const auto yaw_control=data.yaw_traj_.getControlPoint();
      for(int i=0;i<yaw_control.rows();++i)spline.yaw_pts.push_back(yaw_control(i,0));
      if(yaw_control.rows()>0)spline.yaw_dt=data.yaw_traj_.getKnotSpan();
      spline_pub_->publish(spline);
      nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=node_->now();
      // Evaluate the ORIGINAL optimized spline; dense geometric sampling only.
      const int count=std::max(1,static_cast<int>(std::ceil(duration/.02)));
      Eigen::Vector3d previous=worldPoint(curve.evaluateDeBoorT(0));
      auto append=[&](const Eigen::Vector3d &p){geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
        pose.pose.position.x=p.x();pose.pose.position.y=p.y();pose.pose.orientation.w=1.;path.poses.push_back(pose);};
      append(previous);
      for(int i=1;i<=count;++i) {
        Eigen::Vector3d next=worldPoint(curve.evaluateDeBoorT(duration*i/count));
        const int subdivisions=std::max(1,static_cast<int>(std::ceil((next-previous).head<2>().norm()/spacing_)));
        for(int j=1;j<=subdivisions;++j)append(previous+(next-previous)*(double(j)/subdivisions));
        previous=next;
      }
      for(size_t i=0;i+1<path.poses.size();++i) {
        auto &a=path.poses[i].pose;auto &b=path.poses[i+1].pose;
        const double heading=std::atan2(b.position.y-a.position.y,b.position.x-a.position.x);
        a.orientation.z=std::sin(heading/2);a.orientation.w=std::cos(heading/2);
      }
      if(path.poses.size()>1)path.poses.back().pose.orientation=path.poses[path.poses.size()-2].pose.orientation;
      if(yaw_control.rows()>0) {
        const double goal_yaw=data.yaw_traj_.evaluateDeBoorT(duration)[0]+param<double>("adapter/region_yaw",0.);
        path.poses.back().pose.orientation.z=std::sin(goal_yaw/2);
        path.poses.back().pose.orientation.w=std::cos(goal_yaw/2);
      }
      path_pub_->publish(path);
      RCLCPP_INFO(node_->get_logger(), "[plan_diag] cycle=%lu stage=cycle_done elapsed_ms=%.1f path_points=%zu",
        cycle, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-plan_start).count(), path.poses.size());
      state(entering ? "entering":"executing","native optimized spline sampled for FollowPath",result);
    } catch(const std::exception &e) {state("planning",e.what());}
  }
public:
  explicit SearchEngine(rclcpp::Node::SharedPtr node):node_(node) {
    frame_=param<std::string>("adapter/map_frame","robot_1/map");
    const std::string ns=node_->get_namespace();
    base_frame_=param<std::string>("adapter/base_frame", ns=="/" ? "base_link" : ns.substr(1)+"/base_link");
    if(base_frame_.empty() || base_frame_.front()=='/') {
      throw std::runtime_error("adapter/base_frame must be a nonempty TF frame without leading slash");
    }
    odom_frame_=param<std::string>("adapter/odom_frame", ns=="/" ? "odom" : ns.substr(1)+"/odom");
    if(odom_frame_.empty() || odom_frame_.front()=='/') {
      throw std::runtime_error("adapter/odom_frame must be a nonempty TF frame without leading slash");
    }
    center_<<param<double>("adapter/center_x",0.),param<double>("adapter/center_y",0.),0.;
    rotation_=Eigen::AngleAxisd(-param<double>("adapter/region_yaw",0.),Eigen::Vector3d::UnitZ()).toRotationMatrix();
    timeout_=param<double>("adapter/sensor_timeout_s",3.);spacing_=param<double>("adapter/path_spacing_m",.1);
    search_strategy_=param<std::string>("adapter/search_strategy","co_search");
    parallel_lane_spacing_=param<double>("adapter/parallel_lane_spacing_m",2.);
    parallel_margin_=param<double>("adapter/parallel_boundary_margin_m",.8);
    parallel_waypoint_tolerance_=param<double>("adapter/parallel_waypoint_tolerance_m",.5);
    if(search_strategy_!="co_search" && search_strategy_!="parallel_lines")
      throw std::runtime_error("adapter/search_strategy must be co_search or parallel_lines");
    if(!std::isfinite(parallel_lane_spacing_) || parallel_lane_spacing_<=0. ||
       !std::isfinite(parallel_margin_) || parallel_margin_<=0. ||
       !std::isfinite(parallel_waypoint_tolerance_) || parallel_waypoint_tolerance_<=0.)
      throw std::runtime_error("invalid parallel-line search parameters");
    entry_inset_=param<double>("adapter/entry_inset_m",.8);
    entry_spacing_=param<double>("adapter/entry_sample_spacing_m",.5);
    entry_clearance_=param<double>("adapter/entry_clearance_m",.7);
    entry_cooldown_=param<double>("adapter/entry_retry_cooldown_s",5.);
    entry_timeout_=param<double>("adapter/entry_timeout_s",30.);
    entry_max_attempts_=param<int>("adapter/entry_max_plan_attempts",4);
    entry_max_failures_=param<int>("adapter/entry_max_execution_failures",4);
    if(!std::isfinite(entry_inset_) || !std::isfinite(entry_spacing_) || !std::isfinite(entry_clearance_) ||
       !std::isfinite(entry_cooldown_) || !std::isfinite(entry_timeout_) || entry_clearance_<0 ||
       entry_inset_<=entry_clearance_ || entry_spacing_<=0 || entry_cooldown_<=0 || entry_timeout_<=0 ||
       entry_max_attempts_<1 || entry_max_attempts_>32 || entry_max_failures_<1) {
      throw std::runtime_error("invalid area entry parameters");
    }
    cloud_x_filter_=param<bool>("adapter/cloud_enable_x_filter",true);
    cloud_min_x_=param<double>("adapter/cloud_min_x_m",.05);
    cloud_max_x_=param<double>("adapter/cloud_max_x_m",4.);
    cloud_voxel_size_=param<double>("adapter/cloud_voxel_size_m",.1);
    cloud_future_tolerance_=param<double>("adapter/cloud_max_future_offset_s",.5);
    cloud_tf_lookup_timeout_=param<double>("adapter/cloud_tf_lookup_timeout_s",2.0);
    cloud_window_s_=param<double>("adapter/cloud_accumulation_window_s",1.);
    cloud_min_points_=param<int>("adapter/cloud_min_accumulated_points",10);
    ground_height_=param<double>("sdf_map/ground_height",1.0);
    min_obstacle_height_=param<double>("sdf_map/min_obstacle_height",.2);
    max_obstacle_height_=param<double>("sdf_map/max_obstacle_height",.8);
    if(!std::isfinite(cloud_min_x_) || !std::isfinite(cloud_max_x_) || cloud_min_x_>=cloud_max_x_ ||
       !std::isfinite(cloud_voxel_size_) || cloud_voxel_size_<=0 ||
       !std::isfinite(ground_height_) ||
       !std::isfinite(min_obstacle_height_) || !std::isfinite(max_obstacle_height_) ||
       min_obstacle_height_<0.0 || min_obstacle_height_>=max_obstacle_height_ ||
       !std::isfinite(cloud_future_tolerance_) || cloud_future_tolerance_<0 ||
       !std::isfinite(cloud_tf_lookup_timeout_) || cloud_tf_lookup_timeout_<=0 ||
       !std::isfinite(cloud_window_s_) || cloud_window_s_<=0 ||
       cloud_min_points_<1 || cloud_min_points_>1000000) {
      throw std::runtime_error("invalid adapter cloud filter configuration");
    }
    if(timeout_<=0 || spacing_<=0)throw std::runtime_error("invalid adapter timeout/spacing");
    state_pub_=node_->create_publisher<std_msgs::msg::String>("co_search/engine_state",rclcpp::QoS(1).transient_local());
    path_pub_=node_->create_publisher<nav_msgs::msg::Path>("co_search/path",1);
    spline_pub_=node_->create_publisher<bspline::msg::Bspline>("co_search/native_bspline",1);
    manager_=std::make_unique<FastExplorationManager>();manager_->initialize(node_);
    tf_=std::make_shared<tf2_ros::Buffer>(node_->get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
    // Separate sensor groups so a bounded cloud TF wait cannot block odometry updates.
    odom_cbg_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_cbg_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_opts; odom_opts.callback_group=odom_cbg_;
    rclcpp::SubscriptionOptions cloud_opts; cloud_opts.callback_group=cloud_cbg_;
    odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>(param<std::string>("adapter/odom_topic","odom"),rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){odom(*msg);},odom_opts);
    cloud_sub_=node_->create_subscription<sensor_msgs::msg::PointCloud2>(param<std::string>("adapter/cloud_topic","livox/lidar"),rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){cloud(*msg);},cloud_opts);
    // Everything below shares the node default group with the native manager callbacks.
    enable_sub_=node_->create_subscription<std_msgs::msg::Bool>("co_search/enable",1,[this](std_msgs::msg::Bool::ConstSharedPtr msg){if(msg->data&&!enabled_)manager_->MarkExplorationStart();enabled_=msg->data;});
    entry_failure_sub_=node_->create_subscription<std_msgs::msg::Bool>("co_search/entry_failed",1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if(!msg->data || !entry_active_)return;
        rejectEntry();
        entry_failed_=++entry_execution_failures_>=entry_max_failures_;
        state(entry_failed_ ? "entry_failed":"entry_retry",
          entry_failed_ ? "area entry FollowPath failure limit reached" :
          "entry FollowPath aborted; quarantining entry and selecting an alternative");
      });
    timer_=node_->create_wall_timer(std::chrono::duration<double>(param<double>("adapter/plan_period_s",1.5)),[this](){plan();});
    integrate_timer_=node_->create_wall_timer(std::chrono::duration<double>(param<double>("adapter/map_period_s",.2)),
      [this](){integrateCloudWindow(node_->now().seconds());});
    state("starting","native co-search initialized, no controller commands published");
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("co_search_engine",rclcpp::NodeOptions());
  SearchEngine engine(node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
  executor.add_node(node);
  executor.spin();
  if(rclcpp::ok())rclcpp::shutdown();return 0;
}
