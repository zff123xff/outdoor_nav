// Native co-search algorithms; this node only adapts live ROS input and spline output.
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
#include <chrono>
#include <cmath>

using namespace fast_planner;
class SearchEngine {
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FastExplorationManager> manager_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr spline_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector3d center_, position_{0,0,.3}, velocity_{0,0,0};
  Eigen::Matrix3d rotation_;
  std::string frame_;
  double yaw_=0, odom_time_=0, cloud_time_=0, last_cloud_process_=0, timeout_, spacing_;
  bool enabled_=false;
  unsigned clouds_=0;
  template<class T> T param(const std::string &name, const T &value) {
    if (!node_->has_parameter(name)) node_->declare_parameter<T>(name,value);
    return node_->get_parameter(name).get_value<T>();
  }
  Eigen::Isometry3d transform(const std::string &source, const builtin_interfaces::msg::Time &stamp) {
    auto msg=tf_->lookupTransform(frame_,source,stamp,rclcpp::Duration::from_seconds(.1));
    auto &q=msg.transform.rotation; auto &t=msg.transform.translation;
    Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
    out.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
    out.translation()<<t.x,t.y,t.z; return out;
  }
  size_t reachableFrontiers() const {
    return std::count_if(manager_->ed_->points_.begin(), manager_->ed_->points_.end(),
      [this](const Eigen::Vector3d &p) { return manager_->isReachableForAdapter(p); });
  }
  void state(const std::string &phase, const std::string &reason, int result=-1) {
    std_msgs::msg::String msg;
    msg.data=nlohmann::json{{"phase",phase},{"reason",reason},{"native_result",result},
      {"clouds",clouds_},{"odom_stamp",odom_time_},{"cloud_stamp",cloud_time_},
      {"reachable_frontiers",reachableFrontiers()},{"frontier_clusters",manager_->ed_->points_.size()}}.dump();
    state_pub_->publish(msg);
  }
  void odom(const nav_msgs::msg::Odometry &msg) {
    try {
      auto world=transform(msg.header.frame_id,msg.header.stamp);
      auto &p=msg.pose.pose.position; auto &q=msg.pose.pose.orientation;
      Eigen::Matrix3d orientation=world.linear()*Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
      position_=rotation_*(world*Eigen::Vector3d(p.x,p.y,p.z)-center_);
      // Native co-search is a 2D ground-vehicle planner at its configured slice.
      position_.z()=.3;
      yaw_=std::atan2((rotation_*orientation)(1,0),(rotation_*orientation)(0,0));
      auto &v=msg.twist.twist.linear;
      velocity_=rotation_*orientation*Eigen::Vector3d(v.x,v.y,v.z); velocity_.z()=0;
      odom_time_=rclcpp::Time(msg.header.stamp).seconds();
      auto &self=manager_->ed_->swarm_state_.at(0);
      self.pos_=position_; self.vel_=velocity_; self.yaw_=yaw_; self.stamp_=odom_time_;
      if (!self.start_pos_valid_) { self.start_pos_=position_; self.start_pos_valid_=true; }
    } catch(const std::exception &e) { state("waiting_odom",e.what()); }
  }
  void cloud(const sensor_msgs::msg::PointCloud2 &msg) {
    const double now=node_->now().seconds();
    if(now-last_cloud_process_<param<double>("adapter/map_period_s",.2)) return;
    const double stamp=rclcpp::Time(msg.header.stamp).seconds();
    if(now-stamp>timeout_ || stamp-now>.5) {state("waiting_sensor","stale or future point cloud");return;}
    try {
      auto world=transform(msg.header.frame_id,msg.header.stamp);
      pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>), filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(msg,*input);
      // Same sensor-frame filter and voxel size as upstream MapROS::processPointCloud.
      pcl::PassThrough<pcl::PointXYZ> pass; pass.setInputCloud(input);pass.setFilterFieldName("x");pass.setFilterLimits(.05,4.0);pass.filter(*filtered);
      pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(filtered);voxel.setLeafSize(.1f,.1f,.1f);
      pcl::PointCloud<pcl::PointXYZ> points;voxel.filter(points);
      for(auto &p:points) {
        Eigen::Vector3d local=rotation_*(world*Eigen::Vector3d(p.x,p.y,p.z)-center_);
        p.x=local.x();p.y=local.y();p.z=local.z();
      }
      if(points.empty()) {state("waiting_sensor","empty upstream-filtered cloud");return;}
      const Eigen::Vector3d origin=rotation_*(world.translation()-center_);
      manager_->sdf_map_->integrateExternalCloud(points,origin);
      cloud_time_=stamp;last_cloud_process_=now;++clouds_;
    } catch(const std::exception &e) {state("waiting_tf",e.what());}
  }
  Eigen::Vector3d worldPoint(const Eigen::Vector3d &p) {return rotation_.transpose()*p+center_;}
  void plan() {
    const double now=node_->now().seconds();
    if(!odom_time_ || !clouds_ || now-odom_time_>timeout_ || now-cloud_time_>timeout_) {
      state("waiting_sensor","fresh real odometry and point cloud required");return;
    }
    if(!enabled_) {state("ready","native map ready; waiting for task enable");return;}
    try {
      const double half_x=param<double>("adapter/task_length_m",10.)/2;
      const double half_y=param<double>("adapter/task_width_m",10.)/2;
      const bool entering=std::abs(position_.x())>half_x || std::abs(position_.y())>half_y;
      int result;
      if(entering) {
        const double inset=param<double>("adapter/entry_inset_m",.15);
        Eigen::Vector3d entry=position_;
        entry.x()=std::clamp(position_.x(),-half_x+inset,half_x-inset);
        entry.y()=std::clamp(position_.y(),-half_y+inset,half_y-inset);
        const double heading=std::atan2(entry.y()-position_.y(),entry.x()-position_.x());
        result=manager_->planTrajToView(position_,velocity_,Eigen::Vector3d::Zero(),Eigen::Vector3d(yaw_,0,0),entry,heading);
      } else {
        manager_->updateFrontierStruct(position_);
        result=manager_->planExploreMotion(position_,velocity_,Eigen::Vector3d::Zero(),Eigen::Vector3d(yaw_,0,0));
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
      path_pub_->publish(path);state(entering ? "entering":"executing","native optimized spline sampled for FollowPath",result);
    } catch(const std::exception &e) {state("planning",e.what());}
  }
public:
  explicit SearchEngine(rclcpp::Node::SharedPtr node):node_(node) {
    frame_=param<std::string>("adapter/map_frame","robot_1/map");
    center_<<param<double>("adapter/center_x",0.),param<double>("adapter/center_y",0.),0.;
    rotation_=Eigen::AngleAxisd(-param<double>("adapter/region_yaw",0.),Eigen::Vector3d::UnitZ()).toRotationMatrix();
    timeout_=param<double>("adapter/sensor_timeout_s",3.);spacing_=param<double>("adapter/path_spacing_m",.1);
    if(timeout_<=0 || spacing_<=0)throw std::runtime_error("invalid adapter timeout/spacing");
    state_pub_=node_->create_publisher<std_msgs::msg::String>("co_search/engine_state",rclcpp::QoS(1).transient_local());
    path_pub_=node_->create_publisher<nav_msgs::msg::Path>("co_search/path",1);
    spline_pub_=node_->create_publisher<bspline::msg::Bspline>("co_search/native_bspline",1);
    manager_=std::make_unique<FastExplorationManager>();manager_->initialize(node_);
    tf_=std::make_shared<tf2_ros::Buffer>(node_->get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
    odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>(param<std::string>("adapter/odom_topic","odom"),rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){odom(*msg);});
    cloud_sub_=node_->create_subscription<sensor_msgs::msg::PointCloud2>(param<std::string>("adapter/cloud_topic","livox/lidar"),rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){cloud(*msg);});
    enable_sub_=node_->create_subscription<std_msgs::msg::Bool>("co_search/enable",1,[this](std_msgs::msg::Bool::ConstSharedPtr msg){if(msg->data&&!enabled_)manager_->MarkExplorationStart();enabled_=msg->data;});
    timer_=node_->create_wall_timer(std::chrono::duration<double>(param<double>("adapter/plan_period_s",1.5)),[this](){plan();});
    state("starting","native co-search initialized, no controller commands published");
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("co_search_engine",rclcpp::NodeOptions());
  {SearchEngine engine(node);rclcpp::spin(node);}
  if(rclcpp::ok())rclcpp::shutdown();return 0;
}
