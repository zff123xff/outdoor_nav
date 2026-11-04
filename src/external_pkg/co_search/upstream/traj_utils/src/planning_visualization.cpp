#include <std_msgs/msg/color_rgba.hpp>
#include <traj_utils/planning_visualization.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <utility>

using std::cout;
using std::endl;

namespace
{
struct VoxelIdx
{
    int  ix;
    int  iy;
    int  iz;
    bool operator==(const VoxelIdx& o) const
    {
        return ix == o.ix && iy == o.iy && iz == o.iz;
    }
};

struct VoxelIdxHash
{
    std::size_t operator()(const VoxelIdx& v) const noexcept
    {
        // Splitmix-style mix; fine for hash table buckets.
        std::size_t h = static_cast<std::size_t>(v.ix);
        h ^= static_cast<std::size_t>(v.iy) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(v.iz) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return h;
    }
};

VoxelIdx PointToVoxel(const Eigen::Vector3d& p, double inv_voxel_size)
{
    return VoxelIdx{ static_cast<int>(std::floor(p.x() * inv_voxel_size)), static_cast<int>(std::floor(p.y() * inv_voxel_size)),
                     static_cast<int>(std::floor(p.z() * inv_voxel_size)) };
}

}  // namespace

namespace fast_planner
{
PlanningVisualization::PlanningVisualization(rclcpp::Node::SharedPtr node)
{
    node_ = node;

    if (!node->has_parameter("frontier_vis/enabled"))
    {
        node->declare_parameter<bool>("frontier_vis/enabled", false);
    }
    if (!node->has_parameter("frontier_vis/voxel_size"))
    {
        node->declare_parameter<double>("frontier_vis/voxel_size", kFrontierVisDefaultVoxelSize);
    }
    if (!node->has_parameter("frontier_vis/max_display_points"))
    {
        node->declare_parameter<int>("frontier_vis/max_display_points", kFrontierVisDefaultMaxDisplayPoints);
    }
    if (!node->has_parameter("frontier_vis/max_display_points_min"))
    {
        node->declare_parameter<int>("frontier_vis/max_display_points_min", kFrontierVisDefaultMaxDisplayPointsMin);
    }
    frontier_vis_enabled_       = node->get_parameter("frontier_vis/enabled").as_bool();
    frontier_vis_voxel_size_    = node->get_parameter("frontier_vis/voxel_size").as_double();
    const int display_cap_req   = static_cast<int>(node->get_parameter("frontier_vis/max_display_points").as_int());
    const int display_cap_floor = static_cast<int>(node->get_parameter("frontier_vis/max_display_points_min").as_int());
    frontier_vis_max_points_    = std::max(display_cap_floor, display_cap_req);

    // QoS: trajectory/stream KeepLast(5), car_id/path KeepLast(1), all BEST_EFFORT + VOLATILE.
    // RViz Marker display Reliability Policy must be Best Effort (see gazebo/ros2/gazebo.rviz).
    const auto qos_traj_stream   = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    const auto qos_marker_stream = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    const auto qos_marker_id     = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto qos_marker_path   = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    traj_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/trajectory", qos_traj_stream);
    pubs_.push_back(traj_pub_);

    topo_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/topo_path", qos_marker_stream);
    pubs_.push_back(topo_pub_);

    predict_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/prediction", qos_marker_stream);
    pubs_.push_back(predict_pub_);

    visib_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/visib_constraint", qos_marker_stream);
    pubs_.push_back(visib_pub_);

    frontier_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/frontier", qos_marker_stream);
    pubs_.push_back(frontier_pub_);

    yaw_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/yaw", qos_marker_stream);
    pubs_.push_back(yaw_pub_);

    viewpoint_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/viewpoints", qos_marker_stream);
    pubs_.push_back(viewpoint_pub_);

    id_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/car_id", qos_marker_id);
    pubs_.push_back(id_pub_);

    path_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/path", qos_marker_path);
    pubs_.push_back(path_pub_);

    last_topo_path1_num_     = 0;
    last_topo_path2_num_     = 0;
    last_bspline_phase1_num_ = 0;
    last_bspline_phase2_num_ = 0;
    last_frontier_num_       = 0;
}

bool PlanningVisualization::isPublisherEnabled(int pub_id) const
{
    // 默认只开 trajectory(0) 与 car_id(7) 两种核心可视化；frontier(4) 由 launch 参数
    // frontier_vis/enabled（frontier_vis_enabled）控制是否显示；其余流（topo_path/
    // prediction/visib/yaw/viewpoints/path 等）一律关闭，降低 RViz 相关带宽与 CPU。
    // 注意：不能靠注释 create_publisher 或函数体来关闭，否则 pubs_ 索引错位/空指针；
    // 保持所有 publish 函数不动、pubs_ 完整，仅在此开关处把关（各 draw 函数开头即早退）。
    switch (pub_id)
    {
    case 0:  // trajectory
    case 7:  // car_id
        return true;
    case 4:  // frontier：显示与否跟随 frontier_vis_enabled 参数（launch: frontier_vis_enabled）
        return frontier_vis_enabled_;
    default:  // 其他所有可视化流关闭
        return false;
    }
}

void PlanningVisualization::fillBasicInfo(visualization_msgs::msg::Marker& mk, const Eigen::Vector3d& scale, const Eigen::Vector4d& color, const string& ns,
                                          const int& id, const int& shape)
{
    mk.header.frame_id = "world";
    mk.header.stamp    = node_->now();
    mk.id              = id;
    mk.ns              = ns;
    mk.type            = shape;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = scale[0];
    mk.scale.y = scale[1];
    mk.scale.z = scale[2];
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::msg::Marker& mk, const vector<Eigen::Vector3d>& list)
{
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        mk.points.push_back(pt);
    }
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::msg::Marker& mk, const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2)
{
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list1.size()); ++i)
    {
        pt.x = list1[i](0);
        pt.y = list1[i](1);
        pt.z = list1[i](2);
        mk.points.push_back(pt);

        pt.x = list2[i](0);
        pt.y = list2[i](1);
        pt.z = list2[i](2);
        mk.points.push_back(pt);
    }
}

void PlanningVisualization::drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
                                    const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::msg::Marker::CUBE);
    mk.action = visualization_msgs::msg::Marker::DELETE;
    pubs_[pub_id]->publish(mk);

    mk.pose.position.x = center[0];
    mk.pose.position.y = center[1];
    mk.pose.position.z = center[2];
    mk.action          = visualization_msgs::msg::Marker::ADD;

    pubs_[pub_id]->publish(mk);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
}

void PlanningVisualization::drawText(const Eigen::Vector3d& pos, const string& text, const double& scale, const Eigen::Vector4d& color, const string& ns,
                                     const int& id, const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    if (text.empty())
    {
        mk.action = visualization_msgs::msg::Marker::DELETE;
        pubs_[pub_id]->publish(mk);
        return;
    }

    // In ROS2, DELETE+ADD in the same cycle causes visible blink.
    mk.text            = text;
    mk.pose.position.x = pos[0];
    mk.pose.position.y = pos[1];
    mk.pose.position.z = pos[2];
    mk.action          = visualization_msgs::msg::Marker::ADD;
    pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::clearMarker(const string& ns, const int& id, const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector4d(0, 0, 0, 1), ns, id, visualization_msgs::msg::Marker::CUBE_LIST);
    mk.action = visualization_msgs::msg::Marker::DELETE;
    pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
                                        const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::msg::Marker::SPHERE_LIST);
    if (list.empty())
        return;

    fillGeometryInfo(mk, list);
    mk.action = visualization_msgs::msg::Marker::ADD;
    pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
                                      const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::msg::Marker::CUBE_LIST);
    if (list.empty())
        return;

    fillGeometryInfo(mk, list);
    mk.action = visualization_msgs::msg::Marker::ADD;
    pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, const double& scale,
                                      const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::msg::Marker::LINE_LIST);
    if (list1.empty() || list2.empty())
        return;

    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::msg::Marker::ADD;
    pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
                                      const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::msg::Marker::LINE_LIST);
    if (list.size() < 2)
        return;

    // split the single list into two
    vector<Eigen::Vector3d> list1, list2;
    for (int i = 0; i < list.size() - 1; ++i)
    {
        list1.push_back(list[i]);
        list2.push_back(list[i + 1]);
    }

    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::msg::Marker::ADD;
    pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::drawLineStrip(const vector<Eigen::Vector3d>& list, const double& line_width, const Eigen::Vector4d& color, const string& ns,
                                          const int& id, const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(line_width, line_width, line_width), color, ns, id, visualization_msgs::msg::Marker::LINE_STRIP);
    if (list.size() < 2)
        return;

    fillGeometryInfo(mk, list);
    mk.action = visualization_msgs::msg::Marker::ADD;
    pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = node_->now();
    mk.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action          = visualization_msgs::msg::Marker::DELETE;
    mk.id              = id;
    pubs_[pub_id]->publish(mk);

    mk.action             = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        mk.points.push_back(pt);
    }
    pubs_[pub_id]->publish(mk);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = node_->now();
    mk.type            = visualization_msgs::msg::Marker::CUBE_LIST;
    mk.action          = visualization_msgs::msg::Marker::DELETE;
    mk.id              = id;
    pubs_[pub_id]->publish(mk);

    mk.action             = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        mk.points.push_back(pt);
    }
    pubs_[pub_id]->publish(mk);

    rclcpp::sleep_for(std::chrono::milliseconds(1));
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = node_->now();
    mk.type            = visualization_msgs::msg::Marker::LINE_LIST;
    mk.action          = visualization_msgs::msg::Marker::DELETE;
    mk.id              = id;
    pubs_[pub_id]->publish(mk);

    mk.action             = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = line_width;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list1.size()); ++i)
    {
        pt.x = list1[i](0);
        pt.y = list1[i](1);
        pt.z = list1[i](2);
        mk.points.push_back(pt);

        pt.x = list2[i](0);
        pt.y = list2[i](1);
        pt.z = list2[i](2);
        mk.points.push_back(pt);
    }
    pubs_[pub_id]->publish(mk);

    rclcpp::sleep_for(std::chrono::milliseconds(1));
}

void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size)
{
    vector<Eigen::Vector3d> empty;

    for (int i = 0; i < last_bspline_phase1_num_; ++i)
    {
        displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + i % 100);
        displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + i % 100);
    }
    last_bspline_phase1_num_ = bsplines.size();

    for (int i = 0; i < bsplines.size(); ++i)
    {
        drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.2), false, 2 * size, getColor(double(i) / bsplines.size()), i);
    }
}

void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size)
{
    vector<Eigen::Vector3d> empty;

    for (int i = 0; i < last_bspline_phase2_num_; ++i)
    {
        drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i, 0);
        drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i + 50, 0);
        // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + (50 + i) % 100);
        // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + (50 + i) %
        // 100);
    }
    last_bspline_phase2_num_ = bsplines.size();

    for (int i = 0; i < bsplines.size(); ++i)
    {
        drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.6), false, 1.5 * size, getColor(double(i) / bsplines.size()), i);
    }
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, double t_start, const Eigen::Vector3d* anchor_pos)
{
    if (bspline.getControlPoint().size() == 0)
        return;

    vector<Eigen::Vector3d> traj_pts;
    double                  tm, tmp;
    bspline.getTimeSpan(tm, tmp);

    const double t_begin = (t_start >= 0.0) ? std::max(tm, std::min(t_start, tmp)) : tm;
    if (t_begin >= tmp - 1e-6)
    {
        clearMarker("B-Spline", id1, 0);
        return;
    }

    if (anchor_pos != nullptr)
    {
        Eigen::Vector3d pt = *anchor_pos;
        pt.z()             = bspline.evaluateDeBoor(t_begin).z();
        traj_pts.push_back(pt);
    }

    for (double t = t_begin; t <= tmp; t += 0.01)
    {
        traj_pts.push_back(bspline.evaluateDeBoor(t));
    }

    if (traj_pts.size() < 2)
    {
        clearMarker("B-Spline", id1, 0);
        return;
    }

    if (t_start >= 0.0)
        drawLineStrip(traj_pts, size * 2.0, color, "B-Spline", id1, 0);
    else
        drawSpheres(traj_pts, size, color, "B-Spline", id1, 0);

    //！ 改粗细将上面的if else 取消， 采用下面
    // drawSpheres(traj_pts, size, color, "B-Spline", id1, 0);

    // draw the control point
    if (show_ctrl_pts)
    {
        Eigen::MatrixXd         ctrl_pts = bspline.getControlPoint();
        vector<Eigen::Vector3d> ctp;
        for (int i = 0; i < int(ctrl_pts.rows()); ++i)
        {
            Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
            ctp.push_back(pt);
        }
        // displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
        drawSpheres(ctp, size2, color2, "B-Spline", id1 + 50, 0);
    }
}

void PlanningVisualization::drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width, const Eigen::Vector4d& color1,
                                          const Eigen::Vector4d& color2, const Eigen::Vector4d& color3, int id)
{
    // clear exsiting node and edge (drawn last time)
    vector<Eigen::Vector3d> empty;
    displaySphereList(empty, point_size, color1, GRAPH_NODE, 1);
    displaySphereList(empty, point_size, color1, GRAPH_NODE + 50, 1);
    displayLineList(empty, empty, line_width, color3, GRAPH_EDGE, 1);

    /* draw graph node */
    vector<Eigen::Vector3d> guards, connectors;
    for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter)
    {
        if ((*iter)->type_ == GraphNode::Guard)
        {
            guards.push_back((*iter)->pos_);
        }
        else if ((*iter)->type_ == GraphNode::Connector)
        {
            connectors.push_back((*iter)->pos_);
        }
    }
    displaySphereList(guards, point_size, color1, GRAPH_NODE, 1);
    displaySphereList(connectors, point_size, color2, GRAPH_NODE + 50, 1);

    /* draw graph edge */
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;
    for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter)
    {
        for (int k = 0; k < (*iter)->neighbors_.size(); ++k)
        {
            edge_pt1.push_back((*iter)->pos_);
            edge_pt2.push_back((*iter)->neighbors_[k]->pos_);
        }
    }
    displayLineList(edge_pt1, edge_pt2, line_width, color3, GRAPH_EDGE, 1);
}

void PlanningVisualization::drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width)
{
    // clear drawn paths
    Eigen::Vector4d color1(1, 1, 1, 1);
    for (int i = 0; i < last_topo_path1_num_; ++i)
    {
        vector<Eigen::Vector3d> empty;
        displayLineList(empty, empty, line_width, color1, SELECT_PATH + i % 100, 1);
        displaySphereList(empty, line_width, color1, PATH + i % 100, 1);
    }

    last_topo_path1_num_ = paths.size();

    // draw new paths
    for (int i = 0; i < paths.size(); ++i)
    {
        vector<Eigen::Vector3d> edge_pt1, edge_pt2;

        for (int j = 0; j < paths[i].size() - 1; ++j)
        {
            edge_pt1.push_back(paths[i][j]);
            edge_pt2.push_back(paths[i][j + 1]);
        }

        displayLineList(edge_pt1, edge_pt2, line_width, getColor(double(i) / (last_topo_path1_num_)), SELECT_PATH + i % 100, 1);
    }
}

void PlanningVisualization::drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double size)
{
    // clear drawn paths
    Eigen::Vector4d color1(1, 1, 1, 1);
    for (int i = 0; i < last_topo_path2_num_; ++i)
    {
        vector<Eigen::Vector3d> empty;
        displayLineList(empty, empty, size, color1, FILTERED_PATH + i % 100, 1);
    }

    last_topo_path2_num_ = paths.size();

    // draw new paths
    for (int i = 0; i < paths.size(); ++i)
    {
        vector<Eigen::Vector3d> edge_pt1, edge_pt2;

        for (int j = 0; j < paths[i].size() - 1; ++j)
        {
            edge_pt1.push_back(paths[i][j]);
            edge_pt2.push_back(paths[i][j + 1]);
        }

        displayLineList(edge_pt1, edge_pt2, size, getColor(double(i) / (last_topo_path2_num_), 0.2), FILTERED_PATH + i % 100, 1);
    }
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id)
{
    vector<Eigen::Vector3d> goal_vec = { goal };
    displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution, const Eigen::Vector4d& color, int id)
{
    displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color, int id)
{
    vector<Eigen::Vector3d> poly_pts;
    poly_traj.getSamplePoints(poly_pts);
    displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

void PlanningVisualization::drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id)
{
    const auto   time_now   = node_->now();
    double       start_time = (time_now - ObjHistory::global_start_time_).seconds();
    const double range      = 5.6;

    vector<Eigen::Vector3d> traj;
    for (int i = 0; i < pred->size(); i++)
    {
        PolynomialPrediction poly = pred->at(i);
        if (!poly.valid())
            continue;

        for (double t = start_time; t <= start_time + range; t += 0.8)
        {
            Eigen::Vector3d pt = poly.evaluateConstVel(t);
            traj.push_back(pt);
        }
    }
    displaySphereList(traj, resolution, color, id % 100, 2);
}

void PlanningVisualization::drawVisibConstraint(const Eigen::MatrixXd& ctrl_pts, const vector<Eigen::Vector3d>& block_pts)
{
    int visible_num = ctrl_pts.rows() - block_pts.size();

    /* draw block points, their projection rays and visible pairs */
    vector<Eigen::Vector3d> pts1, pts2, pts3, pts4;
    int                     n = ctrl_pts.rows() - visible_num;

    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector3d qb = block_pts[i];

        if (fabs(qb[2] + 10086) > 1e-3)
        {
            // compute the projection
            Eigen::Vector3d qi  = ctrl_pts.row(i);
            Eigen::Vector3d qj  = ctrl_pts.row(i + visible_num);
            Eigen::Vector3d dir = (qj - qi).normalized();
            Eigen::Vector3d qp  = qi + dir * ((qb - qi).dot(dir));

            pts1.push_back(qb);
            pts2.push_back(qp);
            pts3.push_back(qi);
            pts4.push_back(qj);
        }
    }

    displayCubeList(pts1, 0.1, Eigen::Vector4d(1, 1, 0, 1), 0, 3);
    displaySphereList(pts4, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, 3);
    displayLineList(pts1, pts2, 0.015, Eigen::Vector4d(0, 1, 1, 1), 2, 3);
    displayLineList(pts3, pts4, 0.015, Eigen::Vector4d(0, 1, 0, 1), 3, 3);
}

void PlanningVisualization::drawVisibConstraint(const Eigen::MatrixXd& pts, const vector<VisiblePair>& pairs)
{
    vector<Eigen::Vector3d> pts1, pts2, pts3, pts4;
    for (auto pr : pairs)
    {
        Eigen::Vector3d qb  = pr.qb_;
        Eigen::Vector3d qi  = pts.row(pr.from_);
        Eigen::Vector3d qj  = pts.row(pr.to_);
        Eigen::Vector3d dir = (qj - qi).normalized();
        Eigen::Vector3d qp  = qi + dir * ((qb - qi).dot(dir));
        pts1.push_back(qb);
        pts2.push_back(qp);
        pts3.push_back(qi);
        pts4.push_back(qj);
    }
    displayCubeList(pts1, 0.1, Eigen::Vector4d(1, 1, 0, 1), 0, 3);
    displaySphereList(pts4, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, 3);
    displayLineList(pts1, pts2, 0.015, Eigen::Vector4d(0, 1, 1, 1), 2, 3);
    displayLineList(pts3, pts4, 0.015, Eigen::Vector4d(0, 1, 0, 1), 3, 3);
}

void PlanningVisualization::drawViewConstraint(const ViewConstraint& vc)
{
    if (!isPublisherEnabled(3))
        return;

    if (vc.idx_ < 0)
        return;
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id    = "world";
    mk.header.stamp       = node_->now();
    mk.id                 = 0;
    mk.type               = visualization_msgs::msg::Marker::ARROW;
    mk.action             = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x            = 0.1;
    mk.scale.y            = 0.2;
    mk.scale.z            = 0.3;
    mk.color.r            = 1.0;
    mk.color.g            = 0.5;
    mk.color.b            = 0.0;
    mk.color.a            = 1.0;

    geometry_msgs::msg::Point pt;
    pt.x = vc.pt_[0];
    pt.y = vc.pt_[1];
    pt.z = vc.pt_[2];
    mk.points.push_back(pt);
    pt.x = vc.pt_[0] + vc.dir_[0];
    pt.y = vc.pt_[1] + vc.dir_[1];
    pt.z = vc.pt_[2] + vc.dir_[2];
    mk.points.push_back(pt);
    pubs_[3]->publish(mk);

    vector<Eigen::Vector3d> pts = { vc.pcons_ };
    displaySphereList(pts, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, 3);
}

void PlanningVisualization::drawFrontier(const vector<vector<Eigen::Vector3d>>& frontiers)
{
    drawFrontierMerged(frontiers, 0.1, "frontier", 0, 4);
}

void PlanningVisualization::drawFrontierMerged(const vector<vector<Eigen::Vector3d>>& frontiers, const double& scale, const string& ns, const int& id,
                                               const int& pub_id)
{
    if (!isPublisherEnabled(pub_id))
        return;

    visualization_msgs::msg::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), Eigen::Vector4d(1, 1, 1, 1), ns, id, visualization_msgs::msg::Marker::CUBE_LIST);

    size_t total_pts = 0;
    for (const auto& cluster : frontiers)
    {
        total_pts += cluster.size();
    }

    if (total_pts == 0)
    {
        clearMarker(ns, id, pub_id);
    }
    else
    {
        std::vector<std::pair<Eigen::Vector3d, std_msgs::msg::ColorRGBA>> display_samples;
        display_samples.reserve(std::min(total_pts, static_cast<size_t>(frontier_vis_max_points_)));

        const size_t denom     = std::max<size_t>(frontiers.size(), 1);
        const bool   use_voxel = frontier_vis_voxel_size_ > 1e-9;
        const double inv_voxel = use_voxel ? 1.0 / frontier_vis_voxel_size_ : 0.0;
        std::unordered_map<VoxelIdx, std::pair<Eigen::Vector3d, std_msgs::msg::ColorRGBA>, VoxelIdxHash> voxel_map;

        for (size_t i = 0; i < frontiers.size(); ++i)
        {
            const auto               vivid_color = getColor(static_cast<double>(i) / static_cast<double>(denom), 1.0);
            const Eigen::Vector3d    soft_rgb    = 0.45 * Eigen::Vector3d(1.0, 1.0, 1.0) + 0.55 * vivid_color.head<3>();
            std_msgs::msg::ColorRGBA c;
            c.r = soft_rgb(0);
            c.g = soft_rgb(1);
            c.b = soft_rgb(2);
            c.a = 0.55;

            for (const auto& pt_eigen : frontiers[i])
            {
                if (use_voxel)
                {
                    const VoxelIdx key = PointToVoxel(pt_eigen, inv_voxel);
                    voxel_map.emplace(key, std::make_pair(pt_eigen, c));
                }
                else
                {
                    display_samples.emplace_back(pt_eigen, c);
                }
            }
        }

        if (use_voxel)
        {
            display_samples.reserve(voxel_map.size());
            for (const auto& kv : voxel_map)
            {
                display_samples.push_back(kv.second);
            }
        }

        if (display_samples.size() > static_cast<size_t>(frontier_vis_max_points_))
        {
            const size_t                                                      n    = display_samples.size();
            const size_t                                                      cap  = static_cast<size_t>(frontier_vis_max_points_);
            const size_t                                                      step = (n + cap - 1) / cap;
            std::vector<std::pair<Eigen::Vector3d, std_msgs::msg::ColorRGBA>> thinned;
            thinned.reserve(cap);
            for (size_t j = 0; j < n; j += step)
            {
                thinned.push_back(display_samples[j]);
            }
            display_samples.swap(thinned);
        }

        mk.points.reserve(display_samples.size());
        mk.colors.reserve(display_samples.size());
        for (const auto& sample : display_samples)
        {
            geometry_msgs::msg::Point pt;
            pt.x = sample.first(0);
            pt.y = sample.first(1);
            pt.z = sample.first(2);
            mk.points.push_back(pt);
            mk.colors.push_back(sample.second);
        }

        mk.action = visualization_msgs::msg::Marker::ADD;
        pubs_[pub_id]->publish(mk);
    }

    // Clear legacy per-id markers from previous multi-id publishing.
    for (int legacy_id = 1; legacy_id <= last_frontier_num_; ++legacy_id)
    {
        clearMarker(ns, legacy_id, pub_id);
    }
    last_frontier_num_ = 1;
}

void PlanningVisualization::drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt)
{
    double                  duration = pos.getTimeSum();
    vector<Eigen::Vector3d> pts1, pts2;

    for (double tc = 0.0; tc <= duration + 1e-3; tc += dt)
    {
        Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
        pc[2] += 0.15;
        double          yc = yaw.evaluateDeBoorT(tc)[0];
        Eigen::Vector3d dir(cos(yc), sin(yc), 0);
        Eigen::Vector3d pdir = pc + 1.0 * dir;
        pts1.push_back(pc);
        pts2.push_back(pdir);
    }
    displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), 0, 5);
}

void PlanningVisualization::drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt)
{
    vector<Eigen::Vector3d> pts1, pts2;

    for (int i = 0; i < yaw.size(); ++i)
    {
        Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
        pc[2] += 0.3;
        Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
        Eigen::Vector3d pdir = pc + 1.0 * dir;
        pts1.push_back(pc);
        pts2.push_back(pdir);
    }
    displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), 1, 5);
}

Eigen::Vector4d PlanningVisualization::getSwarmDroneColor(int drone_id, double alpha)
{
    static const Eigen::Vector3d palette[] = {
        { 0.95, 0.42, 0.38 },  // bright coral
        { 0.30, 0.62, 0.95 },  // sky blue
        { 0.28, 0.82, 0.52 },  // fresh green
        { 0.22, 0.82, 0.80 },  // cyan
        { 0.62, 0.48, 0.92 },  // violet
        { 0.95, 0.72, 0.28 },  // amber
    };
    const int   n   = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
    const int   idx = n > 0 ? ((drone_id - 1) % n + n) % n : 0;
    const auto& c   = palette[idx];
    return Eigen::Vector4d(c.x(), c.y(), c.z(), alpha);
}

Eigen::Vector4d PlanningVisualization::getColor(const double& h, double alpha)
{
    double h1 = h;
    if (h1 < 0.0 || h1 > 1.0)
    {
        std::cout << "h out of range" << std::endl;
        h1 = 0.0;
    }

    double          lambda;
    Eigen::Vector4d color1, color2;
    if (h1 >= -1e-4 && h1 < 1.0 / 6)
    {
        lambda = (h1 - 0.0) * 6;
        color1 = Eigen::Vector4d(1, 0, 0, 1);
        color2 = Eigen::Vector4d(1, 0, 1, 1);
    }
    else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6)
    {
        lambda = (h1 - 1.0 / 6) * 6;
        color1 = Eigen::Vector4d(1, 0, 1, 1);
        color2 = Eigen::Vector4d(0, 0, 1, 1);
    }
    else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6)
    {
        lambda = (h1 - 2.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 0, 1, 1);
        color2 = Eigen::Vector4d(0, 1, 1, 1);
    }
    else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6)
    {
        lambda = (h1 - 3.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 1, 1, 1);
        color2 = Eigen::Vector4d(0, 1, 0, 1);
    }
    else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6)
    {
        lambda = (h1 - 4.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 1, 0, 1);
        color2 = Eigen::Vector4d(1, 1, 0, 1);
    }
    else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4)
    {
        lambda = (h1 - 5.0 / 6) * 6;
        color1 = Eigen::Vector4d(1, 1, 0, 1);
        color2 = Eigen::Vector4d(1, 0, 0, 1);
    }

    Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
    fcolor(3)              = alpha;

    return fcolor;
}
// PlanningVisualization::
}  // namespace fast_planner