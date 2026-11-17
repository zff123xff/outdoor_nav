#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

#include <lkh_tsp_solver/lkh_interface.h>
#include <lkh_tsp_solver/srv/solve_tsp.hpp>

std::string tsp_dir_;
int drone_id_;

void tspCallback(
  const std::shared_ptr<lkh_tsp_solver::srv::SolveTSP::Request> req,
  std::shared_ptr<lkh_tsp_solver::srv::SolveTSP::Response> resp)
{
  solveTSPLKH(tsp_dir_.c_str());
  RCLCPP_WARN(rclcpp::get_logger("tsp_server"), "TSP server %d finish", drone_id_);
  
  // TODO: Set response to indicate the service call is complete
  // resp->empty = 0;  // Just set to any value to complete the response
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tsp_node");

  // Read mtsp file dir
  node->declare_parameter<std::string>("exploration.tsp_dir", "null");
  node->declare_parameter<int>("exploration.drone_id", 1);
  
  node->get_parameter("exploration.tsp_dir", tsp_dir_);
  node->get_parameter("exploration.drone_id", drone_id_);

  tsp_dir_ = tsp_dir_ + "/drone_" + std::to_string(drone_id_) + ".par";

  auto tsp_server = node->create_service<lkh_tsp_solver::srv::SolveTSP>(
    "/solve_tsp_" + std::to_string(drone_id_), tspCallback);

  RCLCPP_WARN(rclcpp::get_logger("tsp_server"), "TSP server %d is ready.", drone_id_);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
