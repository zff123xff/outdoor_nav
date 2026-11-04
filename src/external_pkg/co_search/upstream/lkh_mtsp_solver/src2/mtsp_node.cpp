#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <sys/wait.h>
#include <unistd.h>

#include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_mtsp_solver/srv/solve_mtsp.hpp>

using std::string;

std::string mtsp_dir1_;
std::string mtsp_dir2_;
std::string mtsp_dir3_;
std::string lkh_binary_path_;
bool lkh_binary_available_ = false;
bool lkh_unavailable_logged_ = false;
int drone_id_, problem_id_;

bool ResolveExecutablePath(const std::string& bin_name, std::string* resolved_path)
{
  if (bin_name.empty()) {
    return false;
  }

  if (bin_name.find('/') != std::string::npos) {
    if (access(bin_name.c_str(), X_OK) == 0) {
      *resolved_path = bin_name;
      return true;
    }
    return false;
  }

  const char* path_env = std::getenv("PATH");
  if (path_env == nullptr) {
    return false;
  }

  const std::string path_list(path_env);
  size_t start = 0;
  while (start <= path_list.size()) {
    const size_t sep = path_list.find(':', start);
    const std::string dir = sep == std::string::npos
                              ? path_list.substr(start)
                              : path_list.substr(start, sep - start);
    const std::string candidate = dir + "/" + bin_name;
    if (access(candidate.c_str(), X_OK) == 0) {
      *resolved_path = candidate;
      return true;
    }
    if (sep == std::string::npos) {
      break;
    }
    start = sep + 1;
  }

  return false;
}

const char* SignalName(int sig)
{
  switch (sig) {
    case SIGABRT: return "SIGABRT";
    case SIGSEGV: return "SIGSEGV";
    case SIGFPE: return "SIGFPE";
    case SIGILL: return "SIGILL";
    case SIGBUS: return "SIGBUS";
    case SIGKILL: return "SIGKILL";
    case SIGTERM: return "SIGTERM";
    case SIGINT: return "SIGINT";
    default: return "UNKNOWN";
  }
}

void LogSystemCallFailure(const int status, const std::string& par_path)
{
  auto logger = rclcpp::get_logger("mtsp_server");
  if (status == -1) {
    RCLCPP_ERROR(
      logger,
      "External LKH command failed to launch, errno=%d (%s), par=%s",
      errno, std::strerror(errno), par_path.c_str());
    return;
  }

  if (WIFEXITED(status)) {
    const int exit_code = WEXITSTATUS(status);
    RCLCPP_ERROR(
      logger,
      "External LKH command exited abnormally, wait_status=%d, exit_code=%d, par=%s",
      status, exit_code, par_path.c_str());
    return;
  }

  if (WIFSIGNALED(status)) {
    const int signal_num = WTERMSIG(status);
    RCLCPP_ERROR(
      logger,
      "External LKH command terminated by signal, wait_status=%d, signal=%d(%s), core_dump=%s, par=%s",
      status, signal_num, SignalName(signal_num),
      WCOREDUMP(status) ? "true" : "false", par_path.c_str());
    return;
  }

  RCLCPP_ERROR(
    logger,
    "External LKH command failed with unknown status, wait_status=%d, par=%s",
    status, par_path.c_str());
}

void mtspCallback(
  const std::shared_ptr<lkh_mtsp_solver::srv::SolveMTSP::Request> req,
  std::shared_ptr<lkh_mtsp_solver::srv::SolveMTSP::Response> resp)
{
  // Log that the request was received
  RCLCPP_INFO(rclcpp::get_logger("mtsp_server"), "MTSP server %d received prob %d", drone_id_, req->prob);

  // Execute the computation based on the problem type
  int result = 0;
  if (req->prob == 1) {
    RCLCPP_INFO(rclcpp::get_logger("mtsp_server"), "Solving MTSP problem 1 with file: %s", mtsp_dir1_.c_str());
    result = solveMTSPWithLKH3(mtsp_dir1_.c_str());
  }
  else if (req->prob == 2) {
    RCLCPP_INFO(rclcpp::get_logger("mtsp_server"), "Solving MTSP problem 2 with file: %s", mtsp_dir2_.c_str());
    result = solveMTSPWithLKH3(mtsp_dir2_.c_str());
  }
  else if (req->prob == 3) {
    if (lkh_binary_available_) {
      string cmd = lkh_binary_path_ + " " + mtsp_dir3_;
      RCLCPP_INFO(
        rclcpp::get_logger("mtsp_server"),
        "Solving MTSP problem 3 with external LKH command: %s",
        cmd.c_str());
      result = system(cmd.c_str());
      if (result != 0) {
        LogSystemCallFailure(result, mtsp_dir3_);
      }
    } else {
      result = -1;
      if (!lkh_unavailable_logged_) {
        RCLCPP_ERROR(
          rclcpp::get_logger("mtsp_server"),
          "External LKH binary is unavailable. configured=%s, par=%s",
          lkh_binary_path_.c_str(), mtsp_dir3_.c_str());
        lkh_unavailable_logged_ = true;
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("mtsp_server"), "MTSP server %d finished prob %d, result: %d", drone_id_, req->prob, result);

  // Set response to indicate the service call is complete
  resp->empty = static_cast<uint8_t>(result);  // Return the result of the computation
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mtsp_node");

  // Read mtsp file dir
  node->declare_parameter<std::string>("exploration/mtsp_dir", "null");
  node->declare_parameter<int>("exploration/drone_id", 1);
  node->declare_parameter<int>("exploration/problem_id", 1);
  node->declare_parameter<std::string>("exploration/lkh_binary", "LKH");
  
  node->get_parameter("exploration/mtsp_dir", mtsp_dir1_);
  node->get_parameter("exploration/drone_id", drone_id_);
  node->get_parameter("exploration/problem_id", problem_id_);
  node->get_parameter("exploration/lkh_binary", lkh_binary_path_);

  std::string base_path = mtsp_dir1_;
  mtsp_dir1_ = base_path + "/amtsp_" + std::to_string(drone_id_) + ".par";
  mtsp_dir2_ = base_path + "/amtsp2_" + std::to_string(drone_id_) + ".par";
  mtsp_dir3_ = base_path + "/amtsp3_" + std::to_string(drone_id_) + ".par";

  std::string resolved_lkh_binary;
  lkh_binary_available_ = ResolveExecutablePath(lkh_binary_path_, &resolved_lkh_binary);
  if (lkh_binary_available_) {
    lkh_binary_path_ = resolved_lkh_binary;
    RCLCPP_INFO(
      rclcpp::get_logger("mtsp_server"),
      "Resolved LKH binary for drone %d: %s",
      drone_id_, lkh_binary_path_.c_str());
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("mtsp_server"),
      "Cannot resolve LKH binary for drone %d, configured=%s",
      drone_id_, lkh_binary_path_.c_str());
  }

  string service_name;
  if (problem_id_ == 1) {  // TSP
    service_name = "/solve_tsp_" + std::to_string(drone_id_);
  } else if (problem_id_ == 2) {  // ACVRP
    service_name = "/solve_acvrp_" + std::to_string(drone_id_);
  }
  
  auto mtsp_server = node->create_service<lkh_mtsp_solver::srv::SolveMTSP>(
    service_name, mtspCallback);

  RCLCPP_WARN(rclcpp::get_logger("mtsp_server"), "MTSP server %d is ready.", drone_id_);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
