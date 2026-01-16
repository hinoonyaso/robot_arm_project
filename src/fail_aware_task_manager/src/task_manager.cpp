#include "fail_aware_task_manager/task_manager.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace fail_aware_task_manager {
namespace {

std::string to_string(TaskState state) {
  switch (state) {
    case TaskState::IDLE:
      return "IDLE";
    case TaskState::SPAWN_OR_SELECT_OBJECT:
      return "SPAWN_OR_SELECT_OBJECT";
    case TaskState::PLAN_APPROACH:
      return "PLAN_APPROACH";
    case TaskState::EXEC_APPROACH:
      return "EXEC_APPROACH";
    case TaskState::PLAN_GRASP:
      return "PLAN_GRASP";
    case TaskState::EXEC_GRASP:
      return "EXEC_GRASP";
    case TaskState::ATTACH:
      return "ATTACH";
    case TaskState::PLAN_PLACE:
      return "PLAN_PLACE";
    case TaskState::EXEC_PLACE:
      return "EXEC_PLACE";
    case TaskState::DETACH:
      return "DETACH";
    case TaskState::SUCCESS:
      return "SUCCESS";
    case TaskState::RECOVERY:
      return "RECOVERY";
    case TaskState::FAIL_SAFE:
      return "FAIL_SAFE";
  }
  return "UNKNOWN";
}

std::string now_string() {
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%dT%H:%M:%S");
  return ss.str();
}

}  // namespace

TaskManager::TaskManager(const rclcpp::NodeOptions &options)
    : Node("fail_aware_task_manager", options),
      state_(TaskState::IDLE),
      retry_count_(0),
      max_retries_(5),
      run_id_(declare_parameter<std::string>("run_id", "run_local")),
      log_dir_(declare_parameter<std::string>("log_dir", "data/runs")) {
  auto log_path = log_dir_ + "/" + run_id_ + "/events.jsonl";
  event_log_.open(log_path, std::ios::app);
  if (!event_log_) {
    RCLCPP_WARN(get_logger(), "Unable to open log file: %s", log_path.c_str());
  }

  current_recovery_ = {"Z_OFFSET", 0.03, 0.0, 0.8, 2.0};

  tick_timer_ = create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&TaskManager::on_tick, this));
  enter_state(TaskState::SPAWN_OR_SELECT_OBJECT);
}

void TaskManager::enter_state(TaskState next_state) {
  log_event("state_exit", {{"state", to_string(state_)}});
  state_ = next_state;
  log_event("state_enter", {{"state", to_string(state_)}});
}

void TaskManager::log_event(const std::string &event_type,
                            const std::unordered_map<std::string, std::string> &fields) {
  if (!event_log_) {
    return;
  }
  event_log_ << "{\"timestamp\":\"" << now_string() << "\"";
  event_log_ << ",\"event\":\"" << event_type << "\"";
  for (const auto &kv : fields) {
    event_log_ << ",\"" << kv.first << "\":\"" << kv.second << "\"";
  }
  event_log_ << "}\n";
  event_log_.flush();
}

void TaskManager::rotate_recovery_action() {
  if (current_recovery_.name == "Z_OFFSET") {
    current_recovery_ = {"YAW_JITTER", 0.0, 0.15, 0.8, 2.5};
  } else if (current_recovery_.name == "YAW_JITTER") {
    current_recovery_ = {"TIMEOUT_UP", 0.0, 0.0, 0.9, 5.0};
  } else {
    current_recovery_ = {"SPEED_DOWN", 0.0, 0.0, 0.5, 4.0};
  }
}

void TaskManager::on_tick() {
  switch (state_) {
    case TaskState::SPAWN_OR_SELECT_OBJECT:
      enter_state(TaskState::PLAN_APPROACH);
      break;
    case TaskState::PLAN_APPROACH:
      log_event("plan_success", {{"planning_time_ms", "120"}});
      enter_state(TaskState::EXEC_APPROACH);
      break;
    case TaskState::EXEC_APPROACH:
      log_event("exec_success", {{"exec_time_ms", "540"}});
      enter_state(TaskState::PLAN_GRASP);
      break;
    case TaskState::PLAN_GRASP:
      log_event("plan_fail", {{"failure_type", "IK"}});
      enter_state(TaskState::RECOVERY);
      break;
    case TaskState::RECOVERY:
      retry_count_++;
      log_event("recovery_action", {{"action", current_recovery_.name},
                                    {"retry_count", std::to_string(retry_count_)}});
      if (retry_count_ >= max_retries_) {
        enter_state(TaskState::FAIL_SAFE);
      } else {
        rotate_recovery_action();
        enter_state(TaskState::PLAN_GRASP);
      }
      break;
    case TaskState::FAIL_SAFE:
      log_event("fail_safe", {{"reason", "retries_exceeded"}});
      tick_timer_->cancel();
      break;
    case TaskState::SUCCESS:
    case TaskState::PLAN_PLACE:
    case TaskState::EXEC_PLACE:
    case TaskState::EXEC_GRASP:
    case TaskState::ATTACH:
    case TaskState::DETACH:
    case TaskState::IDLE:
      break;
  }
}

}  // namespace fail_aware_task_manager

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fail_aware_task_manager::TaskManager>());
  rclcpp::shutdown();
  return 0;
}
