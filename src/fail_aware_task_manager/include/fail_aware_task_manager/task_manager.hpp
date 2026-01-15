#pragma once

#include <fstream>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

namespace fail_aware_task_manager {

enum class TaskState {
  IDLE,
  SPAWN_OR_SELECT_OBJECT,
  PLAN_APPROACH,
  EXEC_APPROACH,
  PLAN_GRASP,
  EXEC_GRASP,
  ATTACH,
  PLAN_PLACE,
  EXEC_PLACE,
  DETACH,
  SUCCESS,
  RECOVERY,
  FAIL_SAFE
};

struct RecoveryAction {
  std::string name;
  double z_offset;
  double yaw_jitter;
  double speed_scale;
  double planning_timeout;
};

class TaskManager : public rclcpp::Node {
public:
  explicit TaskManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void enter_state(TaskState next_state);
  void on_tick();
  void log_event(const std::string &event_type,
                 const std::unordered_map<std::string, std::string> &fields = {});
  void rotate_recovery_action();

  TaskState state_;
  std::size_t retry_count_;
  std::size_t max_retries_;
  std::ofstream event_log_;
  std::string run_id_;
  std::string log_dir_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  RecoveryAction current_recovery_;
};

}  // namespace fail_aware_task_manager
