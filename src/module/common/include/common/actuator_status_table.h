#pragma once
#include <string>
#include <map>
#include <mutex>

struct ActuatorStatus {
  bool is_disabled = false;
  float current = 0.0f;
};

class ActuatorStatusTable {
public:
  static ActuatorStatusTable& Instance() {
    static ActuatorStatusTable instance;
    return instance;
  }

  void SetStatus(const std::string& name, bool disabled, float current) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_map_[name] = {disabled, current};
  }

  std::map<std::string, ActuatorStatus> GetSnapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_map_;
  }

private:
  std::map<std::string, ActuatorStatus> status_map_;
  std::mutex mutex_;
};
