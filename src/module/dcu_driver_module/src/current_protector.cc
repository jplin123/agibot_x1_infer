#include "dcu_driver_module/current_protector.h"
#include <algorithm>
#include <iostream>

bool CurrentProtector::Update(const std::string& name, float current) {
  auto now = std::chrono::steady_clock::now();
  auto& entry = data_[name];
  entry.history.emplace_back(now, current);

  while (!entry.history.empty() && now - entry.history.front().first > window_) {
    entry.history.pop_front();
  }

  int total = entry.history.size();
  int over = std::count_if(entry.history.begin(), entry.history.end(), [&](auto& p) {
    return p.second > threshold_;
  });

  if (!entry.is_disabled && total >= 5 && static_cast<float>(over) / total >= overlimit_ratio_) {
    entry.last_disabled = now;
    entry.is_disabled = true;
    std::cerr << "[PROTECTOR] Disabling " << name << " due to sustained overcurrent\n";
    return true;
  }

  return entry.is_disabled;
}

bool CurrentProtector::IsDisabled(const std::string& name) const {
  auto it = data_.find(name);
  return it != data_.end() && it->second.is_disabled;
}

void CurrentProtector::Reset(const std::string& name) {
  data_.erase(name);
}