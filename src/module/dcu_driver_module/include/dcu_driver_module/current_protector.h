#pragma once
#include <map>
#include <deque>
#include <string>
#include <chrono>

class CurrentProtector {
public:
  CurrentProtector(float threshold, float overlimit_ratio, int window_ms)
      : threshold_(threshold),
        overlimit_ratio_(overlimit_ratio),
        window_(std::chrono::milliseconds(window_ms)) {}

  bool Update(const std::string& name, float current);
  bool IsDisabled(const std::string& name) const;
  void Reset(const std::string& name);
  void LogParams() const;

private:
  struct Entry {
    std::deque<std::pair<std::chrono::steady_clock::time_point, float>> history;
    std::chrono::steady_clock::time_point last_disabled;
    bool is_disabled = false;
  };

  std::map<std::string, Entry> data_;
  float threshold_;
  float overlimit_ratio_;
  std::chrono::milliseconds window_;
};