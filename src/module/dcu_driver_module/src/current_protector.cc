
#include "current_protector.h"
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
        std::cerr << "[WARN] Overcurrent protection triggered for " << name << std::endl;
        return true;
    }

    if (entry.is_disabled && now - entry.last_disabled >= cooldown_ && current < threshold_ * margin_) {
        entry.is_disabled = false;
        std::cerr << "[INFO] Auto-recovered actuator " << name << std::endl;
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

std::map<std::string, float> CurrentProtector::GetCurrentMap() const {
    std::map<std::string, float> result;
    for (const auto& [name, entry] : data_) {
        if (!entry.history.empty())
            result[name] = entry.history.back().second;
    }
    return result;
}
