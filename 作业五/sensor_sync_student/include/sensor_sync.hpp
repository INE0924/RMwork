#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

enum class SensorType { CAMERA, IMU };

struct SensorData {
  SensorType type;
  std::uint32_t sequence;
  std::uint64_t timestamp_us;
};

struct SyncResult {
  SensorData camera;
  std::optional<SensorData> imu;
  std::uint64_t delta_us = 0;
};

// Sort both streams and scan IMU timestamps only once (two-pointer method).
// Complexity: O(C log C + I log I + C + I); memory: O(C + I).
inline std::vector<SyncResult> synchronize(
    std::vector<SensorData> cameras,
    std::vector<SensorData> imus,
    std::uint64_t max_delta_us) {
  const auto order = [](const SensorData& a, const SensorData& b) {
    if (a.timestamp_us != b.timestamp_us)
      return a.timestamp_us < b.timestamp_us;
    return a.sequence < b.sequence;
  };
  std::sort(cameras.begin(), cameras.end(), order);
  std::sort(imus.begin(), imus.end(), order);

  // For duplicate IMU timestamps, the smallest sequence wins.
  std::vector<SensorData> unique_imus;
  unique_imus.reserve(imus.size());
  for (const auto& imu : imus) {
    if (unique_imus.empty() ||
        unique_imus.back().timestamp_us != imu.timestamp_us) {
      unique_imus.push_back(imu);
    }
  }

  std::vector<SyncResult> results;
  results.reserve(cameras.size());
  std::size_t right = 0;  // First IMU timestamp >= the current camera timestamp.

  for (const auto& camera : cameras) {
    while (right < unique_imus.size() &&
           unique_imus[right].timestamp_us < camera.timestamp_us) {
      ++right;
    }

    const SensorData* best = nullptr;
    std::uint64_t best_delta = 0;

    if (right > 0) {
      best = &unique_imus[right - 1];
      best_delta = camera.timestamp_us - best->timestamp_us;
    }
    if (right < unique_imus.size()) {
      const auto& candidate = unique_imus[right];
      const std::uint64_t delta = candidate.timestamp_us - camera.timestamp_us;
      // Strictly less: if distances tie, keep the earlier IMU timestamp.
      if (best == nullptr || delta < best_delta) {
        best = &candidate;
        best_delta = delta;
      }
    }

    SyncResult result{camera, std::nullopt, 0};
    if (best != nullptr && best_delta <= max_delta_us) {
      result.imu = *best;
      result.delta_us = best_delta;
    }
    results.push_back(result);
  }
  return results;
}
