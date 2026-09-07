#pragma once

#include "rust/cxx.h"

#include <cstdint>
#include <memory>
#include <string>

namespace fusion {

struct PlanarConfig;
struct PlanarEstimate;
struct GpsUpdateResult;
enum class GpsUpdate : std::uint8_t;

class GtsamEkfEstimator {
 public:
  explicit GtsamEkfEstimator(const PlanarConfig& config);
  ~GtsamEkfEstimator();

  void process_imu(std::int64_t measurement_time_ns, double yaw_rate_radps,
                   double forward_acceleration_mps2);
  GpsUpdateResult process_gps(double position_world_x_m,
                              double position_world_y_m,
                              double horizontal_position_variance_m2);
  PlanarEstimate estimate() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

std::unique_ptr<GtsamEkfEstimator> new_gtsam_ekf_estimator(
    const PlanarConfig& config);
const std::string& gtsam_version();

}  // namespace fusion
