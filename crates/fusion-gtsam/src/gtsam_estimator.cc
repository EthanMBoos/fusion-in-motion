#include "fusion-gtsam/src/lib.rs.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/config.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fusion {
namespace {

constexpr int kStateDimension = 6;
constexpr int kPositionX = 0;
constexpr int kPositionY = 1;
constexpr int kYaw = 2;
constexpr int kSpeed = 3;
constexpr int kGyroBias = 4;
constexpr int kAccelBias = 5;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

using StateVector = Eigen::Matrix<double, kStateDimension, 1>;
using StateMatrix = Eigen::Matrix<double, kStateDimension, kStateDimension>;

double wrap_angle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTau = 2.0 * kPi;
  double wrapped = std::fmod(angle + kPi, kTau);
  if (wrapped < 0.0) {
    wrapped += kTau;
  }
  wrapped -= kPi;
  return wrapped == -kPi && angle > 0.0 ? kPi : wrapped;
}

class PlanarState {
 public:
  enum { dimension = kStateDimension };

  PlanarState() : values_(StateVector::Zero()) {}
  explicit PlanarState(StateVector values) : values_(std::move(values)) {
    values_[kYaw] = wrap_angle(values_[kYaw]);
  }

  const StateVector& values() const { return values_; }

  void print(const std::string& label = "") const {
    gtsam::print(gtsam::Vector(values_), label);
  }

  bool equals(const PlanarState& other, double tolerance = 1e-9) const {
    return localCoordinates(other).norm() <= tolerance;
  }

  PlanarState retract(const StateVector& correction) const {
    return PlanarState(values_ + correction);
  }

  StateVector localCoordinates(const PlanarState& other) const {
    StateVector difference = other.values_ - values_;
    difference[kYaw] = wrap_angle(difference[kYaw]);
    return difference;
  }

 private:
  StateVector values_;
};

}  // namespace
}  // namespace fusion

namespace gtsam {
template <>
struct traits<fusion::PlanarState>
    : public internal::Manifold<fusion::PlanarState> {};
}  // namespace gtsam

namespace fusion {
namespace {

struct MotionNoise {
  StateMatrix coordinates;
  gtsam::SharedNoiseModel model;
};

MotionNoise motion_noise(const StateMatrix& covariance) {
  Eigen::SelfAdjointEigenSolver<StateMatrix> solver(covariance);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("could not factor the IMU process covariance");
  }

  const auto eigenvalues = solver.eigenvalues();
  const double tolerance = std::max(eigenvalues.maxCoeff(), 1.0e-30) * 1.0e-10;
  StateVector sigmas;
  for (int index = 0; index < kStateDimension; ++index) {
    if (eigenvalues[index] < -tolerance) {
      throw std::runtime_error("IMU process covariance is not positive semidefinite");
    }
    sigmas[index] = eigenvalues[index] <= tolerance
                        ? 0.0
                        : std::sqrt(eigenvalues[index]);
  }
  return {solver.eigenvectors().transpose(),
          gtsam::noiseModel::Constrained::MixedSigmas(sigmas)};
}

class PlanarMotionFactor
    : public gtsam::NoiseModelFactorN<PlanarState, PlanarState> {
 public:
  using Base = gtsam::NoiseModelFactorN<PlanarState, PlanarState>;

  PlanarMotionFactor(gtsam::Key previous_key, gtsam::Key next_key, double dt_s,
                     double yaw_rate_radps,
                     double forward_acceleration_mps2,
                     const StateMatrix& process_covariance)
      : PlanarMotionFactor(previous_key, next_key, dt_s, yaw_rate_radps,
                           forward_acceleration_mps2,
                           motion_noise(process_covariance)) {}

  gtsam::Vector evaluateError(
      const PlanarState& previous, const PlanarState& next,
      boost::optional<gtsam::Matrix&> previous_jacobian = boost::none,
      boost::optional<gtsam::Matrix&> next_jacobian = boost::none) const override {
    StateMatrix transition;
    const PlanarState prediction = predict(previous, &transition);
    if (previous_jacobian) {
      *previous_jacobian = -coordinates_ * transition;
    }
    if (next_jacobian) {
      *next_jacobian = coordinates_;
    }
    return coordinates_ * prediction.localCoordinates(next);
  }

 private:
  PlanarMotionFactor(gtsam::Key previous_key, gtsam::Key next_key, double dt_s,
                     double yaw_rate_radps,
                     double forward_acceleration_mps2, MotionNoise noise)
      : Base(noise.model, previous_key, next_key),
        dt_s_(dt_s),
        yaw_rate_radps_(yaw_rate_radps),
        forward_acceleration_mps2_(forward_acceleration_mps2),
        coordinates_(std::move(noise.coordinates)) {}

  PlanarState predict(const PlanarState& previous,
                      StateMatrix* transition) const {
    const StateVector& state = previous.values();
    const double yaw = state[kYaw];
    const double speed = state[kSpeed];
    const double acceleration = forward_acceleration_mps2_ - state[kAccelBias];
    const double distance =
        speed * dt_s_ + 0.5 * acceleration * dt_s_ * dt_s_;

    StateVector predicted = state;
    predicted[kPositionX] += std::cos(yaw) * distance;
    predicted[kPositionY] += std::sin(yaw) * distance;
    predicted[kYaw] =
        wrap_angle(yaw + (yaw_rate_radps_ - state[kGyroBias]) * dt_s_);
    predicted[kSpeed] += acceleration * dt_s_;

    transition->setIdentity();
    (*transition)(kPositionX, kYaw) = -distance * std::sin(yaw);
    (*transition)(kPositionX, kSpeed) = std::cos(yaw) * dt_s_;
    (*transition)(kPositionX, kAccelBias) =
        -0.5 * std::cos(yaw) * dt_s_ * dt_s_;
    (*transition)(kPositionY, kYaw) = distance * std::cos(yaw);
    (*transition)(kPositionY, kSpeed) = std::sin(yaw) * dt_s_;
    (*transition)(kPositionY, kAccelBias) =
        -0.5 * std::sin(yaw) * dt_s_ * dt_s_;
    (*transition)(kYaw, kGyroBias) = -dt_s_;
    (*transition)(kSpeed, kAccelBias) = -dt_s_;
    return PlanarState(predicted);
  }

  double dt_s_;
  double yaw_rate_radps_;
  double forward_acceleration_mps2_;
  StateMatrix coordinates_;
};

class PlanarGpsFactor : public gtsam::NoiseModelFactorN<PlanarState> {
 public:
  using Base = gtsam::NoiseModelFactorN<PlanarState>;

  PlanarGpsFactor(gtsam::Key key, double x_m, double y_m,
                  double position_variance_m2)
      : Base(gtsam::noiseModel::Diagonal::Variances(
                 gtsam::Vector2::Constant(position_variance_m2)),
             key),
        position_world_m_(x_m, y_m) {}

  gtsam::Vector evaluateError(
      const PlanarState& state,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    if (jacobian) {
      *jacobian = gtsam::Matrix::Zero(2, kStateDimension);
      (*jacobian)(0, kPositionX) = 1.0;
      (*jacobian)(1, kPositionY) = 1.0;
    }
    return state.values().head<2>() - position_world_m_;
  }

 private:
  gtsam::Vector2 position_world_m_;
};

StateMatrix initial_covariance(const PlanarConfig& config) {
  StateVector diagonal;
  diagonal << config.initial_position_variance_m2,
      config.initial_position_variance_m2, config.initial_yaw_variance_rad2,
      config.initial_speed_variance_m2ps2,
      config.initial_gyro_bias_variance_rad2ps2,
      config.initial_accel_bias_variance_m2ps4;
  if (!diagonal.allFinite() || (diagonal.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "initial covariance variances must be positive and finite");
  }
  return diagonal.asDiagonal();
}

StateMatrix process_covariance(const PlanarConfig& config,
                               const StateVector& state, double dt_s) {
  StateMatrix covariance = StateMatrix::Zero();
  const double yaw = state[kYaw];

  StateVector gyro_sensitivity = StateVector::Zero();
  gyro_sensitivity[kYaw] = dt_s;
  const double gyro_variance =
      config.gyro_white_noise_density_radps_sqrt_hz *
      config.gyro_white_noise_density_radps_sqrt_hz / dt_s;
  covariance +=
      gyro_sensitivity * gyro_sensitivity.transpose() * gyro_variance;

  StateVector accel_sensitivity = StateVector::Zero();
  accel_sensitivity[kPositionX] = 0.5 * std::cos(yaw) * dt_s * dt_s;
  accel_sensitivity[kPositionY] = 0.5 * std::sin(yaw) * dt_s * dt_s;
  accel_sensitivity[kSpeed] = dt_s;
  const double accel_variance =
      config.accel_white_noise_density_mps2_sqrt_hz *
      config.accel_white_noise_density_mps2_sqrt_hz / dt_s;
  covariance +=
      accel_sensitivity * accel_sensitivity.transpose() * accel_variance;

  covariance(kGyroBias, kGyroBias) =
      config.gyro_bias_random_walk_radps_sqrt_s *
      config.gyro_bias_random_walk_radps_sqrt_s * dt_s;
  covariance(kAccelBias, kAccelBias) =
      config.accel_bias_random_walk_mps2_sqrt_s *
      config.accel_bias_random_walk_mps2_sqrt_s * dt_s;
  return covariance;
}

}  // namespace

class GtsamEkfEstimator::Impl {
 public:
  explicit Impl(const PlanarConfig& config)
      : config_(config),
        state_(),
        covariance_(initial_covariance(config)),
        filter_(0, state_,
                gtsam::noiseModel::Gaussian::Covariance(covariance_)) {
    if (!std::isfinite(config_.gps_gate_sigma) || config_.gps_gate_sigma < 0.0) {
      throw std::invalid_argument("GPS gate must be finite and nonnegative");
    }
  }

  void process_imu(std::int64_t measurement_time_ns, double yaw_rate_radps,
                   double forward_acceleration_mps2) {
    if (!std::isfinite(yaw_rate_radps) ||
        !std::isfinite(forward_acceleration_mps2)) {
      throw std::invalid_argument("IMU values must be finite");
    }
    if (!has_imu_time_) {
      last_imu_time_ns_ = measurement_time_ns;
      has_imu_time_ = true;
      return;
    }
    const double dt_s =
        static_cast<double>(measurement_time_ns - last_imu_time_ns_) * 1.0e-9;
    if (!(dt_s > 0.0)) {
      throw std::invalid_argument(
          "ego estimator requires increasing IMU timestamps");
    }
    last_imu_time_ns_ = measurement_time_ns;

    const StateMatrix noise = process_covariance(config_, state_.values(), dt_s);
    PlanarMotionFactor factor(current_key_, current_key_ + 1, dt_s,
                              yaw_rate_radps, forward_acceleration_mps2, noise);
    state_ = filter_.predict(factor);
    ++current_key_;
    update_covariance();
  }

  GpsUpdateResult process_gps(double position_world_x_m,
                              double position_world_y_m,
                              double horizontal_position_variance_m2) {
    if (!std::isfinite(horizontal_position_variance_m2) ||
        horizontal_position_variance_m2 < 0.0) {
      throw std::invalid_argument("GPS variance must be finite and nonnegative");
    }
    const gtsam::Vector2 measurement(position_world_x_m, position_world_y_m);
    const gtsam::Vector2 residual = measurement - state_.values().head<2>();
    if (!residual.allFinite()) {
      return {GpsUpdate::Invalid, kNan};
    }
    const gtsam::Matrix2 innovation_covariance =
        covariance_.topLeftCorner<2, 2>() +
        gtsam::Matrix2::Identity() * horizontal_position_variance_m2;
    const Eigen::LLT<gtsam::Matrix2> cholesky(innovation_covariance);
    if (cholesky.info() != Eigen::Success) {
      return {GpsUpdate::Invalid, kNan};
    }
    const double residual_squared = residual.dot(cholesky.solve(residual));
    if (!std::isfinite(residual_squared) || residual_squared < -1.0e-12) {
      return {GpsUpdate::Invalid, kNan};
    }
    const double normalized_residual = std::sqrt(std::max(0.0, residual_squared));
    if (normalized_residual > config_.gps_gate_sigma) {
      return {GpsUpdate::Rejected, normalized_residual};
    }

    PlanarGpsFactor factor(current_key_, position_world_x_m, position_world_y_m,
                           horizontal_position_variance_m2);
    state_ = filter_.update(factor);
    update_covariance();
    return {GpsUpdate::Applied, normalized_residual};
  }

  PlanarEstimate estimate() const {
    PlanarEstimate estimate;
    estimate.position_world_x_m = state_.values()[kPositionX];
    estimate.position_world_y_m = state_.values()[kPositionY];
    estimate.yaw_world_from_body_rad = state_.values()[kYaw];
    estimate.forward_speed_mps = state_.values()[kSpeed];
    estimate.gyro_bias_z_radps = state_.values()[kGyroBias];
    estimate.accel_bias_x_mps2 = state_.values()[kAccelBias];
    for (int row = 0; row < kStateDimension; ++row) {
      for (int column = 0; column < kStateDimension; ++column) {
        estimate.state_covariance[row * kStateDimension + column] =
            covariance_(row, column);
      }
    }
    return estimate;
  }

 private:
  void update_covariance() {
    const StateMatrix information = filter_.Density()->information();
    const Eigen::LDLT<StateMatrix> decomposition(information);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      throw std::runtime_error("GTSAM returned an invalid information matrix");
    }
    covariance_ = decomposition.solve(StateMatrix::Identity());
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    if (!state_.values().allFinite() || !covariance_.allFinite()) {
      throw std::runtime_error("GTSAM returned a non-finite estimate");
    }
  }

  PlanarConfig config_;
  PlanarState state_;
  StateMatrix covariance_;
  gtsam::ExtendedKalmanFilter<PlanarState> filter_;
  gtsam::Key current_key_ = 0;
  std::int64_t last_imu_time_ns_ = 0;
  bool has_imu_time_ = false;
};

GtsamEkfEstimator::GtsamEkfEstimator(const PlanarConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

GtsamEkfEstimator::~GtsamEkfEstimator() = default;

void GtsamEkfEstimator::process_imu(std::int64_t measurement_time_ns,
                                    double yaw_rate_radps,
                                    double forward_acceleration_mps2) {
  impl_->process_imu(measurement_time_ns, yaw_rate_radps,
                     forward_acceleration_mps2);
}

GpsUpdateResult GtsamEkfEstimator::process_gps(
    double position_world_x_m, double position_world_y_m,
    double horizontal_position_variance_m2) {
  return impl_->process_gps(position_world_x_m, position_world_y_m,
                            horizontal_position_variance_m2);
}

PlanarEstimate GtsamEkfEstimator::estimate() const { return impl_->estimate(); }

std::unique_ptr<GtsamEkfEstimator> new_gtsam_ekf_estimator(
    const PlanarConfig& config) {
  return std::make_unique<GtsamEkfEstimator>(config);
}

const std::string& gtsam_version() {
  static const std::string version = GTSAM_VERSION_STRING;
  return version;
}

}  // namespace fusion
