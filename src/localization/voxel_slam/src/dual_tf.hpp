#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

struct DualTfOptions {
  double query_period_sec = 1.0;
  double max_result_age_sec = 5.0;
  double stale_after_sec = 10.0;
  double global_valid_timeout_sec = 30.0;
  bool sc_correction_jump_check_enabled = true;
  double sc_max_correction_translation_m = 0.5;
  double sc_max_correction_rotation_deg = 10.0;
  double large_translation_m = 0.5;
  double large_rotation_deg = 3.0;
  double confirmation_translation_m = 0.25;
  double confirmation_rotation_deg = 2.0;
  double confirmation_timeout_sec = 5.0;
  double audit_min_delay_sec = 0.5;
  int audit_scans = 10;
  int audit_min_matches = 20;
  double audit_min_overlap = 0.4;
  double audit_max_plane_distance_m = 0.15;
  double audit_max_point_distance_m = 1.0;
  double audit_max_rmse_m = 0.08;
  double audit_max_p95_m = 0.12;
  double audit_min_normal_eigenvalue = 0.005;
  double audit_min_spatial_eigenvalue_m2 = 0.1;
  double audit_prior_comparison_m = 0.3;
  double audit_improvement_margin_m = 0.015;
  void validate() const {
    for (double v : {query_period_sec, max_result_age_sec, stale_after_sec, global_valid_timeout_sec,
                     sc_max_correction_translation_m, sc_max_correction_rotation_deg,
                     large_translation_m, large_rotation_deg,
                     confirmation_translation_m, confirmation_rotation_deg,
                     confirmation_timeout_sec, audit_min_delay_sec, audit_min_overlap,
                     audit_max_plane_distance_m, audit_max_point_distance_m,
                     audit_max_rmse_m, audit_max_p95_m, audit_min_normal_eigenvalue,
                     audit_min_spatial_eigenvalue_m2, audit_prior_comparison_m,
                     audit_improvement_margin_m}) {
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid dual-TF threshold");
    }
    if (sc_max_correction_rotation_deg > 180 || large_rotation_deg > 180 || confirmation_rotation_deg > 180)
      throw std::invalid_argument("invalid dual-TF rotation threshold");
    if(global_valid_timeout_sec < stale_after_sec || audit_scans < 2 || audit_min_matches < 6 || audit_min_overlap > 1 ||
       audit_min_normal_eigenvalue > 1.0 / 3 ||
       audit_min_delay_sec >= max_result_age_sec)
      throw std::invalid_argument("invalid dual-TF audit thresholds");
  }
};

struct DualTfView {
  bool aligned = false;
  bool stale = false;
  bool confirmation_pending = false;
  std::uint64_t epoch = 0;
  std::uint64_t manual_revision = 0;
  double last_correction_stamp = -1;
  double age = std::numeric_limits<double>::infinity();
  Eigen::Isometry3d map_from_odom = Eigen::Isometry3d::Identity();
};

// IESKF errors are [body rotation, parent position]; ROS pose is
// [parent position, parent rotation]. This changes basis, not uncertainty age.
inline Eigen::Matrix<double, 6, 6> dual_tf_pose_covariance(
    const Eigen::Matrix<double, 6, 6> &rotation_position_covariance,
    const Eigen::Matrix3d &parent_from_body) {
  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  J.topRightCorner<3, 3>().setIdentity();
  J.bottomLeftCorner<3, 3>() = parent_from_body;
  return J * rotation_position_covariance * J.transpose();
}

inline Eigen::Isometry3d dual_tf_planar_base_pose(
    const Eigen::Isometry3d &current, const Eigen::Vector2d &xy, double yaw) {
  if(!current.matrix().allFinite() || !xy.allFinite() || !std::isfinite(yaw))
    throw std::invalid_argument("nonfinite manual 2D pose");
  Eigen::Isometry3d desired = current;
  const double current_yaw = std::atan2(current.linear()(1, 0), current.linear()(0, 0));
  desired.linear() = Eigen::AngleAxisd(yaw - current_yaw, Eigen::Vector3d::UnitZ()) * current.linear();
  desired.translation().head<2>() = xy;
  return desired;
}

// Owns only the global coordinate relation, never the local estimator state.
class DualTfAlignment {
 public:
  explicit DualTfAlignment(DualTfOptions options = {}) : options_(options) { options_.validate(); }
  void configure(const DualTfOptions &options) {
    options.validate();
    std::lock_guard<std::mutex> lock(mutex_);
    options_ = options;
  }
  void reset(std::uint64_t epoch) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto next_revision = state_.manual_revision + 1;
    state_ = DualTfView();
    state_.manual_revision = next_revision;
    state_.epoch = epoch;
    pending_ = false;
    last_result_stamp_ = -1;
    manual_cutoff_stamp_ = -1;
  }
  DualTfView view(double now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = state_;
    result.confirmation_pending = pending_ && now - pending_stamp_ <= options_.confirmation_timeout_sec;
    if (result.aligned) {
      result.age = std::max(0.0, now - result.last_correction_stamp);
      result.stale = result.age > options_.stale_after_sec;
    }
    return result;
  }
  bool accept(std::uint64_t epoch, double query_stamp, double now,
              const Eigen::Isometry3d &map_from_query,
              const Eigen::Isometry3d &odom_from_query, std::string &reason,
              double evidence_stamp = -1) {
    return accept_impl(epoch, query_stamp, now, map_from_query, odom_from_query,
                       reason, evidence_stamp, true);
  }
  // SC callers have already passed retrieval, optional position trimming and
  // Quatro/GICP checks. Once aligned, also bound the change at the same query pose.
  // DIRECT must continue using accept() with fresh evidence.
  bool accept_scancontext(std::uint64_t epoch, double query_stamp, double now,
                          const Eigen::Isometry3d &map_from_query,
                          const Eigen::Isometry3d &odom_from_query, std::string &reason) {
    return accept_impl(epoch, query_stamp, now, map_from_query, odom_from_query,
                       reason, -1, false);
  }

  // A human map pose overrides the existing alignment, never the local estimator.
  // In-flight automatic queries at/before now cannot undo this intervention.
  bool accept_manual(std::uint64_t epoch, double reference_stamp, double now,
                     const Eigen::Isometry3d &map_from_base,
                     const Eigen::Isometry3d &odom_from_base, std::string &reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    reason.clear();
    if(epoch != state_.epoch || !std::isfinite(reference_stamp) || !std::isfinite(now) ||
       reference_stamp <= 0 || reference_stamp > now + 0.05 ||
       now - reference_stamp > options_.max_result_age_sec ||
       !valid(map_from_base) || !valid(odom_from_base)) {
      reason = "manual pose requires a recent finite local pose in the current session";
      return false;
    }
    const Eigen::Isometry3d correction = map_from_base * odom_from_base.inverse();
    if(!valid(correction)) { reason = "invalid manual map->odom transform"; return false; }
    state_.map_from_odom = correction;
    state_.aligned = true;
    state_.last_correction_stamp = now;
    ++state_.manual_revision;
    pending_ = false;
    manual_cutoff_stamp_ = now;
    last_result_stamp_ = now;
    return true;
  }

 private:
  bool accept_impl(std::uint64_t epoch, double query_stamp, double now,
                   const Eigen::Isometry3d &map_from_query,
                   const Eigen::Isometry3d &odom_from_query, std::string &reason,
                   double evidence_stamp, bool require_audit) {
    std::lock_guard<std::mutex> lock(mutex_);
    reason.clear();
    if(require_audit && (!std::isfinite(evidence_stamp) || evidence_stamp < query_stamp + options_.audit_min_delay_sec ||
       evidence_stamp > now + 1e-6)) {
      reason = "missing fresh post-query geometric audit";
      return false;
    }
    if (epoch != state_.epoch || !std::isfinite(query_stamp) || !std::isfinite(now) ||
        query_stamp <= last_result_stamp_ || query_stamp <= manual_cutoff_stamp_ ||
        query_stamp > now + 1e-6 ||
        now - query_stamp > options_.max_result_age_sec ||
        !valid(map_from_query) || !valid(odom_from_query)) {
      reason = "stale, out-of-order, invalid or previous-session global result";
      return false;
    }
    const Eigen::Isometry3d candidate = map_from_query * odom_from_query.inverse();
    if(!valid(candidate)) {
      reason = "global correction composition overflowed or is not rigid";
      return false;
    }
    last_result_stamp_ = query_stamp;
    if (!require_audit && state_.aligned && options_.sc_correction_jump_check_enabled) {
      const Eigen::Isometry3d predicted = state_.map_from_odom * odom_from_query;
      const double translation = (predicted.translation() - map_from_query.translation()).norm();
      const double rotation = Eigen::AngleAxisd(predicted.linear().transpose() *
          map_from_query.linear()).angle() * 180.0 / M_PI;
      if (translation > options_.sc_max_correction_translation_m ||
          rotation > options_.sc_max_correction_rotation_deg) {
        reason = "SC correction jump exceeds limit: translation_m=" + std::to_string(translation) +
                 " rotation_deg=" + std::to_string(rotation) +
                 " max_translation_m=" + std::to_string(options_.sc_max_correction_translation_m) +
                 " max_rotation_deg=" + std::to_string(options_.sc_max_correction_rotation_deg);
        return false;
      }
    }
    // Compare the effect at the vehicle, not translations at an arbitrary odom origin.
    if (require_audit && state_.aligned && !near(state_.map_from_odom * odom_from_query, map_from_query,
                               options_.large_translation_m, options_.large_rotation_deg)) {
      if (!pending_ || query_stamp - pending_stamp_ > options_.confirmation_timeout_sec ||
          !near(pending_transform_ * odom_from_query, map_from_query, options_.confirmation_translation_m,
                options_.confirmation_rotation_deg)) {
        pending_ = true;
        pending_stamp_ = query_stamp;
        pending_transform_ = candidate;
        reason = "large correction awaiting a second consistent fresh query";
        return false;
      }
    }
    pending_ = false;
    state_.aligned = true;
    state_.map_from_odom = candidate;
    state_.last_correction_stamp = query_stamp;
    return true;
  }

 private:
  static bool valid(const Eigen::Isometry3d &t) {
    return t.matrix().allFinite() &&
           t.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9) &&
           (t.linear().transpose() * t.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
           std::abs(t.linear().determinant() - 1.0) < 1e-6;
  }
  static bool near(const Eigen::Isometry3d &a, const Eigen::Isometry3d &b,
                   double distance, double angle_deg) {
    return (a.translation() - b.translation()).norm() <= distance &&
           Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle() * 180.0 / M_PI <= angle_deg;
  }
  DualTfOptions options_;
  mutable std::mutex mutex_;
  DualTfView state_;
  bool pending_ = false;
  double pending_stamp_ = -1;
  double last_result_stamp_ = -1;
  double manual_cutoff_stamp_ = -1;
  Eigen::Isometry3d pending_transform_ = Eigen::Isometry3d::Identity();
};
