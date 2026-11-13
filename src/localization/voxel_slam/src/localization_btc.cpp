#include "localization_btc.hpp"
#include "localization_history.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

void LocalizationBtcOptions::validate(int saved_window_scans) const {
  if (saved_window_scans <= 0 || query_initial_scans <= 0 ||
      query_max_scans < query_initial_scans || query_stride_scans <= 0) {
    throw std::invalid_argument("invalid localization BTC query window");
  }
  if (history_window_scans < 0 ||
      (history_window_scans > 0 &&
       history_window_scans % saved_window_scans != 0)) {
    throw std::invalid_argument(
        "Localization.btc.history_window_scans must be zero or a positive "
        "multiple of the saved map win_size");
  }
  if (!std::isfinite(query_leaf_size_m) || query_leaf_size_m < 0 ||
      !std::isfinite(candidate_neighborhood_radius) ||
      candidate_neighborhood_radius < 0 ||
      !std::isfinite(early_icp_eigenvalue) || early_icp_eigenvalue < 0 ||
      !std::isfinite(final_icp_eigenvalue) || final_icp_eigenvalue < 0 ||
      !std::isfinite(score_threshold) || score_threshold < 0 ||
      score_threshold > 1) {
    throw std::invalid_argument("invalid localization BTC geometry threshold");
  }
}

LocalizationBtcOptions read_localization_btc_options(ros::NodeHandle &node) {
  LocalizationBtcOptions result;
#define BTC_PARAM(field) \
  node.param("Localization/btc/" #field, result.field, result.field)
  BTC_PARAM(query_leaf_size_m);
  BTC_PARAM(progressive_query);
  BTC_PARAM(query_initial_scans);
  BTC_PARAM(query_max_scans);
  BTC_PARAM(query_stride_scans);
  BTC_PARAM(history_window_scans);
  BTC_PARAM(candidate_neighborhood_radius);
  BTC_PARAM(early_refinement);
  BTC_PARAM(early_icp_eigenvalue);
  BTC_PARAM(final_icp_eigenvalue);
  BTC_PARAM(score_threshold);
#undef BTC_PARAM
  return result;
}

BtcQuerySchedule::BtcQuerySchedule(const LocalizationBtcOptions &options)
    : options_(options) {
  options_.validate(1);
}

bool BtcQuerySchedule::ready(std::size_t buffered_scans) {
  ++since_query_;
  if (buffered_scans < static_cast<std::size_t>(options_.query_initial_scans) ||
      (queried_ && options_.progressive_query &&
       since_query_ < static_cast<std::size_t>(options_.query_stride_scans))) {
    return false;
  }
  queried_ = true;
  since_query_ = 0;
  return true;
}

std::size_t BtcQuerySchedule::capacity() const {
  return static_cast<std::size_t>(options_.progressive_query
                                      ? options_.query_max_scans
                                      : options_.query_initial_scans);
}

void BtcQuerySchedule::reset() {
  since_query_ = 0;
  queried_ = false;
}

std::vector<SavedBtcFrame> build_localization_btc_history(
    const SavedMap &map, ConfigSetting config, int saved_keyframe_groups,
    double leaf_size_m) {
  if (saved_keyframe_groups <= 0) {
    throw std::invalid_argument("BTC history requires positive keyframe groups");
  }
  std::vector<SavedBtcFrame> result;
  result.reserve(map.keyframes.size());
  for (std::size_t index = 0; index < map.keyframes.size(); ++index) {
    const auto anchor = map.keyframes.begin() + index;
    const std::size_t begin =
        index + 1 > static_cast<std::size_t>(saved_keyframe_groups)
            ? index + 1 - saved_keyframe_groups
            : 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    for (std::size_t j = begin; j <= index; ++j) {
      const auto &source = map.keyframes[j];
      if (!source.cloud) {
        throw std::runtime_error("localization BTC keyframe cloud is missing");
      }
      const Eigen::Matrix3d R = anchor->pose.R.transpose() * source.pose.R;
      const Eigen::Vector3d t =
          anchor->pose.R.transpose() * (source.pose.t - anchor->pose.t);
      for (const auto &point : *source.cloud) {
        const Eigen::Vector3d p =
            R * Eigen::Vector3d(point.x, point.y, point.z) + t;
        pcl::PointXYZI output;
        output.x = p.x();
        output.y = p.y();
        output.z = p.z();
        output.intensity = 0;
        cloud->push_back(output);
      }
    }
    if (cloud->empty()) {
      throw std::runtime_error("localization BTC fused cloud is empty");
    }
    cloud = downsample_localization_cloud(cloud, leaf_size_m);
    STDescManager manager(config);
    SavedBtcFrame frame;
    frame.frame_id = static_cast<int>(index);
    frame.anchor_keyframe_id = anchor->id;
    manager.GenerateSTDescs(cloud, frame.descriptors, frame.frame_id);
    for (auto &descriptor : frame.descriptors) {
      descriptor.frame_number_ = frame.frame_id;
    }
    if (manager.plane_cloud_vec_.empty()) {
      throw std::runtime_error("localization BTC generated no plane cloud");
    }
    frame.plane_cloud = manager.plane_cloud_vec_.back();
    frame.plane_cloud->header.seq = frame.anchor_keyframe_id;
    result.push_back(std::move(frame));
  }
  return result;
}

BtcNeighborhood::BtcNeighborhood(const SavedMap &map, double radius)
    : map_(map), radius_(radius) {
  if (!std::isfinite(radius) || radius < 0) {
    throw std::invalid_argument("invalid BTC neighborhood radius");
  }
  for (const auto &frame : map.btc_frames) {
    for (const auto &keyframe : map.keyframes) {
      if (keyframe.id == frame.anchor_keyframe_id) {
        anchors_.emplace(frame.frame_id, &keyframe);
      }
    }
    if (!anchors_.count(frame.frame_id) || !frame.plane_cloud) {
      throw std::invalid_argument("invalid BTC neighborhood anchor or planes");
    }
  }
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr BtcNeighborhood::target(int frame_id) {
  const auto cached = cache_.find(frame_id);
  if (cached != cache_.end()) return cached->second;
  const auto &anchor = *anchors_.at(frame_id);
  if (radius_ == 0) {
    for (const auto &frame : map_.btc_frames) {
      if (frame.frame_id == frame_id) {
        cache_.emplace(frame_id, frame.plane_cloud);
        return frame.plane_cloud;
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  for (const auto &frame : map_.btc_frames) {
    const auto &keyframe = *anchors_.at(frame.frame_id);
    if ((radius_ == 0 && frame.frame_id != frame_id) ||
        (keyframe.pose.t - anchor.pose.t).norm() > radius_) continue;
    const Eigen::Matrix3d R = anchor.pose.R.transpose() * keyframe.pose.R;
    const Eigen::Vector3d t =
        anchor.pose.R.transpose() * (keyframe.pose.t - anchor.pose.t);
    for (auto point : *frame.plane_cloud) {
      const Eigen::Vector3d p = R * Eigen::Vector3d(point.x, point.y, point.z) + t;
      const Eigen::Vector3d normal = R * Eigen::Vector3d(
          point.normal_x, point.normal_y, point.normal_z);
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      point.normal_x = normal.x();
      point.normal_y = normal.y();
      point.normal_z = normal.z();
      result->push_back(point);
    }
  }
  cache_.emplace(frame_id, result);
  return result;
}
