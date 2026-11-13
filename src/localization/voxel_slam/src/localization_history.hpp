#pragma once

#include "map_package.hpp"

struct LocalizationHistoryOptions {
  std::string source = "saved_keyframes";
  int window_scans = 10;
  double leaf_size_m = 0.0;
  std::string scan_directory;
  std::string pose_file;
  double timestamp_tolerance_sec = 0.001;
  double anchor_position_tolerance_m = 0.02;
  double anchor_angle_tolerance_deg = 0.5;
  void validate(int saved_window_scans) const;
};

LocalizationHistoryOptions read_localization_history_options(
    ros::NodeHandle &node, int default_window_scans, double default_leaf_size_m);

// Builds descriptor inputs only. Original keyframes, IDs, anchors and direct-map targets stay immutable.
SavedMap build_localization_history(const SavedMap &map, int saved_window_scans,
                                   const LocalizationHistoryOptions &options);

pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_localization_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double leaf_size_m);
