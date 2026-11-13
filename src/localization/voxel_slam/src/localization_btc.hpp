#pragma once

#include "map_package.hpp"

#include <unordered_map>

struct LocalizationBtcOptions {
  bool progressive_query = true;
  int query_initial_scans = 10;
  int query_max_scans = 30;
  int query_stride_scans = 10;
  // Zero prefers cached descriptors; without a cache, rebuild one saved group.
  int history_window_scans = 30;
  double candidate_neighborhood_radius = 5.0;
  bool early_refinement = true;
  double early_icp_eigenvalue = 7.0;
  double final_icp_eigenvalue = 7.0;
  double score_threshold = 0.4;
  double query_leaf_size_m = 0.0;

  void validate(int saved_window_scans) const;
};

LocalizationBtcOptions read_localization_btc_options(ros::NodeHandle &node);

class BtcQuerySchedule {
 public:
  explicit BtcQuerySchedule(const LocalizationBtcOptions &options);
  bool ready(std::size_t buffered_scans);
  std::size_t capacity() const;
  void reset();

 private:
  LocalizationBtcOptions options_;
  std::size_t since_query_ = 0;
  bool queried_ = false;
};

// Rebuild from saved fused keyframes, never from already-expanded BTC planes.
std::vector<SavedBtcFrame> build_localization_btc_history(
    const SavedMap &map, ConfigSetting config, int saved_keyframe_groups,
    double leaf_size_m = 0.0);

class BtcNeighborhood {
 public:
  BtcNeighborhood(const SavedMap &map, double radius);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr target(int frame_id);

 private:
  const SavedMap &map_;
  double radius_;
  std::unordered_map<int, const SavedKeyframe *> anchors_;
  std::unordered_map<int, pcl::PointCloud<pcl::PointXYZINormal>::Ptr> cache_;
};
