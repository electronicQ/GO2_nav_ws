#pragma once

#include "map_package.hpp"
#include "scan_map.hpp"
#include <Eigen/Geometry>
#include <memory>

struct ScanContextOptions {
  int query_scans = 5;
  int query_stride_scans = 1;
  int candidates = 3;
  double distance_threshold = 0.2;
  double max_radius_m = 80.0;
  double lidar_height_m = 2.0;
  double leaf_size_m = 0.3; // Legacy fallback.
  double history_leaf_size_m = 0.0;
  double query_leaf_size_m = 0.0;
  double registration_leaf_size_m = 0.3;
  double normal_radius_m = 0.6;
  double fpfh_radius_m = 0.9;
  double noise_bound_m = 0.3;
  int max_correspondences = 350;
  int threads = 4;
  double gicp_max_correspondence_m = 35.0;
  double gicp_fitness_threshold = 1.0;
  double candidate_max_distance_m = 35.0; // Applied only with an established map pose.
  void validate() const;
};

ScanContextOptions read_scancontext_options(ros::NodeHandle &node);
bool scancontext_backend_available();
Eigen::Matrix3d scancontext_level_rotation(const Eigen::Matrix3d &world_from_body);

struct ScanContextCandidate {
  int index = -1;
  double distance = 1.0;
  double yaw_rad = 0.0;
};

struct ScanContextResult {
  bool success = false;
  int keyframe_id = -1;
  double distance = 1.0;
  double descriptor_ms = 0, search_ms = 0, target_ms = 0, quatro_ms = 0, gicp_ms = 0;
  Eigen::Isometry3d map_from_query = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_from_target = Eigen::Isometry3d::Identity();
  double candidate_position_distance_m = -1; // Negative: no map-position prior.
  std::string reason;
};

// Independent, immutable history bank built from saved fused keyframes, not BTC windows.
class ScanContextRelocalizer {
 public:
  ScanContextRelocalizer(const SavedMap &map, ConfigSetting plane_config, ScanContextOptions options,
                         MapCloudLoader loader = {});
  ~ScanContextRelocalizer();
  ScanContextCandidate search(const pcl::PointCloud<pcl::PointXYZI> &gravity_aligned_query);
  ScanContextResult match(const pcl::PointCloud<pcl::PointXYZI> &query,
                         const Eigen::Matrix3d &odom_from_body,
                         const Eigen::Vector3d *predicted_map_position = nullptr);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
