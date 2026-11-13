#pragma once

#include "map_package.hpp"
#include "scan_map.hpp"
#include <Eigen/Geometry>

struct DirectMatchOptions {
  bool enabled = true;
  std::string registration_backend = "point_to_plane";
  double gicp_leaf_size_m = 0.2;
  int gicp_threads = 4;
  int gicp_covariance_neighbors = 20;
  double period_sec = 1.0;
  double translation_m = 0.5;
  double rotation_deg = 10.0;
  double retry_sec = 5.0;
  int query_scans = 3;
  double query_leaf_size_m = 0.2;
  double query_max_range_m = 80.0;
  int min_query_points = 100;
  double target_padding_m = 5.0;
  double cache_padding_m = 5.0;
  double cache_refresh_distance_m = 3.0;
  double target_leaf_size_m = 0.2;
  int max_target_voxels = 500000;
  int recovery_after_failures = 5;
  double recovery_after_sec = 15.0;
  double icp_max_correspondence_m = 1.0;
  int icp_max_iterations = 30;
  void validate() const;
};

DirectMatchOptions read_direct_match_options(ros::NodeHandle &node);

// Direct scan-to-plane refinement uses all query points, not only sparse BTC centroids.
bool register_direct_scan(const pcl::PointCloud<pcl::PointXYZI> &source,
    const pcl::PointCloud<pcl::PointXYZINormal> &target,
    const Eigen::Isometry3d &initial, const DirectMatchOptions &options,
    Eigen::Isometry3d &refined);

bool register_direct_gicp(const pcl::PointCloud<pcl::PointXYZI> &source,
    const pcl::PointCloud<pcl::PointXYZI> &target,
    const Eigen::Isometry3d &initial, const DirectMatchOptions &options,
    Eigen::Isometry3d &refined, double *fitness = nullptr);

class DirectMatchSchedule {
 public:
  explicit DirectMatchSchedule(DirectMatchOptions options);
  bool ready(double wall, double stamp, const Eigen::Isometry3d &local, bool confirmation_pending = false) const;
  void submitted(double wall, double stamp, const Eigen::Isometry3d &local);
  void reset();
 private:
  DirectMatchOptions options_;
  double wall_ = -1, stamp_ = -1;
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
};

struct DirectMapTarget {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr planes;
  pcl::PointCloud<pcl::PointXYZI>::Ptr points;
  // Geometry is stored near this origin to avoid float precision loss in large maps.
  Eigen::Isometry3d map_from_target = Eigen::Isometry3d::Identity();
  std::size_t keyframes = 0, voxels = 0;
  bool cache_hit = false;
};

// AABB coverage index over the authoritative saved fused keyframes.
// Only one immutable target cache is retained; original clouds are never changed.
class DirectLocalMap {
 public:
  DirectLocalMap(const SavedMap &map, ConfigSetting config, DirectMatchOptions options,
                 MapCloudLoader loader = {},
                 std::function<STDescManager::PlaneCloud::Ptr(int)> plane_loader = {});
  DirectMapTarget target(const pcl::PointCloud<pcl::PointXYZI> &query,
                         const Eigen::Isometry3d &map_from_query, std::string &error);
 private:
  struct Coverage {
    const SavedKeyframe *frame;
    Eigen::Vector3d lower, upper;
    STDescManager::PlaneCloud::Ptr planes;
  };
  std::vector<Coverage> coverage_;
  MapCloudLoader loader_;
  std::function<STDescManager::PlaneCloud::Ptr(int)> plane_loader_;
  ConfigSetting config_;
  DirectMatchOptions options_;
  DirectMapTarget cached_;
  Eigen::Vector3d cached_lower_, cached_upper_, cached_position_;
};
