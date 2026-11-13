#include "localization_display.hpp"

#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <stdexcept>

pcl::PointCloud<pcl::PointXYZI> build_localization_display_cloud(
    const SavedMap &map, double leaf_size) {
  if (!std::isfinite(leaf_size) || leaf_size < 0.01) {
    throw std::invalid_argument("Display leaf size must be finite and >= 0.01 m");
  }
  auto merged = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (const SavedKeyframe &keyframe : map.keyframes) {
    if (!keyframe.cloud) continue;
    for (const PointType &point : *keyframe.cloud) {
      const Eigen::Vector3d world = keyframe.pose.R *
          Eigen::Vector3d(point.x, point.y, point.z) + keyframe.pose.t;
      if (!world.allFinite()) continue;
      pcl::PointXYZI display_point;
      display_point.x = world.x();
      display_point.y = world.y();
      display_point.z = world.z();
      display_point.intensity = std::isfinite(point.intensity) ? point.intensity : 0;
      merged->push_back(display_point);
    }
  }
  pcl::PointCloud<pcl::PointXYZI> result;
  if (merged->empty()) return result;
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(merged);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.filter(result);
  return result;
}
