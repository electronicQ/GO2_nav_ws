#pragma once

#include "map_package.hpp"

// Visualization only: never modifies saved keyframes or tracking geometry.
pcl::PointCloud<pcl::PointXYZI> build_localization_display_cloud(
    const SavedMap &map, double leaf_size);
