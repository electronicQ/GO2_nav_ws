#include "localization_display.hpp"

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <limits>

TEST(LocalizationDisplay, TransformsImuCloudAndMergesVoxelsWithoutMutation) {
  SavedMap map;
  SavedKeyframe keyframe;
  keyframe.pose.R = Eigen::AngleAxisd(
      1.5707963267948966, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  keyframe.pose.t = Eigen::Vector3d(10, 20, 30);
  keyframe.cloud.reset(new pcl::PointCloud<PointType>());
  PointType point{};
  point.x = 1.1f; point.y = 0; point.z = 0; point.intensity = 8;
  keyframe.cloud->push_back(point);
  point.x = 1.2f;
  keyframe.cloud->push_back(point);
  map.keyframes.push_back(keyframe);
  map.keyframes.push_back(keyframe);

  const auto display = build_localization_display_cloud(map, 1.0);
  ASSERT_EQ(display.size(), 1U);
  EXPECT_NEAR(display[0].x, 10, 1e-5);
  EXPECT_NEAR(display[0].y, 21.15, 1e-5);
  EXPECT_NEAR(display[0].z, 30, 1e-5);
  EXPECT_NEAR(display[0].intensity, 8, 1e-5);
  ASSERT_EQ(keyframe.cloud->size(), 2U);
  EXPECT_FLOAT_EQ((*keyframe.cloud)[0].x, 1.1f);
  EXPECT_DOUBLE_EQ(map.keyframes[0].pose.t.x(), 10);
}

TEST(LocalizationDisplay, EmptyMissingAndNonFiniteCloudsAreSafe) {
  SavedMap map;
  EXPECT_TRUE(build_localization_display_cloud(map, .5).empty());
  map.keyframes.emplace_back();
  EXPECT_TRUE(build_localization_display_cloud(map, .5).empty());
  map.keyframes[0].cloud.reset(new pcl::PointCloud<PointType>());
  PointType point{};
  point.x = std::numeric_limits<float>::quiet_NaN();
  map.keyframes[0].cloud->push_back(point);
  EXPECT_TRUE(build_localization_display_cloud(map, .5).empty());
}

TEST(LocalizationDisplay, RejectsInvalidVoxelSizes) {
  SavedMap map;
  EXPECT_THROW(build_localization_display_cloud(map, 0), std::invalid_argument);
  EXPECT_THROW(build_localization_display_cloud(map, -.5), std::invalid_argument);
  EXPECT_THROW(build_localization_display_cloud(
      map, std::numeric_limits<double>::infinity()), std::invalid_argument);
}
