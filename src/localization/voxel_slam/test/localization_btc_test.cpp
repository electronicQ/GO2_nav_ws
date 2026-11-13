#include "localization_btc.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace {

SavedMap exampleMap() {
  SavedMap map;
  for (int i = 0; i < 3; ++i) {
    SavedKeyframe key;
    key.id = 10 + i * 10;
    key.pose.stamp = i;
    key.pose.t = Eigen::Vector3d(i * 2, 0, 0);
    key.cloud.reset(new pcl::PointCloud<PointType>());
    for (int x = 0; x < 20; ++x) for (int y = 0; y < 20; ++y) {
      PointType p{};
      p.x = x * .1F;
      p.y = y * .1F;
      p.z = 0;
      key.cloud->push_back(p);
    }
    map.keyframes.push_back(key);
    SavedBtcFrame frame;
    frame.frame_id = i;
    frame.anchor_keyframe_id = key.id;
    frame.plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::PointXYZINormal p{};
    p.x = 1;
    p.normal_x = 1;
    frame.plane_cloud->push_back(p);
    map.btc_frames.push_back(frame);
  }
  return map;
}

TEST(LocalizationBtc, ExperimentDefaultsAndValidation) {
  LocalizationBtcOptions options;
  EXPECT_NO_THROW(options.validate(10));
  EXPECT_EQ(options.history_window_scans, 30);
  EXPECT_DOUBLE_EQ(options.early_icp_eigenvalue, 7);
  EXPECT_DOUBLE_EQ(options.final_icp_eigenvalue, 7);
  EXPECT_DOUBLE_EQ(options.candidate_neighborhood_radius, 5);
  options.history_window_scans = 25;
  EXPECT_THROW(options.validate(10), std::invalid_argument);
  options.history_window_scans = 0;
  EXPECT_NO_THROW(options.validate(10));
  options.query_max_scans = 9;
  EXPECT_THROW(options.validate(10), std::invalid_argument);
}

TEST(LocalizationBtc, RejectsInvalidThresholdsAndStride) {
  LocalizationBtcOptions options;
  options.early_icp_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(options.validate(10), std::invalid_argument);
  options.early_icp_eigenvalue = 7;
  options.candidate_neighborhood_radius = -1;
  EXPECT_THROW(options.validate(10), std::invalid_argument);
  options.candidate_neighborhood_radius = 5;
  options.query_stride_scans = 0;
  EXPECT_THROW(options.validate(10), std::invalid_argument);
  options.query_stride_scans = 10;
  options.score_threshold = 1.1;
  EXPECT_THROW(options.validate(10), std::invalid_argument);
}

TEST(LocalizationBtc, ProgressiveScheduleMatchesReplayAndResets) {
  LocalizationBtcOptions options;
  BtcQuerySchedule schedule(options);
  std::vector<std::size_t> windows;
  for (std::size_t i = 1; i <= 50; ++i) {
    const auto buffered = std::min(i, schedule.capacity());
    if (schedule.ready(buffered)) windows.push_back(buffered);
  }
  EXPECT_EQ(windows, (std::vector<std::size_t>{10, 20, 30, 30, 30}));
  schedule.reset();
  for (std::size_t i = 1; i < 10; ++i) EXPECT_FALSE(schedule.ready(i));
  EXPECT_TRUE(schedule.ready(10));
}

TEST(LocalizationBtc, FixedWindowsAndNonDefaultStride) {
  LocalizationBtcOptions options;
  options.progressive_query = false;
  BtcQuerySchedule fixed(options);
  EXPECT_EQ(fixed.capacity(), 10U);
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (std::size_t i = 1; i < 10; ++i) EXPECT_FALSE(fixed.ready(i));
    EXPECT_TRUE(fixed.ready(10));
    fixed.reset();
  }
  options.progressive_query = true;
  options.query_initial_scans = 7;
  options.query_max_scans = 13;
  options.query_stride_scans = 3;
  BtcQuerySchedule variable(options);
  std::vector<std::size_t> at;
  for (std::size_t i = 1; i <= 16; ++i)
    if (variable.ready(std::min(i, variable.capacity()))) at.push_back(i);
  EXPECT_EQ(at, (std::vector<std::size_t>{7, 10, 13, 16}));
}

TEST(LocalizationBtc, HistoryPreservesAnchorsAndOriginalMap) {
  auto map = exampleMap();
  ConfigSetting config;
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  ros::NodeHandle node;
  read_parameters(node, config, 1);
  auto rebuilt = build_localization_btc_history(map, config, 3);
  ASSERT_EQ(rebuilt.size(), 3U);
  for (std::size_t i = 0; i < rebuilt.size(); ++i) {
    EXPECT_EQ(rebuilt[i].frame_id, map.btc_frames[i].frame_id);
    EXPECT_EQ(rebuilt[i].anchor_keyframe_id, map.keyframes[i].id);
    EXPECT_EQ(rebuilt[i].plane_cloud->header.seq, unsigned(map.keyframes[i].id));
    EXPECT_EQ(map.keyframes[i].cloud->size(), 400U);
    EXPECT_EQ(map.btc_frames[i].plane_cloud->size(), 1U);
    EXPECT_TRUE(map.btc_frames[i].descriptors.empty());
  }
}

TEST(LocalizationBtc, MissingHistoryInputFailsExplicitly) {
  auto map = exampleMap();
  ConfigSetting config;
  EXPECT_THROW(build_localization_btc_history(map, config, 0), std::invalid_argument);
  map.keyframes[0].cloud.reset();
  EXPECT_THROW(build_localization_btc_history(map, config, 3), std::runtime_error);
}

TEST(LocalizationBtc, HistoryNeedsOnlyKeyframesNotCachedDescriptorsOrAnchors) {
  auto map = exampleMap();
  map.btc_frames.clear();
  ConfigSetting config;
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  ros::NodeHandle node;
  read_parameters(node, config, 1);
  const auto rebuilt = build_localization_btc_history(map, config, 3);
  ASSERT_EQ(rebuilt.size(), map.keyframes.size());
  for (std::size_t i = 0; i < rebuilt.size(); ++i) {
    EXPECT_EQ(rebuilt[i].frame_id, static_cast<int>(i));
    EXPECT_EQ(rebuilt[i].anchor_keyframe_id, map.keyframes[i].id);
    ASSERT_TRUE(rebuilt[i].plane_cloud);
  }
  EXPECT_TRUE(map.btc_frames.empty());
}

TEST(LocalizationBtc, NeighborhoodUsesAnchorRadiusTransformsNormalsAndCaches) {
  auto map = exampleMap();
  map.keyframes[1].pose.R = Eigen::AngleAxisd(
      M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  BtcNeighborhood neighborhood(map, 2.0);
  auto cloud = neighborhood.target(0);
  ASSERT_EQ(cloud->size(), 2U);
  EXPECT_FLOAT_EQ((*cloud)[0].x, 1);
  EXPECT_NEAR((*cloud)[1].x, 2, 1e-6);
  EXPECT_NEAR((*cloud)[1].y, 1, 1e-6);
  EXPECT_NEAR((*cloud)[1].normal_x, 0, 1e-6);
  EXPECT_NEAR((*cloud)[1].normal_y, 1, 1e-6);
  EXPECT_EQ(cloud, neighborhood.target(0));
  EXPECT_EQ(map.btc_frames[1].plane_cloud->front().normal_x, 1);
  EXPECT_THROW(neighborhood.target(99), std::out_of_range);
}

TEST(LocalizationBtc, ZeroRadiusUsesOnlyCandidateAndInvalidInputsFail) {
  auto map = exampleMap();
  map.keyframes[1].pose.t = map.keyframes[0].pose.t;
  BtcNeighborhood neighborhood(map, 0);
  EXPECT_EQ(neighborhood.target(0)->size(), 1U);
  EXPECT_THROW(BtcNeighborhood(map, -1), std::invalid_argument);
  map.btc_frames[0].plane_cloud.reset();
  EXPECT_THROW(BtcNeighborhood(map, 5), std::invalid_argument);
}

TEST(LocalizationBtc, InvalidRefinementCannotBeInstalled) {
  STDescManager manager;
  EXPECT_THROW(manager.set_candidate_refinement({}, {}, 7), std::invalid_argument);
}

}  // namespace
