#include "direct_matching.hpp"
#include <gtest/gtest.h>

namespace {
Eigen::Isometry3d local(double x = 0, double yaw_deg = 0) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = x;
  pose.linear() = Eigen::AngleAxisd(yaw_deg * M_PI / 180, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}
TEST(DirectSchedule, MotionOrRotationOrForcedRetryWithNewDataAndPeriodLimit) {
  DirectMatchSchedule schedule{DirectMatchOptions()};
  EXPECT_TRUE(schedule.ready(100, 1, local()));
  schedule.submitted(100, 1, local());
  EXPECT_FALSE(schedule.ready(100.5, 2, local(10)));
  EXPECT_FALSE(schedule.ready(101, 2, local(.1)));
  EXPECT_TRUE(schedule.ready(101, 2, local(.6)));
  EXPECT_TRUE(schedule.ready(101, 2, local(0, 11)));
  EXPECT_TRUE(schedule.ready(105, 2, local()));
  EXPECT_TRUE(schedule.ready(101, 2, local(), true));
  EXPECT_FALSE(schedule.ready(100.5, 2, local(), true));
  EXPECT_FALSE(schedule.ready(101, 1, local(), true));
  EXPECT_FALSE(schedule.ready(105, 1, local(10)));
  schedule.submitted(105, 2, local(.6));
  EXPECT_FALSE(schedule.ready(106, 3, local(.7)));
  schedule.reset();
  EXPECT_TRUE(schedule.ready(106, 3, local(.7)));
}
TEST(DirectSchedule, RejectsInvalidSettingsAndNonfiniteInputs) {
  DirectMatchOptions options;
  options.query_scans = 0;
  EXPECT_THROW(DirectMatchSchedule{options}, std::invalid_argument);
  options.query_scans = 1; options.retry_sec = .1;
  EXPECT_THROW(DirectMatchSchedule{options}, std::invalid_argument);
  DirectMatchSchedule schedule{DirectMatchOptions()};
  EXPECT_FALSE(schedule.ready(NAN, 1, local()));
  auto pose = local(); pose.translation().x() = NAN;
  EXPECT_FALSE(schedule.ready(1, 1, pose));
  options = DirectMatchOptions(); options.registration_backend = "invalid";
  EXPECT_THROW(options.validate(), std::invalid_argument);
  options.registration_backend = "nano_gicp"; options.gicp_threads = 0;
  EXPECT_THROW(options.validate(), std::invalid_argument);
  options.gicp_threads = 2; options.gicp_covariance_neighbors = 2;
  EXPECT_THROW(options.validate(), std::invalid_argument);
}
SavedMap map_with_remote_anchor() {
  SavedMap map;
  SavedKeyframe frame;
  frame.id = 7; frame.pose.t = Eigen::Vector3d(100, 0, 0);
  frame.cloud.reset(new pcl::PointCloud<PointType>());
  for(int axis = 0; axis < 3; ++axis)
    for(int i = 0; i < 50; ++i)
      for(int j = 0; j < 50; ++j) {
        Eigen::Vector3d xyz(20, 20, 20);
        xyz[(axis + 1) % 3] += i * .05;
        xyz[(axis + 2) % 3] += j * .05;
        xyz -= frame.pose.t;
        PointType p;
        p.x = xyz.x(); p.y = xyz.y(); p.z = xyz.z();
        frame.cloud->push_back(p);
      }
  map.keyframes.push_back(frame);
  return map;
}
pcl::PointCloud<pcl::PointXYZI> query_at(double x) {
  pcl::PointCloud<pcl::PointXYZI> query;
  pcl::PointXYZI p;
  p.x = x; p.y = 20; p.z = 20; query.push_back(p);
  return query;
}
TEST(DirectMap, UsesPointCoverageNotAnchorDistanceAndCachesImmutableTarget) {
  auto map = map_with_remote_anchor();
  ConfigSetting config{};
  DirectMatchOptions options;
  options.target_leaf_size_m = .05;
  DirectLocalMap index(map, config, options);
  std::string error;
  const auto before = map.keyframes[0].cloud->at(0);
  auto result = index.target(query_at(20), local(), error);
  ASSERT_TRUE(result.planes) << error;
  EXPECT_EQ(result.keyframes, 1U);
  EXPECT_FALSE(result.planes->empty());
  EXPECT_FALSE(result.cache_hit);
  EXPECT_FALSE(result.points);
  auto cached = index.target(query_at(20.1), local(.1), error);
  EXPECT_TRUE(cached.cache_hit);
  EXPECT_EQ(cached.planes.get(), result.planes.get());
  EXPECT_FLOAT_EQ(map.keyframes[0].cloud->at(0).x, before.x);
  auto missing = index.target(query_at(20), local(200), error);
  EXPECT_FALSE(missing.planes);
  EXPECT_FALSE(error.empty());
  // A failed refresh cannot mutate an in-flight candidate's target.
  EXPECT_FALSE(result.planes->empty());
}
TEST(DirectMap, BudgetFailureIsExplicitAndNoDescriptorHistoryIsGenerated) {
  auto map = map_with_remote_anchor();
  ConfigSetting config{};
  config.voxel_size_ = .2;
  DirectMatchOptions options;
  options.max_target_voxels = 100; options.target_leaf_size_m = .05;
  DirectLocalMap index(map, config, options);
  std::string error;
  EXPECT_FALSE(index.target(query_at(20), local(), error).planes);
  EXPECT_NE(error.find("budget exceeded"), std::string::npos);
  STDescManager extractor(config);
  pcl::PointCloud<pcl::PointXYZI>::Ptr empty(new pcl::PointCloud<pcl::PointXYZI>());
  EXPECT_TRUE(extractor.ExtractPlanes(empty)->empty());
  EXPECT_TRUE(extractor.data_base_.empty());
  EXPECT_TRUE(extractor.plane_cloud_vec_.empty());
}
TEST(DirectMap, GicpBuildsRealPointNeighborhoodsSeparatelyFromAuditPlanes) {
  auto map = map_with_remote_anchor();
  ConfigSetting config{};
  DirectMatchOptions options; options.registration_backend = "nano_gicp";
  DirectLocalMap index(map, config, options);
  std::string error;
  auto result = index.target(query_at(20), local(), error);
  ASSERT_TRUE(result.planes) << error;
  ASSERT_TRUE(result.points);
  EXPECT_GT(result.points->size(), result.planes->size());
  const auto cached = index.target(query_at(20), local(), error);
  EXPECT_TRUE(cached.cache_hit);
  EXPECT_EQ(cached.points.get(), result.points.get());
  options.max_target_voxels = 100; options.gicp_leaf_size_m = .01;
  DirectLocalMap limited(map, config, options);
  EXPECT_FALSE(limited.target(query_at(20), local(), error).planes);
  EXPECT_NE(error.find("GICP target voxel budget exceeded"), std::string::npos);
}
TEST(DirectRegistration, RawScanRefinesSmallPriorErrorWithoutModifyingInput) {
  pcl::PointCloud<pcl::PointXYZI> source;
  pcl::PointCloud<pcl::PointXYZINormal> target;
  for(int axis = 0; axis < 3; ++axis)
    for(int i = 0; i < 15; ++i)
      for(int j = 0; j < 15; ++j) {
        pcl::PointXYZINormal p{};
        p.getVector3fMap().setZero(); p.getNormalVector3fMap().setZero();
        p.getVector3fMap()[(axis + 1) % 3] = i * .2;
        p.getVector3fMap()[(axis + 2) % 3] = j * .2;
        p.getNormalVector3fMap()[axis] = 1;
        target.push_back(p);
        pcl::PointXYZI q{};
        q.x = p.x; q.y = p.y; q.z = p.z;
        source.push_back(q);
      }
  Eigen::Isometry3d initial = local(.1, 1), refined;
  ASSERT_TRUE(register_direct_scan(source, target, initial, DirectMatchOptions(), refined));
  EXPECT_LT(refined.translation().norm(), .01);
  EXPECT_LT(Eigen::AngleAxisd(refined.linear()).angle(), .01);
  EXPECT_FLOAT_EQ(source[0].x, 0);
  pcl::PointCloud<pcl::PointXYZI> empty;
  EXPECT_FALSE(register_direct_scan(empty, target, initial, DirectMatchOptions(), refined));
  DirectMatchOptions options; options.registration_backend = "nano_gicp";
  const auto before = source;
  ASSERT_TRUE(register_direct_gicp(source, source, initial, options, refined));
  EXPECT_LT(refined.translation().norm(), .02);
  EXPECT_LT(Eigen::AngleAxisd(refined.linear()).angle(), .01);
  EXPECT_FLOAT_EQ(source[0].x, before[0].x);
  EXPECT_FALSE(register_direct_gicp(empty, source, initial, options, refined));
  EXPECT_FALSE(register_direct_gicp(source, empty, initial, options, refined));
  auto invalid = source; invalid[0].x = NAN;
  EXPECT_FALSE(register_direct_gicp(source, invalid, initial, options, refined));
  initial.translation().x() = NAN;
  EXPECT_FALSE(register_direct_gicp(source, source, initial, options, refined));
}
}  // namespace
