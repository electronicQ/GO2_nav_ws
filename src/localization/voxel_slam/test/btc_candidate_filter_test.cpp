#include "BTC.h"
#include "relocalizer.hpp"

#include <gtest/gtest.h>

#include <unordered_set>
#include <limits>
#include <vector>

namespace {

BinaryDescriptor binary(const Eigen::Vector3d &location) {
  BinaryDescriptor result;
  result.occupy_array_ = {true, true, true};
  result.summary_ = 3;
  result.location_ = location;
  return result;
}

STD descriptor(double triangle_seed, int frame_number) {
  STD result;
  result.triangle_ = Eigen::Vector3d::Constant(triangle_seed);
  result.angle_ = Eigen::Vector3d::Zero();
  result.center_ = Eigen::Vector3d::Constant(triangle_seed + 0.25);
  result.frame_number_ = frame_number;
  result.binary_A_ = binary(Eigen::Vector3d(1.0, 0.0, 0.0));
  result.binary_B_ = binary(Eigen::Vector3d(0.0, 1.0, 0.0));
  result.binary_C_ = binary(Eigen::Vector3d(0.0, 0.0, 1.0));
  return result;
}

STDescManager makeManager() {
  ConfigSetting config;
  config.candidate_num_ = 5;
  config.rough_dis_threshold_ = 0.2F;
  config.similarity_threshold_ = 0.5F;
  config.skip_near_num_ = 50;
  STDescManager manager(config);
  manager.plane_cloud_vec_.resize(2);
  manager.plane_cloud_vec_[0].reset(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  manager.plane_cloud_vec_[1].reset(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  manager.AddSTDescs(
      {descriptor(10.2, 0), descriptor(10.2, 1), descriptor(20.2, 0)});
  return manager;
}

std::vector<STD> makeQueries() {
  std::vector<STD> queries;
  for (int i = 0; i < 6; ++i) {
    queries.push_back(descriptor(10.2, 0));
  }
  for (int i = 0; i < 6; ++i) {
    queries.push_back(descriptor(20.2, 0));
  }
  return queries;
}

TEST(BtcCandidateFilter, DisallowedHighVoteFrameCannotHideAllowedFrame) {
  STDescManager manager = makeManager();
  std::vector<STD> queries = makeQueries();
  std::vector<STDMatchList> candidates;
  const std::unordered_set<int> allowed{1};

  manager.candidate_selector(queries, candidates, &allowed, true);

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates[0].match_id_.second, 1);
  EXPECT_EQ(candidates[0].match_list_.size(), 6U);
}

TEST(BtcCandidateFilter, EmptyAllowedSetDoesNotBecomeGlobalSearch) {
  STDescManager manager = makeManager();
  std::vector<STD> queries = makeQueries();
  std::vector<STDMatchList> candidates;
  const std::unordered_set<int> allowed;

  manager.candidate_selector(queries, candidates, &allowed, true);
  EXPECT_TRUE(candidates.empty());

  std::pair<int, double> loop_result(123, 123.0);
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> transform;
  std::vector<std::pair<STD, STD>> matches;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_planes(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  manager.SearchLoop(queries, loop_result, transform, matches, current_planes,
                     allowed);
  EXPECT_EQ(loop_result.first, -1);
  EXPECT_DOUBLE_EQ(loop_result.second, 0.0);
}

TEST(BtcCandidateFilter, CrossSessionBypassesTemporalSkipOnlyWhenRequested) {
  STDescManager manager = makeManager();
  std::vector<STD> queries = makeQueries();
  std::vector<STDMatchList> candidates;

  manager.candidate_selector(queries, candidates, nullptr, false);
  EXPECT_TRUE(candidates.empty());

  manager.candidate_selector(queries, candidates, nullptr, true);
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].match_id_.second, 0);
  EXPECT_EQ(candidates[1].match_id_.second, 1);
}

TEST(BtcCandidateFilter, IgnoresNegativeAndOutOfRangeHistoryFrameIds) {
  STDescManager manager = makeManager();
  manager.AddSTDescs({descriptor(10.2, -1), descriptor(10.2, 99)});
  std::vector<STD> queries = makeQueries();
  std::vector<STDMatchList> candidates;

  manager.candidate_selector(queries, candidates, nullptr, true);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].match_id_.second, 0);
  EXPECT_EQ(candidates[1].match_id_.second, 1);
}

TEST(BtcRadiusPolicy, DoublesAndClampsWithoutRepeatingMaximum) {
  EXPECT_DOUBLE_EQ(next_btc_radius(30.0, 100.0), 60.0);
  EXPECT_DOUBLE_EQ(next_btc_radius(60.0, 100.0), 100.0);
  EXPECT_DOUBLE_EQ(next_btc_radius(100.0, 100.0), 100.0);

  const std::vector<double> expanded =
      make_btc_radius_schedule(30.0, 100.0, true);
  ASSERT_EQ(expanded.size(), 3U);
  EXPECT_DOUBLE_EQ(expanded[0], 30.0);
  EXPECT_DOUBLE_EQ(expanded[1], 60.0);
  EXPECT_DOUBLE_EQ(expanded[2], 100.0);

  const std::vector<double> single =
      make_btc_radius_schedule(30.0, 100.0, false);
  ASSERT_EQ(single.size(), 1U);
  EXPECT_DOUBLE_EQ(single[0], 30.0);
  EXPECT_TRUE(make_btc_radius_schedule(0.0, 100.0, true).empty());
  EXPECT_TRUE(make_btc_radius_schedule(30.0, 20.0, true).empty());
}

TEST(BtcCandidateRefinement, UsesConfiguredGateAndRejectsWorseOrInvalidUpdate) {
  for (int mode = 0; mode < 4; ++mode) {
    STDescManager manager = makeManager();
    manager.config_setting_.icp_threshold_ = .15;
    manager.config_setting_.normal_threshold_ = .2;
    manager.config_setting_.dis_threshold_ = .5;
    auto planes = STDescManager::PlaneCloud::Ptr(new STDescManager::PlaneCloud());
    pcl::PointXYZINormal point{};
    point.normal_z = 1;
    planes->push_back(point);
    manager.plane_cloud_vec_[1] = planes;
    int calls = 0;
    manager.set_candidate_refinement(
        [&](int id) { EXPECT_EQ(id, 1); return planes; },
        [&](STDescManager::PlaneCloud &, STDescManager::PlaneCloud &,
            STDescManager::Transform &transform, double gate) {
          ++calls;
          EXPECT_DOUBLE_EQ(gate, 7);
          if (mode == 0) transform.first.z() = 1;
          if (mode == 1) transform.first.z() = std::numeric_limits<double>::quiet_NaN();
          if (mode == 3) transform.first.x() = .1;
          return mode != 2;
        }, 7);
    auto queries = makeQueries();
    std::pair<int, double> result;
    STDescManager::Transform transform;
    std::vector<std::pair<STD, STD>> matches;
    manager.SearchLoop(queries, result, transform, matches, planes,
                       std::unordered_set<int>{1});
    EXPECT_GT(calls, 0);
    EXPECT_EQ(result.first, 1);
    EXPECT_DOUBLE_EQ(result.second, 1);
    EXPECT_NEAR(transform.first.z(), 0, 1e-8);
    if (mode == 3) {
      EXPECT_NEAR(transform.first.x(), .1, 1e-8);
    }
  }
}

TEST(BtcCandidateRefinement, IsOptInForEachManager) {
  STDescManager manager = makeManager();
  manager.config_setting_.icp_threshold_ = .15;
  manager.config_setting_.normal_threshold_ = .2;
  manager.config_setting_.dis_threshold_ = .5;
  auto planes = STDescManager::PlaneCloud::Ptr(new STDescManager::PlaneCloud());
  pcl::PointXYZINormal point{};
  point.normal_z = 1;
  planes->push_back(point);
  manager.plane_cloud_vec_[1] = planes;
  auto queries = makeQueries();
  std::pair<int, double> result;
  STDescManager::Transform transform;
  std::vector<std::pair<STD, STD>> matches;
  manager.SearchLoop(queries, result, transform, matches, planes,
                     std::unordered_set<int>{1});
  EXPECT_EQ(result.first, 1);
  EXPECT_DOUBLE_EQ(result.second, 1);
}

}  // namespace
