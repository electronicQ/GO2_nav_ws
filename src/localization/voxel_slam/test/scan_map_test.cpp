#include "scan_map.hpp"
#include "direct_matching.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace {
struct Fixture : ::testing::Test {
  std::filesystem::path root=std::filesystem::temp_directory_path()/
      ("sc_map_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  ~Fixture() { std::filesystem::remove_all(root); }
  void write(int count=6) {
    ScanMapWriter writer(root.string(),.1);
    std::vector<std::pair<int,SavedPose>> final;
    for(int i=0;i<count;++i) {
      pcl::PointCloud<PointType> cloud;
      for(int j=0;j<10;++j) { PointType p{}; p.x=j*.2f; p.y=1; cloud.push_back(p); }
      SavedPose pose; pose.stamp=i+1; pose.t.x()=i*.3;
      const int id=writer.append(cloud,pose,0);
      final.emplace_back(id,pose);
    }
    writer.finish(final);
  }
};
TEST_F(Fixture, StoresOnlyScansAndPosesAndLoadsMetadataWithoutClouds) {
  write();
  EXPECT_TRUE(std::filesystem::exists(root/"scans/0.pcd"));
  EXPECT_FALSE(std::filesystem::exists(root/"keyframes"));
  EXPECT_FALSE(std::filesystem::exists(root/"btc_frames.bin"));
  ScanMapStore store(root.string(),1024*1024);
  EXPECT_EQ(store.disk_reads(),0U);
  EXPECT_EQ(store.cache_entries(),0U);
  ASSERT_EQ(store.map().keyframes.size(),6U);
  EXPECT_FALSE(store.map().keyframes.front().cloud);
  EXPECT_EQ(store.scan(0)->size(),10U);
  EXPECT_EQ(store.disk_reads(),1U);
  store.scan(0); EXPECT_EQ(store.disk_reads(),1U);
}
TEST_F(Fixture, BoundedCacheEvictsAndReloadsAndWindowsPreserveCoordinates) {
  write();
  ScanMapStore store(root.string(),1);
  auto retained=store.scan(0);
  EXPECT_LE(store.resident_bytes(),1U);
  EXPECT_EQ(retained->size(),10U);
  store.scan(0); EXPECT_EQ(store.disk_reads(),2U);
  EXPECT_EQ(store.evictions(),2U);
  auto fused=store.fuse(2,3);
  ASSERT_EQ(fused->size(),30U);
  float minimum=std::numeric_limits<float>::infinity();
  for(const auto &p : *fused) minimum=std::min(minimum,p.x);
  EXPECT_NEAR(minimum,-.6,1e-6);
  EXPECT_THROW(store.fuse(1,3),std::runtime_error);
}
TEST_F(Fixture, HistoricalMotionGateIsIndependentOfWindow) {
  write();
  ScanMapStore store(root.string(),1024);
  auto history=store.history(3,.5,10);
  ASSERT_EQ(history.keyframes.size(),2U);
  EXPECT_EQ(history.keyframes[0].id,2);
  EXPECT_EQ(history.keyframes[1].id,4);
  EXPECT_THROW(store.history(0,.5,10),std::invalid_argument);
}
TEST_F(Fixture, RefusesOverwriteAndOldMapsAndDetectsCorruptClouds) {
  write();
  EXPECT_THROW(ScanMapWriter(root.string(),.1),std::runtime_error);
  std::filesystem::remove(root/"scans/2.pcd");
  EXPECT_THROW(ScanMapStore(root.string(),1024),std::filesystem::filesystem_error);
}
TEST_F(Fixture, TailUsesFinalAnchorCorrection) {
  ScanMapWriter writer(root.string(),.1);
  pcl::PointCloud<PointType> cloud; PointType p{}; p.x=1; cloud.push_back(p);
  SavedPose first; first.stamp=1;
  SavedPose last=first; last.stamp=2; last.t.x()=1;
  writer.append(cloud,first,0); writer.append(cloud,last,0);
  first.t.y()=5;
  writer.finish({{0,first}});
  ScanMapStore store(root.string(),1024);
  EXPECT_NEAR(store.map().keyframes[1].pose.t.x(),1,1e-9);
  EXPECT_NEAR(store.map().keyframes[1].pose.t.y(),5,1e-9);
}
}
