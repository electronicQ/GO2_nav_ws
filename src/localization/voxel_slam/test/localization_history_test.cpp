#include "localization_history.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <chrono>
#include <fstream>
#include <pcl/io/pcd_io.h>

namespace {
SavedMap anchors() {
  SavedMap map;
  for(int i=0;i<3;++i) {
    SavedKeyframe key; key.id=100+i; key.pose.stamp=i;
    key.pose.t.x()=i;
    key.cloud.reset(new pcl::PointCloud<PointType>);
    PointType point{}; point.x=1; key.cloud->push_back(point);
    map.keyframes.push_back(key);
  }
  return map;
}
TEST(LocalizationHistory, ValidatesSavedAndIndividualWindows) {
  LocalizationHistoryOptions o;
  o.window_scans=3;
  EXPECT_THROW(o.validate(10), std::invalid_argument);
  o.source="individual_scans";
  EXPECT_THROW(o.validate(10), std::invalid_argument);
  o.scan_directory="/tmp";
  EXPECT_NO_THROW(o.validate(10));
  o.leaf_size_m=-1;
  EXPECT_THROW(o.validate(10), std::invalid_argument);
}
TEST(LocalizationHistory, FusionTransformsIntoFixedAnchorsWithoutChangingMap) {
  const auto original=anchors();
  LocalizationHistoryOptions o; o.window_scans=20;
  const auto history=build_localization_history(original,10,o);
  ASSERT_EQ(history.keyframes.size(),3U);
  EXPECT_EQ(history.keyframes[2].id,102);
  ASSERT_EQ(history.keyframes[2].cloud->size(),2U);
  EXPECT_FLOAT_EQ(history.keyframes[2].cloud->front().x,0);
  EXPECT_FLOAT_EQ(history.keyframes[2].cloud->back().x,1);
  EXPECT_EQ(original.keyframes[2].cloud->size(),1U);
}
TEST(LocalizationHistory, IndividualScansUsePoseRowsAndRejectMismatchedAnchors) {
  const auto directory=std::filesystem::temp_directory_path()/
      ("voxel_history_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  struct Cleanup { std::filesystem::path p; ~Cleanup(){std::filesystem::remove_all(p);} } cleanup{directory};
  {
    std::ofstream poses(directory/"alidarState.txt");
    for(int i=0;i<3;++i) {
      poses<<i<<" "<<i<<" 0 0 0 0 0 1\n";
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::PointXYZI point{}; point.x=1; cloud.push_back(point);
      pcl::io::savePCDFileBinary((directory/(std::to_string(i)+".pcd")).string(),cloud);
    }
  }
  auto map=anchors();
  LocalizationHistoryOptions o; o.source="individual_scans"; o.window_scans=2;
  o.scan_directory=directory.string();
  const auto history=build_localization_history(map,10,o);
  ASSERT_EQ(history.keyframes[2].cloud->size(),2U);
  EXPECT_FLOAT_EQ(history.keyframes[2].cloud->front().x,0);
  map.keyframes[2].pose.t.x()+=1;
  EXPECT_THROW(build_localization_history(map,10,o),std::runtime_error);
  map=anchors();
  std::filesystem::remove(directory/"2.pcd");
  EXPECT_THROW(build_localization_history(map,10,o),std::runtime_error);
}
TEST(LocalizationHistory, ZeroLeafPreservesCloudAndPositiveLeafReducesPoints) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointXYZI p{}; p.x=0.01; cloud->push_back(p); p.x=0.02; cloud->push_back(p);
  EXPECT_EQ(downsample_localization_cloud(cloud,0),cloud);
  EXPECT_EQ(downsample_localization_cloud(cloud,0.1)->size(),1U);
  EXPECT_EQ(cloud->size(),2U);
}
}
