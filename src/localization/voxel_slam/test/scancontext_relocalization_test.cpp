#include "scancontext_relocalization.hpp"
#include <gtest/gtest.h>

TEST(ScanContextOptions, RejectsInvalidWindowBackendScalesAndThreads) {
  ScanContextOptions o;
  EXPECT_NO_THROW(o.validate());
  o.query_scans=0; EXPECT_THROW(o.validate(), std::invalid_argument);
  o=ScanContextOptions(); o.query_scans=31; EXPECT_THROW(o.validate(), std::invalid_argument);
  o=ScanContextOptions(); o.fpfh_radius_m=0.1; EXPECT_THROW(o.validate(), std::invalid_argument);
  o=ScanContextOptions(); o.distance_threshold=2; EXPECT_THROW(o.validate(), std::invalid_argument);
  o=ScanContextOptions(); o.threads=0; EXPECT_THROW(o.validate(), std::invalid_argument);
  o=ScanContextOptions(); o.candidate_max_distance_m=0; EXPECT_THROW(o.validate(), std::invalid_argument);
}

TEST(ScanContextFrames, LevelingAndDelayedCorrectionPreserveFullPoseConvention) {
  const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(1.2,Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.3,Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.2,Eigen::Vector3d::UnitX())).toRotationMatrix();
  const auto level=scancontext_level_rotation(rotation);
  const Eigen::Matrix3d yaw=rotation*level.transpose();
  EXPECT_NEAR((yaw*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm(),0,1e-12);
  EXPECT_NEAR((yaw*level-rotation).norm(),0,1e-12);
  Eigen::Isometry3d anchor=Eigen::Isometry3d::Identity(), coarse=anchor, query_level=anchor;
  anchor.linear()=yaw; anchor.translation()=Eigen::Vector3d(10,20,30);
  coarse.linear()=Eigen::AngleAxisd(-0.8,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  coarse.translation()=Eigen::Vector3d(1,2,3); query_level.linear()=level;
  const Eigen::Vector3d point(4,5,6);
  EXPECT_NEAR(((anchor*coarse*query_level)*point - anchor*(coarse*(level*point))).norm(),0,1e-12);
}

TEST(ScanContextHistory, FullBankIncludesLastFrameAndSupportsSmallMaps) {
  if(!scancontext_backend_available()) GTEST_SKIP() << "Optional TEASER++ backend not built";
  SavedMap map;
  pcl::PointCloud<pcl::PointXYZI> query;
  for(int f=0; f<2; ++f) {
    SavedKeyframe frame; frame.id=10+f; frame.cloud.reset(new pcl::PointCloud<PointType>);
    for(int r=1; r<=16; ++r) for(int s=0; s<60; ++s) {
      const double angle=s*2*M_PI/60;
      PointType p{}; p.x=r*3*std::cos(angle); p.y=r*3*std::sin(angle);
      p.z=0.1*r+std::sin((f+1)*angle)*r*0.2+0.03*s;
      frame.cloud->push_back(p);
      if(f==1) { pcl::PointXYZI q{}; q.x=p.x; q.y=p.y; q.z=p.z; query.push_back(q); }
    }
    map.keyframes.push_back(frame);
  }
  ScanContextRelocalizer relocalizer(map, ConfigSetting(), ScanContextOptions());
  const auto candidate=relocalizer.search(query);
  EXPECT_EQ(candidate.index,1);
  EXPECT_LT(candidate.distance,0.02);
  pcl::PointCloud<pcl::PointXYZI> empty;
  EXPECT_LT(relocalizer.search(empty).index,0);
  EXPECT_FALSE(relocalizer.match(empty,Eigen::Matrix3d::Identity()).success);
}

TEST(ScanContextHistory, RecoveryDistanceTrimsSelectedCandidateBeforeRegistration) {
  if(!scancontext_backend_available()) GTEST_SKIP();
  SavedMap map;SavedKeyframe frame;frame.id=7;frame.pose.t=Eigen::Vector3d(100,0,0);
  frame.cloud.reset(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZI> query;
  for(int r=1;r<=16;++r) for(int s=0;s<60;++s) {
    const double a=s*2*M_PI/60;PointType p{};
    p.x=r*3*std::cos(a);p.y=r*3*std::sin(a);p.z=.1*r+.2*r*std::sin(2*a)+.03*s;
    frame.cloud->push_back(p);pcl::PointXYZI q{};q.x=p.x;q.y=p.y;q.z=p.z;query.push_back(q);
  }
  map.keyframes.push_back(frame);
  ScanContextRelocalizer relocalizer(map,ConfigSetting(),ScanContextOptions());
  for(const auto &prediction : {Eigen::Vector3d(0,0,0),Eigen::Vector3d(65,0,0)}) {
    const auto result=relocalizer.match(query,Eigen::Matrix3d::Identity(),&prediction);
    EXPECT_FALSE(result.success);EXPECT_EQ(result.keyframe_id,7);
    EXPECT_EQ(result.reason,"Scan Context candidate outside predicted-position distance");
    EXPECT_GE(result.candidate_position_distance_m,35);
    EXPECT_DOUBLE_EQ(result.quatro_ms,0);EXPECT_DOUBLE_EQ(result.gicp_ms,0);
  }
  // Initial localization has no map-position prior, even for a distant map origin.
  const auto initial=relocalizer.match(query,Eigen::Matrix3d::Identity());
  EXPECT_EQ(initial.keyframe_id,7);
  EXPECT_LT(initial.candidate_position_distance_m,0);
  EXPECT_NE(initial.reason,"Scan Context candidate outside predicted-position distance");
  EXPECT_GT(initial.quatro_ms,0);
}
