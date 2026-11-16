#include "dual_tf.hpp"
#include "global_match_audit.hpp"
#include <gtest/gtest.h>

namespace {
// Existing transform tests explicitly supply successful post-query evidence.
struct AuditedAlignment : DualTfAlignment {
  using DualTfAlignment::DualTfAlignment;
  bool accept(std::uint64_t epoch, double stamp, double now,
              const Eigen::Isometry3d &map, const Eigen::Isometry3d &odom, std::string &reason) {
    return DualTfAlignment::accept(epoch, stamp, now + 0.5,
                                   map, odom, reason, stamp + 0.5);
  }
};
Eigen::Isometry3d pose(double x, double yaw = 0) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation().x() = x;
  p.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}
TEST(DualTf, LateQueryCorrectsCurrentPoseNotFrontend) {
  AuditedAlignment a;
  std::string reason;
  auto local_at_nine = pose(20), local_at_ten = pose(22);
  ASSERT_TRUE(a.accept(0, 9, 10, pose(100), local_at_nine, reason)) << reason;
  const auto view = a.view(10);
  EXPECT_DOUBLE_EQ((view.map_from_odom * local_at_ten).translation().x(), 102);
  EXPECT_DOUBLE_EQ(local_at_ten.translation().x(), 22);
}
TEST(DualTf, RotationAndMotionComposeInCorrectOrder) {
  AuditedAlignment a;
  std::string reason;
  const auto correction = pose(10, M_PI / 2), local = pose(3, .2);
  ASSERT_TRUE(a.accept(0, 1, 1, correction * local, local, reason));
  EXPECT_TRUE(a.view(1).map_from_odom.matrix().isApprox(correction.matrix()));
  EXPECT_NEAR((a.view(1).map_from_odom * pose(4)).translation().y(), 4, 1e-9);
}
TEST(DualTf, StartsUnalignedAndReportsStaleWithoutDroppingTransform) {
  AuditedAlignment a;
  std::string reason;
  EXPECT_FALSE(a.view(1).aligned);
  ASSERT_TRUE(a.accept(0, 1, 2, pose(10), pose(0), reason));
  EXPECT_FALSE(a.view(11).stale);
  EXPECT_TRUE(a.view(12).stale);
  EXPECT_TRUE(a.view(12).aligned);
  EXPECT_DOUBLE_EQ(a.view(12).map_from_odom.translation().x(), 10);
}
TEST(DualTf, RejectsOldEpochOldTimestampFutureNonFiniteAndDelayedResults) {
  AuditedAlignment a;
  std::string reason;
  EXPECT_FALSE(a.accept(1, 1, 1, pose(1), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 2, 1, pose(1), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 1, 7, pose(1), pose(0), reason));
  auto invalid = pose(0);
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(a.accept(0, 1, 1, invalid, pose(0), reason));
  ASSERT_TRUE(a.accept(0, 2, 2, pose(1), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 1, 2, pose(1), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 2, 2, pose(1), pose(0), reason));
}
TEST(DualTf, LargeCorrectionNeedsFreshConsistentQueriesNotStationaryPositions) {
  AuditedAlignment a;
  std::string reason;
  ASSERT_TRUE(a.accept(0, 1, 1, pose(10), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 2, 2, pose(20), pose(0), reason));
  EXPECT_TRUE(a.view(2.5).confirmation_pending);
  EXPECT_FALSE(a.view(8).confirmation_pending);
  EXPECT_DOUBLE_EQ(a.view(2).map_from_odom.translation().x(), 10);
  // Vehicle moved 5 m; the correction is still exactly the same 20 m.
  ASSERT_TRUE(a.accept(0, 3, 3, pose(25), pose(5), reason)) << reason;
  EXPECT_DOUBLE_EQ(a.view(3).map_from_odom.translation().x(), 20);
  EXPECT_FALSE(a.view(3).confirmation_pending);
}
TEST(DualTf, ExpiredOrInconsistentLargeCorrectionsDoNotConfirm) {
  AuditedAlignment a;
  std::string reason;
  ASSERT_TRUE(a.accept(0, 1, 1, pose(0), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 2, 2, pose(10), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 8, 8, pose(10), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 9, 9, pose(20), pose(0), reason));
  EXPECT_DOUBLE_EQ(a.view(9).map_from_odom.translation().x(), 0);
  a.reset(1);
  EXPECT_FALSE(a.view(10).aligned);
  EXPECT_FALSE(a.accept(0, 10, 10, pose(20), pose(0), reason));
  EXPECT_TRUE(a.accept(1, 10, 10, pose(20), pose(0), reason));
}
TEST(DualTf, LargeRotationAlsoRequiresConfirmation) {
  AuditedAlignment a;
  std::string reason;
  ASSERT_TRUE(a.accept(0, 1, 1, pose(0), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 2, 2, pose(0, .5), pose(0), reason));
  EXPECT_TRUE(a.view(2).map_from_odom.matrix().isApprox(pose(0).matrix()));
  ASSERT_TRUE(a.accept(0, 3, 3, pose(0, .51), pose(0), reason));
  EXPECT_TRUE(a.view(3).map_from_odom.matrix().isApprox(pose(0, .51).matrix()));
}
TEST(DualTf, RejectsInvalidConfiguration) {
  DualTfOptions options;
  options.max_result_age_sec = -1;
  EXPECT_THROW(DualTfAlignment{options}, std::invalid_argument);
}
TEST(DualTf, CovarianceReordersPositionAndRotatesBodyOrientationErrors) {
  Eigen::Matrix<double, 6, 6> p = Eigen::Matrix<double, 6, 6>::Zero();
  p.diagonal() << 1, 2, 3, 4, 5, 6;
  const auto transformed = dual_tf_pose_covariance(p, pose(0, M_PI / 2).linear());
  Eigen::Matrix<double, 6, 1> expected;
  expected << 4, 5, 6, 2, 1, 3;
  EXPECT_TRUE(transformed.diagonal().isApprox(expected));
}
TEST(DualTf, FiniteInputsCannotOverflowThePublishedCorrection) {
  AuditedAlignment a;
  std::string reason;
  auto world = pose(1e308), local = pose(-1e308);
  EXPECT_FALSE(a.accept(0, 1, 1, world, local, reason));
  EXPECT_FALSE(a.view(1).aligned);
}
TEST(DualTf, CannotPublishWithoutNewAuditedScans) {
  DualTfAlignment a;
  std::string reason;
  EXPECT_FALSE(a.accept(0, 1, 2, pose(10), pose(0), reason));
  EXPECT_FALSE(a.accept(0, 1, 2, pose(10), pose(0), reason, 1.1));
  EXPECT_FALSE(a.accept(0, 1, 2, pose(10), pose(0), reason, 3));
  EXPECT_FALSE(a.view(2).aligned);
  EXPECT_TRUE(a.accept(0, 1, 2, pose(10), pose(0), reason, 2));
}
TEST(DualTf, GateMeasuresDisplacementAtVehicleNotOdomOrigin) {
  AuditedAlignment a;
  std::string reason;
  const auto local = pose(1000);
  ASSERT_TRUE(a.accept(0, 1, 1, local, local, reason));
  const auto rotation = pose(0, .002);
  // Only 0.11 degrees but a 2 m position jump at the vehicle: must be pending.
  EXPECT_FALSE(a.accept(0, 2, 2, rotation * local, local, reason));
  EXPECT_TRUE(a.view(2).map_from_odom.matrix().isApprox(pose(0).matrix()));
}
TEST(DualTf, RecordedStadtgartenTwoMeterCorrectionCannotImmediatelyReachOutput) {
  auto from_record = [](double x, double y, double z, double qx, double qy, double qz, double qw) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, y, z);
    t.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    return t;
  };
  // Frozen old/new map->odom and local pose from the 801.45 s baseline jump.
  const auto old = from_record(-.9220508139249262, 1.8235716323089406, -2.015881381266066,
      .005980413735688502, .0004070226278378741, -.01089731015012318, .9999226558167499);
  const auto bad = from_record(1.206338010736772, 1.4806240547200389, -1.6763042392666634,
      .004186280581035228, .0006727682010779774, -.010562796899723368, .9999352227817059);
  const auto local = from_record(183.18903952101624, 72.22662612380027, 32.17116523520297,
      .08240229304935576, -.2905054097327362, .24127150222001256, .922282240549346);
  EXPECT_NEAR(((bad * local).translation() - (old * local).translation()).norm(), 2.10357791486, 1e-8);
  DualTfAlignment alignment;
  std::string reason;
  ASSERT_TRUE(alignment.accept(0, 1, 2, old * local, local, reason, 2));
  // Even granting a successful future audit, this one-off proposal stays pending.
  EXPECT_FALSE(alignment.accept(0, 3, 4, bad * local, local, reason, 4));
  EXPECT_EQ(reason, "large correction awaiting a second consistent fresh query");
  EXPECT_TRUE(alignment.view(4).map_from_odom.matrix().isApprox(old.matrix()));
}
TEST(GlobalAudit, ExactDistributedThreePlaneGeometryPassesAndWrongTransformFails) {
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  for(int axis = 0; axis < 3; ++axis)
    for(int i = -5; i <= 5; ++i)
      for(int j = -5; j <= 5; ++j) {
        pcl::PointXYZINormal p;
        p.getVector3fMap().setZero(); p.getNormalVector3fMap().setZero();
        p.getVector3fMap()[(axis + 1) % 3] = i * .3;
        p.getVector3fMap()[(axis + 2) % 3] = j * .3;
        p.getNormalVector3fMap()[axis] = 1;
        cloud.push_back(p);
      }
  const auto original = cloud;
  DualTfOptions options;
  EXPECT_TRUE(audit_global_match(cloud, cloud, pose(0), options).passed);
  EXPECT_FALSE(audit_global_match(cloud, cloud, pose(3), options).passed);
  EXPECT_TRUE(cloud[0].getVector3fMap().isApprox(original[0].getVector3fMap()));
  pcl::PointCloud<pcl::PointXYZINormal> empty;
  EXPECT_FALSE(audit_global_match(empty, cloud, pose(0), options).passed);
}
TEST(GlobalAudit, SinglePlaneIsNotEnoughEvenWithPerfectResidual) {
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  for(int i = 0; i < 100; ++i) {
    pcl::PointXYZINormal p;
    p.x = (i % 10) * .3; p.y = (i / 10) * .3; p.z = 0;
    p.normal_x = 0; p.normal_y = 0; p.normal_z = 1;
    cloud.push_back(p);
  }
  const auto result = audit_global_match(cloud, cloud, pose(0), DualTfOptions());
  EXPECT_DOUBLE_EQ(result.overlap, 1);
  EXPECT_DOUBLE_EQ(result.rmse, 0);
  EXPECT_FALSE(result.passed);
}
TEST(GlobalAudit, LargeChangeNeedsImprovementOverGoodIncumbent) {
  GlobalMatchAudit candidate, prior;
  candidate.passed = prior.passed = true;
  candidate.overlap = prior.overlap = .8;
  candidate.rmse = prior.rmse = .03;
  DualTfOptions options;
  EXPECT_FALSE(audit_improves_prior(candidate, prior, 2, 0, options));
  EXPECT_TRUE(audit_improves_prior(candidate, prior, .1, 0, options));
  prior.rmse = .07;
  EXPECT_TRUE(audit_improves_prior(candidate, prior, 2, 0, options));
  candidate.passed = false;
  EXPECT_FALSE(audit_improves_prior(candidate, prior, .1, 0, options));
}
}  // namespace

TEST(DualTfScanContext, InitialAndSmallContinuousCommitWithoutPlaneEvidenceOrConfirmation) {
  DualTfAlignment a;
  std::string reason;
  ASSERT_TRUE(a.accept_scancontext(0, 1, 1.1, pose(100), pose(5), reason)) << reason;
  EXPECT_NEAR(a.view(1.1).map_from_odom.translation().x(), 95, 1e-12);
  ASSERT_TRUE(a.accept_scancontext(0, 2, 2.1, pose(101.2), pose(6), reason)) << reason;
  EXPECT_NEAR(a.view(2.1).map_from_odom.translation().x(), 95.2, 1e-12);
  EXPECT_FALSE(a.view(2.1).confirmation_pending);
  // Local motion is untouched; the accepted relation corrects the current pose.
  EXPECT_NEAR((a.view(2.1).map_from_odom * pose(7)).translation().x(), 102.2, 1e-12);
  EXPECT_FALSE(a.accept(0, 3, 3.1, pose(121), pose(7), reason));
  EXPECT_EQ(reason, "missing fresh post-query geometric audit");
}

TEST(DualTfScanContext, RetainsSessionFreshnessOrderingAndRigidTransformChecks) {
  DualTfAlignment a;
  std::string reason;
  EXPECT_FALSE(a.accept_scancontext(1, 1, 1.1, pose(1), pose(0), reason));
  EXPECT_FALSE(a.accept_scancontext(0, 2, 1.1, pose(1), pose(0), reason));
  EXPECT_FALSE(a.accept_scancontext(0, 1, 6.1, pose(1), pose(0), reason));
  auto bad=pose(1);bad.linear()(0,0)=2;
  EXPECT_FALSE(a.accept_scancontext(0, 1, 1.1, bad, pose(0), reason));
  ASSERT_TRUE(a.accept_scancontext(0, 2, 2.1, pose(10), pose(0), reason));
  EXPECT_FALSE(a.accept_scancontext(0, 2, 2.2, pose(11), pose(0), reason));
  EXPECT_FALSE(a.accept_scancontext(0, 1, 2.2, pose(11), pose(0), reason));
  a.reset(1);
  EXPECT_FALSE(a.accept_scancontext(0, 3, 3.1, pose(11), pose(0), reason));
  ASSERT_TRUE(a.accept_scancontext(1, 3, 3.1, pose(11), pose(0), reason));
}

TEST(DualTfScanContext, RejectsJumpWithoutChangingAlignmentOrFreshness) {
  DualTfAlignment a; std::string reason;
  ASSERT_TRUE(a.accept_scancontext(0, 1, 1.1, pose(100), pose(5), reason));
  ASSERT_FALSE(a.accept_scancontext(0, 2, 2.1, pose(102), pose(6), reason));
  EXPECT_NE(reason.find("SC correction jump exceeds limit"), std::string::npos);
  EXPECT_DOUBLE_EQ(a.view(2.1).last_correction_stamp, 1);
  EXPECT_DOUBLE_EQ(a.view(2.1).map_from_odom.translation().x(), 95);
  EXPECT_FALSE(a.view(2.1).confirmation_pending);
  // A rejected result cannot be replayed with a changed transform.
  EXPECT_FALSE(a.accept_scancontext(0, 2, 2.2, pose(101), pose(6), reason));
  // New queries can still commit against the retained alignment.
  EXPECT_TRUE(a.accept_scancontext(0, 3, 3.1, pose(102.5), pose(7), reason));
}
TEST(DualTfScanContext, RotationGateAndQueryAnchorRatherThanOdomOrigin) {
  DualTfAlignment a; std::string reason;
  ASSERT_TRUE(a.accept_scancontext(0, 1, 1.1, pose(1000), pose(1000), reason));
  EXPECT_FALSE(a.accept_scancontext(0, 2, 2.1, pose(1000, 11*M_PI/180), pose(1000), reason));
  // Rotating at this distant query anchor creates a large map->odom translation;
  // the actual query position does not move and must remain acceptable.
  ASSERT_TRUE(a.accept_scancontext(0, 3, 3.1, pose(1000, 5*M_PI/180), pose(1000), reason));
  EXPECT_GT(a.view(3.1).map_from_odom.translation().norm(), 50);
}
TEST(DualTfScanContext, ConfigurableLimitsAndEpochReset) {
  DualTfOptions o; o.sc_max_correction_translation_m=2; o.sc_max_correction_rotation_deg=30;
  DualTfAlignment a(o); std::string reason;
  ASSERT_TRUE(a.accept_scancontext(0, 1, 1.1, pose(0), pose(0), reason));
  EXPECT_TRUE(a.accept_scancontext(0, 2, 2.1, pose(1, 20*M_PI/180), pose(0), reason));
  a.reset(1);
  EXPECT_TRUE(a.accept_scancontext(1, 3, 3.1, pose(100, 170*M_PI/180), pose(0), reason));
  o.sc_max_correction_translation_m=0; EXPECT_THROW(o.validate(),std::invalid_argument);
  o.sc_max_correction_translation_m=1; o.sc_max_correction_rotation_deg=181;
  EXPECT_THROW(o.validate(),std::invalid_argument);
}

TEST(DualTfScanContext, DisabledJumpCheckAllowsLargeCorrectionsButRetainsResultChecks) {
  DualTfOptions o; o.sc_correction_jump_check_enabled=false;
  DualTfAlignment a(o); std::string reason;
  ASSERT_TRUE(a.accept_scancontext(0, 1, 1.1, pose(0), pose(0), reason));
  ASSERT_TRUE(a.accept_scancontext(0, 2, 2.1, pose(100, 90*M_PI/180), pose(0), reason));
  EXPECT_DOUBLE_EQ(a.view(2.1).map_from_odom.translation().x(), 100);
  EXPECT_FALSE(a.accept_scancontext(0, 2, 2.2, pose(200), pose(0), reason));
  EXPECT_FALSE(a.accept_scancontext(1, 3, 3.1, pose(200), pose(0), reason));
  EXPECT_FALSE(a.accept_scancontext(0, 3, 9, pose(200), pose(0), reason));
  auto bad=pose(200);bad.linear()(0,0)=2;
  EXPECT_FALSE(a.accept_scancontext(0, 3, 3.1, bad, pose(0), reason));
  // Re-enabling checks retains the established relation and applies the limits.
  o.sc_correction_jump_check_enabled=true; a.configure(o);
  EXPECT_FALSE(a.accept_scancontext(0, 4, 4.1, pose(200), pose(0), reason));
  EXPECT_NE(reason.find("SC correction jump exceeds limit"),std::string::npos);
  EXPECT_DOUBLE_EQ(a.view(4.1).last_correction_stamp, 2);
}

TEST(DualTf, ManualPoseOverridesWrongAlignmentAndRetainsBaseExtrinsic) {
  DualTfAlignment a;
  std::string reason;
  ASSERT_TRUE(a.accept_scancontext(0, 1, 1, pose(100), pose(0), reason));
  const auto local_imu = pose(5, .2);
  auto imu_base = pose(-.1);
  imu_base.translation().z() = -.65;
  const Eigen::Isometry3d local_base = local_imu * imu_base;
  const auto desired = dual_tf_planar_base_pose(a.view(2).map_from_odom * local_base,
                                               Eigen::Vector2d(10, 20), 1.1);
  ASSERT_TRUE(a.accept_manual(0, 2, 2, desired, local_base, reason)) << reason;
  const auto corrected = a.view(2).map_from_odom * local_base;
  EXPECT_NEAR(corrected.translation().x(), 10, 1e-9);
  EXPECT_NEAR(corrected.translation().y(), 20, 1e-9);
  EXPECT_NEAR(corrected.translation().z(), -.65, 1e-9);
  EXPECT_NEAR(std::atan2(corrected.linear()(1,0), corrected.linear()(0,0)), 1.1, 1e-9);
  EXPECT_DOUBLE_EQ(local_imu.translation().x(), 5);
  EXPECT_FALSE(a.view(2).confirmation_pending);
  // The automatic computation started before the click must not undo it,
  // even when its fresh audit samples were collected after the click.
  EXPECT_FALSE(a.accept(0, 1.9, 3, pose(100), pose(0), reason, 2.9));
  EXPECT_FALSE(a.accept_scancontext(0, 2, 3, pose(100), pose(0), reason));
  ASSERT_TRUE(a.accept_scancontext(0, 3, 3, a.view(3).map_from_odom * local_imu,
                                  local_imu, reason)) << reason;
}

TEST(DualTf, ManualPoseWorksBeforeAutomaticLocalizationAndRejectsInvalidInputs) {
  DualTfAlignment a;
  a.reset(4);
  std::string reason;
  ASSERT_TRUE(a.accept_manual(4, 10, 10, pose(20), pose(3), reason));
  EXPECT_TRUE(a.view(10).aligned);
  EXPECT_NEAR(a.view(10).map_from_odom.translation().x(), 17, 1e-9);
  const auto revision = a.view(10).manual_revision;
  EXPECT_FALSE(a.accept_manual(3, 10, 10, pose(30), pose(3), reason));
  EXPECT_FALSE(a.accept_manual(4, 1, 10, pose(30), pose(3), reason));
  EXPECT_FALSE(a.accept_manual(4, 12, 10, pose(30), pose(3), reason));
  auto bad = pose(0); bad.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(a.accept_manual(4, 10, 10, bad, pose(3), reason));
  EXPECT_EQ(a.view(10).manual_revision, revision);
  a.reset(5);
  EXPECT_FALSE(a.view(10).aligned);
  // Clock reset in a new local session removes the old timestamp barrier.
  EXPECT_TRUE(a.accept_scancontext(5, 1, 1, pose(0), pose(0), reason));
}

TEST(DualTf, PlanarManualPosePreservesHeightAndTilt) {
  Eigen::Isometry3d current = pose(3, .4);
  current.linear() *= Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  current.linear() *= Eigen::AngleAxisd(-.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  current.translation().z() = 1.7;
  const auto desired = dual_tf_planar_base_pose(current, Eigen::Vector2d(8, 9), -.6);
  EXPECT_NEAR(desired.translation().z(), 1.7, 1e-12);
  EXPECT_TRUE(desired.linear().row(2).isApprox(current.linear().row(2), 1e-12));
  EXPECT_NEAR(std::atan2(desired.linear()(1,0), desired.linear()(0,0)), -.6, 1e-12);
  EXPECT_THROW(dual_tf_planar_base_pose(current, Eigen::Vector2d(0, 0),
                 std::numeric_limits<double>::infinity()), std::invalid_argument);
}
