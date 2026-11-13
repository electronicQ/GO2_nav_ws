#pragma once

#include "dual_tf.hpp"
#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <vector>

struct GlobalMatchAudit {
  std::size_t matches = 0;
  double overlap = 0;
  double rmse = std::numeric_limits<double>::infinity();
  double p95 = std::numeric_limits<double>::infinity();
  double normal_eigenvalue = 0;
  double spatial_eigenvalue = 0;
  bool passed = false;
};

// Evaluate a frozen transform. No ICP or state update is allowed in this audit.
inline GlobalMatchAudit audit_global_match(
    const pcl::PointCloud<pcl::PointXYZINormal> &source,
    const pcl::PointCloud<pcl::PointXYZINormal> &target,
    const Eigen::Isometry3d &target_from_source, const DualTfOptions &options) {
  GlobalMatchAudit result;
  if(source.empty() || target.empty() || !target_from_source.matrix().allFinite()) return result;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<std::size_t> ids;
  for(std::size_t i = 0; i < target.size(); ++i) {
    if(!target[i].getVector3fMap().allFinite() || !target[i].getNormalVector3fMap().allFinite()) continue;
    points->push_back(pcl::PointXYZ(target[i].x, target[i].y, target[i].z));
    ids.push_back(i);
  }
  if(points->empty()) return result;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(points);
  std::vector<int> index(1);
  std::vector<float> squared_distance(1);
  std::vector<double> residuals;
  Eigen::Matrix3d normal_information = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d second_moment = Eigen::Matrix3d::Zero();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  double square_sum = 0;
  for(const auto &p : source) {
    if(!p.getVector3fMap().allFinite() || !p.getNormalVector3fMap().allFinite()) continue;
    const Eigen::Vector3d x = target_from_source * p.getVector3fMap().cast<double>();
    Eigen::Vector3d normal = target_from_source.linear() * p.getNormalVector3fMap().cast<double>();
    if(!x.allFinite() || !normal.allFinite()) continue;
    if(normal.norm() < 1e-6) continue;
    normal.normalize();
    if(!tree.nearestKSearch(pcl::PointXYZ(x.x(), x.y(), x.z()), 1, index, squared_distance) ||
       squared_distance[0] > options.audit_max_point_distance_m * options.audit_max_point_distance_m) continue;
    const auto &q = target[ids[index[0]]];
    Eigen::Vector3d n = q.getNormalVector3fMap().cast<double>();
    if(n.norm() < 1e-6) continue;
    n.normalize();
    if(std::min((normal - n).norm(), (normal + n).norm()) > 0.2) continue;
    const double residual = std::abs(n.dot(x - q.getVector3fMap().cast<double>()));
    if(residual > options.audit_max_plane_distance_m) continue;
    residuals.push_back(residual);
    square_sum += residual * residual;
    normal_information += n * n.transpose();
    // Coordinates relative to the target frame keep this covariance translation invariant.
    mean += x;
    second_moment += x * x.transpose();
  }
  result.matches = residuals.size();
  result.overlap = double(result.matches) / source.size();
  if(result.matches < 3) return result;
  result.rmse = std::sqrt(square_sum / result.matches);
  std::sort(residuals.begin(), residuals.end());
  result.p95 = residuals[std::size_t(std::ceil(0.95 * residuals.size())) - 1];
  mean /= result.matches;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normals(normal_information / result.matches);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spatial(second_moment / result.matches - mean * mean.transpose());
  if(normals.info() != Eigen::Success || spatial.info() != Eigen::Success) return result;
  result.normal_eigenvalue = normals.eigenvalues()[0];
  result.spatial_eigenvalue = spatial.eigenvalues()[1];
  result.passed = result.matches >= std::size_t(options.audit_min_matches) &&
      result.overlap >= options.audit_min_overlap && result.rmse <= options.audit_max_rmse_m &&
      result.p95 <= options.audit_max_p95_m &&
      result.normal_eigenvalue >= options.audit_min_normal_eigenvalue &&
      result.spatial_eigenvalue >= options.audit_min_spatial_eigenvalue_m2;
  return result;
}

inline bool audit_improves_prior(const GlobalMatchAudit &candidate,
                                const GlobalMatchAudit &prior, double displacement,
                                double rotation_deg, const DualTfOptions &options) {
  if(!candidate.passed) return false;
  if(displacement <= options.audit_prior_comparison_m && rotation_deg <= options.large_rotation_deg)
    return true;
  // A good incumbent must not be replaced by an equally plausible large correction.
  return !prior.passed || candidate.overlap > prior.overlap + 0.1 ||
      (candidate.overlap >= prior.overlap * 0.9 &&
       candidate.rmse + options.audit_improvement_margin_m < prior.rmse);
}
