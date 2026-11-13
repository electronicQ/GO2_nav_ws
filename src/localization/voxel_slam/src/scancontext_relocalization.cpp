#include "scancontext_relocalization.hpp"
#include "direct_matching.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <pcl/filters/voxel_grid.h>
#ifdef VOXEL_SLAM_HAS_SC_QN
#include "Scancontext.h"
#include <quatro/quatro_module.h>
#include <omp.h>
#endif

namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
double milliseconds(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}
Cloud::Ptr prepare(const Cloud &input, const Eigen::Matrix3d &rotation, double radius, double leaf) {
  Cloud::Ptr finite(new Cloud), output(new Cloud);
  for(const auto &p : input) {
    if(!p.getVector3fMap().allFinite()) continue;
    const Eigen::Vector3d v = rotation * p.getVector3fMap().cast<double>();
    if(v.head<2>().norm() > radius || v.head<2>().norm() < 1e-6) continue;
    pcl::PointXYZI q{}; q.x = v.x(); q.y = v.y(); q.z = v.z(); q.intensity = p.intensity;
    finite->push_back(q);
  }
  if(leaf == 0) return finite;
  if(!finite->empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(finite); filter.setLeafSize(leaf, leaf, leaf); filter.filter(*output);
  }
  return output;
}
}

void ScanContextOptions::validate() const {
  if(query_scans < 1 || query_scans > 30 || query_stride_scans < 1 || query_stride_scans > 30 ||
     candidates < 1 || candidates > 100 || threads < 1 || threads > 64 || max_correspondences < 10)
    throw std::invalid_argument("invalid Scan Context limits");
  for(double v : {distance_threshold, max_radius_m, registration_leaf_size_m, normal_radius_m, fpfh_radius_m,
                 noise_bound_m, gicp_max_correspondence_m, gicp_fitness_threshold, candidate_max_distance_m})
    if(!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid Scan Context threshold");
  if(!std::isfinite(history_leaf_size_m) || history_leaf_size_m < 0 ||
     !std::isfinite(query_leaf_size_m) || query_leaf_size_m < 0)
    throw std::invalid_argument("invalid Scan Context voxel leaf");
  if(distance_threshold > 1 || fpfh_radius_m < 1.5 * normal_radius_m ||
     normal_radius_m <= registration_leaf_size_m || !std::isfinite(lidar_height_m))
    throw std::invalid_argument("inconsistent Scan Context feature scales");
}

ScanContextOptions read_scancontext_options(ros::NodeHandle &node) {
  ScanContextOptions options;
#define SC_PARAM(field) node.param("Localization/scancontext/" #field, options.field, options.field)
  SC_PARAM(query_scans); SC_PARAM(query_stride_scans); SC_PARAM(candidates);
  SC_PARAM(distance_threshold); SC_PARAM(max_radius_m); SC_PARAM(lidar_height_m);
  SC_PARAM(leaf_size_m); SC_PARAM(normal_radius_m); SC_PARAM(fpfh_radius_m);
  SC_PARAM(noise_bound_m); SC_PARAM(max_correspondences); SC_PARAM(threads);
  SC_PARAM(gicp_max_correspondence_m); SC_PARAM(gicp_fitness_threshold); SC_PARAM(candidate_max_distance_m);
#undef SC_PARAM
  options.registration_leaf_size_m = options.leaf_size_m;
  node.param("Localization/history/leaf_size_m", options.history_leaf_size_m, options.history_leaf_size_m);
  node.param("Localization/scancontext/query_leaf_size_m", options.query_leaf_size_m, options.query_leaf_size_m);
  node.param("Localization/scancontext/registration_leaf_size_m", options.registration_leaf_size_m, options.registration_leaf_size_m);
  options.validate(); return options;
}

Eigen::Matrix3d scancontext_level_rotation(const Eigen::Matrix3d &world_from_body) {
  const double yaw = std::atan2(world_from_body(1, 0), world_from_body(0, 0));
  return Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * world_from_body;
}

#ifdef VOXEL_SLAM_HAS_SC_QN
bool scancontext_backend_available() { return true; }
struct ScanContextRelocalizer::Impl {
  const SavedMap &map;
  ScanContextOptions options;
  SCManager manager;
  std::unique_ptr<InvKeyTree> tree;
  MapCloudLoader loader;
  std::vector<Eigen::Isometry3d> poses;
  Impl(const SavedMap &m, ConfigSetting, ScanContextOptions o, MapCloudLoader l)
      : map(m), options(o), loader(std::move(l)) {
    options.validate();
    manager.setMaximumRadius(options.max_radius_m);
    manager.LIDAR_HEIGHT = options.lidar_height_m;
    for(const auto &frame : map.keyframes) {
      const auto raw = frame.cloud ? frame.cloud : loader(frame.id);
      if(!raw || raw->empty()) throw std::invalid_argument("empty Scan Context history keyframe");
      Cloud input;
      for(const auto &p : *raw) {
        pcl::PointXYZI q{}; q.x=p.x; q.y=p.y; q.z=p.z; q.intensity=p.intensity; input.push_back(q);
      }
      const Eigen::Matrix3d level = scancontext_level_rotation(frame.pose.R);
      auto cloud = prepare(input, level, options.max_radius_m, options.history_leaf_size_m);
      if(cloud->empty()) throw std::invalid_argument("empty finite Scan Context history cloud");

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = frame.pose.R * level.transpose(); pose.translation() = frame.pose.t;
      poses.push_back(pose);
      manager.makeAndSaveScancontextAndKeys(*cloud);
    }
    if(poses.empty()) throw std::invalid_argument("empty Scan Context history bank");
    // Historical localization has no recent-loop exclusion; query all saved keyframes.
    tree.reset(new InvKeyTree(manager.PC_NUM_RING, manager.polarcontext_invkeys_mat_, 10));
  }
  ScanContextCandidate find(const Cloud &query, double *descriptor_ms = nullptr, double *search_ms = nullptr) {
    auto start = std::chrono::steady_clock::now();
    auto descriptor = manager.makeScancontext(query);
    auto key = manager.makeRingkeyFromScancontext(descriptor);
    auto values = eig2stdvec(key);
    if(descriptor_ms) *descriptor_ms = milliseconds(start);
    ScanContextCandidate result;
    if(!descriptor.allFinite() || descriptor.norm() < 1e-9) return result;
    start = std::chrono::steady_clock::now();
    const size_t k = std::min(size_t(options.candidates), poses.size());
    std::vector<size_t> indices(k); std::vector<float> distances(k);
    nanoflann::KNNResultSet<float> nearest(k);
    nearest.init(indices.data(), distances.data());
    tree->index->findNeighbors(nearest, values.data(), nanoflann::SearchParams(10));
    double best = options.distance_threshold;
    for(size_t j=0; j<nearest.size(); ++j) {
      auto &history = manager.polarcontexts_.at(indices[j]);
      const auto distance = manager.distanceBtnScanContext(descriptor, history);
      if(std::isfinite(distance.first) && distance.first < best) {
        best=distance.first; result.index=int(indices[j]); result.distance=best;
        result.yaw_rad=distance.second * manager.PC_UNIT_SECTORANGLE * M_PI / 180.0;
      }
    }
    if(search_ms) *search_ms = milliseconds(start);
    return result;
  }
};

ScanContextRelocalizer::ScanContextRelocalizer(const SavedMap &map, ConfigSetting config, ScanContextOptions options, MapCloudLoader loader)
    : impl_(new Impl(map, config, options, std::move(loader))) {}
ScanContextRelocalizer::~ScanContextRelocalizer() = default;
ScanContextCandidate ScanContextRelocalizer::search(const Cloud &query) { return impl_->find(query); }
ScanContextResult ScanContextRelocalizer::match(const Cloud &query, const Eigen::Matrix3d &odom_from_body,
    const Eigen::Vector3d *predicted_map_position) {
  ScanContextResult result;
  auto &p = *impl_; const auto &o = p.options;
  if(!odom_from_body.allFinite()) { result.reason="non-finite query orientation"; return result; }
  const auto level = scancontext_level_rotation(odom_from_body);
  auto source = prepare(query, level, o.max_radius_m, o.query_leaf_size_m);
  if(source->size() < 100) { result.reason="insufficient finite query points"; return result; }
  const auto candidate = p.find(*source, &result.descriptor_ms, &result.search_ms);
  result.distance = candidate.distance;
  if(candidate.index < 0) { result.reason="Scan Context found no candidate"; return result; }
  result.keyframe_id = p.map.keyframes.at(candidate.index).id;
  if(predicted_map_position) {
    result.candidate_position_distance_m =
        (p.poses.at(candidate.index).translation() - *predicted_map_position).norm();
    if(!predicted_map_position->allFinite() ||
       !std::isfinite(result.candidate_position_distance_m) ||
       result.candidate_position_distance_m >= o.candidate_max_distance_m) {
      result.reason="Scan Context candidate outside predicted-position distance";
      return result;
    }
  }
  const auto target_started = std::chrono::steady_clock::now();
  auto registration_source = prepare(query, level, o.max_radius_m, o.registration_leaf_size_m);
  auto history_cloud = [&](size_t i, double leaf) {
    const auto &frame=p.map.keyframes.at(i);
    auto raw=frame.cloud ? frame.cloud : p.loader(frame.id);
    Cloud input;
    for(const auto &point : *raw) {
      pcl::PointXYZI q{}; q.x=point.x; q.y=point.y; q.z=point.z; q.intensity=point.intensity;
      input.push_back(q);
    }
    return prepare(input, scancontext_level_rotation(frame.pose.R), o.max_radius_m, leaf);
  };
  const auto target_cloud=history_cloud(candidate.index,o.registration_leaf_size_m);
  const auto &target = *target_cloud;
  result.target_ms = milliseconds(target_started);
  auto start = std::chrono::steady_clock::now();
  // As in SC-QN, Quatro uses a single candidate scan, not a large map neighborhood.
  struct ThreadLimit {
    int previous = omp_get_max_threads();
    explicit ThreadLimit(int threads) { omp_set_num_threads(threads); }
    ~ThreadLimit() { omp_set_num_threads(previous); }
  } thread_limit(o.threads);
  quatro<pcl::PointXYZI> coarse(o.normal_radius_m, o.fpfh_radius_m, o.noise_bound_m,
      1.4, 0.0001, 50, false, true, o.max_radius_m, o.max_correspondences);
  bool valid = false;
  const Eigen::Matrix4d matrix = coarse.align(*registration_source, target, valid);
  result.quatro_ms = milliseconds(start);
  if(!valid || !matrix.allFinite() ||
     (matrix.topLeftCorner<3,3>().transpose()*matrix.topLeftCorner<3,3>()-Eigen::Matrix3d::Identity()).norm()>1e-3 ||
     matrix.topLeftCorner<3,3>().determinant()<0.99) {
    result.reason="Quatro rejected coarse registration"; return result;
  }
  Eigen::Isometry3d initial = Eigen::Isometry3d::Identity(), refined;
  initial.matrix() = matrix;
  DirectMatchOptions fine;
  fine.gicp_threads=o.threads; fine.gicp_leaf_size_m=o.registration_leaf_size_m; fine.gicp_covariance_neighbors=15;
  fine.icp_max_iterations=32; fine.icp_max_correspondence_m=o.gicp_max_correspondence_m;
  start = std::chrono::steady_clock::now();
  double fitness = std::numeric_limits<double>::infinity();
  const bool converged = register_direct_gicp(*registration_source, target, initial, fine, refined, &fitness);
  result.gicp_ms = milliseconds(start);
  if(!converged || !std::isfinite(fitness) || fitness >= o.gicp_fitness_threshold) {
    result.reason="Nano-GICP convergence/fitness failed"; return result;
  }
  Eigen::Isometry3d query_level = Eigen::Isometry3d::Identity(); query_level.linear()=level;
  result.map_from_target=p.poses.at(candidate.index);
  result.map_from_query=result.map_from_target * refined * query_level;
  result.success=true; return result;
}
#else
bool scancontext_backend_available() { return false; }
struct ScanContextRelocalizer::Impl {};
ScanContextRelocalizer::ScanContextRelocalizer(const SavedMap &, ConfigSetting, ScanContextOptions, MapCloudLoader) {
  throw std::runtime_error("Scan Context/Quatro was not built: install TEASER++ and reconfigure");
}
ScanContextRelocalizer::~ScanContextRelocalizer() = default;
ScanContextCandidate ScanContextRelocalizer::search(const Cloud &) { return {}; }
ScanContextResult ScanContextRelocalizer::match(const Cloud &, const Eigen::Matrix3d &, const Eigen::Vector3d *) { return {}; }
#endif
