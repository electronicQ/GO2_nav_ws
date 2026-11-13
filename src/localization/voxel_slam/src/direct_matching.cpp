#include "direct_matching.hpp"
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <nano_gicp/nano_gicp.hpp>

void DirectMatchOptions::validate() const {
  for(double value : {period_sec, translation_m, rotation_deg, retry_sec,
      query_max_range_m, target_padding_m, cache_padding_m, cache_refresh_distance_m,
      target_leaf_size_m, recovery_after_sec, icp_max_correspondence_m, gicp_leaf_size_m})
    if(!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid direct-match threshold");
  if(!std::isfinite(query_leaf_size_m) || query_leaf_size_m <= 0)
    throw std::invalid_argument("invalid DIRECT query leaf");
  if(rotation_deg > 180 || retry_sec < period_sec || query_scans < 1 || query_scans > 30 ||
     min_query_points < 6 || max_target_voxels < 100 || recovery_after_failures < 1 || icp_max_iterations < 1 ||
     gicp_threads < 1 || gicp_threads > 64 || gicp_covariance_neighbors < 5 || gicp_covariance_neighbors > 100)
    throw std::invalid_argument("invalid direct-match limits");
  if(registration_backend != "point_to_plane" && registration_backend != "nano_gicp")
    throw std::invalid_argument("unknown direct-match registration backend");
}

DirectMatchOptions read_direct_match_options(ros::NodeHandle &node) {
  DirectMatchOptions options;
#define DIRECT_PARAM(field) node.param("Localization/direct_match/" #field, options.field, options.field)
  DIRECT_PARAM(enabled);
  DIRECT_PARAM(registration_backend);
  DIRECT_PARAM(gicp_leaf_size_m);
  DIRECT_PARAM(gicp_threads);
  DIRECT_PARAM(gicp_covariance_neighbors);
  DIRECT_PARAM(period_sec);
  DIRECT_PARAM(translation_m);
  DIRECT_PARAM(rotation_deg);
  DIRECT_PARAM(retry_sec);
  DIRECT_PARAM(query_scans);
  DIRECT_PARAM(query_leaf_size_m);
  DIRECT_PARAM(query_max_range_m);
  DIRECT_PARAM(min_query_points);
  DIRECT_PARAM(target_padding_m);
  DIRECT_PARAM(cache_padding_m);
  DIRECT_PARAM(cache_refresh_distance_m);
  DIRECT_PARAM(target_leaf_size_m);
  DIRECT_PARAM(max_target_voxels);
  DIRECT_PARAM(recovery_after_failures);
  DIRECT_PARAM(recovery_after_sec);
  DIRECT_PARAM(icp_max_correspondence_m);
  DIRECT_PARAM(icp_max_iterations);
#undef DIRECT_PARAM
  options.validate();
  return options;
}

bool register_direct_scan(const pcl::PointCloud<pcl::PointXYZI> &source,
    const pcl::PointCloud<pcl::PointXYZINormal> &target,
    const Eigen::Isometry3d &initial, const DirectMatchOptions &options,
    Eigen::Isometry3d &refined) {
  if(source.empty() || target.empty() || !initial.matrix().allFinite()) return false;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr input(new pcl::PointCloud<pcl::PointXYZINormal>());
  input->reserve(source.size());
  for(const auto &p : source) {
    if(!p.getVector3fMap().allFinite()) continue;
    pcl::PointXYZINormal q{};
    q.x = p.x; q.y = p.y; q.z = p.z;
    // The asymmetric point-to-plane objective uses target normals only.
    q.normal_x = q.normal_y = q.normal_z = 0;
    input->push_back(q);
  }
  if(input->size() < std::size_t(options.min_query_points)) return false;
  pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
  icp.setUseSymmetricObjective(false);
  icp.setInputSource(input);
  icp.setInputTarget(target.makeShared());
  icp.setMaximumIterations(options.icp_max_iterations);
  icp.setMaxCorrespondenceDistance(options.icp_max_correspondence_m);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-6);
  pcl::PointCloud<pcl::PointXYZINormal> aligned;
  icp.align(aligned, initial.matrix().cast<float>());
  const Eigen::Matrix4d matrix = icp.getFinalTransformation().cast<double>();
  if(!icp.hasConverged() || !matrix.allFinite()) return false;
  refined = Eigen::Isometry3d::Identity();
  refined.linear() = Eigen::Quaterniond(Eigen::Matrix3d(matrix.topLeftCorner<3, 3>())).normalized().toRotationMatrix();
  refined.translation() = matrix.topRightCorner<3, 1>();
  return refined.matrix().allFinite();
}

bool register_direct_gicp(const pcl::PointCloud<pcl::PointXYZI> &source,
    const pcl::PointCloud<pcl::PointXYZI> &target,
    const Eigen::Isometry3d &initial, const DirectMatchOptions &options,
    Eigen::Isometry3d &refined, double *fitness) {
  if(!initial.matrix().allFinite() || target.size() < std::size_t(options.gicp_covariance_neighbors))
    return false;
  pcl::PointCloud<pcl::PointXYZI>::Ptr finite(new pcl::PointCloud<pcl::PointXYZI>());
  for(const auto &p : source) if(p.getVector3fMap().allFinite()) finite->push_back(p);
  pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(options.gicp_leaf_size_m, options.gicp_leaf_size_m, options.gicp_leaf_size_m);
  filter.setInputCloud(finite); filter.filter(*input);
  if(input->size() < std::size_t(std::max(options.min_query_points, options.gicp_covariance_neighbors)))
    return false;
  for(const auto &p : target) if(!p.getVector3fMap().allFinite()) return false;
  nano_gicp::NanoGICP<pcl::PointXYZI, pcl::PointXYZI> gicp;
  gicp.setNumThreads(options.gicp_threads);
  gicp.setCorrespondenceRandomness(options.gicp_covariance_neighbors);
  gicp.setMaximumIterations(options.icp_max_iterations);
  gicp.setMaxCorrespondenceDistance(options.icp_max_correspondence_m);
  // Nano-GICP checks the translation increment itself, unlike PCL ICP's squared criterion.
  gicp.setTransformationEpsilon(1e-4);
  gicp.setRotationEpsilon(1e-4);
  gicp.setEuclideanFitnessEpsilon(1e-6);
  gicp.setInputSource(input);
  gicp.setInputTarget(target.makeShared());
  pcl::PointCloud<pcl::PointXYZI> aligned;
  gicp.align(aligned, initial.matrix().cast<float>());
  if(fitness) *fitness = gicp.getFitnessScore();
  const Eigen::Matrix4d matrix = gicp.getFinalTransformation().cast<double>();
  if(!gicp.hasConverged() || !matrix.allFinite()) return false;
  refined = Eigen::Isometry3d::Identity();
  refined.linear() = Eigen::Quaterniond(Eigen::Matrix3d(matrix.topLeftCorner<3, 3>())).normalized().toRotationMatrix();
  refined.translation() = matrix.topRightCorner<3, 1>();
  return refined.matrix().allFinite();
}

DirectMatchSchedule::DirectMatchSchedule(DirectMatchOptions options) : options_(options) {
  options_.validate();
}
bool DirectMatchSchedule::ready(double wall, double stamp, const Eigen::Isometry3d &local, bool confirmation_pending) const {
  if(!std::isfinite(wall) || !std::isfinite(stamp) || !local.matrix().allFinite()) return false;
  if(wall_ < 0) return true;
  if(stamp <= stamp_ || wall - wall_ < options_.period_sec) return false;
  return confirmation_pending || wall - wall_ >= options_.retry_sec ||
      (local.translation() - pose_.translation()).norm() >= options_.translation_m ||
      Eigen::AngleAxisd(pose_.linear().transpose() * local.linear()).angle() * 180 / M_PI >= options_.rotation_deg;
}
void DirectMatchSchedule::submitted(double wall, double stamp, const Eigen::Isometry3d &local) {
  wall_ = wall; stamp_ = stamp; pose_ = local;
}
void DirectMatchSchedule::reset() { wall_ = stamp_ = -1; pose_.setIdentity(); }

DirectLocalMap::DirectLocalMap(const SavedMap &map, ConfigSetting config, DirectMatchOptions options,
    MapCloudLoader loader, std::function<STDescManager::PlaneCloud::Ptr(int)> plane_loader)
    : loader_(std::move(loader)), plane_loader_(std::move(plane_loader)), config_(config), options_(options) {
  options_.validate();
  for(const auto &frame : map.keyframes) {
    if(!frame.pose.R.allFinite() || !frame.pose.t.allFinite()) continue;
    Coverage entry{&frame, Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
                   Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity()), nullptr};
    pcl::PointCloud<PointType> corners;
    if(!frame.cloud && loader_ && frame.local_lower.allFinite() && frame.local_upper.allFinite())
      for(int mask=0;mask<8;++mask) {
        PointType p{}; p.x=(mask&1)?frame.local_upper.x():frame.local_lower.x();
        p.y=(mask&2)?frame.local_upper.y():frame.local_lower.y();
        p.z=(mask&4)?frame.local_upper.z():frame.local_lower.z(); corners.push_back(p);
      }
    const auto &coverage_cloud=frame.cloud ? *frame.cloud : corners;
    for(const auto &p : coverage_cloud) {
      const Eigen::Vector3d world = frame.pose.R * p.getVector3fMap().cast<double>() + frame.pose.t;
      if(!world.allFinite()) continue;
      entry.lower = entry.lower.cwiseMin(world);
      entry.upper = entry.upper.cwiseMax(world);
    }
    if(entry.lower.allFinite()) coverage_.push_back(entry);
  }
}

DirectMapTarget DirectLocalMap::target(const pcl::PointCloud<pcl::PointXYZI> &query,
    const Eigen::Isometry3d &map_from_query, std::string &error) {
  error.clear();
  DirectMapTarget empty;
  Eigen::Vector3d lower = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper = -lower;
  if(!map_from_query.matrix().allFinite()) { error = "nonfinite map prediction"; return empty; }
  for(const auto &p : query) {
    const Eigen::Vector3d point = p.getVector3fMap().cast<double>();
    if(!point.allFinite() || point.norm() > options_.query_max_range_m) continue;
    const Eigen::Vector3d world = map_from_query * point;
    lower = lower.cwiseMin(world); upper = upper.cwiseMax(world);
  }
  if(!lower.allFinite()) { error = "empty query footprint"; return empty; }
  lower.array() -= options_.target_padding_m;
  upper.array() += options_.target_padding_m;
  if(cached_.planes && (lower.array() >= cached_lower_.array()).all() &&
      (upper.array() <= cached_upper_.array()).all() &&
      (map_from_query.translation() - cached_position_).norm() <= options_.cache_refresh_distance_m) {
    auto result = cached_; result.cache_hit = true; return result;
  }
  lower.array() -= options_.cache_padding_m;
  upper.array() += options_.cache_padding_m;
  DirectMapTarget result;
  result.map_from_target.translation() = map_from_query.translation();
  // Preserve each view's planes before merging: fitting one coarse world voxel
  // over every traversal can erase narrow surfaces and mix unrelated planes.
  struct Cell {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero(), normal = Eigen::Vector3d::Zero();
    std::size_t count = 0;
  };
  std::map<std::array<std::int64_t, 6>, Cell> cells;
  std::map<std::array<std::int64_t, 3>, Cell> point_cells;
  for(auto &entry : coverage_) {
    if((entry.upper.array() < lower.array()).any() || (entry.lower.array() > upper.array()).any()) continue;
    ++result.keyframes;
    if(options_.registration_backend == "nano_gicp") {
      const auto source=entry.frame->cloud ? entry.frame->cloud : loader_(entry.frame->id);
      for(const auto &p : *source) {
        const Eigen::Vector3d world = entry.frame->pose.R * p.getVector3fMap().cast<double>() + entry.frame->pose.t;
        if(!world.allFinite() || (world.array() < lower.array()).any() || (world.array() > upper.array()).any()) continue;
        const Eigen::Vector3d local = world - result.map_from_target.translation();
        std::array<std::int64_t, 3> id;
        for(int axis = 0; axis < 3; ++axis)
          id[axis] = std::int64_t(std::floor(local[axis] / options_.gicp_leaf_size_m));
        auto &cell = point_cells[id]; cell.sum += local; ++cell.count;
        if(point_cells.size() > std::size_t(options_.max_target_voxels)) {
          error = "GICP target voxel budget exceeded; target not silently truncated"; return empty;
        }
      }
    }
    auto view_planes = entry.planes;
    if(plane_loader_) view_planes = plane_loader_(entry.frame->id);
    if(!view_planes) {
      const auto source=entry.frame->cloud ? entry.frame->cloud : loader_(entry.frame->id);
      pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
      input->reserve(source->size());
      for(const auto &p : *source) {
        if(!p.getVector3fMap().allFinite()) continue;
        pcl::PointXYZI q{}; q.x = p.x; q.y = p.y; q.z = p.z;
        input->push_back(q);
      }
      STDescManager extractor(config_);
      view_planes = extractor.ExtractPlanes(input);
      if(!loader_) entry.planes = view_planes;
    }
    for(const auto &p : *view_planes) {
      const Eigen::Vector3d world = entry.frame->pose.R * p.getVector3fMap().cast<double>() + entry.frame->pose.t;
      if(!world.allFinite() || (world.array() < lower.array()).any() || (world.array() > upper.array()).any()) continue;
      const Eigen::Vector3d local = world - result.map_from_target.translation();
      Eigen::Vector3d normal = entry.frame->pose.R * p.getNormalVector3fMap().cast<double>();
      if(!normal.allFinite() || normal.norm() < 1e-6) continue;
      normal.normalize();
      Eigen::Index major; normal.cwiseAbs().maxCoeff(&major);
      if(normal[major] < 0) normal = -normal;
      std::array<std::int64_t, 6> id;
      for(int axis = 0; axis < 3; ++axis) {
        id[axis] = std::int64_t(std::floor(local[axis] / options_.target_leaf_size_m));
        id[axis + 3] = std::int64_t(std::round(normal[axis] * 4));
      }
      auto &cell = cells[id]; cell.sum += local; cell.normal += normal; ++cell.count;
      if(cells.size() > std::size_t(options_.max_target_voxels)) {
        error = "target voxel budget exceeded; target not silently truncated"; return empty;
      }
    }
  }
  if(cells.empty()) { error = "no saved point coverage at query footprint"; return empty; }
  result.planes.reset(new STDescManager::PlaneCloud());
  result.planes->reserve(cells.size());
  for(const auto &entry : cells) {
    const auto xyz = entry.second.sum / entry.second.count;
    const Eigen::Vector3d normal = entry.second.normal.normalized();
    pcl::PointXYZINormal p{};
    p.x = xyz.x(); p.y = xyz.y(); p.z = xyz.z();
    p.normal_x = normal.x(); p.normal_y = normal.y(); p.normal_z = normal.z();
    result.planes->push_back(p);
  }
  result.voxels = cells.size();
  if(options_.registration_backend == "nano_gicp") {
    result.points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    result.points->reserve(point_cells.size());
    for(const auto &entry : point_cells) {
      const Eigen::Vector3d xyz = entry.second.sum / entry.second.count;
      pcl::PointXYZI p{}; p.x = xyz.x(); p.y = xyz.y(); p.z = xyz.z();
      result.points->push_back(p);
    }
  }
  if(result.planes->empty()) { error = "no usable historical planes"; return empty; }
  cached_ = result; cached_lower_ = lower; cached_upper_ = upper;
  cached_position_ = map_from_query.translation();
  return result;
}
