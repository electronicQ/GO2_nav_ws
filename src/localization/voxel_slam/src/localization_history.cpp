#include "localization_history.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

void LocalizationHistoryOptions::validate(int saved_window_scans) const {
  if(source != "saved_keyframes" && source != "individual_scans")
    throw std::invalid_argument("unknown Localization.history.source");
  if(saved_window_scans < 1 || window_scans < 0 || window_scans > 1000 ||
     !std::isfinite(leaf_size_m) || leaf_size_m < 0)
    throw std::invalid_argument("invalid Localization.history window or voxel leaf");
  if(source == "saved_keyframes" && window_scans > 0 && window_scans % saved_window_scans != 0)
    throw std::invalid_argument("saved_keyframes history window must be a multiple of saved win_size; use individual_scans for 3/5-scan windows");
  if(source == "individual_scans" && (window_scans < 1 || scan_directory.empty()))
    throw std::invalid_argument("individual_scans requires scan_directory and a positive window_scans");
  for(double value : {timestamp_tolerance_sec, anchor_position_tolerance_m, anchor_angle_tolerance_deg})
    if(!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid history anchor tolerance");
}

LocalizationHistoryOptions read_localization_history_options(
    ros::NodeHandle &node, int default_window_scans, double default_leaf_size_m) {
  LocalizationHistoryOptions options;
  options.window_scans = default_window_scans;
  options.leaf_size_m = default_leaf_size_m;
#define HISTORY_PARAM(field) node.param("Localization/history/" #field, options.field, options.field)
  HISTORY_PARAM(source); HISTORY_PARAM(window_scans); HISTORY_PARAM(leaf_size_m);
  HISTORY_PARAM(scan_directory); HISTORY_PARAM(pose_file); HISTORY_PARAM(timestamp_tolerance_sec);
  HISTORY_PARAM(anchor_position_tolerance_m); HISTORY_PARAM(anchor_angle_tolerance_deg);
#undef HISTORY_PARAM
  return options;
}

namespace {
std::vector<SavedPose> load_scan_poses(const std::filesystem::path &path) {
  std::ifstream file(path);
  if(!file) throw std::runtime_error("cannot read individual-scan poses: " + path.string());
  std::vector<SavedPose> poses;
  std::string line;
  while(std::getline(file, line)) {
    if(line.empty() || line.front() == '#') continue;
    std::istringstream row(line);
    SavedPose pose; double qx, qy, qz, qw;
    if(!(row >> pose.stamp >> pose.t.x() >> pose.t.y() >> pose.t.z() >> qx >> qy >> qz >> qw))
      throw std::runtime_error("individual-scan poses require timestamp, xyz, qx qy qz qw");
    const Eigen::Quaterniond q(qw, qx, qy, qz);
    if(!std::isfinite(pose.stamp) || !pose.t.allFinite() || !q.coeffs().allFinite() ||
       std::abs(q.norm()-1.0) > 1e-3 || (!poses.empty() && pose.stamp <= poses.back().stamp))
      throw std::runtime_error("invalid or unordered individual-scan pose");
    pose.R = q.normalized().toRotationMatrix(); poses.push_back(pose);
  }
  if(poses.empty()) throw std::runtime_error("individual-scan pose file is empty");
  return poses;
}

void append_cloud(const pcl::PointCloud<PointType> &cloud, const SavedPose &pose,
                  const SavedPose &anchor, pcl::PointCloud<PointType> &output) {
  const Eigen::Matrix3d rotation = anchor.R.transpose()*pose.R;
  const Eigen::Vector3d translation = anchor.R.transpose()*(pose.t-anchor.t);
  for(const auto &point : cloud) {
    if(!point.getVector3fMap().allFinite()) continue;
    const Eigen::Vector3d v=rotation*point.getVector3fMap().cast<double>()+translation;
    PointType out{}; out.x=v.x(); out.y=v.y(); out.z=v.z(); out.intensity=point.intensity;
    output.push_back(out);
  }
}
}

SavedMap build_localization_history(const SavedMap &map, int saved_window_scans,
                                   const LocalizationHistoryOptions &options) {
  options.validate(saved_window_scans);
  if(map.keyframes.empty()) throw std::invalid_argument("empty history anchor map");
  SavedMap history;
  history.keyframes.reserve(map.keyframes.size());
  std::vector<SavedPose> poses;
  if(options.source == "individual_scans") {
    std::filesystem::path path=options.pose_file.empty() ? "alidarState.txt" : options.pose_file;
    if(path.is_relative()) path=std::filesystem::path(options.scan_directory)/path;
    poses=load_scan_poses(path);
  }
  std::unordered_map<std::size_t, pcl::PointCloud<PointType>::Ptr> scan_cache;
  for(std::size_t index=0; index<map.keyframes.size(); ++index) {
    SavedKeyframe key=map.keyframes[index];
    key.cloud.reset(new pcl::PointCloud<PointType>);
    if(options.source == "saved_keyframes") {
      const std::size_t groups=std::max(1, options.window_scans/saved_window_scans);
      const std::size_t begin=index+1>groups ? index+1-groups : 0;
      if(groups==1) {
        if(!map.keyframes[index].cloud) throw std::runtime_error("missing saved keyframe cloud");
        key.cloud=map.keyframes[index].cloud;
      } else for(std::size_t j=begin; j<=index; ++j) {
        const auto &source=map.keyframes[j];
        if(!source.cloud) throw std::runtime_error("missing saved keyframe cloud");
        append_cloud(*source.cloud, source.pose, key.pose, *key.cloud);
      }
    } else {
      const auto found=std::lower_bound(poses.begin(), poses.end(), key.pose.stamp,
          [](const SavedPose &pose, double stamp) { return pose.stamp < stamp; });
      std::size_t end=found==poses.end() ? poses.size()-1 : std::size_t(found-poses.begin());
      if(end>0 && std::abs(poses[end-1].stamp-key.pose.stamp)<std::abs(poses[end].stamp-key.pose.stamp)) --end;
      const auto &pose=poses[end];
      if(std::abs(pose.stamp-key.pose.stamp)>options.timestamp_tolerance_sec ||
         (pose.t-key.pose.t).norm()>options.anchor_position_tolerance_m ||
         Eigen::AngleAxisd(key.pose.R.transpose()*pose.R).angle()*180/M_PI>options.anchor_angle_tolerance_deg)
        throw std::runtime_error("individual scans do not match saved map anchor " + std::to_string(key.id));
      const std::size_t begin=end+1>std::size_t(options.window_scans) ? end+1-options.window_scans : 0;
      for(std::size_t j=begin; j<=end; ++j) {
        auto &cloud=scan_cache[j];
        if(!cloud) {
          pcl::PointCloud<pcl::PointXYZI> input;
          const auto path=std::filesystem::path(options.scan_directory)/(std::to_string(j)+".pcd");
          if(pcl::io::loadPCDFile(path.string(), input)<0 || input.empty())
            throw std::runtime_error("missing or empty individual scan: " + path.string());
          cloud.reset(new pcl::PointCloud<PointType>);
          for(const auto &p : input) {
            PointType out{}; out.x=p.x; out.y=p.y; out.z=p.z; out.intensity=p.intensity; cloud->push_back(out);
          }
        }
        append_cloud(*cloud, poses[j], key.pose, *key.cloud);
      }
    }
    if(key.cloud->empty()) throw std::runtime_error("empty descriptor history keyframe");
    history.keyframes.push_back(std::move(key));
  }
  return history;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_localization_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double leaf_size_m) {
  if(!cloud || !std::isfinite(leaf_size_m) || leaf_size_m<0)
    throw std::invalid_argument("invalid localization voxel input");
  if(leaf_size_m==0 || cloud->empty()) return cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(cloud); filter.setLeafSize(leaf_size_m,leaf_size_m,leaf_size_m); filter.filter(*output);
  return output;
}
