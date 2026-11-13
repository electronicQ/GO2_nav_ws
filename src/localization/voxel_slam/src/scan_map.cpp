#include "scan_map.hpp"
#include "tools.hpp"
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {
void valid_pose(const SavedPose &p) {
  if(!std::isfinite(p.stamp) || !p.t.allFinite() || !p.R.allFinite() ||
     (p.R.transpose()*p.R-Eigen::Matrix3d::Identity()).norm()>1e-5 || p.R.determinant()<.999)
    throw std::runtime_error("invalid scan map pose");
}
std::string filename(int id) { return "scans/"+std::to_string(id)+".pcd"; }
void bounds(const pcl::PointCloud<PointType> &cloud, Eigen::Vector3d &lo, Eigen::Vector3d &hi) {
  lo.setConstant(std::numeric_limits<double>::infinity()); hi=-lo;
  for(const auto &p : cloud) {
    if(!p.getVector3fMap().allFinite()) throw std::runtime_error("nonfinite scan point");
    lo=lo.cwiseMin(p.getVector3fMap().cast<double>());
    hi=hi.cwiseMax(p.getVector3fMap().cast<double>());
  }
  if(!lo.allFinite()) throw std::runtime_error("empty scan cloud");
}
void write_yaml(const std::filesystem::path &p, const YAML::Node &node) {
  std::ofstream out(p); out << node;
  out.flush();
  if(!out) throw std::runtime_error("cannot write scan map metadata: "+p.string());
}
}

ScanMapWriter::ScanMapWriter(const std::string &directory, double leaf)
    : directory_(directory), leaf_(leaf) {
  if(directory.empty() || !std::isfinite(leaf) || leaf<=0)
    throw std::invalid_argument("scan map export requires path and positive leaf");
  if(std::filesystem::exists(directory_))
    throw std::runtime_error("refusing to overwrite existing map: "+directory);
  staging_=directory+".incomplete_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  std::filesystem::create_directories(staging_/"scans");
}
int ScanMapWriter::append(const pcl::PointCloud<PointType> &input, const SavedPose &pose, uint64_t session) {
  valid_pose(pose);
  if(!map_.keyframes.empty() && pose.stamp<=map_.keyframes.back().pose.stamp)
    throw std::runtime_error("scan export timestamps must increase; restart mapping after clock rollback");
  pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>(input)), cloud(new pcl::PointCloud<PointType>);
  Eigen::Vector3d lo,hi; bounds(input,lo,hi);
  *cloud=*source;
  down_sampling_voxel(*cloud,leaf_);
  bounds(*cloud,lo,hi);
  SavedKeyframe frame; frame.id=map_.keyframes.size(); frame.pose=pose;
  pcl::PointCloud<pcl::PointXYZI> stored;
  stored.reserve(cloud->size());
  for(const auto &p : *cloud) {
    pcl::PointXYZI q{}; q.x=p.x; q.y=p.y; q.z=p.z; q.intensity=p.intensity; stored.push_back(q);
  }
  if(pcl::io::savePCDFileBinary((staging_/filename(frame.id)).string(),stored)<0)
    throw std::runtime_error("cannot write scan cloud");
  map_.keyframes.push_back(frame); sessions_.push_back(session);
  counts_.push_back(cloud->size()); lower_.push_back(lo); upper_.push_back(hi);
  return frame.id;
}
void ScanMapWriter::set_pose(int id, const SavedPose &pose) {
  valid_pose(pose); map_.keyframes.at(id).pose=pose;
}
void ScanMapWriter::finish(const std::vector<std::pair<int,SavedPose>> &optimized) {
  if(map_.keyframes.empty() || optimized.empty()) throw std::runtime_error("no optimized scans to export");
  const auto before=map_;
  std::unordered_map<int,SavedPose> final;
  for(const auto &item : optimized) {
    valid_pose(item.second);
    if(item.first<0 || std::size_t(item.first)>=map_.keyframes.size() || !final.emplace(item).second)
      throw std::runtime_error("invalid final scan association");
  }
  // Only the unretired tail lacks graph vertices. Preserve its local-BA relative
  // motion under the nearest optimized scan's final rigid correction.
  for(std::size_t i=0;i<map_.keyframes.size();++i) {
    const auto exact=final.find(i);
    if(exact!=final.end()) { map_.keyframes[i].pose=exact->second; continue; }
    int nearest=-1; double dt=std::numeric_limits<double>::infinity();
    for(const auto &item : final) if(sessions_[i]==sessions_[item.first]) {
      const double d=std::abs(before.keyframes[i].pose.stamp-before.keyframes[item.first].pose.stamp);
      if(d<dt) { dt=d; nearest=item.first; }
    }
    if(nearest<0) throw std::runtime_error("scan session has no optimized pose anchor");
    const auto &old=before.keyframes[nearest].pose, &now=final.at(nearest);
    auto &p=map_.keyframes[i].pose;
    const Eigen::Matrix3d R=now.R*old.R.transpose();
    p.t=R*(p.t-old.t)+now.t; p.R=R*p.R;
  }
  YAML::Node index;
  for(std::size_t i=0;i<map_.keyframes.size();++i) {
    const auto &f=map_.keyframes[i]; const Eigen::Quaterniond q(f.pose.R);
    YAML::Node row; row["id"]=f.id; row["session"]=sessions_[i]; row["stamp"]=f.pose.stamp;
    for(double v : {f.pose.t.x(),f.pose.t.y(),f.pose.t.z(),q.x(),q.y(),q.z(),q.w()}) row["pose"].push_back(v);
    for(int a=0;a<3;++a) { row["lower"].push_back(lower_[i][a]); row["upper"].push_back(upper_[i][a]); }
    row["points"]=counts_[i]; row["file"]=filename(i);
    row["pose_source"]=final.count(i) ? "optimized" : "relative_to_optimized";
    index.push_back(row);
  }
  write_yaml(staging_/"scans.yaml",index);
  YAML::Node manifest;
  manifest["format"]="voxel_slam_sc_scans"; manifest["version"]=4;
  manifest["frame"]="imu"; manifest["leaf_size_m"]=leaf_;
  manifest["scan_count"]=map_.keyframes.size();
  write_yaml(staging_/"manifest.yaml",manifest);
  if(std::filesystem::exists(directory_)) throw std::runtime_error("map destination appeared during export");
  std::filesystem::rename(staging_,directory_);
}

ScanMapStore::ScanMapStore(const std::string &directory, std::size_t cache_bytes)
    : directory_(std::filesystem::canonical(directory)), limit_(cache_bytes) {
  if(limit_==0) throw std::invalid_argument("point-cloud cache must be positive");
  const auto manifest=YAML::LoadFile((directory_/"manifest.yaml").string());
  if(manifest["format"].as<std::string>("")!="voxel_slam_sc_scans" ||
     manifest["version"].as<int>(0)!=4 || manifest["frame"].as<std::string>("")!="imu")
    throw std::runtime_error("SC-only localization requires a new v4 scan map; remap with this branch");
  leaf_=manifest["leaf_size_m"].as<double>();
  if(!std::isfinite(leaf_) || leaf_<=0) throw std::runtime_error("invalid map leaf");
  const auto rows=YAML::LoadFile((directory_/"scans.yaml").string());
  if(!rows.IsSequence() || rows.size()==0 || rows.size()!=manifest["scan_count"].as<std::size_t>())
    throw std::runtime_error("invalid scan map index");
  for(const auto &row : rows) {
    SavedKeyframe frame; frame.id=row["id"].as<int>(); frame.pose.stamp=row["stamp"].as<double>();
    if(frame.id!=int(map_.keyframes.size()) || row["file"].as<std::string>()!=filename(frame.id) ||
       row["pose"].size()!=7 || row["lower"].size()!=3 || row["upper"].size()!=3)
      throw std::runtime_error("invalid scan map row");
    auto p=row["pose"]; Eigen::Quaterniond q(p[6].as<double>(),p[3].as<double>(),p[4].as<double>(),p[5].as<double>());
    if(!q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-5) throw std::runtime_error("invalid scan quaternion");
    frame.pose.R=q.normalized().toRotationMatrix();
    for(int a=0;a<3;++a) {
      frame.pose.t[a]=p[a].as<double>();
      frame.local_lower[a]=row["lower"][a].as<double>(); frame.local_upper[a]=row["upper"][a].as<double>();
    }
    valid_pose(frame.pose);
    if(!frame.local_lower.allFinite() || !frame.local_upper.allFinite() ||
       (frame.local_lower.array()>frame.local_upper.array()).any() ||
       (!map_.keyframes.empty() && frame.pose.stamp<=map_.keyframes.back().pose.stamp))
      throw std::runtime_error("invalid scan bounds or timestamps");
    const auto file=std::filesystem::canonical(directory_/filename(frame.id));
    if(file.parent_path()!=directory_/"scans") throw std::runtime_error("scan path escapes map");
    map_.keyframes.push_back(frame); sessions_.push_back(row["session"].as<uint64_t>());
    counts_.push_back(row["points"].as<std::size_t>());
    if(!counts_.back()) throw std::runtime_error("empty indexed scan");
  }
}
void ScanMapStore::trim() {
  while(bytes_>limit_ && !lru_.empty()) {
    // Drop raw clouds before cheap reusable plane views. Borrowed buffers are
    // valid until their caller releases them and are not resident cache memory.
    auto victim=lru_.end();
    for(auto it=lru_.end();it!=lru_.begin();) {
      --it;
      if(cache_.at(*it).cloud) { victim=it; break; }
    }
    if(victim==lru_.end()) victim=std::prev(lru_.end());
    auto &entry=cache_.at(*victim);
    if(entry.cloud && entry.planes) {
      const auto size=entry.cloud->points.capacity()*sizeof(PointType);
      entry.cloud.reset(); entry.bytes-=size; bytes_-=size;
    } else {
      bytes_-=entry.bytes; cache_.erase(*victim); lru_.erase(victim);
    }
    ++evictions_;
  }
}
pcl::PointCloud<PointType>::Ptr ScanMapStore::scan(int id) {
  map_.keyframes.at(id);
  auto found=cache_.find(id);
  if(found!=cache_.end() && found->second.cloud) {
    lru_.splice(lru_.begin(),lru_,found->second.lru);
    return found->second.cloud;
  }
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZI> stored;
  if(pcl::io::loadPCDFile((directory_/filename(id)).string(),stored)<0 || stored.size()!=counts_[id])
    throw std::runtime_error("missing or corrupt scan "+std::to_string(id));
  cloud->reserve(stored.size());
  for(const auto &p : stored) {
    PointType q{}; q.x=p.x; q.y=p.y; q.z=p.z; q.intensity=p.intensity; cloud->push_back(q);
  }
  Eigen::Vector3d lo,hi; bounds(*cloud,lo,hi);
  const auto &frame=map_.keyframes[id];
  if((lo-frame.local_lower).norm()>1e-3 || (hi-frame.local_upper).norm()>1e-3)
    throw std::runtime_error("scan bounds differ from metadata");
  if(found==cache_.end()) {
    lru_.push_front(id); Entry entry; entry.lru=lru_.begin();
    found=cache_.emplace(id,std::move(entry)).first;
  } else lru_.splice(lru_.begin(),lru_,found->second.lru);
  const auto size=cloud->points.capacity()*sizeof(PointType);
  found->second.cloud=cloud; found->second.bytes+=size; bytes_+=size;
  ++reads_; trim();
  return cloud;
}
pcl::PointCloud<PointType>::Ptr ScanMapStore::fuse(int end, int scans) {
  if(scans<1 || end+1<scans) throw std::runtime_error("incomplete history window");
  const auto &anchor=map_.keyframes.at(end).pose;
  pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
  for(int i=end+1-scans;i<=end;++i) {
    if(sessions_[i]!=sessions_[end]) throw std::runtime_error("history window crosses session boundary");
    const auto cloud=scan(i); const auto &pose=map_.keyframes[i].pose;
    const Eigen::Matrix3d R=anchor.R.transpose()*pose.R;
    const Eigen::Vector3d t=anchor.R.transpose()*(pose.t-anchor.t);
    for(const auto &p : *cloud) {
      const Eigen::Vector3d v=R*p.getVector3fMap().cast<double>()+t;
      PointType q{}; q.x=v.x(); q.y=v.y(); q.z=v.z(); q.intensity=p.intensity; output->push_back(q);
    }
  }
  return output;
}
SavedMap ScanMapStore::history(int scans, double distance, double angle) const {
  if(scans<1 || scans>30 || !std::isfinite(distance) || distance<=0 || !std::isfinite(angle) || angle<=0 || angle>180)
    throw std::invalid_argument("invalid SC history selection");
  SavedMap result; int start=0; uint64_t session=sessions_.front();
  for(int i=0;i<int(map_.keyframes.size());++i) {
    if(sessions_[i]!=session) { start=i; session=sessions_[i]; }
    if(i-start+1<scans) continue;
    const auto &frame=map_.keyframes[i];
    if(!result.keyframes.empty() && sessions_[result.keyframes.back().id]==session) {
      const auto &previous=result.keyframes.back().pose;
      if((frame.pose.t-previous.t).norm()<distance &&
         Eigen::AngleAxisd(previous.R.transpose()*frame.pose.R).angle()*180/M_PI<angle) continue;
    }
    result.keyframes.push_back(frame);
  }
  if(result.keyframes.empty()) throw std::runtime_error("map has no complete SC history window");
  return result;
}
STDescManager::PlaneCloud::Ptr ScanMapStore::planes(int id, const ConfigSetting &config) {
  auto found=cache_.find(id);
  if(found!=cache_.end() && found->second.planes) {
    lru_.splice(lru_.begin(),lru_,found->second.lru);
    return found->second.planes;
  }
  auto cloud=scan(id);
  pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
  for(const auto &p : *cloud) {
    pcl::PointXYZI q{}; q.x=p.x; q.y=p.y; q.z=p.z; input->push_back(q);
  }
  auto setting=config;
  STDescManager extractor(setting); auto result=extractor.ExtractPlanes(input);
  found=cache_.find(id);
  if(found!=cache_.end()) {
    const auto extra=result->points.capacity()*sizeof(pcl::PointXYZINormal);
    found->second.planes=result; found->second.bytes+=extra; bytes_+=extra; trim();
  }
  return result;
}
