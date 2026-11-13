#pragma once
#include "map_package.hpp"
#include <functional>
#include <list>
#include <unordered_map>
#include <filesystem>

using MapCloudLoader = std::function<pcl::PointCloud<PointType>::Ptr(int)>;

// SC-only format: independent, deskewed IMU-frame scans and final map poses.
// The manifest is published last. An unfinished directory is not a usable map.
class ScanMapWriter {
 public:
  ScanMapWriter(const std::string &directory, double leaf);
  int append(const pcl::PointCloud<PointType> &cloud, const SavedPose &pose, uint64_t session);
  void set_pose(int id, const SavedPose &pose);
  void finish(const std::vector<std::pair<int, SavedPose>> &optimized);
 private:
  std::filesystem::path directory_, staging_;
  double leaf_;
  SavedMap map_;
  std::vector<uint64_t> sessions_;
  std::vector<Eigen::Vector3d> lower_, upper_;
  std::vector<std::size_t> counts_;
};

class ScanMapStore {
 public:
  explicit ScanMapStore(const std::string &directory, std::size_t cache_bytes);
  const SavedMap &map() const { return map_; }
  SavedMap history(int scans, double distance, double angle_deg) const;
  pcl::PointCloud<PointType>::Ptr scan(int id);
  pcl::PointCloud<PointType>::Ptr fuse(int end, int scans);
  STDescManager::PlaneCloud::Ptr planes(int id, const ConfigSetting &config);
  std::size_t resident_bytes() const { return bytes_; }
  std::size_t cache_entries() const { return cache_.size(); }
  std::size_t disk_reads() const { return reads_; }
  std::size_t evictions() const { return evictions_; }
  double leaf() const { return leaf_; }
 private:
  struct Entry {
    pcl::PointCloud<PointType>::Ptr cloud;
    STDescManager::PlaneCloud::Ptr planes;
    std::list<int>::iterator lru;
    std::size_t bytes = 0;
  };
  void trim();
  std::filesystem::path directory_;
  SavedMap map_;
  double leaf_ = 0;
  std::vector<uint64_t> sessions_;
  std::vector<std::size_t> counts_;
  std::list<int> lru_;
  std::unordered_map<int, Entry> cache_;
  std::size_t limit_, bytes_ = 0, reads_ = 0, evictions_ = 0;
};
