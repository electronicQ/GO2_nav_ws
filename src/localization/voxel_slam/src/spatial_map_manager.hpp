#pragma once

#include "spatial_map_io.hpp"

#include <Eigen/Core>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct SpatialMapRuntimeConfig {
  double preload_radius = 15.0;
  double load_radius = 30.0;
  double unload_radius = 40.0;
  double prefetch_time_sec = 2.0;
  double map_gap_timeout_sec = 1.0;
  double preload_timeout_sec = 10.0;
  int max_background_loads = 2;
  double max_block_memory_mb = 0.0;  // Accounted block/reservation quota; 0 disables.
  double io_rate_mb_per_sec = 0.0;  // Shared logical-read service rate; 0 disables.
  double io_fixed_latency_ms = 0.0;
  std::string metrics_directory;  // Empty disables CSV instrumentation.

  bool validate(std::string &error) const;
};

struct SpatialMapSnapshot {
  std::uint64_t generation = 0U;
  std::unordered_map<BlockIndex, std::shared_ptr<const StaticMapBlock>,
                     BlockIndexHash>
      blocks;
};

struct SpatialMapStatus {
  std::uint64_t generation = 0U;
  bool core_ready = false;
  bool idle = true;
  bool map_gap = false;
  std::size_t core_block_count = 0U;
  std::size_t loaded_block_count = 0U;
  std::size_t pending_block_count = 0U;
  std::size_t failed_block_count = 0U;
  std::size_t failed_core_block_count = 0U;
  std::uint64_t load_sequence = 0U;
  double last_load_ms = 0.0;
  std::uint64_t load_attempts = 0U;
  std::uint64_t load_failures = 0U;
  std::uint64_t discarded_loads = 0U;
  std::uint64_t read_bytes = 0U;
  std::uint64_t evictions = 0U;
  std::uint64_t budget_waits = 0U;
  std::uint64_t live_block_bytes = 0U;
  std::uint64_t pinned_block_bytes = 0U;
  std::uint64_t reserved_bytes = 0U;
  std::uint64_t peak_accounted_bytes = 0U;
  bool budget_blocked = false;
  std::string last_error;
};

struct SpatialFrameMetrics {
  double sensor_time = 0.0;
  std::string state;
  bool valid = false;
  double schedule_ms = 0.0;
  double match_ms = 0.0;
  double frame_compute_ms = 0.0;
  std::size_t query_points = 0U;
  std::size_t missing_block_points = 0U;
  int matched_points = 0;
  double residual = 0.0;
};

class SpatialBlockSource {
 public:
  virtual ~SpatialBlockSource() = default;
  virtual const SpatialMapCatalog &catalog() const = 0;
  virtual bool load(const BlockIndex &index, StaticMapBlock &block,
                    std::string &error) = 0;
  virtual bool load_with_metrics(const BlockIndex &index, StaticMapBlock &block,
                                std::string &error, SpatialBlockLoadMetrics &metrics) {
    metrics = SpatialBlockLoadMetrics();
    return load(index, block, error);
  }
  virtual std::uint64_t read_cost(const SpatialBlockIndexEntry &) const { return 0U; }
  virtual std::uint64_t scratch_cost(const SpatialBlockIndexEntry &) const { return 0U; }
};

class FileSpatialBlockSource : public SpatialBlockSource {
 public:
  FileSpatialBlockSource(std::string directory, SpatialMapCatalog catalog);
  const SpatialMapCatalog &catalog() const override;
  bool load(const BlockIndex &index, StaticMapBlock &block,
                    std::string &error) override;
  bool load_with_metrics(const BlockIndex &index, StaticMapBlock &block,
                        std::string &error, SpatialBlockLoadMetrics &metrics) override;
  std::uint64_t read_cost(const SpatialBlockIndexEntry &entry) const override;
  std::uint64_t scratch_cost(const SpatialBlockIndexEntry &entry) const override;

 private:
  std::string directory_;
  SpatialMapCatalog catalog_;
};

class SpatialMapManager {
 public:
  SpatialMapManager(std::shared_ptr<SpatialBlockSource> source,
                    const SpatialMapRuntimeConfig &config);
  ~SpatialMapManager();

  SpatialMapManager(const SpatialMapManager &) = delete;
  SpatialMapManager &operator=(const SpatialMapManager &) = delete;

  std::uint64_t begin_session();
  bool request(const Eigen::Vector3d &position,
               const Eigen::Vector3d &velocity,
               std::uint64_t generation, std::string *error = nullptr);

  std::shared_ptr<const SpatialMapSnapshot> snapshot() const;
  SpatialMapStatus status() const;
  bool wait_until_idle(std::chrono::milliseconds timeout);
  bool metrics_enabled() const { return !config_.metrics_directory.empty(); }
  void record_frame(const SpatialFrameMetrics &frame);

 private:
  struct LoadRequest {
    BlockIndex index;
    std::uint64_t generation = 0U;
    bool core = false;
    double distance = 0.0;
    std::uint64_t id = 0U;
    std::chrono::steady_clock::time_point queued_at;
  };

  struct LiveBlock {
    std::weak_ptr<const StaticMapBlock> block;
    std::uint64_t bytes = 0U;
  };

  void worker_loop();
  void publish_locked();
  void update_status_locked();
  SpatialMapStatus measured_status_locked() const;
  bool admit_locked(const LoadRequest &request, std::uint64_t reservation);
  void event_locked(const char *event, const LoadRequest &request,
                    const SpatialBlockLoadMetrics &metrics = SpatialBlockLoadMetrics(),
                    double service_ms = 0.0, const std::string &error = "");

  std::shared_ptr<SpatialBlockSource> source_;
  SpatialMapRuntimeConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable idle_cv_;
  bool stop_ = false;
  std::size_t active_loads_ = 0U;
  std::uint64_t generation_ = 0U;
  std::uint64_t load_sequence_ = 0U;
  std::uint64_t next_request_id_ = 0U;
  std::uint64_t memory_limit_ = 0U;
  std::uint64_t reserved_bytes_ = 0U;
  std::vector<LiveBlock> live_blocks_;
  std::unordered_map<BlockIndex, std::size_t, BlockIndexHash> priority_;
  std::chrono::steady_clock::time_point io_ready_at_;
  std::chrono::steady_clock::time_point started_ = std::chrono::steady_clock::now();
  std::ofstream events_csv_;
  std::ofstream frames_csv_;
  std::deque<LoadRequest> queue_;
  std::unordered_set<BlockIndex, BlockIndexHash> queued_;
  std::unordered_map<BlockIndex, std::uint64_t, BlockIndexHash>
      active_requests_;
  std::unordered_set<BlockIndex, BlockIndexHash> core_required_;
  std::unordered_set<BlockIndex, BlockIndexHash> desired_;
  std::unordered_set<BlockIndex, BlockIndexHash> failed_;
  std::unordered_map<BlockIndex, std::shared_ptr<const StaticMapBlock>,
                     BlockIndexHash>
      staging_;
  std::shared_ptr<const SpatialMapSnapshot> snapshot_;
  SpatialMapStatus status_;
  std::vector<std::thread> workers_;
};
