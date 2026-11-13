#include "spatial_map_manager.hpp"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>
#include <unistd.h>

namespace {

bool finiteNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

double distanceToAabb(const Eigen::Vector3d &point,
                      const BlockAabb &bounds) {
  double squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    double delta = 0.0;
    if (point[axis] < bounds.minimum[axis]) {
      delta = bounds.minimum[axis] - point[axis];
    } else if (point[axis] > bounds.maximum[axis]) {
      delta = point[axis] - bounds.maximum[axis];
    }
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

}  // namespace

bool SpatialMapRuntimeConfig::validate(std::string &error) const {
  error.clear();
  if (!finiteNonNegative(preload_radius) ||
      !finiteNonNegative(load_radius) ||
      !finiteNonNegative(unload_radius) ||
      !finiteNonNegative(prefetch_time_sec) ||
      !finiteNonNegative(map_gap_timeout_sec) ||
      !finiteNonNegative(preload_timeout_sec) ||
      !finiteNonNegative(max_block_memory_mb) ||
      !finiteNonNegative(io_rate_mb_per_sec) ||
      !finiteNonNegative(io_fixed_latency_ms) ||
      max_block_memory_mb > 1e9 || io_rate_mb_per_sec > 1e9 ||
      (max_block_memory_mb > 0.0 && max_block_memory_mb < 1.0 / 1048576.0) ||
      (io_rate_mb_per_sec > 0.0 && io_rate_mb_per_sec < 0.001) ||
      io_fixed_latency_ms > 3600000.0) {
    error = "spatial map runtime values must be finite and non-negative";
    return false;
  }
  if (preload_radius > load_radius || load_radius >= unload_radius) {
    error = "spatial map radii must satisfy preload <= load < unload";
    return false;
  }
  if (max_background_loads <= 0) {
    error = "max_background_loads must be positive";
    return false;
  }
  return true;
}

FileSpatialBlockSource::FileSpatialBlockSource(
    std::string directory, SpatialMapCatalog catalog)
    : directory_(std::move(directory)), catalog_(std::move(catalog)) {}

const SpatialMapCatalog &FileSpatialBlockSource::catalog() const {
  return catalog_;
}

bool FileSpatialBlockSource::load(const BlockIndex &index,
                                  StaticMapBlock &block,
                                  std::string &error) {
  return SpatialMapIO::load_block(directory_, catalog_, index, block, error);
}

bool FileSpatialBlockSource::load_with_metrics(
    const BlockIndex &index, StaticMapBlock &block, std::string &error,
    SpatialBlockLoadMetrics &metrics) {
  return SpatialMapIO::load_block(directory_, catalog_, index, block, error, &metrics);
}

std::uint64_t FileSpatialBlockSource::read_cost(const SpatialBlockIndexEntry &entry) const {
  return spatial_block_read_bytes(entry);
}

std::uint64_t FileSpatialBlockSource::scratch_cost(const SpatialBlockIndexEntry &entry) const {
  return spatial_block_scratch_bytes(entry);
}

SpatialMapManager::SpatialMapManager(
    std::shared_ptr<SpatialBlockSource> source,
    const SpatialMapRuntimeConfig &config)
    : source_(std::move(source)), config_(config) {
  if (!source_) {
    throw std::invalid_argument("spatial block source is null");
  }
  std::string error;
  if (!config_.validate(error)) {
    throw std::invalid_argument(error);
  }
  std::string catalog_error;
  if (!source_->catalog().config.validate(catalog_error)) {
    throw std::invalid_argument("invalid spatial block catalog: " +
                                catalog_error);
  }
  memory_limit_ = static_cast<std::uint64_t>(config_.max_block_memory_mb * 1048576.0);
  io_ready_at_ = started_;
  for (const auto &entry : source_->catalog().entries) {
    const auto decoded = spatial_block_decoded_bytes(entry);
    const auto scratch = source_->scratch_cost(entry);
    if (scratch > std::numeric_limits<std::uint64_t>::max() - decoded) {
      throw std::invalid_argument("spatial block reservation overflow");
    }
    const auto read_cost = source_->read_cost(entry);
    if (config_.io_rate_mb_per_sec > 0.0 &&
        read_cost / (config_.io_rate_mb_per_sec * 1048576.0) > 86400.0)
      throw std::invalid_argument("single-block I/O service time exceeds one day");
  }
  if (metrics_enabled()) {
    const boost::filesystem::path directory(config_.metrics_directory);
    if (!directory.parent_path().empty())
      boost::filesystem::create_directories(directory.parent_path());
    if (!boost::filesystem::create_directory(directory))
      throw std::invalid_argument("metrics_directory already exists: " + directory.string());
    events_csv_.open((directory / "block_events.csv").string());
    frames_csv_.open((directory / "frames.csv").string());
    if (!events_csv_ || !frames_csv_)
      throw std::runtime_error("cannot create spatial map metrics files");
    events_csv_ << "steady_sec,event,generation,request_id,x,y,z,queue_age_ms,service_ms,decode_ms,read_bytes,live_block_bytes,reserved_bytes,error\n";
    frames_csv_ << "steady_sec,sensor_time,state,valid,schedule_ms,match_ms,frame_compute_ms,query_points,missing_block_points,matched_points,residual,core_ready,active,pending,live_block_bytes,pinned_block_bytes,reserved_bytes,peak_accounted_bytes,read_bytes,load_attempts,load_failures,discarded_loads,evictions,budget_waits,budget_blocked,rss_bytes\n";
    std::ofstream config_file((directory / "resource_config.txt").string());
    config_file << "memory_scope=block_objects_and_vector_capacity_plus_inflight_estimates\n"
                << "memory_limit_bytes=" << memory_limit_ << '\n'
                << "io_scope=shared_zero_burst_logical_read_service\n"
                << "io_rate_mib_per_sec=" << config_.io_rate_mb_per_sec << '\n'
                << "io_fixed_latency_ms=" << config_.io_fixed_latency_ms << '\n';
  }
  std::shared_ptr<SpatialMapSnapshot> initial(new SpatialMapSnapshot());
  snapshot_ = initial;
  workers_.reserve(static_cast<std::size_t>(config_.max_background_loads));
  for (int i = 0; i < config_.max_background_loads; ++i) {
    workers_.emplace_back(&SpatialMapManager::worker_loop, this);
  }
}

SpatialMapManager::~SpatialMapManager() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    for (const auto &request : queue_) event_locked("cancelled_queue", request);
    queue_.clear();
    queued_.clear();
  }
  work_cv_.notify_all();
  for (std::thread &worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

std::uint64_t SpatialMapManager::begin_session() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("spatial map session generation exhausted");
  }
  ++generation_;
  for (const auto &request : queue_) event_locked("cancelled_queue", request);
  queue_.clear();
  queued_.clear();
  core_required_.clear();
  desired_.clear();
  failed_.clear();
  staging_.clear();
  io_ready_at_ = std::chrono::steady_clock::now();
  priority_.clear();
  status_.last_error.clear();
  status_.budget_blocked = false;
  status_.generation = generation_;
  status_.load_sequence = load_sequence_;
  publish_locked();
  update_status_locked();
  idle_cv_.notify_all();
  work_cv_.notify_all();
  return generation_;
}

bool SpatialMapManager::request(const Eigen::Vector3d &position,
                                const Eigen::Vector3d &velocity,
                                std::uint64_t generation,
                                std::string *error) {
  if (error != nullptr) {
    error->clear();
  }
  if (!position.array().isFinite().all() ||
      !velocity.array().isFinite().all()) {
    if (error != nullptr) {
      *error = "spatial map request pose or velocity is non-finite";
    }
    return false;
  }
  const Eigen::Vector3d predicted =
      position + velocity * config_.prefetch_time_sec;
  if (!predicted.array().isFinite().all()) {
    if (error != nullptr) {
      *error = "spatial map predicted position is non-finite";
    }
    return false;
  }

  try {
    const SpatialMapCatalog &catalog = source_->catalog();
    const std::vector<BlockIndex> core_candidates =
        blocks_intersecting_sphere(position, config_.preload_radius,
                                   catalog.config);
    const std::vector<BlockIndex> current_candidates =
        blocks_intersecting_sphere(position, config_.load_radius,
                                   catalog.config);
    const std::vector<BlockIndex> predicted_candidates =
        blocks_intersecting_sphere(predicted, config_.load_radius,
                                   catalog.config);

    std::unordered_set<BlockIndex, BlockIndexHash> core;
    std::unordered_set<BlockIndex, BlockIndexHash> desired;
    auto add_if_present = [&](const BlockIndex &index,
                              std::unordered_set<BlockIndex, BlockIndexHash>
                                  &target) {
      if (catalog.lookup.find(index) != catalog.lookup.end()) {
        target.insert(index);
      }
    };
    for (const BlockIndex &index : core_candidates) {
      add_if_present(index, core);
      add_if_present(index, desired);
    }
    for (const BlockIndex &index : current_candidates) {
      add_if_present(index, desired);
    }
    for (const BlockIndex &index : predicted_candidates) {
      add_if_present(index, desired);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != generation_) {
        if (error != nullptr) {
          *error = "spatial map request generation is stale";
        }
        return false;
      }
      core_required_ = std::move(core);
      desired_ = std::move(desired);
      std::unordered_map<BlockIndex, LoadRequest, BlockIndexHash> previous;
      for (const auto &old : queue_) previous.emplace(old.index, old);
      queue_.clear();
      queued_.clear();

      bool removed = false;
      for (auto it = staging_.begin(); it != staging_.end();) {
        const BlockAabb bounds = block_aabb(it->first, catalog.config);
        if (desired_.count(it->first) == 0U &&
            !sphere_intersects_aabb(position, config_.unload_radius,
                                    bounds)) {
          LoadRequest evicted;
          evicted.index = it->first;
          evicted.generation = generation_;
          evicted.queued_at = std::chrono::steady_clock::now();
          it = staging_.erase(it);
          ++status_.evictions;
          event_locked("evicted_hysteresis", evicted);
          removed = true;
        } else {
          ++it;
        }
      }

      std::vector<LoadRequest> ranked;
      ranked.reserve(desired_.size());
      for (const BlockIndex &index : desired_) {
        const auto catalog_entry = catalog.lookup.find(index);
        if (catalog_entry == catalog.lookup.end() ||
            catalog_entry->second >= catalog.entries.size()) {
          continue;
        }
        LoadRequest request;
        request.index = index;
        request.generation = generation_;
        request.core = core_required_.count(index) != 0U;
        request.distance = distanceToAabb(
            position, catalog.entries[catalog_entry->second].bounds);
        ranked.push_back(request);
      }
      std::sort(ranked.begin(), ranked.end(),
                [](const LoadRequest &left, const LoadRequest &right) {
                  if (left.core != right.core) {
                    return left.core && !right.core;
                  }
                  if (left.distance != right.distance) {
                    return left.distance < right.distance;
                  }
                  return left.index < right.index;
                });
      priority_.clear();
      for (std::size_t rank = 0U; rank < ranked.size(); ++rank) {
        auto request = ranked[rank];
        priority_[request.index] = rank;
        if (staging_.count(request.index) || failed_.count(request.index)) continue;
        const auto active = active_requests_.find(request.index);
        if (active != active_requests_.end() && active->second == generation_) continue;
        const auto old = previous.find(request.index);
        if (old != previous.end()) {
          request.id = old->second.id;
          request.queued_at = old->second.queued_at;
          previous.erase(old);
        } else {
          request.id = ++next_request_id_;
          request.queued_at = std::chrono::steady_clock::now();
          event_locked("queued", request);
        }
        queue_.push_back(request);
        queued_.insert(request.index);
      }
      for (const auto &old : previous) event_locked("cancelled_queue", old.second);
      if (removed) {
        publish_locked();
      }
      update_status_locked();
      if (queue_.empty() && active_loads_ == 0U) {
        idle_cv_.notify_all();
      }
    }
    work_cv_.notify_all();
    return true;
  } catch (const std::exception &exception) {
    if (error != nullptr) {
      *error = std::string("spatial map request failed: ") + exception.what();
    }
    return false;
  }
}

std::shared_ptr<const SpatialMapSnapshot> SpatialMapManager::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

SpatialMapStatus SpatialMapManager::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return measured_status_locked();
}

bool SpatialMapManager::wait_until_idle(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  return idle_cv_.wait_for(lock, timeout, [&] {
    return queue_.empty() && active_loads_ == 0U;
  });
}

void SpatialMapManager::worker_loop() {
  while (true) {
    LoadRequest request;
    std::uint64_t reservation = 0U;
    std::uint64_t decoded_reservation = 0U;
    bool cancelled = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [&] { return stop_ || !queue_.empty(); });
      if (stop_) {
        return;
      }
      request = queue_.front();
      const auto &entry = source_->catalog().entries.at(
          source_->catalog().lookup.at(request.index));
      decoded_reservation = spatial_block_decoded_bytes(entry);
      reservation = decoded_reservation + source_->scratch_cost(entry);
      if (!admit_locked(request, reservation)) {
        status_.budget_blocked = true;
        ++status_.budget_waits;
        update_status_locked();
        // Snapshot owners release references without notifying the manager.
        work_cv_.wait_for(lock, std::chrono::milliseconds(10));
        continue;
      }
      status_.budget_blocked = false;
      reserved_bytes_ += reservation;
      queue_.pop_front();
      queued_.erase(request.index);
      active_requests_[request.index] = request.generation;
      ++active_loads_;
      ++status_.load_attempts;
      update_status_locked();
      event_locked("admitted", request);

      double service_sec = config_.io_fixed_latency_ms / 1000.0;
      if (config_.io_rate_mb_per_sec > 0.0)
        service_sec += source_->read_cost(entry) /
                       (config_.io_rate_mb_per_sec * 1048576.0);
      if (service_sec > 0.0) {
        // One shared zero-burst service timeline, independent of worker count.
        io_ready_at_ = std::max(io_ready_at_, std::chrono::steady_clock::now()) +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(service_sec));
        const auto ready = io_ready_at_;
        work_cv_.wait_until(lock, ready, [&] {
          return stop_ || request.generation != generation_ ||
                 desired_.count(request.index) == 0U;
        });
      }
      cancelled = stop_ || request.generation != generation_ ||
                  desired_.count(request.index) == 0U;
    }

    const std::chrono::steady_clock::time_point load_started =
        std::chrono::steady_clock::now();
    StaticMapBlock loaded;
    std::string load_error;
    SpatialBlockLoadMetrics metrics;
    bool success = false;
    if (!cancelled) {
      try {
        success = source_->load_with_metrics(request.index, loaded, load_error, metrics);
      } catch (const std::exception &exception) {
        load_error = exception.what();
      }
    }
    const double load_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - load_started).count();
    if (success && loaded.index != request.index) {
      success = false;
      load_error = "block source returned a different block index";
    }
    if (success && memory_limit_ != 0U &&
        spatial_block_memory_bytes(loaded) > decoded_reservation) {
      success = false;
      load_error = "decoded block exceeds catalog memory reservation";
    }
    std::shared_ptr<const StaticMapBlock> immutable;
    if (success) {
      immutable.reset(new StaticMapBlock(std::move(loaded)));
    } else {
      loaded = StaticMapBlock();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      reserved_bytes_ -= reservation;
      if (immutable)
        live_blocks_.push_back({immutable, spatial_block_memory_bytes(*immutable)});
      status_.read_bytes += metrics.read_bytes;
      if (!success && !cancelled) ++status_.load_failures;
      if (active_loads_ != 0U) {
        --active_loads_;
      }
      const auto active = active_requests_.find(request.index);
      if (active != active_requests_.end() &&
          active->second == request.generation) {
        active_requests_.erase(active);
      }
      if (request.generation == generation_ &&
          desired_.count(request.index) != 0U) {
        if (load_sequence_ !=
            std::numeric_limits<std::uint64_t>::max()) {
          ++load_sequence_;
        }
        status_.load_sequence = load_sequence_;
        status_.last_load_ms = load_ms;
        if (success) {
          staging_[request.index] = std::move(immutable);
          publish_locked();
          event_locked("published", request, metrics, load_ms);
        } else {
          if (!cancelled) failed_.insert(request.index);
          status_.last_error = "failed loading block " +
                               std::to_string(request.index.x) + "," +
                               std::to_string(request.index.y) + "," +
                               std::to_string(request.index.z) + ": " +
                               load_error;
          event_locked(cancelled ? "cancelled_active" : "failed", request, metrics,
                       load_ms, load_error);
        }
      } else {
        ++status_.discarded_loads;
        event_locked("discarded", request, metrics, load_ms, load_error);
      }
      immutable.reset();
      update_status_locked();
      if (queue_.empty() && active_loads_ == 0U) {
        idle_cv_.notify_all();
      }
    }
    work_cv_.notify_all();
  }
}

SpatialMapStatus SpatialMapManager::measured_status_locked() const {
  SpatialMapStatus measured = status_;
  measured.live_block_bytes = 0U;
  for (const auto &record : live_blocks_)
    if (!record.block.expired()) measured.live_block_bytes += record.bytes;
  std::uint64_t staging_bytes = 0U;
  for (const auto &item : staging_) staging_bytes += spatial_block_memory_bytes(*item.second);
  measured.pinned_block_bytes = measured.live_block_bytes > staging_bytes
      ? measured.live_block_bytes - staging_bytes : 0U;
  measured.reserved_bytes = reserved_bytes_;
  measured.peak_accounted_bytes = std::max(measured.peak_accounted_bytes,
                                         measured.live_block_bytes + reserved_bytes_);
  return measured;
}

bool SpatialMapManager::admit_locked(const LoadRequest &request,
                                    std::uint64_t reservation) {
  if (memory_limit_ == 0U) return true;
  if (reservation > memory_limit_) return false;
  while (true) {
    const auto measured = measured_status_locked();
    if (measured.live_block_bytes + reserved_bytes_ <= memory_limit_ - reservation)
      return true;
    auto victim = staging_.end();
    std::size_t worst_rank = priority_.at(request.index);
    for (auto it = staging_.begin(); it != staging_.end(); ++it) {
      if (core_required_.count(it->first)) continue;
      const auto rank = priority_.find(it->first);
      const auto value = rank == priority_.end() ? std::numeric_limits<std::size_t>::max()
                                               : rank->second;
      if (value > worst_rank || (value == worst_rank && victim != staging_.end() &&
                                victim->first < it->first)) {
        victim = it;
        worst_rank = value;
      }
    }
    if (victim == staging_.end()) return false;
    LoadRequest evicted;
    evicted.index = victim->first;
    evicted.generation = generation_;
    evicted.queued_at = std::chrono::steady_clock::now();
    staging_.erase(victim);
    ++status_.evictions;
    publish_locked();
    event_locked("evicted_budget", evicted);
  }
}

void SpatialMapManager::event_locked(const char *event, const LoadRequest &request,
                                    const SpatialBlockLoadMetrics &metrics,
                                    double service_ms, const std::string &error) {
  if (!metrics_enabled()) return;
  const auto now = std::chrono::steady_clock::now();
  const auto measured = measured_status_locked();
  std::string escaped;
  for (char value : error) {
    if (value == '"') escaped += '"';
    escaped += value == '\n' || value == '\r' ? ' ' : value;
  }
  events_csv_ << std::setprecision(17)
      << std::chrono::duration<double>(now - started_).count() << ',' << event << ','
      << request.generation << ',' << request.id << ',' << request.index.x << ','
      << request.index.y << ',' << request.index.z << ','
      << std::chrono::duration<double, std::milli>(now - request.queued_at).count() << ','
      << service_ms << ',' << metrics.decode_ms << ',' << metrics.read_bytes << ','
      << measured.live_block_bytes << ',' << reserved_bytes_ << ",\"" << escaped << "\"\n";
  events_csv_.flush();
}

void SpatialMapManager::record_frame(const SpatialFrameMetrics &frame) {
  if (!metrics_enabled()) return;
  std::uint64_t pages = 0U, resident_pages = 0U;
  std::ifstream proc("/proc/self/statm");
  proc >> pages >> resident_pages;
  const long page_size = sysconf(_SC_PAGESIZE);
  const std::uint64_t rss = page_size > 0 ? resident_pages * page_size : 0U;
  std::lock_guard<std::mutex> lock(mutex_);
  const auto measured = measured_status_locked();
  frames_csv_ << std::setprecision(17)
      << std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count()
      << ',' << frame.sensor_time << ',' << frame.state << ',' << frame.valid
      << ',' << frame.schedule_ms << ',' << frame.match_ms << ',' << frame.frame_compute_ms
      << ',' << frame.query_points << ',' << frame.missing_block_points
      << ',' << frame.matched_points << ',' << frame.residual
      << ',' << measured.core_ready << ',' << measured.loaded_block_count
      << ',' << measured.pending_block_count << ',' << measured.live_block_bytes
      << ',' << measured.pinned_block_bytes << ',' << measured.reserved_bytes
      << ',' << measured.peak_accounted_bytes << ',' << measured.read_bytes
      << ',' << measured.load_attempts << ',' << measured.load_failures
      << ',' << measured.discarded_loads << ',' << measured.evictions
      << ',' << measured.budget_waits << ',' << measured.budget_blocked << ',' << rss << '\n';
  frames_csv_.flush();
}

void SpatialMapManager::publish_locked() {
  std::shared_ptr<SpatialMapSnapshot> next(new SpatialMapSnapshot());
  next->generation = generation_;
  next->blocks = staging_;
  snapshot_ = next;
}

void SpatialMapManager::update_status_locked() {
  live_blocks_.erase(std::remove_if(live_blocks_.begin(), live_blocks_.end(),
      [](const LiveBlock &record) { return record.block.expired(); }), live_blocks_.end());
  status_ = measured_status_locked();
  status_.generation = generation_;
  status_.core_block_count = core_required_.size();
  status_.loaded_block_count = staging_.size();
  status_.failed_block_count = failed_.size();
  status_.failed_core_block_count = 0U;
  for (const BlockIndex &index : core_required_) {
    if (failed_.count(index) != 0U) {
      ++status_.failed_core_block_count;
    }
  }
  std::size_t current_active_loads = 0U;
  for (const auto &active : active_requests_) {
    if (active.second == generation_) {
      ++current_active_loads;
    }
  }
  status_.pending_block_count = queue_.size() + current_active_loads;
  status_.idle = status_.pending_block_count == 0U;
  if (status_.idle) status_.budget_blocked = false;
  status_.core_ready = !core_required_.empty();
  for (const BlockIndex &index : core_required_) {
    if (staging_.find(index) == staging_.end()) {
      status_.core_ready = false;
      break;
    }
  }
  status_.map_gap = !status_.core_ready && status_.idle;
}
