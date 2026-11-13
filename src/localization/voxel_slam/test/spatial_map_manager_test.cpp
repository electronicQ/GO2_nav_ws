#include "spatial_map_manager.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <boost/filesystem.hpp>
#include <condition_variable>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <thread>

namespace {

class ControllableBlockSource : public SpatialBlockSource {
 public:
  explicit ControllableBlockSource(const SpatialMapCatalog &catalog)
      : catalog_(catalog) {}

  const SpatialMapCatalog &catalog() const override { return catalog_; }

  bool load(const BlockIndex &index, StaticMapBlock &block,
            std::string &error) override {
    std::unique_lock<std::mutex> lock(mutex_);
    requested_.insert(index);
    ++load_counts_[index];
    requested_cv_.notify_all();
    release_cv_.wait(lock, [&] { return released_ || stopped_; });
    if (stopped_) {
      error = "source stopped";
      return false;
    }
    if (failed_.count(index) != 0U) {
      error = "injected corrupt block";
      return false;
    }
    block.index = index;
    RootVoxelRecord root;
    root.index = RootVoxelIndex(index.x * 20, index.y * 20, index.z * 10);
    root.root_node_index = 0U;
    block.roots.push_back(root);
    StaticOctreeNode node;
    node.center = Eigen::Vector3d(root.index.x + 0.5,
                                  root.index.y + 0.5,
                                  root.index.z + 0.5);
    node.half_length = 0.5;
    block.nodes.push_back(node);
    return true;
  }

  void release_all() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    release_cv_.notify_all();
  }

  void fail(const BlockIndex &index) { failed_.insert(index); }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    release_cv_.notify_all();
  }

  int load_count(const BlockIndex &index) {
    std::lock_guard<std::mutex> lock(mutex_);
    return load_counts_[index];
  }

 private:
  SpatialMapCatalog catalog_;
  std::mutex mutex_;
  std::condition_variable requested_cv_;
  std::condition_variable release_cv_;
  std::set<BlockIndex> requested_;
  std::map<BlockIndex, int> load_counts_;
  std::set<BlockIndex> failed_;
  bool released_ = false;
  bool stopped_ = false;
};

SpatialMapCatalog catalogFixture() {
  SpatialMapCatalog catalog;
  for (int x = -1; x <= 6; ++x) {
    SpatialBlockIndexEntry entry;
    entry.index = BlockIndex(x, 0, 0);
    entry.root_count = 1U;
    entry.node_count = 1U;
    entry.length = 136U;
    entry.bounds = block_aabb(entry.index, catalog.config);
    catalog.lookup.emplace(entry.index, catalog.entries.size());
    catalog.entries.push_back(entry);
  }
  return catalog;
}

SpatialMapRuntimeConfig runtimeConfig() {
  SpatialMapRuntimeConfig config;
  config.preload_radius = 15.0;
  config.load_radius = 30.0;
  config.unload_radius = 40.0;
  config.prefetch_time_sec = 2.0;
  config.max_background_loads = 2;
  return config;
}

const std::chrono::milliseconds kTimeout(2000);

TEST(SpatialMapManager, LoadsCoreThenPrefetchesAndUnloadsWithHysteresis) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  generation);
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_TRUE(manager.status().core_ready);
  const auto first = manager.snapshot();
  ASSERT_FALSE(first->blocks.empty());

  manager.request(Eigen::Vector3d(100.0, 0.0, 0.0),
                  Eigen::Vector3d::Zero(), generation);
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  const auto second = manager.snapshot();
  EXPECT_NE(second.get(), first.get());
  EXPECT_EQ(second->blocks.count(BlockIndex(0, 0, 0)), 0U);
  EXPECT_GT(second->blocks.count(BlockIndex(5, 0, 0)), 0U);
}

TEST(SpatialMapManager, DiscardsLoadsFromOldSessionGeneration) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t old_generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  old_generation);
  const std::uint64_t new_generation = manager.begin_session();
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(manager.status().generation, new_generation);
  EXPECT_TRUE(manager.snapshot()->blocks.empty());
}

TEST(SpatialMapManager, ReportsLoadFailureWithoutPublishingCorruptBlock) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->fail(BlockIndex(0, 0, 0));
  source->release_all();
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  generation);
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  const SpatialMapStatus status = manager.status();
  EXPECT_FALSE(status.last_error.empty());
  EXPECT_FALSE(status.core_ready);
  EXPECT_EQ(manager.snapshot()->blocks.count(BlockIndex(0, 0, 0)), 0U);
  EXPECT_GT(status.failed_core_block_count, 0U);
  const std::uint64_t failed_load_sequence = status.load_sequence;
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(), generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(source->load_count(BlockIndex(0, 0, 0)), 1);
  EXPECT_EQ(manager.status().load_sequence, failed_load_sequence);
}

TEST(SpatialMapManager, ConcurrentSnapshotsRemainImmutable) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->release_all();
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  const auto before = manager.snapshot();
  std::thread requester([&] {
    manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                    generation);
  });
  for (int i = 0; i < 100; ++i) {
    const auto current = manager.snapshot();
    ASSERT_TRUE(current);
    EXPECT_EQ(before->blocks.size(), 0U);
  }
  requester.join();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_FALSE(manager.snapshot()->blocks.empty());
  EXPECT_TRUE(before->blocks.empty());
}

TEST(SpatialMapManager, RepeatedRequestsDoNotDuplicateActiveLoads) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(), generation));
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), generation));
  }
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  for (const auto &entry : manager.snapshot()->blocks) {
    EXPECT_EQ(source->load_count(entry.first), 1);
  }
}

TEST(SpatialMapManager, KeepsDesiredPrefetchBlocksOutsideCurrentUnloadRadius) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->release_all();
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  const Eigen::Vector3d velocity(50.0, 0.0, 0.0);
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), velocity, generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  ASSERT_GT(manager.snapshot()->blocks.count(BlockIndex(5, 0, 0)), 0U);
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), velocity, generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(source->load_count(BlockIndex(5, 0, 0)), 1);
}

TEST(SpatialMapRuntimeConfig, RejectsInvalidRadiusOrdering) {
  SpatialMapRuntimeConfig config = runtimeConfig();
  config.unload_radius = config.load_radius - 1.0;
  std::string error;
  EXPECT_FALSE(config.validate(error));
}

TEST(SpatialMapManager, BudgetCountsOldSnapshotsUntilTheyAreReleased) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->release_all();
  auto config = runtimeConfig();
  config.preload_radius = 0.0;
  config.load_radius = 1.0;
  config.unload_radius = 2.0;
  config.prefetch_time_sec = 0.0;
  const auto cost = spatial_block_decoded_bytes(source->catalog().entries[1]);
  config.max_block_memory_mb = cost / 1048576.0;
  SpatialMapManager manager(source, config);
  auto generation = manager.begin_session();
  ASSERT_TRUE(manager.request(Eigen::Vector3d(10, 10, 5), Eigen::Vector3d::Zero(), generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  auto first = manager.snapshot();
  ASSERT_TRUE(manager.status().core_ready);
  ASSERT_TRUE(manager.request(Eigen::Vector3d(30, 10, 5), Eigen::Vector3d::Zero(), generation));
  EXPECT_FALSE(manager.wait_until_idle(std::chrono::milliseconds(40)));
  EXPECT_EQ(manager.status().pinned_block_bytes, cost);
  EXPECT_EQ(source->load_count(BlockIndex(1, 0, 0)), 0);
  EXPECT_LE(manager.status().peak_accounted_bytes, cost);
  first.reset();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_TRUE(manager.status().core_ready);
  EXPECT_EQ(manager.status().pinned_block_bytes, 0U);
  EXPECT_EQ(manager.status().live_block_bytes, cost);
  EXPECT_LE(manager.status().peak_accounted_bytes, cost);
}

TEST(SpatialMapManager, InsufficientBudgetDoesNotLoadOrPretendCoreIsReady) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->release_all();
  auto config = runtimeConfig();
  config.max_block_memory_mb = 1.0 / 1048576.0;
  SpatialMapManager manager(source, config);
  const auto generation = manager.begin_session();
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), generation));
  EXPECT_FALSE(manager.wait_until_idle(std::chrono::milliseconds(40)));
  EXPECT_TRUE(manager.status().budget_blocked);
  EXPECT_FALSE(manager.status().core_ready);
  EXPECT_EQ(manager.status().load_attempts, 0U);
  EXPECT_TRUE(manager.snapshot()->blocks.empty());
}

class CostedBlockSource : public ControllableBlockSource {
 public:
  using ControllableBlockSource::ControllableBlockSource;
  std::uint64_t read_cost(const SpatialBlockIndexEntry &) const override { return 65536U; }
  std::uint64_t scratch_cost(const SpatialBlockIndexEntry &) const override { return 1024U; }
  bool load_with_metrics(const BlockIndex &index, StaticMapBlock &block,
                        std::string &error, SpatialBlockLoadMetrics &metrics) override {
    const bool result = load(index, block, error);
    metrics.read_bytes = 65536U;
    return result;
  }
};

TEST(SpatialMapManager, SharedRateDoesNotMultiplyWithWorkersAndCountsEachReadOnce) {
  auto source = std::make_shared<CostedBlockSource>(catalogFixture());
  source->release_all();
  auto config = runtimeConfig();
  config.max_background_loads = 4;
  config.io_rate_mb_per_sec = 1.0;
  SpatialMapManager manager(source, config);
  const auto generation = manager.begin_session();
  const auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(manager.request(Eigen::Vector3d(10, 10, 5), Eigen::Vector3d::Zero(), generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  const auto status = manager.status();
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GT(status.load_attempts, 1U);
  EXPECT_EQ(status.read_bytes, status.load_attempts * 65536U);
  EXPECT_GE(elapsed, status.read_bytes / 1048576.0 - 0.005);
  ASSERT_TRUE(manager.request(Eigen::Vector3d(10, 10, 5), Eigen::Vector3d::Zero(), generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(manager.status().read_bytes, status.read_bytes);
}

TEST(SpatialMapManager, SessionCancellationInterruptsLongServiceWait) {
  auto source = std::make_shared<CostedBlockSource>(catalogFixture());
  source->release_all();
  auto config = runtimeConfig();
  config.io_fixed_latency_ms = 10000.0;
  SpatialMapManager manager(source, config);
  const auto generation = manager.begin_session();
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), generation));
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (manager.status().load_attempts == 0U && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_GT(manager.status().load_attempts, 0U);
  manager.begin_session();
  // begin_session must wake waits, not leave old workers blocking for ten seconds.
  EXPECT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(manager.status().read_bytes, 0U);
  EXPECT_EQ(manager.status().reserved_bytes, 0U);
}

TEST(SpatialMapManager, ConcurrentInFlightReservationsShareTheSameQuota) {
  auto source = std::make_shared<CostedBlockSource>(catalogFixture());
  source->release_all();
  auto config = runtimeConfig();
  config.io_fixed_latency_ms = 10000.0;
  config.max_background_loads = 4;
  const auto cost = spatial_block_decoded_bytes(source->catalog().entries.front()) + 1024U;
  config.max_block_memory_mb = cost / 1048576.0;
  SpatialMapManager manager(source, config);
  const auto generation = manager.begin_session();
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), generation));
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (manager.status().load_attempts == 0U && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(manager.status().load_attempts, 1U);
  EXPECT_EQ(manager.status().reserved_bytes, cost);
  EXPECT_LE(manager.status().peak_accounted_bytes, cost);
  manager.begin_session();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(manager.status().reserved_bytes, 0U);
}

TEST(SpatialMapManager, MetricsAreOptInAndExistingRunDirectoryIsRejected) {
  const auto directory = boost::filesystem::temp_directory_path() /
                         boost::filesystem::unique_path("voxel_metrics_%%%%-%%%%");
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->release_all();
  auto config = runtimeConfig();
  config.metrics_directory = directory.string();
  {
    SpatialMapManager manager(source, config);
    const auto generation = manager.begin_session();
    ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), generation));
    ASSERT_TRUE(manager.wait_until_idle(kTimeout));
    SpatialFrameMetrics frame;
    frame.state = "IN_MAP_TRACKING";
    frame.valid = true;
    manager.record_frame(frame);
    EXPECT_THROW(SpatialMapManager(source, config), std::invalid_argument);
  }
  std::ifstream input((directory / "block_events.csv").string());
  const std::string contents((std::istreambuf_iterator<char>(input)), {});
  EXPECT_NE(contents.find(",published,"), std::string::npos);
  EXPECT_NE(contents.find("request_id"), std::string::npos);
  boost::filesystem::remove_all(directory);
}

TEST(SpatialMapRuntimeConfig, RejectsInvalidResourceLimits) {
  std::string error;
  for (double value : {-1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    auto config = runtimeConfig();
    config.max_block_memory_mb = value;
    EXPECT_FALSE(config.validate(error));
    config = runtimeConfig();
    config.io_rate_mb_per_sec = value;
    EXPECT_FALSE(config.validate(error));
  }
}

}  // namespace
