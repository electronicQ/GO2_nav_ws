"""Guard lock placement around shared voxel/slide-window lifecycle paths.

These are source contracts, not a substitute for a sanitizer or replay test.
"""
from pathlib import Path
import unittest


class LocalizationPipelineLockContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (Path(__file__).resolve().parents[1] / 'src/voxelslam.cpp').read_text()
        cls.odometry = source.split('void thd_odometry_localmapping(ros::NodeHandle &n)', 1)[1].split(
            'const SavedKeyframe *find_localization_keyframe', 1)[0]
        cls.btc = source.split('void thd_dual_tf_sc(', 1)[1].split(
            '// Build the pose graph', 1)[0]

    def test_pipeline_lock_precedes_idle_recycling(self):
        lock = self.odometry.index('unique_lock<mutex>(frontend_pipeline_mutex)')
        idle = self.odometry.index('if(!sync_packages(')
        recycle = self.odometry.index('delete sws[0].back();')
        self.assertLess(lock, idle)
        self.assertLess(idle, recycle)

    def test_shutdown_cleanup_uses_same_pipeline_lock(self):
        cleanup = self.odometry.split('unique_lock<mutex> localization_cleanup_lock;', 1)[1]
        lock = cleanup.index('unique_lock<mutex>(frontend_pipeline_mutex)')
        self.assertLess(lock, cleanup.index('delete iter->second;'))
        self.assertLess(cleanup.index('surf_map.clear();'), cleanup.index('delete sws[0][i];'))

    def test_global_worker_never_mutates_frontend(self):
        self.assertNotIn('apply_frontend_global_correction(', self.btc)
        self.assertNotIn('start_spatial_map_preparation(', self.btc)
        self.assertIn('while(n.ok() && !is_finish.load())', self.btc)

if __name__ == '__main__':
    unittest.main()
