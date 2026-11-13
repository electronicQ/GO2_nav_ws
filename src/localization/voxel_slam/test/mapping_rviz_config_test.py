"""Keep ROS2 mapping visualization consistent with the ROS1 map streams."""
from pathlib import Path
import unittest

import yaml


class MappingRvizConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((cls.package / 'rviz_cfg/ros2.rviz').read_text())
        cls.displays = {display['Name']: display
                        for display in config['Visualization Manager']['Displays']}
        cls.config = config

    def test_mapping_streams_accumulate_and_use_reset_aware_plugin(self):
        for name, topic, decay in [('Current Map', '/map_cmap', 300),
                                   ('Previous Map', '/map_pmap', 1000000)]:
            display = self.displays[name]
            self.assertTrue(display['Enabled'])
            self.assertEqual(display['Class'], 'voxelslam_pointcloud2/VoxelSLAMPointCloud2')
            self.assertEqual(display['Topic']['Value'], topic)
            self.assertEqual(display['Topic']['Durability Policy'], 'Volatile')
            self.assertEqual(display['Decay Time'], decay)

    def test_current_scan_remains_single_frame(self):
        self.assertEqual(self.displays['Map Scan']['Topic'], '/map_scan')
        self.assertEqual(self.displays['Map Scan']['Decay Time'], 0)
        self.assertEqual(self.config['Visualization Manager']['Global Options']['Fixed Frame'],
                         'camera_init')

    def test_mid360_and_avia_mapping_load_this_configuration(self):
        for name in ('vxlm_mid360.launch.py', 'vxlm_avia_fly.launch.py'):
            self.assertIn("'ros2.rviz'", (self.package / 'launch' / name).read_text())


if __name__ == '__main__':
    unittest.main()
