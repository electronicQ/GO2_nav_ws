"""Keep both ROS2 localization views wired to the retained display map."""
from pathlib import Path
import unittest

import yaml


class LocalizationRvizConfigTest(unittest.TestCase):
    package = Path(__file__).resolve().parents[1]

    def test_localization_presets_use_prebuilt_display_pcd(self):
        for suffix in ('mid360', 'avia_fly'):
            with self.subTest(sensor=suffix):
                params = yaml.safe_load((self.package / 'config' /
                    f'ros2_localization_{suffix}.yaml').read_text())['/**']['ros__parameters']
                visualization = params['Localization']['visualization']
                self.assertTrue(visualization['publish_map_cloud'])
                self.assertEqual(visualization['map_cloud_path'], '')
                self.assertNotIn('leaf_size', visualization)
                self.assertEqual(params['General']['mode'], 'localization')
                self.assertEqual(params['General']['map_export_path'], '')

    def test_launches_load_their_localization_views(self):
        for suffix, view in (('mid360', 'mid360'), ('avia_fly', 'avia')):
            with self.subTest(sensor=suffix):
                launch = (self.package / 'launch' /
                          f'vxlm_localization_{suffix}.launch.py').read_text()
                self.assertIn(f"'ros2_localization_{view}.rviz'", launch)

    def test_map_is_retained_and_current_scan_does_not_accumulate(self):
        for suffix in ('mid360', 'avia'):
            with self.subTest(sensor=suffix):
                manager = yaml.safe_load((self.package / 'rviz_cfg' /
                    f'ros2_localization_{suffix}.rviz').read_text())['Visualization Manager']
                displays = {display['Name']: display for display in manager['Displays']}
                history = displays['Historical Map']
                self.assertTrue(history['Enabled'])
                self.assertEqual(history['Topic']['Value'], '/localization/map_cloud')
                self.assertEqual(history['Topic']['Durability Policy'], 'Transient Local')
                self.assertEqual(history['Topic']['Reliability Policy'], 'Reliable')
                self.assertEqual(history['Topic']['Depth'], 1)
                scan = displays['Current Scan']
                self.assertEqual(scan['Topic']['Value'], '/map_scan')
                self.assertEqual(scan['Decay Time'], 0)
                self.assertEqual(scan['Topic']['Durability Policy'], 'Volatile')
                self.assertEqual(displays['Current Path']['Topic']['Value'], '/map_path')
                self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')


if __name__ == '__main__':
    unittest.main()
