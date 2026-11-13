"""Localization must not create or save legacy mapping output directories."""
from pathlib import Path
import unittest


class LocalizationOutputContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (Path(__file__).resolve().parents[1] / 'src/voxelslam.cpp').read_text()
        cls.constructor = source.split('VOXEL_SLAM(ros::NodeHandle &n)', 1)[1].split(
            'sub_imu = ', 1)[0]

    def test_localization_disables_legacy_save_flag_after_reading_parameter(self):
        parameter = self.constructor.index('n.param<int>("General/is_save_map"')
        guard = self.constructor.index('if(operation_mode == "localization")\n      is_save_map = 0;')
        self.assertLess(parameter, guard)

    def test_legacy_directory_access_and_creation_are_mapping_only(self):
        start = self.constructor.index('if(operation_mode == "mapping")\n    {\n      int ss = 0;')
        opening = self.constructor.index('{', start)
        depth = 1
        end = opening + 1
        while depth:
            depth += (self.constructor[end] == '{') - (self.constructor[end] == '}')
            end += 1
        block = self.constructor[opening:end]
        self.assertIn('access((savepath+bagname+"/").c_str(), X_OK)', block)
        self.assertIn('ss = system(cmd.c_str());', block)
        self.assertIn('if(ss != 0 && is_save_map == 1)', block)
        self.assertNotIn('system(', self.constructor[:start] + self.constructor[end:])


if __name__ == '__main__':
    unittest.main()
