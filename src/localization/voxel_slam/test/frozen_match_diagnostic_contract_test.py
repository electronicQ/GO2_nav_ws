"""Source contracts for opt-in diagnostics; numerical evidence comes from replay."""
from pathlib import Path
import unittest


class FrozenMatchDiagnosticContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (Path(__file__).resolve().parents[1] / 'src/voxelslam.cpp').read_text()
        cls.diagnostic = source.split('void run_frozen_match_diagnostic(', 1)[1].split(
            '// The point-to-plane alignment for initialization', 1)[0]

    def test_opt_in_and_single_capture(self):
        self.assertIn('std::getenv("VOXEL_SLAM_MATCH_DIAGNOSTIC_DIR")', self.diagnostic)
        self.assertIn('if(captured || !directory || !*directory || !loaded) return;', self.diagnostic)

    def test_live_state_restored_on_exception(self):
        self.assertIn('~StateRestore() { target = saved; }', self.diagnostic)
        self.assertLess(self.diagnostic.index('restore{x_curr, live_state}'),
                        self.diagnostic.index('SpatialMapIO::load_catalog('))

    def test_frozen_map_comparison_uses_identical_input(self):
        self.assertIn('compare("loaded", loaded, input);', self.diagnostic)
        self.assertIn('compare("complete", complete, input);', self.diagnostic)
        self.assertIn('(x_curr.cov - live_state.cov).norm() > 1e-10', self.diagnostic)

    def test_mapping_reference_changes_only_pose(self):
        control = self.diagnostic.split('IMUST reference = input;', 1)[1].split(
            'std::ofstream ref(', 1)[0]
        self.assertIn('reference.p =', control)
        self.assertIn('reference.R =', control)
        self.assertNotIn('reference.cov =', control)
        self.assertNotIn('reference.v =', control)


if __name__ == '__main__':
    unittest.main()
