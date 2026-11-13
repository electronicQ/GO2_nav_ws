from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]

def test_sc_only_defaults_and_independent_mapping_sampling():
    cfg = yaml.safe_load((ROOT/'config/ros2_scancontext_relocalization.yaml').read_text())['/**']['ros__parameters']['Localization']
    assert cfg['history']['window_scans'] == cfg['scancontext']['query_scans'] == 5
    assert cfg['history']['translation_m'] == 0.5
    assert cfg['history']['rotation_deg'] == 10
    assert cfg['scancontext']['query_leaf_size_m'] == 0
    assert cfg['scancontext']['registration_leaf_size_m'] == 0.3
    assert not (ROOT/'config/ros2_btc_relocalization.yaml').exists()
    common=yaml.safe_load((ROOT/'config/ros2_localization_common.yaml').read_text())['/**']['ros__parameters']['Localization']
    assert common['direct_match']['query_scans'] == 3
    assert common['direct_match']['query_leaf_size_m'] == 0.2
    assert common['dual_tf']['audit_scans'] == 10
    for sensor in ('avia_fly','mid360'):
        launch=(ROOT/f'launch/vxlm_localization_{sensor}.launch.py').read_text()
        assert 'ros2_scancontext_relocalization.yaml' in launch
        assert 'btc_config' not in launch

def test_sc_candidate_uses_latest_dense_scans_and_separate_sc_commit():
    source=(ROOT/'src/voxelslam.cpp').read_text()
    worker=source.split('void thd_dual_tf_sc(',1)[1].split('// Build the pose graph',1)[0]
    assert 'latest.end() - scancontext_options.query_scans' in worker
    assert 'scan_map_store->fuse(id,history_scans)' in worker
    assert worker.count('dual_alignment.accept(')==1
    assert worker.count('dual_alignment.accept_scancontext(')==1
    assert 'GenerateSTDescs' not in worker
    assert 'BtcRelocalizer' not in worker
    assert 'global_points' in source
    assert 'detail_window.back()' in source

def test_history_bank_keeps_descriptors_not_all_pointclouds():
    source=(ROOT/'src/scancontext_relocalization.cpp').read_text()
    assert 'detectLoopClosureIDGivenScan(' not in source
    assert 'manager.polarcontext_invkeys_mat_' in source
    assert 'std::min(size_t(options.candidates), poses.size())' in source
    assert 'std::vector<Cloud::Ptr>' not in source
    assert 'p.loader(frame.id)' in source
