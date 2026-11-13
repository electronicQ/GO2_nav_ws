from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_worker_never_changes_frontend_or_runs_session_graph():
    source = (ROOT / 'src/voxelslam.cpp').read_text()
    worker = source.split('void thd_dual_tf_sc(', 1)[1].split('// Build the pose graph', 1)[0]
    assert 'dual_alignment.accept' in worker
    for forbidden in ('apply_frontend_global_correction', 'system_reset(',
                      'start_spatial_map_preparation', 'align_current_session_to_map',
                      'x_curr =', 'surf_map'):
        assert forbidden not in worker


def test_new_branch_defaults_and_old_map_compatibility():
    btc = yaml.safe_load((ROOT / 'config/ros2_localization_common.yaml').read_text())
    assert btc['/**']['ros__parameters']['Localization']['dual_tf']['enabled']
    for sensor in ('avia_fly', 'mid360'):
        mapping = yaml.safe_load((ROOT / f'config/ros2_{sensor}.yaml').read_text())
        assert mapping['/**']['ros__parameters']['General']['map_scan_leaf_size_m'] == 0.1
        cfg = yaml.safe_load((ROOT / f'config/ros2_localization_{sensor}.yaml').read_text())
        assert cfg['/**']['ros__parameters']['Odometry']['odom_frame'] == 'odom'
        assert 'btc_search_radius' not in cfg['/**']['ros__parameters']['Localization']


def test_clock_reset_precedes_waiting_for_old_scan_and_clears_initialization():
    source = (ROOT / 'src/voxelslam.cpp').read_text()
    frontend = source.split('void thd_odometry_localmapping(', 1)[1]
    assert frontend.index('if(dual_tf && frontend_reset_requested.exchange(false))') < frontend.index('if(!sync_packages(')
    for field in ('initialization_clouds', 'initialization_times', 'initialization_imus'):
        assert field + '.clear();' in source.split('void system_reset(', 1)[1].split('// After local BA', 1)[0]
    assert 'static vector<pcl::PointCloud<PointType>::Ptr> pl_origs' not in source


def test_direct_commit_keeps_future_audit_and_sc_has_separate_acceptance():
    source = (ROOT / 'src/voxelslam.cpp').read_text()
    worker = source.split('void thd_dual_tf_sc(', 1)[1].split('// Build the pose graph', 1)[0]
    assert worker.count('dual_alignment.accept(') == 1
    assert 'scan.state.t > pending.anchor.t' in worker
    assert 'audit_global_match(planes, *target' in worker
    assert 'audit_improves_prior(' in worker
    assert 'error, pending.fresh.back().state.t)' in worker
    assert worker.count('dual_alignment.accept_scancontext(') == 1
    assert 'alignment.aligned ? &predicted_position : nullptr' in worker


def test_direct_match_is_prior_gated_independent_of_btc_and_has_single_commit_path():
    source = (ROOT / 'src/voxelslam.cpp').read_text()
    worker = source.split('void thd_dual_tf_sc(', 1)[1].split('// Build the pose graph', 1)[0]
    direct = worker.split('if(direct_due &&', 1)[1].split('if(!btc_due)', 1)[0]
    assert 'alignment.aligned' in worker.split('const bool direct_due', 1)[1].split('if(direct_due', 1)[0]
    assert 'GenerateSTDescs' not in direct
    assert 'ExtractPlanes' in direct
    assert 'direct_map->target' in direct
    assert 'pending.active = true' in direct
    assert 'dual_alignment.accept' not in direct
    assert 'direct_local, alignment.confirmation_pending)' in worker
    assert 'dual_latest_scans.clear()' in source.split('void system_reset(', 1)[1].split('// After local BA', 1)[0]
