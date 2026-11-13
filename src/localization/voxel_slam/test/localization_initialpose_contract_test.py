import pathlib
import re
import unittest


VOXELSLAM_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = VOXELSLAM_ROOT.parent
LOCALIZATION_SOURCE = VOXELSLAM_ROOT / "src" / "localization_state.cpp"
RVIZ_CONFIG = VOXELSLAM_ROOT / "rviz_cfg" / "back.rviz"
LOCALIZATION_CONFIG = VOXELSLAM_ROOT / "config" / "localization_mid360.yaml"
README = REPOSITORY_ROOT / "README.md"
PACKAGE_README = VOXELSLAM_ROOT / "README.md"


def function_body(source, signature):
    start = source.find(signature)
    if start < 0:
        raise AssertionError("missing function: {}".format(signature))
    brace = source.find("{", start)
    if brace < 0:
        raise AssertionError("missing body: {}".format(signature))
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError("unterminated body: {}".format(signature))


def if_condition(source, required_token):
    for match in re.finditer(r"\bif\s*\(", source):
        opening = source.find("(", match.start())
        depth = 0
        for index in range(opening, len(source)):
            if source[index] == "(":
                depth += 1
            elif source[index] == ")":
                depth -= 1
                if depth == 0:
                    condition = source[opening + 1:index]
                    if required_token in condition:
                        return condition
                    break
    raise AssertionError(
        "missing if condition containing: {}".format(required_token)
    )


def evaluate_rejection_predicate(frame_id, compared_frames, operators):
    comparisons = [frame_id != allowed for allowed in compared_frames]
    result = comparisons[0]
    for operator, comparison in zip(operators, comparisons[1:]):
        if operator == "&&":
            result = result and comparison
        elif operator == "||":
            result = result or comparison
        else:
            raise AssertionError(
                "unsupported logical operator: {}".format(operator)
            )
    return result


def markdown_section(source, start_heading, end_heading):
    start = source.find(start_heading)
    if start < 0:
        raise AssertionError("missing heading: {}".format(start_heading))
    end = source.find(end_heading, start + len(start_heading))
    if end < 0:
        raise AssertionError("missing heading: {}".format(end_heading))
    return source[start:end]


class LocalizationInitialPoseContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.localization_source = LOCALIZATION_SOURCE.read_text(encoding="utf-8")
        cls.rviz_config = RVIZ_CONFIG.read_text(encoding="utf-8")
        cls.localization_config = LOCALIZATION_CONFIG.read_text(encoding="utf-8")
        cls.readme = README.read_text(encoding="utf-8")
        cls.package_readme = PACKAGE_README.read_text(encoding="utf-8")

    def test_default_rviz_frame_matches_initialpose_contract(self):
        self.assertRegex(
            self.rviz_config,
            r"(?m)^\s+Fixed Frame:\s*camera_init\s*$",
        )
        submit_body = function_body(
            self.localization_source, "bool InitialPoseRequestBuffer::submit("
        )
        predicate = if_condition(submit_body, "frame_id")
        compared_frames = re.findall(r'frame_id\s*!=\s*"([^"]*)"', predicate)
        operators = re.findall(r"&&|\|\|", predicate)

        expected_frames = {"camera_init", "/camera_init", "map", "/map"}
        self.assertEqual(set(compared_frames), expected_frames)
        self.assertEqual(len(compared_frames), len(expected_frames))
        self.assertEqual(operators, ["&&", "&&", "&&"])
        self.assertNotIn("||", predicate)

        for frame in sorted(expected_frames):
            self.assertFalse(
                evaluate_rejection_predicate(frame, compared_frames, operators),
                "allowed frame rejected by extracted predicate: {}".format(frame),
            )
        for frame in ("odom", "base_link", ""):
            self.assertTrue(
                evaluate_rejection_predicate(frame, compared_frames, operators),
                "unsupported frame accepted by extracted predicate: {!r}".format(
                    frame
                ),
            )
        self.assertIn("allowed map frames", submit_body)

    def test_readme_documents_default_and_compatible_initialpose_frames(self):
        self.assertIn("定位 RViz 的 Fixed Frame 为 `map`", self.readme)
        self.assertIn("不启用旧的 RViz 人工初始化路径", self.readme)
        self.assertIn("map → odom → aft_mapped", self.readme)

    def test_readme_defines_map_aliases_as_same_coordinate_without_tf_lookup(self):
        # The old API still has same-coordinate aliases, but the current branch
        # must not advertise that API as its dual-TF initialization path.
        self.assertIn("首次定位不使用锚点位置先验", self.readme)
        self.assertIn("不额外发布 `map→aft_mapped`", self.readme)
        submit_body = function_body(
            self.localization_source, "bool InitialPoseRequestBuffer::submit("
        )
        self.assertIn(
            "map aliases are same-coordinate compatibility aliases",
            submit_body,
        )
        self.assertIn("no TF transform occurs", submit_body)

    def test_scan_planner_steps_distinguish_existing_session_and_full_stack(self):
        section = markdown_section(self.readme, "## 7. 输出与状态检查", "## 8. 常见问题")
        self.assertIn("ros2 launch voxel_slam scan_planner_adapter.launch.py", section)
        self.assertIn("不会启动规划器或控制器", section)
        self.assertNotIn("roslaunch ", section)
        self.assertNotIn("catkin_ws", section)

    def test_scan_planner_world_cloud_and_pose_extrinsic_are_independent(self):
        self.assertIn("`/map_scan` 是 `odom` 系当前点云", self.readme)
        self.assertIn("点云坐标系设置与雷达–IMU 外参是两件事", self.readme)
        self.assertIn("[README](../README.md)", self.package_readme)

    def test_scan_planner_mid360_extrinsic_matches_localization_config(self):
        self.assertRegex(
            self.localization_config,
            r"(?m)^\s+extrinsic_tran:\s*\[-0\.011,\s*-0\.02329,\s*0\.04412\]\s*$",
        )
        self.assertRegex(
            self.localization_config,
            r"(?ms)^\s+extrinsic_rota:\s*\[1,\s*0,\s*0,\s*"
            r"0,\s*1,\s*0,\s*0,\s*0,\s*1\]\s*$",
        )
        self.assertIn("真实标定或 URDF", self.readme)
        self.assertIn("General.extrinsic_tran", self.readme)
        self.assertIn("General.extrinsic_rota", self.readme)

    def test_scan_planner_is_external_and_documents_safe_launch_integration(self):
        self.assertIn("不会启动规划器或控制器", self.readme)
        self.assertIn("不是实车验收", self.readme)
        self.assertIn("外部规划器接入", self.readme)
        self.assertIn("[README](../README.md)", self.package_readme)
        self.assertIn("ROS 2", self.package_readme)

    def test_dependencies_include_boost_and_rosdep_guidance(self):
        self.assertRegex(
            self.readme,
            r"Boost[^\n]{0,80}(filesystem|FileSystem)[^\n]{0,80}system",
        )
        self.assertIn("rosdep install", self.readme)


if __name__ == "__main__":
    unittest.main()
