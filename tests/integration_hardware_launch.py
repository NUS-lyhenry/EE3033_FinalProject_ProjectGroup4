"""Exercise real launch argument scoping with driver stand-ins; no hardware/ROS nodes.

The base stand-in reads its YAML exactly as the installed lab launch does. This
reproduces mission's parent ``config`` overriding the lab default without opening
a serial port. Run in the sourced ROS environment.
"""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, SetLaunchConfiguration


HARDWARE = Path(__file__).resolve().parents[1]/'src/ee3033_project/launch/hardware.launch.py'
spec = importlib.util.spec_from_file_location('project_hardware_launch', HARDWARE)
hardware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hardware)


class HardwareConfiguration(unittest.TestCase):
    def launch_driver(self):
        with tempfile.TemporaryDirectory(prefix='group4_launch_scope_') as tmp:
            root = Path(tmp)
            lab = root/'balance_robot_base'
            (lab/'launch').mkdir(parents=True)
            (lab/'config').mkdir()
            base_config = lab/'config/base_params.yaml'
            base_config.write_text('balance_robot_node:\n  ros__parameters:\n'
                                   '    laser_frame_id: laser\n    control_loop_freq: 50.0\n')
            project_config = root/'project.yaml'
            project_config.write_text('project_mission:\n  ros__parameters:\n    target_class: bottle\n')
            result = root/'driver.json'
            (lab/'launch/balance_robot_base.launch.py').write_text('''
import json
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def inspect(context):
    config = LaunchConfiguration('config').perform(context)
    params = yaml.safe_load(Path(config).read_text()).get('balance_robot_node', {}).get('ros__parameters', {})
    Path(RESULT).write_text(json.dumps(dict(config=config, laser_frame=params.get('laser_frame_id', 'laser_link'),
                                          control_loop_freq=params.get('control_loop_freq'))))
    return []

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=BASE_CONFIG),
                              OpaqueFunction(function=inspect)])
'''.replace('RESULT', repr(str(result))).replace('BASE_CONFIG', repr(str(base_config))))
            context = LaunchContext()
            context.launch_configurations.update(base='true', lidar='false', camera='false')
            with patch.object(hardware, 'get_package_share_directory', return_value=str(lab)):
                actions = hardware.setup(context)
            after = {}

            def inspect_parent(context):
                after['config'] = context.launch_configurations['config']
                return []

            service = LaunchService(argv=[])
            service.include_launch_description(LaunchDescription([
                SetLaunchConfiguration('config', str(project_config)), *actions,
                OpaqueFunction(function=inspect_parent)]))
            self.assertEqual(service.run(), 0)
            return json.loads(result.read_text()), after, str(base_config), str(project_config)

    def test_mission_config_does_not_replace_installed_base_settings(self):
        driver, _, base_config, _ = self.launch_driver()
        self.assertEqual(driver, dict(config=base_config, laser_frame='laser', control_loop_freq=50.0))

    def test_base_argument_does_not_replace_downstream_project_config(self):
        _, after, _, project_config = self.launch_driver()
        self.assertEqual(after['config'], project_config)


if __name__ == '__main__':
    unittest.main()
