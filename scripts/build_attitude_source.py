#!/usr/bin/env python3
"""Prepare an isolated opt-in fresh-attitude base-driver overlay; never launch it.

Source anchors are from the inspected balance_robot_base driver. Unknown source
layouts fail before any output is written. The original driver is never edited.
Run only on a stopped robot when explicitly authorized to build/deploy hardware
software. Preparation, build, launch, and live verification are separate steps.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


CPP_PUBLISHER = '    euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("robot_euler", 20);'
HEADER_PUBLISHER = '    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher_;'
READ_CALL = '    // Read and parse data\n    get_sensor_data();'
PUBLISH_CALL = '    publish_voltage();\n    publish_euler();'
SENSOR_ANCHOR = ('                    stm32_pitch_ = (float)imu_trans(receive_data_.rx_buffer[24], receive_data_.rx_buffer[25]) / 100.0f;\n'
                '                    stm32_roll_  = (float)imu_trans(receive_data_.rx_buffer[26], receive_data_.rx_buffer[27]) / 100.0f;\n'
                '                    stm32_yaw_   = (float)imu_trans(receive_data_.rx_buffer[28], receive_data_.rx_buffer[29]) / 100.0f;\n\n'
                '                    has_new_data = true;')
SENSOR_PARSER_SHA256 = '4186070e2e8a925253b2099edc29ad95c2c9b58b477e997dc3b7c19e6eb19102'


def sensor_parser(cpp):
    start = cpp.index('bool BalanceRobot::get_sensor_data()')
    return cpp[start:cpp.index('\n}\n', start)+3]


def replace_once(text, anchor, replacement):
    if text.count(anchor) != 1:
        raise ValueError('Driver source anchor missing or ambiguous: '+anchor.splitlines()[0])
    return text.replace(anchor, replacement, 1)


def patch_sources(cpp, header):
    if 'publish_body_attitude' in cpp or 'body_attitude_publisher_' in header:
        raise ValueError('Driver already contains a body-attitude patch; use a fresh baseline copy')
    for anchor in (SENSOR_ANCHOR, '    bool has_new_data = false;', '    return has_new_data;',
                   'bool BalanceRobot::get_sensor_data()', 'void BalanceRobot::control_loop()'):
        if cpp.count(anchor) != 1:
            raise ValueError('Unsupported sensor-packet freshness implementation')
    if cpp.count('has_new_data = true;') != 1:
        raise ValueError('Freshness flag has an unexpected writer')
    # Verify the entire inspected parser, not just the flag setter: moving it
    # outside the CRC branch would otherwise retain all superficial anchors.
    if hashlib.sha256(sensor_parser(cpp).encode()).hexdigest() != SENSOR_PARSER_SHA256:
        raise ValueError('Sensor parser differs from the CRC-validated inspected baseline')
    cpp = replace_once(cpp, CPP_PUBLISHER, CPP_PUBLISHER+'\n'
        '    publish_body_attitude_ = this->declare_parameter<bool>("publish_body_attitude", false);\n'
        '    if (publish_body_attitude_) {\n'
        '        body_attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/project/body_attitude", 20);\n'
        '    }')
    header = replace_once(header, HEADER_PUBLISHER, HEADER_PUBLISHER+'\n'
        '    bool publish_body_attitude_ = false;\n'
        '    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_attitude_publisher_;')
    cpp = replace_once(cpp, READ_CALL, '    // Read and parse data\n'
        '    const bool fresh_attitude_packet = get_sensor_data();\n'
        '    const auto attitude_packet_received = this->get_clock()->now();')
    cpp = replace_once(cpp, PUBLISH_CALL, PUBLISH_CALL+'\n'
        '    // Fresh CRC-validated sensor packets only; legacy Euler is unchanged.\n'
        '    // Timestamp is host receipt time, not an MCU exposure timestamp.\n'
        '    if (publish_body_attitude_ && fresh_attitude_packet) {\n'
        '        geometry_msgs::msg::Vector3Stamped attitude;\n'
        '        attitude.header.stamp = attitude_packet_received;\n'
        '        attitude.header.frame_id = robot_frame_id_;\n'
        '        attitude.vector.x = stm32_roll_ * M_PI / 180.0;\n'
        '        attitude.vector.y = stm32_pitch_ * M_PI / 180.0;\n'
        '        attitude.vector.z = 0.0; // Map yaw stays in localization TF.\n'
        '        body_attitude_publisher_->publish(attitude);\n'
        '    }')
    return cpp, header


def digest(data):
    return hashlib.sha256(data).hexdigest()


def prepare(source, output, enable_source=False):
    source, output = Path(source).resolve(), Path(output).resolve()
    cpp_path = source/'src/balance_robot.cpp'
    header_path = source/'include/balance_robot_base/balance_robot.h'
    originals = {str(p.relative_to(source)): p.read_bytes() for p in (cpp_path, header_path)}
    patched = patch_sources(originals[str(cpp_path.relative_to(source))].decode(),
                            originals[str(header_path.relative_to(source))].decode())
    config_path = source/'config/base_params.yaml'
    config_text = None
    if enable_source:
        import yaml
        config = yaml.safe_load(config_path.read_text())
        entries = [value['ros__parameters'] for value in config.values()
                   if isinstance(value, dict) and isinstance(value.get('ros__parameters'), dict)]
        if len(entries) != 1:
            raise ValueError('Expected one base-driver ROS parameter block')
        entries[0]['publish_body_attitude'] = True
        config_text = yaml.safe_dump(config, sort_keys=False)
    if output.exists() or source == output or source in output.parents:
        raise ValueError('Use a new isolated overlay directory outside the source package')
    output.mkdir(parents=True)
    target = output/'src/balance_robot_base'
    shutil.copytree(source, target)
    backup = output/'original'
    backup.mkdir()
    manifest = dict(source=str(source), source_enabled=enable_source, built=False,
        topic='/project/body_attitude', timestamp_basis='host receipt after validated sensor packet',
        files={})
    for (name, original), replacement in zip(originals.items(), patched):
        saved = backup/name
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(original)
        (target/name).write_text(replacement)
        manifest['files'][name] = dict(original_sha256=digest(original),
                                       patched_sha256=digest(replacement.encode()))
    if config_text is not None:
        saved = backup/'config/base_params.yaml'
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(config_path.read_bytes())
        (target/'config/base_params.yaml').write_text(config_text)
        manifest['files']['config/base_params.yaml'] = dict(original_sha256=digest(saved.read_bytes()),
            patched_sha256=digest(config_text.encode()))
    (output/'attitude-source-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-package', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--enable-source', action='store_true', help='Enable only the copied overlay config')
    parser.add_argument('--build', action='store_true', help='Explicit opt-in colcon build; never starts the driver')
    args = parser.parse_args()
    manifest = prepare(args.source_package, args.output, args.enable_source)
    output = Path(args.output).resolve()
    if args.build:
        subprocess.run(['colcon', 'build', '--base-paths', str(output/'src'),
                        '--packages-select', 'balance_robot_base', '--build-base', str(output/'build'),
                        '--install-base', str(output/'install')], cwd=output, check=True)
        manifest['built'] = True
        (output/'attitude-source-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(overlay=str(output), manifest=str(output/'attitude-source-manifest.json'),
        config=str(output/'src/balance_robot_base/config/base_params.yaml'),
        built=manifest['built'], source_enabled=args.enable_source, deployed=False), indent=2))


if __name__ == '__main__':
    main()
