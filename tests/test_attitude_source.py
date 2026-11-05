import importlib.util
import hashlib
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location('attitude_source', Path(__file__).parents[1]/'scripts/build_attitude_source.py')
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


def source_fixture():
    return ('void constructor() {\n'+driver.CPP_PUBLISHER+'\n}\n'
        'bool BalanceRobot::get_sensor_data()\n{\n    bool has_new_data = false;\n'
        '    if (valid_sensor_checksum) {\n'+driver.SENSOR_ANCHOR+'\n    }\n'
        '    return has_new_data;\n}\nvoid BalanceRobot::control_loop()\n{\n'
        '    unchanged_motor_control();\n'+driver.READ_CALL+'\n    unchanged_odometry();\n'
        +driver.PUBLISH_CALL+'\n}\n'), driver.HEADER_PUBLISHER+'\n'


@pytest.fixture(autouse=True)
def fixture_parser_digest(monkeypatch):
    # This synthetic parser substitutes only the known-baseline hash; production
    # carries the digest of the complete inspected MCU parser, including CRC.
    cpp, _ = source_fixture()
    monkeypatch.setattr(driver, 'SENSOR_PARSER_SHA256',
                        hashlib.sha256(driver.sensor_parser(cpp).encode()).hexdigest())


def test_patch_gates_new_topic_on_sensor_packet_and_keeps_original_control_and_legacy():
    cpp, header = source_fixture()
    patched, _ = driver.patch_sources(cpp, header)
    assert 'const bool fresh_attitude_packet = get_sensor_data();' in patched
    assert 'if (publish_body_attitude_ && fresh_attitude_packet)' in patched
    assert 'declare_parameter<bool>("publish_body_attitude", false)' in patched
    for section in ('unchanged_motor_control();', 'unchanged_odometry();', driver.PUBLISH_CALL, driver.SENSOR_ANCHOR):
        assert section in patched
    assert 'attitude.vector.z = 0.0' in patched
    assert 'attitude.header.stamp = attitude_packet_received;' in patched


def test_unknown_or_repatched_source_fails_before_output():
    cpp, header = source_fixture()
    for invalid in (cpp.replace('has_new_data = true;', 'has_new_data = true; has_new_data = true;'),
                    cpp.replace('    get_sensor_data();', '    custom_read();')):
        with pytest.raises(ValueError):
            driver.patch_sources(invalid, header)
    patched = driver.patch_sources(cpp, header)
    with pytest.raises(ValueError, match='already'):
        driver.patch_sources(*patched)
    with pytest.raises(ValueError, match='CRC-validated'):
        driver.patch_sources(cpp.replace('valid_sensor_checksum', 'true'), header)


def test_prepare_preserves_original_and_enables_only_explicit_overlay(tmp_path):
    source, output = tmp_path/'base', tmp_path/'overlay'
    cpp, header = source_fixture()
    for name, data in [('src/balance_robot.cpp', cpp),
                       ('include/balance_robot_base/balance_robot.h', header),
                       ('config/base_params.yaml', 'balance_robot_node:\n  ros__parameters:\n    max_accel_x: 0.8\n')]:
        path = source/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(data)
    manifest = driver.prepare(source, output, enable_source=True)
    assert not manifest['built']
    assert (source/'src/balance_robot.cpp').read_text() == cpp
    assert (output/'original/src/balance_robot.cpp').read_text() == cpp
    assert 'publish_body_attitude: true' in (output/'src/balance_robot_base/config/base_params.yaml').read_text()
    assert 'publish_body_attitude' not in (source/'config/base_params.yaml').read_text()
    with pytest.raises(ValueError, match='new isolated'):
        driver.prepare(source, output)
