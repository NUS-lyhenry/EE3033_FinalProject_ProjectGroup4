"""Slow camera rendering cannot hold commands/physics or publish mixed worlds."""
from collections import deque
import json
import math
import threading
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
from test_simulation_camera import robot_stub
from simulation_camera import default_camera_settings
from simulation_scenarios import default_scenario, validate_scenario


def paused_renderer(monkeypatch, robot):
    entered, release = threading.Event(), threading.Event()
    captured = []
    def render(*args):
        captured.append(args)
        entered.set()
        assert release.wait(2.), 'Test renderer was not released'
        return np.zeros((2, 2, 3), np.uint8)
    monkeypatch.setitem(robot.publish_camera.__func__.__globals__, 'render_camera', render)
    result = []
    thread = threading.Thread(target=lambda: result.append(robot.camera_tick()))
    thread.start()
    assert entered.wait(1.)
    return thread, release, captured, result


def test_command_and_physics_progress_during_slow_render_and_exposure_stamp_is_preserved(monkeypatch):
    robot, _, images, infos = robot_stub()
    monkeypatch.setitem(robot.publish_camera.__func__.__globals__, 'time', time)
    robot.accept_commands = True
    robot.command_queue = deque()
    before = list(robot.pose)
    clock = [NS(sec=1, nanosec=0)]
    robot.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: clock[0]))
    thread, release, captured, result = paused_renderer(monkeypatch, robot)
    finished = threading.Event()
    def fast_callbacks():
        robot.on_command(NS(linear=NS(x=.12), angular=NS(z=.4)))
        robot.physics_tick = lambda: setattr(robot, 'pose', [1., 2., 3.])
        robot.tick()
        finished.set()
    worker = threading.Thread(target=fast_callbacks)
    worker.start()
    try:
        assert finished.wait(.3), 'Camera renderer blocked command/physics callbacks'
        clock[0] = NS(sec=2, nanosec=0)
    finally:
        release.set()
        worker.join(1.)
        thread.join(1.)
    assert result == [True]
    assert captured[0][0] == before
    assert robot.pose == [1., 2., 3.]
    assert images[-1].header.stamp.sec == 1
    assert images[-1].header is infos[-1].header


@pytest.mark.parametrize('change', ['camera', 'scene', 'phase'])
def test_inflight_camera_frame_is_discarded_after_configuration_change(monkeypatch, change):
    robot, _, images, infos = robot_stub()
    scope = robot.publish_camera.__func__.__globals__
    scope.update(validate_scenario=validate_scenario, json=json, time=time)
    robot.scene_status_after = -math.inf
    robot.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000,
                                                   to_msg=lambda: NS(sec=1, nanosec=0)))
    thread, release, _, result = paused_renderer(monkeypatch, robot)
    try:
        if change == 'camera':
            robot.apply_camera(dict(default_camera_settings(), camera_height=.31))
        elif change == 'scene':
            scene = default_scenario()
            scene['target'] = [-.7, .7, 0.]
            robot.apply_scenario(scene)
        else:
            robot.on_status(NS(data=json.dumps(dict(stamp=1., phase='mapping'))))
    finally:
        release.set()
        thread.join(1.)
    assert result == [False]
    assert images == [] and infos == []
