from ee3033_project.capture_frame import CaptureFrameGate


def test_missing_capture_transform_defers_original_frame_once():
    gate = CaptureFrameGate()
    first, newer = object(), object()
    assert gate.select(first, lambda _: False) is None
    assert gate.select(newer, lambda _: False) is first
    assert gate.select(newer, lambda _: True) is newer


def test_ready_frame_is_processed_immediately():
    frame = object()
    gate = CaptureFrameGate()
    assert gate.select(frame, lambda _: True) is frame
    assert gate.select(None, lambda _: False) is None
