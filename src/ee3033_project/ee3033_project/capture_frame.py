"""Bounded nonblocking wait for a frame's capture-time transform."""


class CaptureFrameGate:
    def __init__(self):
        self.pending = None

    def select(self, latest, ready):
        # Hold at most one frame for one inference period. The second attempt
        # always proceeds so missing TF still produces detection diagnostics.
        if self.pending is not None:
            frame, self.pending = self.pending, None
            return frame
        if latest is not None and not ready(latest):
            self.pending = latest
            return None
        return latest
