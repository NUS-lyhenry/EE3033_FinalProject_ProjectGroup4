"""Uncertainty-weighted live map estimates, separate from mission confirmation."""
from collections import deque
import math
import numpy as np


class LiveTargetEstimate:
    def __init__(self, seconds=8., retain_seconds=30., max_samples=30):
        self.seconds, self.retain_seconds = seconds, retain_seconds
        self.samples = deque(maxlen=max_samples)
        self.label = None

    def reset(self):
        self.samples.clear()
        self.label = None

    def update(self, detections, now, label):
        if label != self.label:
            self.reset()
            self.label = label
        choices = []
        for d in detections:
            try:
                values = [float(d[k]) for k in ('stamp', 'x', 'y', 'sigma', 'confidence')]
                stamp, x, y, sigma, confidence = values
                if (d.get('label') == label and all(math.isfinite(v) for v in values)
                        and sigma > 0 and math.isfinite(sigma*sigma)
                        and 0 <= now-stamp <= 3. and 0 <= confidence <= 1):
                    choices.append((confidence, stamp, x, y, sigma))
            except (KeyError, TypeError, ValueError):
                continue
        if not choices:
            return
        _, stamp, x, y, sigma = max(choices)
        if self.samples and stamp <= self.samples[-1][0]:
            return  # Polling/repeated messages do not create new observations.
        while self.samples and stamp-self.samples[0][0] > self.seconds:
            self.samples.popleft()
        self.samples.append((stamp, x, y, sigma))

    def snapshot(self, now):
        if not self.samples:
            return None
        age = now-self.samples[-1][0]
        if not 0 <= age <= self.retain_seconds:
            return None
        a = np.asarray(self.samples)
        weights = (a[:, 3].min()/a[:, 3])**2 * np.exp((a[:, 0]-a[-1, 0])/3.)
        weights /= weights.sum()
        center = weights @ a[:, 1:3]
        residual = a[:, 1:3]-center
        # Camera frames share mount/AMCL errors. Do not claim 1/sqrt(N) accuracy.
        covariance = (residual.T*weights) @ residual + np.eye(2)*float(weights @ (a[:, 3]**2))
        sigma = math.sqrt(float(np.linalg.eigvalsh(covariance).max()))
        return dict(x=float(center[0]), y=float(center[1]), sigma=sigma,
                    covariance=covariance.tolist(), label=self.label, samples=len(a),
                    stamp=float(a[-1, 0]), age=age, stale=age > 2., confirmed=False,
                    method='recent inverse-variance weighted estimate')
