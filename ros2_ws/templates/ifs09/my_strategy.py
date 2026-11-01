"""MY cone-detection strategy: start here if your idea is a new algorithm.

A strategy is what the perception node runs on every scan. It has the same
interface as the real pipeline's (cone_detection_strategy.py), so an idea
written here can go back to IFS09-DV-PIPELINE as a new strategy next to "base".

  configure()                  one-off set-up (e.g. Numba warm-up)
  detect_cones(point_cloud)    (N, 3) x, y, z in the sensor frame -> DetectionResult
                               with a ConeObservation(x, y, height_m) per cone

For now it's the real algorithm, unchanged (it inherits BaseConeDetection).
Override detect_cones() to replace the whole thing, or only the parts you
need. Your config is self._config (my_cone_detection/config.py).
"""
from __future__ import annotations

import numpy as np

from my_cone_detection.strategies.base_cone_detection import BaseConeDetection
from my_cone_detection.strategies.cone_detection_strategy import DetectionResult


class MyConeDetection(BaseConeDetection):

    def detect_cones(self, point_cloud: np.ndarray, **kwargs) -> DetectionResult:
        # Your idea here. For now: the real algorithm.
        return super().detect_cones(point_cloud, **kwargs)
