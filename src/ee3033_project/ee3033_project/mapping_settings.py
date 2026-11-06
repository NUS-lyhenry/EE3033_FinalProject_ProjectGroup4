"""Validated settings loaded when a new SLAM mapping process starts."""
import math

DEFAULT_SLAM_SETTINGS=dict(occupancy_threshold=.15,minimum_pass_through=2)
# The operator/API label is descriptive; Humble's actual ROS parameter is shorter.
SLAM_NATIVE_NAMES=dict(occupancy_threshold='occupancy_threshold',minimum_pass_through='min_pass_through')


def validate_slam_settings(values):
    if not isinstance(values,dict) or set(values)!=set(DEFAULT_SLAM_SETTINGS):
        raise ValueError('Provide occupancy_threshold and minimum_pass_through')
    threshold=values['occupancy_threshold'];passes=values['minimum_pass_through']
    if (isinstance(threshold,bool) or not isinstance(threshold,(int,float))
            or not math.isfinite(threshold) or not .01<=threshold<=.99):
        raise ValueError('Occupancy threshold must be a finite number between 0.01 and 0.99')
    if type(passes) is not int or not 0<=passes<=100:
        raise ValueError('Minimum pass-through must be an integer between 0 and 100')
    return dict(occupancy_threshold=float(threshold),minimum_pass_through=passes)


def same_slam_settings(selected,live):
    return bool(live is not None and
        math.isclose(selected['occupancy_threshold'],live['occupancy_threshold'],abs_tol=1e-8,rel_tol=0.) and
        selected['minimum_pass_through']==live['minimum_pass_through'])
