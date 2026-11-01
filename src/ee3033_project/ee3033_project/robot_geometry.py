"""One explicit robot-size profile for navigation, safety and simulation.

The wheel-inclusive body is 11.5 cm long (forward x) and 18 cm wide (lateral y).
This two-wheel balancing chassis is enclosed by a circle.
That circle is orientation independent and conservative; it is not a rectangular
footprint or a model of the balancing chassis' dynamics. The legacy profile
retains the previous 14 cm body radius and estimated 20 cm width for stress tests.

Select exactly once at process startup with EE3033_ROBOT_GEOMETRY. No simulator,
map, platform, or other environment variable implicitly changes the selection.
"""
import math
import os


GEOMETRY_PROFILE = os.environ.get('EE3033_ROBOT_GEOMETRY', 'measured')
if GEOMETRY_PROFILE not in ('measured', 'legacy'):
    raise ValueError('EE3033_ROBOT_GEOMETRY must be measured or legacy')

FOOTPRINT_PADDING = 0.01
if GEOMETRY_PROFILE == 'measured':
    ROBOT_LENGTH = 0.115
    ROBOT_WIDTH = 0.18
    # Round outward to a micrometre, so even numerical rounding cannot put a
    # measured corner outside the body circle. The exact diagonal is retained
    # in metadata for reproducibility, rather than being recomputed elsewhere.
    EXACT_CIRCUMRADIUS = math.hypot(ROBOT_LENGTH/2., ROBOT_WIDTH/2.)
    BODY_RADIUS = math.ceil(EXACT_CIRCUMRADIUS*1_000_000.)/1_000_000.
    DIMENSION_SOURCE = 'wheel_inclusive_user_measurement'
    MODEL = 'conservative_circumscribed_circle'
else:
    ROBOT_LENGTH = None  # The old model never established a measured length.
    ROBOT_WIDTH = 0.20
    EXACT_CIRCUMRADIUS = None
    BODY_RADIUS = 0.14
    DIMENSION_SOURCE = 'legacy_radius_and_estimated_width'
    MODEL = 'legacy_conservative_circle'

PHYSICAL_RADIUS = BODY_RADIUS
# Nav2's requested planning radius; final motion checks retain measured geometry.
NAVIGATION_RADIUS = 0.10 if GEOMETRY_PROFILE == 'measured' else BODY_RADIUS
FOOTPRINT_RADIUS = round(BODY_RADIUS+FOOTPRINT_PADDING, 6)
# Routing budget for SLAM pose error and controller tracking at wall tips.
# Separate from measured body geometry and the final motion collision envelope.
ROUTE_TRACKING_RESERVE = .03
ROUTE_RADIUS = round(FOOTPRINT_RADIUS+ROUTE_TRACKING_RESERVE,6)
DEFAULT_STOP_DISTANCE = 0.12 if GEOMETRY_PROFILE == 'measured' else round(FOOTPRINT_RADIUS+0.07, 6)
ROTATION_CLEARANCES = ({'cautious':0.16,'balanced':0.15,'bold':0.14}
    if GEOMETRY_PROFILE=='legacy' else
    {name:round(FOOTPRINT_RADIUS+reserve,6) for name,reserve in
        [('cautious',.02),('balanced',.01),('bold',0.)]})
GEOMETRY_METADATA = dict(profile=GEOMETRY_PROFILE, model=MODEL,
    length=ROBOT_LENGTH, width=ROBOT_WIDTH, body_radius=BODY_RADIUS,
    padding=FOOTPRINT_PADDING, footprint_radius=FOOTPRINT_RADIUS,
    navigation_radius=NAVIGATION_RADIUS, route_radius=ROUTE_RADIUS,
    route_tracking_reserve=ROUTE_TRACKING_RESERVE,
    exact_circumradius=EXACT_CIRCUMRADIUS, dimension_source=DIMENSION_SOURCE,
    front_stop_distance=DEFAULT_STOP_DISTANCE,
    rotation_clearances=dict(ROTATION_CLEARANCES),
    selection_environment='EE3033_ROBOT_GEOMETRY')


def geometry_metadata():
    """Return a fresh JSON-compatible record of the selected immutable sizes."""
    return {**GEOMETRY_METADATA,'rotation_clearances':dict(ROTATION_CLEARANCES)}
