"""One runtime source for the commissioned chassis geometry in simulation."""
from pathlib import Path
import math
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/chassis_sim.yaml'


def load_chassis(path=DEFAULT_CONFIG):
    values = yaml.safe_load(Path(path).read_text())
    required = ('wheelbase_m', 'track_m', 'wheel_radius_m', 'max_forward_mps',
                'max_reverse_mps', 'max_center_steer_deg',
                'firmware_center_steer_deg', 'command_timeout_sec',
                'steering_settle_rad')
    if not isinstance(values, dict) or any(key not in values for key in required):
        raise ValueError('Incomplete simulation chassis configuration')
    values = {key: float(values[key]) for key in required}
    if not all(math.isfinite(value) and value > 0 for value in values.values()):
        raise ValueError('Chassis values must be finite and positive')
    if (values['max_center_steer_deg'] > values['firmware_center_steer_deg']
            or values['firmware_center_steer_deg'] >= 90):
        raise ValueError('Invalid centre steering limit')
    return values


def inner_wheel_limit(chassis):
    """Physical joint allowance for the inner tire at maximum centre steer."""
    angle = math.radians(chassis['max_center_steer_deg'])
    radius = chassis['wheelbase_m'] / math.tan(angle)
    if radius <= chassis['track_m'] / 2:
        raise ValueError('Steering radius crosses the inner kingpin')
    return math.atan(chassis['wheelbase_m'] / (radius - chassis['track_m'] / 2))
