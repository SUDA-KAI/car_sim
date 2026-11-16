"""Fresh obstacle readings must gate fixed-map drive commands."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fixed_map_command_gate import input_health


class FixedMapGateTest(unittest.TestCase):
    def test_all_inputs_fresh_then_one_sonar_stale(self):
        samples = {'scan': (10., 20.), 'odom': (10., 20.)}
        samples.update({f'sonar_{i}': (10., 20.) for i in range(1, 5)})
        self.assertEqual(input_health(samples, 10.1, 20.1),
                         dict(scan=True, odom=True, sonar=True))
        samples['sonar_4'] = (8., 20.)
        self.assertFalse(input_health(samples, 10.1, 20.1)['sonar'])

    def test_paused_simulation_stops_on_wall_age(self):
        samples = {'scan': (10., 20.), 'odom': (10., 20.)}
        samples.update({f'sonar_{i}': (10., 20.) for i in range(1, 5)})
        self.assertFalse(all(input_health(samples, 10., 23.).values()))


if __name__ == '__main__':
    unittest.main()
