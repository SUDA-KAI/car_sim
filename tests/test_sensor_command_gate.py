from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from sensor_command_gate import input_health

class SensorGateTest(unittest.TestCase):
    def samples(self):return {key:(10.,100.) for key in ['lio','visual','fused',*[f'sonar_{i}' for i in range(1,5)]]}
    def test_fresh_prediction_cannot_hide_loss_of_both_position_sources(self):
        samples=self.samples();samples['lio']=(1.,100.);samples['visual']=(1.,100.)
        health=input_health(samples,10.1,100.1)
        self.assertTrue(health['fused']);self.assertFalse(health['position'])
        self.assertFalse(all(health.values()))
    def test_one_healthy_position_source_and_all_sonars_required(self):
        samples=self.samples();samples['visual']=(1.,100.)
        self.assertTrue(all(input_health(samples,10.1,100.1).values()))
        samples.pop('sonar_3')
        self.assertFalse(input_health(samples,10.1,100.1)['sonar'])
    def test_wall_watchdog_and_future_measurements(self):
        samples=self.samples()
        self.assertFalse(any(input_health(samples,10.1,105.).values()))
        samples['fused']=(11.,100.)
        self.assertFalse(input_health(samples,10.1,100.1)['fused'])
if __name__=='__main__':unittest.main()
