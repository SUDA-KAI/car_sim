import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from proximity_alarm import Proximity


class ProximityTest(unittest.TestCase):
    def test_threshold_float32_hysteresis_and_channels(self):
        p=Proximity()
        for i in range(1,5):
            p.update(i,.3,.03,.55,1.,10.)
        self.assertFalse(p.snapshot(1.,10.)['unknown'])
        p.update(3,.10000000149,.03,.55,1.,10.)
        self.assertTrue(p.snapshot(1.,10.)['alarm'])
        p.update(3,.11,.03,.55,1.,10.)
        self.assertTrue(p.snapshot(1.,10.)['alarm'])
        p.update(3,.121,.03,.55,1.,10.)
        self.assertFalse(p.snapshot(1.,10.)['alarm'])

    def test_missing_invalid_no_echo_too_close_and_retained_warning(self):
        p=Proximity()
        self.assertTrue(p.snapshot(0.,0.)['unknown'])
        p.update(1,-math.inf,.03,.55,1.,10.)
        p.update(2,math.inf,.03,.55,1.,10.)
        p.update(3,math.nan,.03,.55,1.,10.)
        p.update(4,.02,.03,.55,1.,10.)
        result=p.snapshot(1.,10.)
        self.assertEqual([c['kind'] for c in result['channels']],['too_close','no_echo','invalid','too_close'])
        self.assertTrue(result['alarm']);self.assertTrue(result['unknown'])
        p.update(1,math.nan,.03,.55,2.,11.)
        self.assertTrue(p.snapshot(2.,11.)['channels'][0]['near'])
        self.assertEqual(p.snapshot(4.1,11.)['channels'][0]['kind'],'stale')
        self.assertTrue(p.snapshot(4.,11.)['alarm'])

    def test_stale_message_and_clock_reset(self):
        p=Proximity()
        p.update(1,.3,.03,.55,1.,10.)
        self.assertEqual(p.snapshot(1.,14.)['channels'][0]['kind'],'stale')
        self.assertEqual(p.snapshot(1.,0.)['channels'][0]['kind'],'stale')
        self.assertEqual(p.snapshot(4.,10.)['channels'][0]['kind'],'stale')
        p.update(1,.3,.6,.5,4.,11.)
        self.assertEqual(p.snapshot(4.,11.)['channels'][0]['kind'],'invalid')

if __name__=='__main__':unittest.main()
