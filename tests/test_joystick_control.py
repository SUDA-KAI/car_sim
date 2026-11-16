import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from joystick_control import JoystickState,Settings

class JoystickTest(unittest.TestCase):
    def sample(self,state,speed=0.,steering=0.,enabled=False,t=0.):
        state.receive([0.,speed,1.,steering],[0,0,0,0,int(enabled)],t)
    def test_startup_requires_neutral_and_released_enable(self):
        s=JoystickState();self.sample(s,1,1,True)
        self.assertEqual(s.output(0),(0,0))
        self.sample(s,0,0,False,t=.1);self.sample(s,1,1,True,t=.2)
        v,w=s.output(.2);self.assertAlmostEqual(v,.5)
        self.assertAlmostEqual(w,.5*math.tan(s.cfg.max_steering_angle)/.79)
    def test_right_stick_steers_and_left_horizontal_is_ignored(self):
        s=JoystickState();self.sample(s)
        s.receive([1.,1.,1.,0.],[0,0,0,0,1],.1)
        self.assertEqual(s.output(.1),(.5,0.))
        s.receive([0.,1.,1.,-1.],[0,0,0,0,1],.2)
        self.assertLess(s.output(.2)[1],0.)

    def test_reverse_turn_and_stationary_steering(self):
        s=JoystickState();self.sample(s)
        self.sample(s,-1,1,True,t=.1);v,w=s.output(.1)
        self.assertAlmostEqual(v,-.2);self.assertLess(w,0)
        self.sample(s,0,1,True,t=.2);self.assertEqual(s.output(.2),(0,0))
    def test_release_timeout_and_reconnect_stop(self):
        s=JoystickState();self.sample(s);self.sample(s,1,0,True,t=.1)
        self.assertEqual(s.output(.46),(0,0))
        self.sample(s,1,0,True,t=.5);self.assertEqual(s.output(.5),(0,0))
        self.sample(s,t=.6);self.sample(s,1,0,True,t=.7);self.assertGreater(s.output(.7)[0],0)
        self.sample(s,1,0,False,t=.8);self.assertEqual(s.output(.8),(0,0))
    def test_invalid_packet_disconnect_and_deadzone(self):
        s=JoystickState();self.sample(s);self.sample(s,1,0,True,t=.1)
        s.receive([],[],.2);self.assertEqual(s.output(.2),(0,0));self.assertFalse(s.armed)
        self.sample(s,t=.3);self.sample(s,.05,.1,True,t=.4);self.assertEqual(s.output(.4),(0,0))
        s.receive([float('nan'),1],[0,0,0,0,1],.5);self.assertFalse(s.armed)
    def test_config_sign_and_unsafe_parameters(self):
        s=JoystickState(Settings(speed_sign=-1.));self.sample(s);self.sample(s,-1,0,True,t=.1)
        self.assertAlmostEqual(s.output(.1)[0],.5)
        with self.assertRaises(ValueError):JoystickState(Settings(wheelbase=0))
        with self.assertRaises(ValueError):JoystickState(Settings(enable_button=-1))
        with self.assertRaises(ValueError):JoystickState(Settings(deadzone=1))
if __name__=='__main__':unittest.main()
