"""Map geometry, import isolation, and the local HTTP control boundary."""
import base64
from http.server import ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from gz_sim.web_console.catalog import Catalog, ROOT
from gz_sim.web_console.server import make_handler, json_safe
from gz_sim.web_console.runtime import Runtime
from gz_sim.web_console.ros_bridge import RosBridge


class BridgeBoundaryTests(unittest.TestCase):
    def preview_bridge(self):
        from concurrent.futures import Future
        from types import SimpleNamespace as NS
        bridge = RosBridge.__new__(RosBridge)
        bridge.lock = threading.RLock()
        bridge.generation = 1
        bridge.preview_sequence = 0
        bridge.preview_handle = None
        bridge.reset_data()
        bridge.log = lambda *args: None
        bridge.pose_stamped = lambda p: p
        bridge.plan_actions = (NS(Goal=NS), NS(Goal=NS))
        response = Future()
        client = NS(server_is_ready=lambda: True, send_goal_async=lambda goal: response)
        bridge.plan_clients = (client, client)
        result = Future()
        canceled = []
        handle = NS(accepted=True, get_result_async=lambda: result, cancel_goal_async=lambda: canceled.append(True))
        return bridge, response, result, handle, canceled

    def test_path_preview_never_starts_vehicle_on_failure(self):
        from types import SimpleNamespace as NS
        bridge, response, result, handle, canceled = self.preview_bridge()
        bridge._preview([dict(x=5.01,y=-.03,yaw=0)])
        response.set_result(handle)
        result.set_result(NS(status=6,result=NS(path=NS(header=NS(frame_id='map'),poses=[]))))
        self.assertEqual(bridge.data['mission'],'idle')
        self.assertEqual(bridge.data['preview']['state'],'failed')
        self.assertEqual(bridge.data['preview']['path'],[])
        self.assertIn('尚未启动导航',bridge.data['preview']['message'])

    def test_preview_uses_native_path_and_ignores_old_session(self):
        from types import SimpleNamespace as NS
        bridge, response, result, handle, canceled = self.preview_bridge()
        bridge._preview([dict(x=1,y=1,yaw=0)])
        response.set_result(handle)
        poses=[NS(pose=NS(position=NS(x=x,y=0))) for x in (0.,1.,2.)]
        result.set_result(NS(status=4,result=NS(path=NS(header=NS(frame_id='map'),poses=poses))))
        self.assertEqual(bridge.data['preview']['path'],[[0.,0.],[1.,0.],[2.,0.]])
        self.assertEqual(bridge.data['preview']['length'],2.)
        self.assertEqual(bridge.data['mission'],'idle')
        bridge, response, result, handle, canceled = self.preview_bridge()
        bridge._preview([dict(x=1,y=1,yaw=0)])
        bridge.generation+=1
        response.set_result(handle)
        self.assertEqual(canceled,[True])
        self.assertNotEqual(bridge.data['preview']['state'],'ready')

    def test_refuses_domain_already_used_by_other_nodes(self):
        class FakeNode:
            def get_node_names(self):
                return ['gazebo_web_console', 'chassis_driver']
            def get_name(self):
                return 'gazebo_web_console'
        bridge = RosBridge.__new__(RosBridge)
        bridge.node, bridge.domain = FakeNode(), 73
        with self.assertRaisesRegex(ValueError, '已被其他 ROS 节点占用'):
            bridge._command('session', True)

    def test_stale_telemetry_does_not_report_ready(self):
        import time
        bridge = RosBridge.__new__(RosBridge)
        bridge.lock = threading.RLock()
        bridge.active, bridge.available, bridge.error = True, True, None
        bridge.reset_data()
        bridge.data.update(ready=True, scan=[[1, 2]], ranges=[.2, .3, .4, .5])
        bridge.seen = dict(clock=time.monotonic() - 10, pose=time.monotonic() - 10)
        snapshot = bridge.snapshot()
        self.assertFalse(snapshot['ready'])
        self.assertFalse(snapshot['connected'])
        self.assertFalse(snapshot['pose_fresh'])
        self.assertEqual(snapshot['scan'], [])
        self.assertEqual(snapshot['ranges'], [None] * 4)
        self.assertIsNone(snapshot['speed'])


class MapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = Catalog()

    def test_existing_maps_match_world_metadata_and_spawn(self):
        public = self.catalog.list()
        self.assertGreaterEqual(len(public), 17)
        for item in public:
            with self.subTest(map=item['id']):
                self.assertNotIn('free', item)
                self.assertNotIn('world', item)
                entry = self.catalog.get(item['id'])
                self.assertTrue(Path(entry['world']).is_file())
                self.catalog.check_point(entry, *entry['spawn'][:2])
                self.assertTrue(entry['png'].startswith(b'\x89PNG'))

    def test_boundary_unknown_and_nonfinite_goals_rejected(self):
        entry = self.catalog.get('demo')
        for point in [(-100, -100), (100, 100), (math.nan, 0), (math.inf, 0)]:
            with self.subTest(point=point), self.assertRaises(ValueError):
                self.catalog.check_point(entry, *point)

    def payload(self, origin=(1., 2., .7), size=80):
        image = Image.new('L', (size, size), 254)
        for i in range(size):
            image.putpixel((i, 0), 0)
            image.putpixel((i, size - 1), 0)
            image.putpixel((0, i), 0)
            image.putpixel((size - 1, i), 0)
        output = io.BytesIO()
        image.save(output, format='PPM')
        cfg = dict(image='map.pgm', resolution=.05, origin=list(origin), negate=0,
                   free_thresh=.196, occupied_thresh=.65)
        return dict(title='旋转地图', yaml=yaml.safe_dump(cfg), image_name='map.pgm',
                    image=base64.b64encode(output.getvalue()).decode(), wall_height=1.5)

    def test_import_builds_world_and_preserves_rotated_coordinates(self):
        with tempfile.TemporaryDirectory() as tmp:
            catalog = Catalog(data=tmp)
            imported = catalog.import_map(self.payload())
            entry = catalog.get(imported['id'])
            self.assertEqual(imported['origin'], [1., 2., .7])
            self.assertEqual(imported['title'], '旋转地图')
            self.assertEqual(imported['category'], '导入')
            self.assertTrue(Path(entry['world']).exists())
            catalog.check_point(entry, *imported['spawn'][:2])
            catalog.refresh()
            self.assertEqual(catalog.get(imported['id'])['title'], '旋转地图')

    def test_failed_import_cleans_up_generated_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            catalog = Catalog(data=tmp)
            with self.assertRaisesRegex(ValueError, '余量'):
                catalog.import_map(self.payload(size=12))
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_import_does_not_trust_image_path_from_yaml(self):
        with tempfile.TemporaryDirectory() as tmp:
            catalog = Catalog(data=tmp)
            payload = self.payload()
            cfg = yaml.safe_load(payload['yaml'])
            cfg['image'] = '/etc/this-does-not-exist/map.pgm'
            payload['yaml'] = yaml.safe_dump(cfg)
            entry = catalog.get(catalog.import_map(payload)['id'])
            self.assertTrue(Path(entry['map']).is_relative_to(tmp))
            self.assertEqual(entry['cfg']['image'], 'image.pgm')

    def test_invalid_resolution_and_threshold_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            catalog = Catalog(data=tmp)
            for key, value in [('resolution', 0), ('resolution', float('inf')), ('free_thresh', .9), ('mode', 'raw')]:
                payload = self.payload()
                cfg = yaml.safe_load(payload['yaml'])
                cfg[key] = value
                payload['yaml'] = yaml.safe_dump(cfg)
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    catalog.import_map(payload)
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_hardware_default_domain_disallowed(self):
        with self.assertRaises(ValueError):
            Runtime(self.catalog, domain=0)


class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        class FakeRuntime:
            def snapshot(self, after=0):
                return dict(state='idle', telemetry=dict(speed=float('nan')), events=[])
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(FakeRuntime(), Catalog()))
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f'http://127.0.0.1:{cls.server.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def test_page_and_map_endpoints(self):
        with urlopen(self.url) as response:
            self.assertEqual(response.status, 200)
            self.assertIn('NAV STUDIO', response.read().decode())
            self.assertIn("frame-ancestors 'none'", response.headers['Content-Security-Policy'])
        with urlopen(self.url + '/api/state') as response:
            self.assertIsNone(json.load(response)['telemetry']['speed'])

    def test_static_allowlist_prevents_file_access(self):
        for path in ['/../README.md', '/data/maps/file', '/api/maps/nope/image.png']:
            with self.subTest(path=path), self.assertRaises(HTTPError) as error:
                urlopen(self.url + path)
            self.assertIn(error.exception.code, (400, 404))

    def test_cross_origin_command_rejected(self):
        request = Request(self.url + '/api/simulation/start', b'{}',
                          headers={'Content-Type': 'application/json', 'Origin': 'http://another-site.test'})
        with self.assertRaises(HTTPError) as error:
            urlopen(request)
        self.assertEqual(error.exception.code, 403)

    def test_non_json_command_rejected(self):
        request = Request(self.url + '/api/simulation/start', b'map_id=demo', headers={'Content-Type': 'application/x-www-form-urlencoded'})
        with self.assertRaises(HTTPError) as error:
            urlopen(request)
        self.assertEqual(error.exception.code, 400)


if __name__ == '__main__':
    unittest.main()
