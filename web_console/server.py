"""No web framework required: local HTTP + native ROS 2 actions."""
import argparse
from concurrent.futures import TimeoutError
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import threading
import yaml
from urllib.parse import parse_qs, urlsplit
import webbrowser

from .catalog import Catalog
from .runtime import Runtime

STATIC = Path(__file__).resolve().parent / 'static'


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def make_handler(runtime, catalog):
    mutations = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def send(self, code, data, content_type='application/json; charset=utf-8'):
            if not isinstance(data, bytes):
                data = json.dumps(json_safe(data), ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            if self.command != 'HEAD':
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        def do_GET(self):
            url = urlsplit(self.path)
            try:
                if url.path == '/api/maps':
                    self.send(200, dict(maps=catalog.list()))
                elif url.path == '/api/state':
                    after = max(0, int(parse_qs(url.query).get('after', ['0'])[0]))
                    self.send(200, runtime.snapshot(after))
                elif url.path.startswith('/api/maps/') and url.path.endswith('/image.png'):
                    ident = url.path.split('/')[3]
                    self.send(200, catalog.get(ident)['png'], 'image/png')
                elif url.path in ('/', '/index.html', '/app.js', '/style.css'):
                    name = 'index.html' if url.path == '/' else url.path[1:]
                    mime = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8', 'css': 'text/css; charset=utf-8'}[name.rsplit('.', 1)[1]]
                    self.send(200, (STATIC / name).read_bytes(), mime)
                elif url.path == '/favicon.ico':
                    self.send(204, b'', 'image/x-icon')
                else:
                    self.send(404, dict(error='页面或接口不存在'))
            except (ValueError, KeyError) as error:
                self.send(400, dict(error=str(error)))

        def do_HEAD(self):
            self.do_GET()

        def do_POST(self):
            if self.headers.get('Origin') not in (None, 'http://' + self.headers.get('Host', '')):
                self.send(403, dict(error='不接受跨站控制请求'))
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if self.headers.get_content_type() != 'application/json' or not 0 < size <= 18_000_000:
                    raise ValueError('请使用 JSON 请求，大小不能超过 18 MB')
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError('请求必须是 JSON 对象')
                path = urlsplit(self.path).path
                with mutations:
                    if path == '/api/simulation/start':
                        result = runtime.start(payload)
                    elif path == '/api/simulation/stop':
                        result = runtime.stop()
                    elif path == '/api/navigate':
                        result = runtime.navigate(payload)
                    elif path == '/api/path/preview':
                        result = runtime.navigate(payload, preview=True)
                    elif path == '/api/navigation/cancel':
                        result = runtime.bridge.command('cancel')
                    elif path == '/api/initial-pose':
                        result = runtime.initial_pose(payload)
                    elif path == '/api/maps/import':
                        result = dict(map=catalog.import_map(payload))
                        runtime.log('success', '新地图已导入：' + result['map']['title'])
                    else:
                        self.send(404, dict(error='接口不存在'))
                        return
                self.send(200, result)
            except (ValueError, KeyError, TypeError, AttributeError, OverflowError, yaml.YAMLError) as error:
                self.send(400, dict(error=str(error)))
            except TimeoutError:
                self.send(503, dict(error='ROS 响应超时，请查看连接状态'))
            except Exception as error:
                runtime.log('error', '操作失败：' + str(error))
                self.send(500, dict(error='操作失败：' + str(error)))

        def log_message(self, fmt, *args):
            if '/api/state' not in args[0]:
                print(fmt % args, flush=True)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8770)
    parser.add_argument('--domain', type=int, default=73)
    parser.add_argument('--partition', default=os.environ.get('SIM_IGN_PARTITION'))
    parser.add_argument('--open', action='store_true')
    args = parser.parse_args()
    args.partition = args.partition or f'patrol_web_console_{args.domain}'
    # Two consoles must not control the same ROS graph or Gazebo partition.
    lockfile = open(f'/tmp/nav-studio-{os.getuid()}-domain-{args.domain}.lock', 'a')
    try:
        fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(1, f'仿真域 {args.domain} 已有网页控制台运行，请更换 --domain\n')
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    catalog = Catalog()
    runtime = Runtime(catalog, args.domain, args.partition)
    try:
        server = ThreadingHTTPServer((args.host, args.port), make_handler(runtime, catalog))
    except OSError:
        runtime.close()
        raise
    server.daemon_threads = True
    url = f'http://{args.host}:{server.server_port}'
    print(f'\n  NAV STUDIO · 仿真导航控制台\n  {url}\n  ROS_DOMAIN_ID={args.domain} · IGN_PARTITION={args.partition}\n', flush=True)
    if args.open:
        threading.Timer(.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        print('\n正在关闭控制台与所属仿真…', flush=True)
    finally:
        server.server_close()
        runtime.close()
        lockfile.close()


if __name__ == '__main__':
    main()
