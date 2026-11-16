#!/usr/bin/python3
"""Local 3D bracket assembly editor; edits stay separate from live simulation."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import subprocess
import threading
from urllib.parse import urlsplit
import webbrowser
from model import Store,ConflictError
HERE=Path(__file__).resolve().parent


def handler(store):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(HERE/'web'),**kwargs)
        def payload(self,status,data):
            data=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers()
            if self.command!='HEAD':self.wfile.write(data)
        def do_GET(self):
            path=urlsplit(self.path).path
            if path=='/api/state':
                try:self.payload(200,dict(state=store.load(),assets=store.assets,path=str(store.path)))
                except (OSError,ValueError) as error:self.payload(500,dict(error=str(error)))
            elif path.startswith('/meshes/'):
                key=path.removeprefix('/meshes/');asset=store.assets.get(key)
                if asset is None:self.send_error(404);return
                data=Path(asset['path']).read_bytes();self.send_response(200);self.send_header('Content-Type','application/octet-stream');self.send_header('Content-Length',str(len(data)));self.end_headers()
                if self.command!='HEAD':self.wfile.write(data)
            elif path.startswith('/api/') or '..' in path:self.send_error(404)
            else:super().do_GET()
        def do_HEAD(self):self.do_GET()
        def do_POST(self):
            path=urlsplit(self.path).path
            if path not in ('/api/state','/api/export'):self.payload(404,dict(error='接口不存在'));return
            if self.headers.get('Origin') not in (None,'http://'+self.headers.get('Host','')):self.payload(403,dict(error='不接受跨站保存'));return
            try:
                size=int(self.headers.get('Content-Length','0'))
                if self.headers.get_content_type()!='application/json' or not 0<size<=256000:raise ValueError('保存格式或大小无效')
                raw=json.loads(self.rfile.read(size))
                if path=='/api/state':self.payload(200,dict(state=store.save(raw),path=str(store.path)))
                else:
                    if raw.get('revision')!=store.load()['revision']:raise ConflictError('请先保存当前草稿再导出')
                    self.payload(200,store.export())
            except ConflictError as error:self.payload(409,dict(error=str(error)))
            except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:self.payload(400,dict(error=str(error)))
            except OSError as error:self.payload(500,dict(error='文件写入失败：'+str(error)))
        def log_message(self,fmt,*args):print(fmt%args,flush=True)
    return Handler


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8767);parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--config',type=Path,default=HERE/'assembly_settings.json');parser.add_argument('--open',action='store_true')
    args=parser.parse_args();store=Store(args.config.resolve());store.load()
    try:server=ThreadingHTTPServer((args.host,args.port),handler(store))
    except OSError as error:parser.exit(1,f'装配页面无法启动：{error}\n')
    url=f'http://{args.host}:{server.server_port}';print(f'三维装配：{url}\n保存位置：{store.path}',flush=True)
    if args.open:
        browser=shutil.which('firefox')
        def open_page():
            if browser:subprocess.Popen([browser,'--new-tab',url],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
            else:webbrowser.open(url)
        threading.Timer(.5,open_page).start()
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


if __name__=='__main__':main()
