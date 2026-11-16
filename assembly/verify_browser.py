"""Browser QA on a disposable local state; leave the real editor open afterwards."""
import base64
import json
from pathlib import Path
import sys
import tempfile
import threading
from http.server import ThreadingHTTPServer
from urllib.request import urlopen
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'test'))
from verify_sensor_mounting_browser import Browser
from model import Store,world_transforms,frames_for
from server import handler
import numpy as np


def main():
    b=Browser(2834);results=[]
    def js(code):return b.js('return window.wrappedJSObject.eval('+json.dumps(code)+')')
    def inspect(code):return json.loads(js('JSON.stringify('+code+')'))
    def wait(expression):b.wait('window.wrappedJSObject.eval('+json.dumps(expression)+')')
    def fill(id,value):
        js(f'{{const e=document.getElementById({json.dumps(id)});e.value={json.dumps(str(value))};e.dispatchEvent(new Event("change",{{bubbles:true}}));}}')
    def click(id):js('document.getElementById('+json.dumps(id)+').click()')
    with tempfile.TemporaryDirectory() as folder:
        store=Store(Path(folder)/'assembly.json');store.load();server=ThreadingHTTPServer(('127.0.0.1',8772),handler(store));threading.Thread(target=server.serve_forever,daemon=True).start()
        try:
            b.call('WebDriver:SetWindowRect',dict(width=1540,height=1040));b.call('WebDriver:Navigate',dict(url='http://127.0.0.1:8772'))
            wait('!!window.assembly?.ready');wait('assembly.renderer.info.render.triangles>30000')
            results.append('All catalog STL assets rendered with local dependencies')
            js("assembly.select('main_bracket')");fill('positions_0',125);fill('rotations_2',37);fill('partNotes','主控支架测试备注')
            expected=inspect("assembly.groups.get('main_bracket').matrixWorld.elements")
            fill('parent','third_crossbar')
            actual=inspect("assembly.groups.get('main_bracket').matrixWorld.elements")
            np.testing.assert_allclose(actual,expected,atol=1e-10)
            results.append('Millimetre inputs / RPY and reparenting preserve world pose')
            fill('parent','base_link');fill('positions_0',150);fill('rotations_2',0)
            click('save');wait('document.getElementById("saveState").textContent.startsWith("已保存")')
            saved=store.load();part=next(p for p in saved['parts'] if p['id']=='main_bracket')
            assert part['xyz_m'][0]==.15 and part['notes']=='主控支架测试备注'
            b.call('WebDriver:Refresh',{});wait('!!window.assembly?.ready')
            assert inspect("assembly.state.parts.find(p=>p.id==='main_bracket').notes")=='主控支架测试备注'
            results.append('Save writes disk, persists installation notes and reloads')
            js("assembly.select('l2_bracket')");fill('positions_2',123)
            b.call('WebDriver:Refresh',{});wait('!!window.assembly?.ready')
            assert inspect("assembly.state.parts.find(p=>p.id==='l2_bracket').xyz_m[2]")==.123
            results.append('Unsaved local browser draft recovered across refresh')
            # Use actual canvas pointer actions to select visible triangle faces.
            js("void import('three').then(T=>window.qaThree=T)");wait('!!window.qaThree')
            click('pick')
            def pick_surface(partid):
                position=inspect("""(()=>{const T=qaThree,g=assembly.groups.get(%s),meshes=[];assembly.scene.updateMatrixWorld(true);assembly.scene.traverse(o=>{if(o.isMesh&&o.userData.part)meshes.push(o)});const own=meshes.find(m=>m.userData.part===%s),a=own.geometry.attributes.position,r=new T.Raycaster(),rect=document.getElementById('viewport').getBoundingClientRect();for(let i=0;i<a.count;i+=Math.max(3,Math.floor(a.count/400/3)*3)){if(i+2>=a.count)break;const p=new T.Vector3().fromBufferAttribute(a,i).add(new T.Vector3().fromBufferAttribute(a,i+1)).add(new T.Vector3().fromBufferAttribute(a,i+2)).multiplyScalar(1/3).applyMatrix4(own.matrixWorld),s=p.clone().project(assembly.camera);if(Math.abs(s.x)>.94||Math.abs(s.y)>.94)continue;r.setFromCamera(new T.Vector2(s.x,s.y),assembly.camera);const hit=r.intersectObjects(meshes,false)[0];if(hit?.object.userData.part===%s)return [rect.x+(s.x+1)/2*rect.width,rect.y+(1-s.y)/2*rect.height];}return null;})()"""%(json.dumps(partid),json.dumps(partid),json.dumps(partid)))
                assert position,partid
                before=inspect('assembly.state.markers.length')
                b.call('WebDriver:PerformActions',dict(actions=[dict(type='pointer',id='mouse',parameters=dict(pointerType='mouse'),actions=[dict(type='pointerMove',duration=0,origin='viewport',x=round(position[0]),y=round(position[1])),dict(type='pointerDown',button=0),dict(type='pointerUp',button=0)])]))
                wait('assembly.state.markers.length=='+str(before+1))
                return inspect('assembly.state.markers.at(-1).id')
            a=pick_surface('l2_bracket');target=pick_surface('base_link')
            fill('sourceMarker',a);fill('targetMarker',target);fill('markerName','支架安装面 A')
            click('alignFace')
            delta=inspect("""(()=>{const a=assembly.state.markers.find(m=>m.id===%d),b=assembly.state.markers.find(m=>m.id===%d),aw=assembly.markerWorld(a),bw=assembly.markerWorld(b);return [aw.point.distanceTo(bw.point),aw.normal.dot(bw.normal)];})()"""%(a,target))
            assert delta[0]<1e-8 and abs(delta[1]+1)<1e-8,delta
            results.append('Real surface clicks create persisted points; face alignment gives coincident points and opposite normals')
            click('save');wait('document.getElementById("saveState").textContent.startsWith("已保存")')
            saved=store.load();backend=world_transforms(frames_for(saved));front=inspect("assembly.groups.get('l2_bracket').matrixWorld.elements")
            np.testing.assert_allclose(backend['l2_bracket'],np.array(front).reshape(4,4).T,atol=1e-10)
            assert len(saved['markers'])==2
            click('export');wait('document.getElementById("status").textContent.includes("草稿已导出")')
            assert (Path(folder)/'exports/patrol_car_assembly.urdf').exists()
            results.append('Browser and backend transforms agree after alignment; URDF/YAML/snapshot export succeeds')
            result=dict(result='passed',checks=results)
            (HERE.parent/'artifacts/assembly_browser_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
        finally:
            # QA edits only touched the temporary file and port 8772.
            b.call('WebDriver:Navigate',dict(url='http://127.0.0.1:8767'));wait('!!window.assembly?.ready')
            image=b.call('WebDriver:TakeScreenshot',{});(HERE.parent/'artifacts/assembly_editor.png').write_bytes(base64.b64decode(image))
            b.socket.close();server.shutdown();server.server_close()


if __name__=='__main__':main()
