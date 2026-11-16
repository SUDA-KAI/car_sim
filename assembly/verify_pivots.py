"""Browser checks for editable percent origins, using a disposable config."""
import base64
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
from urllib.request import urlopen,Request
from urllib.error import HTTPError
import numpy as np
from model import Store,frames_for,world_transforms
from server import handler
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parents[1]/'test'))
from verify_sensor_mounting_browser import Browser


def main():
 b=Browser(2834);checks=[]
 def js(code):return b.js('return window.wrappedJSObject.eval('+json.dumps(code)+')')
 def inspect(code):return json.loads(js('JSON.stringify('+code+')'))
 def wait(expression):b.wait('window.wrappedJSObject.eval('+json.dumps(expression)+')')
 def fill(id,value):js('{const e=document.getElementById('+json.dumps(id)+');e.value='+json.dumps(str(value))+';e.dispatchEvent(new Event("change",{bubbles:true}));}')
 def click(id):js('document.getElementById('+json.dumps(id)+').click()')
 def matrix(id):return np.array(inspect('assembly.groups.get('+json.dumps(id)+').matrixWorld.elements')).reshape(4,4).T
 def pivot(id):return np.array(inspect('assembly.pivotWorld('+json.dumps(id)+').toArray()'))
 with tempfile.TemporaryDirectory() as folder:
  store=Store(Path(folder)/'draft.json');store.load();server=ThreadingHTTPServer(('127.0.0.1',8772),handler(store));threading.Thread(target=server.serve_forever,daemon=True).start()
  try:
   b.call('WebDriver:Navigate',dict(url='http://127.0.0.1:8772'));wait('!!window.assembly?.ready')
   js("assembly.select('main_bracket')")
   for i,v in enumerate([20,-30,45]):fill('rotations_'+str(i),v)
   js("assembly.select('unilidar_lidar')");fill('parent','main_bracket');js("assembly.select('main_bracket')")
   js("assembly.state.markers.push({id:1,part:'main_bracket',point_m:[.01,.02,.03],normal:[0,0,1],label:'保持安装孔'});assembly.changed()")
   physical={id:matrix(id) for id in ['main_bracket','unilidar_lidar']};mark=np.array(inspect('assembly.markerWorld(assembly.state.markers[0]).point.toArray()'))
   click('pivotCenter');fill('pivot_0',25);fill('pivot_1',75);fill('pivot_2',100)
   for id in physical:np.testing.assert_allclose(matrix(id),physical[id],atol=1e-11)
   np.testing.assert_allclose(inspect('assembly.markerWorld(assembly.state.markers[0]).point.toArray()'),mark,atol=1e-11)
   asset=store.assets['main_bracket'];lo=np.array(asset['bounds_min_m'])+asset['offset_m'];hi=np.array(asset['bounds_max_m'])+asset['offset_m'];local=lo+(hi-lo)*[.25,.75,1]
   expected=(physical['main_bracket']@np.r_[local,1])[:3];np.testing.assert_allclose(pivot('main_bracket'),expected,atol=1e-11)
   checks.append('Percent origin uses rotated part-local bounds; mesh, child and marked hole stay fixed')
   original=pivot('main_bracket');x=float(js('document.getElementById("positions_0").value'));fill('positions_0',x+10)
   np.testing.assert_allclose(pivot('main_bracket')-original,[.01,0,0],atol=2e-6)
   fixed_pivot=pivot('main_bracket');fill('rotations_2',95)
   np.testing.assert_allclose(pivot('main_bracket'),fixed_pivot,atol=1e-11)
   checks.append('XYZ moves the chosen O; numeric rotation keeps O fixed')
   # TransformControls manipulates the editor-only proxy; exercise its real event handler.
   fixed_pivot=pivot('main_bracket')
   js("assembly.editPivot.rotateZ(.2);assembly.transform.dispatchEvent({type:'objectChange'})")
   np.testing.assert_allclose(pivot('main_bracket'),fixed_pivot,atol=1e-11)
   checks.append('Gizmo rotation also keeps custom O fixed')
   parent_mat=matrix('main_bracket');parent_o=pivot('main_bracket');js("assembly.select('unilidar_lidar')")
   child_o=pivot('unilidar_lidar');expected=parent_mat[:3,:3].T@(child_o-parent_o)
   fields=np.array([float(js('document.getElementById("positions_'+str(i)+'").value'))/1000 for i in range(3)])
   np.testing.assert_allclose(fields,expected,atol=1e-6)
   before=matrix('unilidar_lidar');click('pivotCenter');np.testing.assert_allclose(matrix('unilidar_lidar'),before,atol=1e-11)
   checks.append('Child coordinates refer to parent P; L2 editor origin does not redefine its physical frame')
   js("assembly.select('main_bracket')");click('save');wait('document.getElementById("saveState").textContent.startsWith("已保存")')
   saved=store.load();assert next(p for p in saved['parts'] if p['id']=='main_bracket')['pivot_percent']==[25,75,100]
   b.call('WebDriver:Refresh',{});wait('!!window.assembly?.ready')
   assert inspect("assembly.state.parts.find(p=>p.id==='main_bracket').pivot_percent")==[25,75,100]
   for id in ['main_bracket','unilidar_lidar']:np.testing.assert_allclose(matrix(id),world_transforms(frames_for(saved))[id],atol=1e-10)
   checks.append('Percent origins persist through disk save/reload; physical export transforms match browser')
   js("assembly.select('third_crossbar')");assert js('document.getElementById("customPivot").disabled')
   bad=store.load();next(p for p in bad['parts'] if p['id']=='main_bracket')['pivot_percent']=[-1,50,0]
   request=Request('http://127.0.0.1:8772/api/state',data=json.dumps(bad).encode(),headers={'Content-Type':'application/json'})
   try:urlopen(request);raise AssertionError('Invalid percent accepted')
   except HTTPError as error:assert error.code==400
   checks.append('Non-geometric reference frames stay fixed; invalid percent saves are rejected')
   result=dict(result='passed',checks=checks);(HERE.parent/'artifacts/assembly_percent_pivot_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
  finally:
   b.call('WebDriver:Navigate',dict(url='http://127.0.0.1:8767'));wait('!!window.assembly?.ready')
   image=b.call('WebDriver:TakeScreenshot',{});(HERE.parent/'artifacts/assembly_percent_pivots.png').write_bytes(base64.b64decode(image))
   b.socket.close();server.shutdown();server.server_close()


if __name__=='__main__':main()
