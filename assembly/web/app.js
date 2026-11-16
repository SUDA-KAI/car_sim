import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {TransformControls} from './vendor/TransformControls.js';
import {STLLoader} from './vendor/STLLoader.js';
const $=id=>document.getElementById(id),rad=THREE.MathUtils.degToRad,deg=THREE.MathUtils.radToDeg;
let data,assets,initial,savePath,selected='main_bracket',dirty=false,sequence=0,picking=false,ready=false;
let history=[],future=[],draftTimer,toastTimer;
const groups=new Map(),meshes=[],markerObjects=new Map(),axesObjects=[];
const viewport=$('viewport');
const scene=new THREE.Scene();scene.background=new THREE.Color('#c7d2dc');
const camera=new THREE.PerspectiveCamera(42,1,.005,200);camera.up.set(0,0,1);
let renderer,orbit,transform;
try{
 renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
 viewport.prepend(renderer.domElement);orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=false;
 transform=new TransformControls(camera,renderer.domElement);scene.add(transform);transform.setSize(.8);
}catch(error){$('loading').textContent='无法启动三维显示：'+error.message;throw error;}
const editPivot=new THREE.Object3D();scene.add(editPivot);
scene.add(new THREE.HemisphereLight(0xffffff,0x5b6b76,2.4));
for(const [xyz,intensity] of [[[2,-2,5],2.2],[[-3,1,2],1.2]]){const light=new THREE.DirectionalLight(0xffffff,intensity);light.position.set(...xyz);scene.add(light);}
const grid=new THREE.GridHelper(4,40,0x8294a4,0xa9b8c4);grid.rotation.x=Math.PI/2;grid.position.z=-.002;scene.add(grid);
const axis=new THREE.AxesHelper(.22);scene.add(axis);axesObjects.push(axis);
// Editor-only references: never included in ray picking, saved data or exports.
const originGuide=new THREE.Group(),parentGuide=new THREE.Group();scene.add(originGuide,parentGuide);
function guideDot(color,radius){const dot=new THREE.Mesh(new THREE.SphereGeometry(radius,14,10),new THREE.MeshBasicMaterial({color,depthTest:false,depthWrite:false}));dot.renderOrder=1000;return dot;}
originGuide.add(guideDot('#ffd35e',.007));parentGuide.add(guideDot('#63c9f0',.005));
const localAxes=new THREE.AxesHelper(.075);for(const material of (Array.isArray(localAxes.material)?localAxes.material:[localAxes.material])){material.depthTest=false;material.depthWrite=false;}localAxes.renderOrder=999;originGuide.add(localAxes);axesObjects.push(localAxes);
let originBounds;
const originConnection=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),new THREE.LineDashedMaterial({color:'#e4c260',dashSize:.015,gapSize:.009,depthTest:false,depthWrite:false}));originConnection.renderOrder=998;scene.add(originConnection);
function updateOriginGuide(){
 if(!data||!groups.has(selected))return;
 const p=part(),g=groups.get(selected);scene.updateMatrixWorld(true);
 const localPivot=pivotLocal(p),position=pivotWorld(p.id);originGuide.position.copy(position);originGuide.quaternion.copy(g.getWorldQuaternion(new THREE.Quaternion()));
 const parent=p.parent?groups.get(p.parent):null,parentPosition=parent?pivotWorld(p.parent):position.clone();parentGuide.position.copy(parentPosition);
 if(originBounds){originGuide.remove(originBounds);originBounds.geometry.dispose();originBounds.material.dispose();originBounds=null;}
 const box=new THREE.Box3();for(const mesh of meshes.filter(m=>m.userData.part===selected)){
  if(!mesh.geometry.boundingBox)mesh.geometry.computeBoundingBox();mesh.updateMatrix();box.union(mesh.geometry.boundingBox.clone().applyMatrix4(mesh.matrix));
 }
 if(!box.isEmpty()){
  const size=box.getSize(new THREE.Vector3()),center=box.getCenter(new THREE.Vector3());
  const geometry=new THREE.BoxGeometry(size.x,size.y,size.z),edges=new THREE.EdgesGeometry(geometry);geometry.dispose();
  originBounds=new THREE.LineSegments(edges,new THREE.LineBasicMaterial({color:'#c39420',transparent:true,opacity:.65,depthTest:false,depthWrite:false}));originBounds.position.copy(center.sub(localPivot));originBounds.renderOrder=997;originGuide.add(originBounds);
 }
 originConnection.geometry.setFromPoints([parentPosition,position]);originConnection.computeLineDistances();
 originGuide.visible=$('showOrigins').checked;parentGuide.visible=$('showOrigins').checked&&!!parent;
 originConnection.visible=$('showOrigins').checked&&!!parent&&position.distanceTo(parentPosition)>1e-6;
}
$('showOrigins').onchange=()=>{updateOriginGuide();draw();};
let scheduled=false;
function draw(){if(scheduled)return;scheduled=true;requestAnimationFrame(()=>{scheduled=false;scene.updateMatrixWorld(true);renderer.render(scene,camera);renderLabels();});}
function resize(){const {width,height}=viewport.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();draw();}
new ResizeObserver(resize).observe(viewport);orbit.addEventListener('change',draw);transform.addEventListener('change',draw);
function part(id=selected){return data.parts.find(p=>p.id===id);}
function partBounds(p){
 if(p.asset==='frame')return null;
 const box=new THREE.Box3();for(const mesh of meshes.filter(m=>m.userData.part===p.id)){
  if(!mesh.geometry.boundingBox)mesh.geometry.computeBoundingBox();mesh.updateMatrix();box.union(mesh.geometry.boundingBox.clone().applyMatrix4(mesh.matrix));
 }return box.isEmpty()?null:box;
}
function pivotLocal(p){
 if(!p.pivot_percent)return new THREE.Vector3();const box=partBounds(p);if(!box)return new THREE.Vector3();
 const size=box.getSize(new THREE.Vector3());return box.min.clone().add(size.multiply(new THREE.Vector3(...p.pivot_percent).multiplyScalar(.01)));
}
function pivotWorld(id=selected){return groups.get(id).localToWorld(pivotLocal(part(id)));}
function pivotPositionInParent(p=part()){
 const group=groups.get(p.id),own=pivotLocal(p).applyQuaternion(group.quaternion).add(group.position);
 return p.parent?own.sub(pivotLocal(part(p.parent))):own;
}
function setPivotPositionInParent(position){
 const p=part(),group=groups.get(selected),parentPoint=p.parent?pivotLocal(part(p.parent)):new THREE.Vector3();
 p.xyz_m=position.clone().add(parentPoint).sub(pivotLocal(p).applyQuaternion(group.quaternion)).toArray();
}
function setRotationAroundPivot(angles){
 const p=part(),group=groups.get(selected),pivot=pivotLocal(p),point=pivot.clone().applyQuaternion(group.quaternion).add(group.position);
 const rotation=new THREE.Quaternion().setFromEuler(new THREE.Euler(...angles.map(rad),'ZYX'));
 p.xyz_m=point.sub(pivot.applyQuaternion(rotation)).toArray();p.rpy_deg=angles;
}
function fixed(p=part()){return ['base_link','third_crossbar'].includes(p.id);}
function descendants(id){const found=new Set([id]);let changed=true;while(changed){changed=false;for(const p of data.parts)if(found.has(p.parent)&&!found.has(p.id)){found.add(p.id);changed=true;}}return found;}
function toast(text,error=false){$('toast').textContent=text;$('toast').className=error?'error':'';$('toast').style.display='block';clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').style.display='none',6500);}
function checkpoint(){history.push(JSON.stringify(data));if(history.length>50)history.shift();future=[];buttons();}
function buttons(){$('undo').disabled=!history.length;$('redo').disabled=!future.length;}
function markDirty(){dirty=true;sequence++;$('saveState').textContent='有修改 · 尚未保存';$('saveState').className='dirty';clearTimeout(draftTimer);draftTimer=setTimeout(()=>{try{localStorage.setItem('assembly-draft:'+savePath,JSON.stringify(data));}catch{}},200);}
function changed(){markDirty();updateObjects();renderPanel();renderList();renderMarkers();draw();}
function invalidatePose(id){for(const key of descendants(id)){const p=part(key);if(!fixed(p))p.confirmed=false;}}
function captureGroup(id){invalidatePose(id);const p=part(id),g=groups.get(id);p.xyz_m=g.position.toArray();const e=new THREE.Euler().setFromQuaternion(g.quaternion,'ZYX');p.rpy_deg=[e.x,e.y,e.z].map(deg);}
function updateObjects(){
 for(const p of data.parts){const g=groups.get(p.id),parent=p.parent?groups.get(p.parent):scene;if(g.parent!==parent)parent.add(g);g.position.set(...p.xyz_m);g.rotation.set(...p.rpy_deg.map(rad),'ZYX');g.visible=p.visible;}
 scene.updateMatrixWorld(true);
 const reference=groups.get('third_crossbar').getWorldPosition(new THREE.Vector3());axis.position.copy(reference);
 $('referenceLabel').textContent='原点：车身地面中心 · 第三横杠：('+reference.toArray().map(v=>Number((v*1000).toFixed(3))).join(', ')+') mm';
}
function renderList(){
 $('parts').replaceChildren();$('references').replaceChildren();
 const assemblyPart=p=>['bracket','attachment'].includes(p.category);
 const ordering=[...data.parts.filter(assemblyPart),...data.parts.filter(p=>!assemblyPart(p))];
 for(const p of ordering){const row=document.createElement('div');row.className='part'+(p.id===selected?' selected':'');row.dataset.part=p.id;
  const color=document.createElement('span');color.className='color';color.style.background=assets[p.asset]?.color||(p.asset==='camera'?'#72c8d0':'#a2b7c7');row.append(color);
  const text=document.createElement('span');text.className='label';text.append(document.createTextNode(p.label));const sub=document.createElement('small');sub.textContent=assemblyPart(p)?(p.confirmed?'已确认安装':p.category==='attachment'?'测距位置已知 · 外形待对齐':'待装配 · 位置未确认'):(p.category==='reference'?'参考坐标':p.asset==='camera'?'相机外形示意':'当前装配位置');text.append(sub);row.append(text);
  const check=document.createElement('input');check.type='checkbox';check.checked=p.visible;check.title='显示 / 隐藏此部件及子部件';check.onclick=e=>e.stopPropagation();check.onchange=()=>{checkpoint();p.visible=check.checked;changed();};row.append(check);
  row.onclick=()=>select(p.id);(p.category==='reference'&&p.id!=='third_crossbar'?$('references'):$('parts')).append(row);
 }
 const pending=data.parts.filter(assemblyPart);$('progress').textContent=pending.filter(p=>p.confirmed).length+' / '+pending.length;
}
for(const [container,keys] of [['positions',['X','Y','Z']],['rotations',['Roll','Pitch','Yaw']]]){
 keys.forEach((label,index)=>{const holder=document.createElement('label');holder.append(document.createTextNode(label));const input=document.createElement('input');input.type='number';input.step=container==='positions'?'1':'1';input.id=container+'_'+index;holder.append(input);$(container).append(holder);
 input.onchange=()=>{if(!data)return;const value=Number(input.value);if(!input.value.trim()||!Number.isFinite(value)||Math.abs(value)>(container==='positions'?10000:3600)){toast('请填写有效的位置或角度',true);renderPanel();return;}checkpoint();invalidatePose(selected);
 if(container==='positions'){const position=pivotPositionInParent();position.setComponent(index,value/1000);setPivotPositionInParent(position);}
 else{const angles=[...part().rpy_deg];angles[index]=value;setRotationAroundPivot(angles);}
 changed();};});
}
for(const [index,name] of ['X %','Y %','Z %'].entries()){
 const label=document.createElement('label');label.append(document.createTextNode(name));const input=document.createElement('input');input.id='pivot_'+index;input.type='number';input.min='0';input.max='100';input.step='.1';label.append(input);$('pivotPercent').append(label);
 input.onchange=()=>{const values=[0,1,2].map(i=>Number($('pivot_'+i).value));if(values.some(v=>!Number.isFinite(v)||v<0||v>100)||[0,1,2].some(i=>!$('pivot_'+i).value.trim())){toast('O 点百分比应在 0% 到 100% 之间',true);renderPanel();return;}setCustomPivot(values);};
}
function facingBottom(){
 const quaternion=groups.get(selected).getWorldQuaternion(new THREE.Quaternion());
 const vertical=[new THREE.Vector3(1,0,0),new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,1)].map(axis=>axis.applyQuaternion(quaternion).z);
 const count=vertical.filter(v=>Math.abs(v)>1e-6).length;
 return {values:vertical.map(v=>Math.abs(v)<1e-6?50:v>0?0:100),label:count===1?'当前朝向底面中心':count===2?'当前朝向最低边中心':'当前朝向最低角点'};
}
function setCustomPivot(values){const previous=pivotLocal(part());checkpoint();if(values)part().pivot_percent=[...values];else delete part().pivot_percent;changed();
 if(previous.distanceTo(pivotLocal(part()))<1e-9)toast('这个点与当前 O 重合，所以不会移动。模型底面按未旋转坐标定义；可另选“当前朝向底部”。');
}
$('customPivot').onchange=()=>setCustomPivot($('customPivot').checked?[0,1,2].map(i=>Number($('pivot_'+i).value)):null);
$('pivotBottom').onclick=()=>setCustomPivot([50,50,0]);$('pivotCenter').onclick=()=>setCustomPivot([50,50,50]);$('pivotOriginal').onclick=()=>setCustomPivot(null);
$('pivotFacingBottom').onclick=()=>setCustomPivot(facingBottom().values);
function renderPanel(){
 const p=part();$('selectedName').textContent=p.label;const a=assets[p.asset];
 $('assetInfo').textContent=a?`${a.size_mm.map(v=>v.toFixed(2)).join(' × ')} mm · ${a.triangles.toLocaleString()} 面\n${p.pivot_percent?'O：自定义百分比参考点':['bracket','attachment'].includes(p.category)?'默认原点：模型包围盒底面中心':'默认沿用原有模型坐标'}`:p.asset==='frame'?'参考坐标，无实体外形':'尺寸为示意，不代表真实设备外壳';
 const ownOrigin=p.id==='base_link'?'车身地面中心，是整车 base_link 的 (0,0,0)。':p.id==='third_crossbar'?'已确认的第三根横杠几何中心，距车尾 690 mm、距左侧 450 mm、离地 495 mm。':p.asset==='l2'?'雷达底部安装面的中心，沿用官方点云坐标原点。':p.asset==='camera'?'当前相机示意盒的几何中心，沿用你填写的相机安装点。':p.asset==='imu'?'当前 IMU 示意盒的几何中心，沿用你填写的 IMU 安装点。':p.asset==='frame'?(p.id.startsWith('ultrasonic_')?'已填写的超声波测距参考点，与支架、本体的编辑原点分开。':'此参考坐标系的 (0,0,0)，没有实体模型。'):'模型未旋转时，包围盒底面矩形的中心。它可能位于空洞内，并不一定是实体表面或螺孔中心。';
 const bounds=partBounds(p),custom=!!p.pivot_percent;
 $('originInfo').textContent=custom?'自选编辑参考点：包围盒 X='+p.pivot_percent[0]+'%、Y='+p.pivot_percent[1]+'%、Z='+p.pivot_percent[2]+'%。':ownOrigin;
 const rawPoint=a?pivotLocal(p).sub(new THREE.Vector3(...a.offset_m)):null;
 $('sourceOrigin').textContent=a&&['bracket','attachment'].includes(p.category)?'原始 STL 中这个点（mm）：('+rawPoint.toArray().map(v=>Number((v*1000).toFixed(3))).join(', ')+')。':'黄色 O 是编辑参考点；黄色线框用于辨认包围盒范围。';
 const size=bounds?.getSize(new THREE.Vector3()),values=p.pivot_percent||(bounds?['x','y','z'].map(k=>THREE.MathUtils.clamp(-bounds.min[k]/(size[k]||1)*100,0,100)):[0,0,0]);
 $('customPivot').checked=custom;$('customPivot').disabled=!bounds;
 for(let i=0;i<3;i++){$('pivot_'+i).value=String(Number(values[i].toFixed(4)));$('pivot_'+i).disabled=!bounds||!custom;}
 for(const id of ['pivotBottom','pivotCenter','pivotOriginal','pivotFacingBottom'])$(id).disabled=!bounds;
 const bottom=facingBottom();$('pivotFacingBottom').textContent=bottom.label+'（'+bottom.values.join(' / ')+'%）';
 $('pivotFacingBottom').title='按整车 Z 向上判断当前包围盒的最低面、边或角；不是地面 Z=0。';
 $('pivotHint').textContent=bounds?'百分比沿模型自身 X/Y/Z 轴计算。旋转后“模型底面”可能不朝下。'+bottom.label+'对应 '+bottom.values.join('%、')+'%。更改 O 不挪动实体或子部件。':'此项是无实体的测量坐标，没有包围盒，保留其原点。';
 const locked=fixed(p);const excluded=descendants(p.id);$('parent').replaceChildren();
 for(const parent of data.parts){if(excluded.has(parent.id))continue;const option=new Option(parent.label,parent.id);$('parent').append(option);}
 $('parent').value=p.parent||'';$('parent').disabled=locked;
 const position=pivotPositionInParent(p).toArray();
 for(const c of ['positions','rotations'])for(let i=0;i<3;i++){const input=$(c+'_'+i),v=(c==='positions'?position[i]*1000:p.rpy_deg[i]);input.value=String(Number(v.toFixed(c==='positions'?3:4)));input.disabled=locked;}
 $('partNotes').value=p.notes;$('confirmed').checked=p.confirmed;$('confirmed').disabled=locked;
 $('zeroRotation').disabled=locked;$('resetPart').disabled=locked;
 const g=groups.get(p.id),v=pivotWorld(p.id);$('worldPose').textContent='O 的整车坐标（mm）：'+v.toArray().map(n=>(n*1000).toFixed(2)).join(' / ');
 if(locked)transform.detach();else{
  if(!transform.dragging){editPivot.position.copy(v);editPivot.quaternion.copy(g.getWorldQuaternion(new THREE.Quaternion()));}
  if(transform.object!==editPivot)transform.attach(editPivot);
 }
 updateOriginGuide();
 buttons();
}
function select(id){selected=id;renderList();renderPanel();draw();}
$('parent').onchange=()=>{
 const p=part();checkpoint();scene.updateMatrixWorld(true);const g=groups.get(p.id),world=g.matrixWorld.clone(),parent=groups.get($('parent').value);
 const local=parent.matrixWorld.clone().invert().multiply(world);parent.add(g);local.decompose(g.position,g.quaternion,g.scale);p.parent=$('parent').value;captureGroup(p.id);changed();
};
$('notes').onchange=()=>{checkpoint();data.notes=$('notes').value;markDirty();};
$('partNotes').onchange=()=>{checkpoint();part().notes=$('partNotes').value;markDirty();};
$('confirmed').onchange=()=>{checkpoint();part().confirmed=$('confirmed').checked;changed();};
$('zeroRotation').onclick=()=>{checkpoint();invalidatePose(selected);setRotationAroundPivot([0,0,0]);changed();};
$('resetPart').onclick=()=>{checkpoint();const p=part(),original=initial.parts.find(x=>x.id===p.id);for(const key of ['parent','xyz_m','rpy_deg','confirmed'])p[key]=structuredClone(original[key]);if(original.pivot_percent)p.pivot_percent=[...original.pivot_percent];else delete p.pivot_percent;changed();};
$('undo').onclick=()=>{if(!history.length)return;future.push(JSON.stringify(data));const revision=data.revision,saved_at=data.saved_at;data=JSON.parse(history.pop());Object.assign(data,{revision,saved_at});$('notes').value=data.notes;changed();};
$('redo').onclick=()=>{if(!future.length)return;history.push(JSON.stringify(data));const revision=data.revision,saved_at=data.saved_at;data=JSON.parse(future.pop());Object.assign(data,{revision,saved_at});$('notes').value=data.notes;changed();};
transform.addEventListener('dragging-changed',e=>{orbit.enabled=!e.value;});transform.addEventListener('mouseDown',()=>checkpoint());
transform.addEventListener('objectChange',()=>{if(!ready)return;
 const p=part(),group=groups.get(selected),parent=group.parent;scene.updateMatrixWorld(true);
 const rotation=editPivot.getWorldQuaternion(new THREE.Quaternion()),position=editPivot.getWorldPosition(new THREE.Vector3()).sub(pivotLocal(p).applyQuaternion(rotation));
 group.position.copy(parent.worldToLocal(position));group.quaternion.copy(parent.getWorldQuaternion(new THREE.Quaternion()).invert().multiply(rotation));
 captureGroup(selected);markDirty();renderPanel();draw();});
function mode(value){transform.setMode(value);$('translate').classList.toggle('active',value==='translate');$('rotate').classList.toggle('active',value==='rotate');draw();}
$('translate').onclick=()=>mode('translate');$('rotate').onclick=()=>mode('rotate');$('space').onchange=()=>transform.setSpace($('space').value);
$('snap').onchange=()=>{transform.setTranslationSnap($('snap').checked?.001:null);transform.setRotationSnap($('snap').checked?rad(5):null);};
window.addEventListener('keydown',e=>{if(['INPUT','TEXTAREA','SELECT'].includes(e.target.tagName))return;if(e.key.toLowerCase()==='w')mode('translate');if(e.key.toLowerCase()==='e')mode('rotate');if(e.key==='Shift')transform.enabled=false;});
window.addEventListener('keyup',e=>{if(e.key==='Shift')transform.enabled=!picking;});window.addEventListener('blur',()=>{transform.enabled=!picking;});
function view(name='iso',focus=false){
 const bounds=new THREE.Box3();if(focus)bounds.setFromObject(groups.get(selected));else for(const p of data.parts)if(p.visible&&p.asset!=='frame')bounds.expandByObject(groups.get(p.id));
 const center=bounds.getCenter(new THREE.Vector3()),size=bounds.getSize(new THREE.Vector3()),extent=Math.max(.15,size.x,size.y,size.z);orbit.target.copy(center);
 const dirs={iso:[1.3,-1.8,1.4],top:[0,0,2.5],front:[2.5,0,.1],left:[0,2.5,.1]};camera.up.set(...(name==='top'?[0,1,0]:[0,0,1]));camera.position.copy(center).add(new THREE.Vector3(...dirs[name]).multiplyScalar(extent));orbit.update();draw();
}
for(const button of document.querySelectorAll('[data-view]'))button.onclick=()=>view(button.dataset.view);$('focus').onclick=()=>view('iso',true);
$('transparent').onchange=()=>{for(const mesh of meshes)if(mesh.userData.part==='base_link'){mesh.material.transparent=$('transparent').checked;mesh.material.opacity=$('transparent').checked?.25:1;mesh.material.depthWrite=!$('transparent').checked;}draw();};
$('axes').onchange=()=>{for(const axis of axesObjects)axis.visible=$('axes').checked;draw();};
function visible(object){while(object){if(!object.visible)return false;object=object.parent;}return true;}
let down;
renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
renderer.domElement.addEventListener('pointerup',e=>{
 if(!ready||!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5||transform.dragging)return;
 const bounds=renderer.domElement.getBoundingClientRect(),mouse=new THREE.Vector2((e.clientX-bounds.left)/bounds.width*2-1,-(e.clientY-bounds.top)/bounds.height*2+1),ray=new THREE.Raycaster();ray.setFromCamera(mouse,camera);
 const hit=ray.intersectObjects(meshes.filter(visible),false)[0];if(!hit)return;
 if(!(e.shiftKey||picking)){if(!transform.axis)select(hit.object.userData.part);return;}
 const id=hit.object.userData.part,g=groups.get(id);scene.updateMatrixWorld(true);
 const local=g.worldToLocal(hit.point.clone()),normal=hit.face.normal.clone().applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(hit.object.matrixWorld));normal.applyQuaternion(g.getWorldQuaternion(new THREE.Quaternion()).invert()).normalize();
 checkpoint();const markerId=Math.max(0,...data.markers.map(m=>m.id))+1;data.markers.push({id:markerId,part:id,point_m:local.toArray(),normal:normal.toArray(),label:''});
 changed();$('sourceMarker').value=String(markerId);markerSelectionChanged();toast(`已标记点 ${markerId} · ${part(id).label}`);
});
$('pick').onclick=()=>{picking=!picking;$('pick').classList.toggle('active',picking);transform.enabled=!picking;toast(picking?'单击模型表面添加标记，完成后再次点击按钮退出。':'已退出表面标记');};
function markerWorld(m){const g=groups.get(m.part);scene.updateMatrixWorld(true);return {point:g.localToWorld(new THREE.Vector3(...m.point_m)),normal:new THREE.Vector3(...m.normal).applyQuaternion(g.getWorldQuaternion(new THREE.Quaternion())).normalize()};}
function renderMarkers(){
 for(const {object,label} of markerObjects.values()){object.parent?.remove(object);object.children.forEach(o=>{o.geometry?.dispose();o.material?.dispose();});label.remove();}markerObjects.clear();
 const oldA=$('sourceMarker').value,oldB=$('targetMarker').value;$('sourceMarker').replaceChildren();$('targetMarker').replaceChildren();
 for(const m of data.markers){const group=new THREE.Group();group.position.set(...m.point_m);
 const sphere=new THREE.Mesh(new THREE.SphereGeometry(.005,12,8),new THREE.MeshBasicMaterial({color:0x0eab86,depthTest:false}));sphere.renderOrder=10;group.add(sphere);
 const arrow=new THREE.ArrowHelper(new THREE.Vector3(...m.normal),new THREE.Vector3(),.045,0x178d79,.012,.006);group.add(arrow);groups.get(m.part).add(group);
 const label=document.createElement('span');label.className='marker-label';label.textContent='#'+m.id+ (m.label?' '+m.label:'');$('markerLabels').append(label);markerObjects.set(m.id,{object:group,label});
 const text=`#${m.id} · ${part(m.part).label}${m.label?' · '+m.label:''}`;for(const id of ['sourceMarker','targetMarker'])$(id).append(new Option(text,String(m.id)));
 }
 if([...$('sourceMarker').options].some(o=>o.value===oldA))$('sourceMarker').value=oldA;
 if([...$('targetMarker').options].some(o=>o.value===oldB))$('targetMarker').value=oldB;
 markerSelectionChanged();
}
function renderLabels(){if(!data)return;const rect=viewport.getBoundingClientRect();for(const m of data.markers){const {object,label}=markerObjects.get(m.id)||{};if(!object)continue;const p=object.getWorldPosition(new THREE.Vector3()).project(camera);label.style.display=visible(object)&&Math.abs(p.z)<=1?'block':'none';label.style.left=((p.x+1)/2*rect.width+7)+'px';label.style.top=((-p.y+1)/2*rect.height-18)+'px';}
 for(const [object,id,offset] of [[originGuide,'originLabel',-28],[parentGuide,'parentOriginLabel',10]]){
  const label=$(id),p=object.getWorldPosition(new THREE.Vector3()).project(camera);label.style.display=object.visible&&Math.abs(p.z)<=1?'block':'none';label.style.left=((p.x+1)/2*rect.width+9)+'px';label.style.top=((-p.y+1)/2*rect.height+offset)+'px';
 }
}
function markerSelectionChanged(){const m=data?.markers.find(m=>String(m.id)===$('sourceMarker').value);$('markerName').value=m?.label||'';$('markerName').disabled=!m;$('deleteMarker').disabled=!m;$('alignPoint').disabled=$('alignFace').disabled=(data?.markers.length||0)<2;}
$('sourceMarker').onchange=markerSelectionChanged;$('targetMarker').onchange=markerSelectionChanged;
$('markerName').onchange=()=>{const m=data.markers.find(m=>String(m.id)===$('sourceMarker').value);if(!m)return;checkpoint();m.label=$('markerName').value;changed();};
$('deleteMarker').onclick=()=>{checkpoint();data.markers=data.markers.filter(m=>String(m.id)!==$ ('sourceMarker').value);changed();};
function align(face){
 const a=data.markers.find(m=>String(m.id)===$('sourceMarker').value),b=data.markers.find(m=>String(m.id)===$('targetMarker').value);
 if(!a||!b||a.part===b.part||fixed(part(a.part))||descendants(a.part).has(b.part)){toast('请选择可移动部件上的点 A，以及其他独立部件上的点 B。',true);return;}
 checkpoint();const g=groups.get(a.part),parent=g.parent,aw=markerWorld(a),bw=markerWorld(b),rotation=g.getWorldQuaternion(new THREE.Quaternion());
 if(face)rotation.premultiply(new THREE.Quaternion().setFromUnitVectors(aw.normal,bw.normal.clone().negate()));
 const position=bw.point.clone().sub(new THREE.Vector3(...a.point_m).applyQuaternion(rotation));
 g.position.copy(parent.worldToLocal(position));g.quaternion.copy(parent.getWorldQuaternion(new THREE.Quaternion()).invert().multiply(rotation));captureGroup(a.part);selected=a.part;changed();toast(face?'已对齐两点，并让安装面法向相对。请检查孔位和剩余绕法向的转角。':'已对齐两点，部件旋转保持原值。');
}
$('alignPoint').onclick=()=>align(false);$('alignFace').onclick=()=>align(true);
async function request(url,options){const r=await fetch(url,options);const value=await r.json();if(!r.ok)throw Error(value.error||'请求失败');return value;}
async function save(){
 $('save').disabled=true;const seq=sequence;
 try{const result=await request('/api/state',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});data.revision=result.state.revision;data.saved_at=result.state.saved_at;
 if(sequence===seq){dirty=false;clearTimeout(draftTimer);localStorage.removeItem('assembly-draft:'+savePath);$('saveState').textContent='已保存 · '+new Date(data.saved_at).toLocaleTimeString('zh-CN');$('saveState').className='';}else markDirty();
 toast('已保存装配位置、安装说明和表面标记。');return !dirty;
 }catch(error){toast(error.message,true);return false;}finally{$('save').disabled=false;}
}
$('save').onclick=save;
function download(name,text,type='application/json'){const link=document.createElement('a'),url=URL.createObjectURL(new Blob([text],{type}));link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
$('download').onclick=()=>download('assembly_settings.json',JSON.stringify(data,null,2)+'\n');
$('export').onclick=async()=>{if(dirty&&!await save())return;try{const result=await request('/api/export',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({revision:data.revision})});$('status').textContent='草稿已导出：'+result.files[0];toast('已在本机导出 URDF、安装 YAML 和装配快照。'+(result.unconfirmed.length?'尚有未确认部件，仅供装配检查。':''));}catch(error){toast(error.message,true);}};
window.addEventListener('beforeunload',e=>{if(dirty){
 try{localStorage.setItem('assembly-draft:'+savePath,JSON.stringify(data));}catch{}
 e.preventDefault();e.returnValue='';
}});
async function main(){
 try{
 const result=await request('/api/state');data=result.state;assets=result.assets;initial=structuredClone(data);savePath=result.path;$('savePath').textContent='保存到：'+savePath;
 // Reset uses measured sensor defaults and intentionally separated bracket inventory.
 const draft=localStorage.getItem('assembly-draft:'+savePath);if(draft){try{const d=JSON.parse(draft),savedIds=new Set(data.parts.map(p=>p.id)),draftIds=new Set(d.parts.map(p=>p.id));
  const correction=data.reference_correction,oldReference=d.parts.find(p=>p.id==='third_crossbar');
  const corrected=correction&&d.revision===correction.from_revision&&data.revision===correction.to_revision&&oldReference&&oldReference.xyz_m.every((v,i)=>Math.abs(v-correction.previous_xyz_m[i])<1e-9);
  const mounts=data.mounting_correction;
  const mountsCorrected=mounts&&d.revision===mounts.from_revision&&data.revision===mounts.to_revision&&Object.entries(mounts.frames).every(([id,pose])=>{const old=d.parts.find(p=>p.id===id);return old&&old.xyz_m.every((v,i)=>Math.abs(v-pose.previous_xyz_m[i])<1e-9);});
  if((d.revision===data.revision||corrected||mountsCorrected)&&draftIds.size===d.parts.length&&d.parts.every(p=>savedIds.has(p.id))&&data.parts.filter(p=>!draftIds.has(p.id)).every(p=>p.id==='camera_right_bracket'||/^ultrasonic_[1-4]_(model|body)$/.test(p.id))){
   const previous=new Map(d.parts.map(p=>[p.id,p]));d.parts=data.parts.map(p=>corrected&&p.id==='third_crossbar'?p:mountsCorrected&&mounts.frames[p.id]?{...previous.get(p.id),xyz_m:[...p.xyz_m]}:previous.get(p.id)||p);
   if(corrected||mountsCorrected){d.revision=data.revision;d.reference_correction=correction;d.mounting_correction=mounts;d.source_mounting_sha256=data.source_mounting_sha256;d.saved_at=data.saved_at;}
   d.parts=d.parts.map(p=>{const canonical=result.state.parts.find(x=>x.id===p.id);return {...p,label:canonical.label,category:canonical.category};});
   data=d;dirty=true;$('saveState').textContent=mountsCorrected?'已修正前探头位置 · 保留未保存装配草稿':corrected?'已修正横杠原点 · 保留未保存装配草稿':'已恢复浏览器草稿 · 尚未保存';$('saveState').className='dirty';}
 }catch{}}
 const loader=new STLLoader(),geometries={};await Promise.all(Object.values(assets).map(async a=>{const geometry=await loader.loadAsync(a.url);geometry.scale(a.scale,a.scale,a.scale);geometry.translate(...a.offset_m);geometry.computeVertexNormals();geometries[a.id]=geometry;}));
 for(const p of data.parts){const group=new THREE.Group();group.name=p.id;groups.set(p.id,group);scene.add(group);
  let geometry=geometries[p.asset],color=assets[p.asset]?.color;if(p.asset==='camera'){geometry=new THREE.BoxGeometry(.06,.05,.04);color=p.id.includes('left')?'#c19c61':'#6a9fc1';}if(p.asset==='imu'){geometry=new THREE.BoxGeometry(.04,.03,.015);color='#a78ac1';}
  if(geometry){const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,roughness:.7,metalness:.12,side:THREE.DoubleSide}));mesh.userData.part=p.id;group.add(mesh);meshes.push(mesh);}
  if(p.asset==='camera'){const lens=new THREE.Mesh(new THREE.CylinderGeometry(.013,.013,.012,20),new THREE.MeshStandardMaterial({color:'#1e3546',roughness:.2}));lens.rotation.z=-Math.PI/2;lens.position.x=.035;lens.userData.part=p.id;group.add(lens);meshes.push(lens);}
  if(p.id==='third_crossbar'){const point=new THREE.Mesh(new THREE.SphereGeometry(.009,12,8),new THREE.MeshBasicMaterial({color:'#b064a4'}));point.userData.part=p.id;group.add(point);meshes.push(point);}
 }
 updateObjects();$('notes').value=data.notes;renderList();renderPanel();renderMarkers();resize();view();ready=true;$('loading').style.display='none';if(!dirty){$('saveState').textContent=data.saved_at?'已保存 · '+new Date(data.saved_at).toLocaleTimeString('zh-CN'):'已载入 · 等待装配';$('saveState').className='';}
 // Readable QA hooks also let future tools inspect precise geometry without screenshots.
 window.assembly={get state(){return data;},get ready(){return ready;},select,groups,markerWorld,align,scene,camera,renderer,changed,view,save,pivotLocal,pivotWorld,pivotPositionInParent,editPivot,transform};draw();
 }catch(error){$('loading').style.display='block';$('loading').textContent='载入失败：'+error.message;toast(error.message,true);console.error(error);}
}
main();
