import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

window.addEventListener('error', e => { const box=document.querySelector('#loading'); box.style.display='grid'; box.textContent=`Simulator error: ${e.message}`; });
window.addEventListener('unhandledrejection', e => { const box=document.querySelector('#loading'); box.style.display='grid'; box.textContent=`Simulator error: ${e.reason?.message||e.reason}`; });

// Browser-only FR3 visualization. No SDK imports or network robot connection.
const root = document.querySelector('#view');
const status = document.querySelector('#status');
const model = await fetch('./assets/fairino3_v6/model.json').then(r => r.json());
const scene = new THREE.Scene();
scene.background = new THREE.Color('#0c151d');
const camera = new THREE.PerspectiveCamera(42, 1, 1, 5000);
camera.up.set(0, 0, 1);
camera.position.set(1030, -1250, 1010);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
root.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(180, -50, 260);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xd9f3ff, 0x27313a, 2.1));
const key = new THREE.DirectionalLight(0xfff0da, 3.3);
key.position.set(650, -400, 1100);
key.castShadow = true;
scene.add(key);
scene.add(new THREE.GridHelper(1200, 24, '#47616d', '#263942').rotateX(Math.PI/2));
const axes = new THREE.AxesHelper(180);
scene.add(axes);

const links = Object.fromEntries(model.links.map(x => [x.name, new THREE.Group()]));
const robot = new THREE.Group();
robot.add(links.base_link);
const pivots = [];
const jointByName = {};
for (const joint of model.joints) {
  const pivot = new THREE.Group();
  pivot.name = joint.name;
  pivot.position.set(...joint.xyz);
  // URDF fixed-axis roll/pitch/yaw uses XYZ composition.
  pivot.rotation.order = 'XYZ';
  pivot.rotation.set(...joint.rpy);
  pivot.userData.axis = new THREE.Vector3(...joint.axis).normalize();
  links[joint.parent].add(pivot);
  pivot.add(links[joint.child]);
  pivots.push(pivot);
  jointByName[joint.name] = pivot;
}
scene.add(robot);
const stl = new STLLoader();
const metal = new THREE.MeshStandardMaterial({color:0xd5e0e5, metalness:.52, roughness:.34});
for (const link of model.links) {
  const geometry = await stl.loadAsync(`./assets/fairino3_v6/${link.mesh}`);
  // FAIRINO's URDF uses metres and the browser scene uses millimetres.
  geometry.scale(1000, 1000, 1000);
  geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(geometry, metal);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  links[link.name].add(mesh);
}
const tcpLocal = new THREE.Vector3(-.45, -.949, 63.886);
const tcpMarker = new THREE.Mesh(new THREE.SphereGeometry(11, 24, 16), new THREE.MeshStandardMaterial({color:'#ff9755',emissive:'#6b2200',emissiveIntensity:.35}));
scene.add(tcpMarker);
const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color:'#ff9b55'}));
scene.add(pathLine);
const targets = new THREE.Group();
scene.add(targets);
const planeGrid = new THREE.Group();
scene.add(planeGrid);

function syncWorld(){ robot.updateMatrixWorld(true); const p=tcpLocal.clone().applyMatrix4(links.wrist3_link.matrixWorld); tcpMarker.position.copy(p); return p; }
function poseQuaternion(rpy){return new THREE.Quaternion().setFromEuler(new THREE.Euler(THREE.MathUtils.degToRad(rpy[0]),THREE.MathUtils.degToRad(rpy[1]),THREE.MathUtils.degToRad(rpy[2]),'ZYX'));}
function solveLinear(a,b){
  const n=b.length,m=a.map((row,i)=>[...row,b[i]]);
  for(let c=0;c<n;c++){let p=c;for(let r=c+1;r<n;r++)if(Math.abs(m[r][c])>Math.abs(m[p][c]))p=r;if(Math.abs(m[p][c])<1e-10)return null;[m[c],m[p]]=[m[p],m[c]];const d=m[c][c];for(let j=c;j<=n;j++)m[c][j]/=d;for(let r=0;r<n;r++)if(r!==c){const f=m[r][c];for(let j=c;j<=n;j++)m[r][j]-=f*m[c][j];}}
  return m.map(row=>row[n]);
}
function orientationError(current,target){const q=target.clone().multiply(current.clone().invert()).normalize();if(q.w<0)q.set(-q.x,-q.y,-q.z,-q.w);const angle=2*Math.acos(THREE.MathUtils.clamp(q.w,-1,1));const s=Math.sqrt(Math.max(1e-12,1-q.w*q.w));return new THREE.Vector3(q.x/s,q.y/s,q.z/s).multiplyScalar(angle);}
function ik(position,quaternion){
  const damping=2.5;
  for(let iteration=0;iteration<80;iteration++){
    robot.updateMatrixWorld(true);
    const current=tcpLocal.clone().applyMatrix4(links.wrist3_link.matrixWorld);
    const epos=position.clone().sub(current), error=[epos.x,epos.y,epos.z];
    if(epos.length()<2.5)return true;
    // Position-only damped least squares is deliberately illustrative: it
    // follows TCP XYZ while leaving wrist attitude out of the preview solver.
    const columns=pivots.map(p=>{
      const origin=p.getWorldPosition(new THREE.Vector3());
      const axis=p.userData.axis.clone().applyQuaternion(p.getWorldQuaternion(new THREE.Quaternion())).normalize();
      const linear=axis.clone().cross(current.clone().sub(origin));
      return [linear.x,linear.y,linear.z];
    });
    const a=Array.from({length:3},(_,r)=>Array.from({length:3},(_,c)=>columns.reduce((s,col)=>s+col[r]*col[c],0)+(r===c?damping*damping:0)));
    const y=solveLinear(a,error);if(!y)return false;
    for(let j=0;j<pivots.length;j++){
      let delta=columns[j].reduce((s,v,k)=>s+v*y[k],0);
      delta=THREE.MathUtils.clamp(delta,-.12,.12);
      pivots[j].rotation.z+=delta;
    }
  }
  robot.updateMatrixWorld(true);
  const p=tcpLocal.clone().applyMatrix4(links.wrist3_link.matrixWorld);
  return p.distanceTo(position)<12;
}
function xyzToBase(plane,xyz){
  return plane.origin.map((v,r)=>v+plane.basis[r][0]*xyz[0]+plane.basis[r][1]*xyz[1]+plane.basis[r][2]*xyz[2]);
}
function makePath(data){
  const plane=(data.planes||[]).find(p=>p.name===data.selected_plane)||data.planes?.[0];
  if(!plane||plane.reference!=='robot_base')throw Error('Import a calibration session with a selected base-reference plane.');
  const task=data.task||{}; const rows=task.rows||3,cols=task.columns||2;
  const points=task.points||Array.from({length:rows*cols},(_,i)=>[i%cols*30,Math.floor(i/cols)*30,0]);
  const orientation=data.xyz?.orientation_reference||[180,0,0];
  const dest=task.destination||[350,0,300,...orientation];
  // Legacy shared routes are copied per source, matching the Studio migration.
  const routes=task.routes||points.map(()=>task.waypoints||[]);
  if(routes.length!==points.length)throw Error('Each array part must have its own waypoint list.');
  const path=[];
  points.forEach((point,i)=>{
    path.push({name:`Pick ${i+1}`,pose:[...xyzToBase(plane,point),...orientation]});
    routes[i].forEach((p,j)=>path.push({name:`Route ${i+1}.${j+1}`,pose:p}));
    path.push({name:`Place ${i+1}`,pose:dest});
    if(i<points.length-1)[...routes[i+1]].reverse().forEach((p,j)=>path.push({name:`Return ${i+1}.${j+1}`,pose:p}));
  });
  return {plane,path,rows,cols,points};
}
function clearGroup(g){while(g.children.length){const o=g.children.pop();o.geometry?.dispose();}}
function drawRecipe(recipe){
  clearGroup(targets);clearGroup(planeGrid);
  const {plane,path,points}=recipe;
  const origin=new THREE.Vector3(...plane.origin);
  const x=new THREE.Vector3(plane.basis[0][0],plane.basis[1][0],plane.basis[2][0]);
  const y=new THREE.Vector3(plane.basis[0][1],plane.basis[1][1],plane.basis[2][1]);
  const z=new THREE.Vector3(plane.basis[0][2],plane.basis[1][2],plane.basis[2][2]);
  const frame=new THREE.Group();frame.position.copy(origin);frame.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x,y,z));planeGrid.add(frame);
  const grid=new THREE.GridHelper(400,20,'#4ab8e3','#254957');grid.rotation.x=Math.PI/2;grid.position.z=1;frame.add(grid);
  const arrow=new THREE.ArrowHelper(z,origin,150,0x4ab8e3,30,15);planeGrid.add(arrow);
  points.forEach((p,i)=>{const pos=new THREE.Vector3(...xyzToBase(plane,p));const m=new THREE.Mesh(new THREE.SphereGeometry(9,18,12),new THREE.MeshStandardMaterial({color:0x54dda6}));m.position.copy(pos);targets.add(m);const sprite=textSprite(`${i+1}`);sprite.position.copy(pos).add(new THREE.Vector3(0,0,22));targets.add(sprite);});
  const coords=path.map(s=>new THREE.Vector3(...s.pose.slice(0,3)));
  if(coords.length>1){pathLine.geometry.dispose();pathLine.geometry=new THREE.BufferGeometry().setFromPoints(coords);}
  document.querySelector('#sequence').innerHTML=path.map((s,i)=>`<div class="step" data-step="${i}"><span class="num">${String(i+1).padStart(2,'0')}</span><span>${s.name}</span><small>${s.pose.slice(0,3).map(v=>v.toFixed(0)).join(', ')}</small></div>`).join('');
  document.querySelector('#recipeInfo').textContent=`${recipe.rows} × ${recipe.cols} array · ${path.length} TCP stops · ${plane.name}`;
  const center=new THREE.Vector3(origin.x*.5,origin.y*.5,260);
  camera.position.add(center.clone().sub(controls.target));
  controls.target.copy(center);
}
function textSprite(text){const c=document.createElement('canvas');c.width=64;c.height=64;const x=c.getContext('2d');x.fillStyle='#07171b';x.beginPath();x.arc(32,32,22,0,Math.PI*2);x.fill();x.fillStyle='#75e6bb';x.font='bold 24px sans-serif';x.textAlign='center';x.textBaseline='middle';x.fillText(text,32,33);const t=new THREE.CanvasTexture(c);const s=new THREE.Sprite(new THREE.SpriteMaterial({map:t,transparent:true}));s.scale.set(25,25,1);return s;}
let recipe=null,playing=false,step=0,segmentFrom=null,segmentProgress=0,lastTime=performance.now();let speed=80,rate=1;
function setRecipe(data){recipe=makePath(data);step=0;playing=false;segmentFrom=null;drawRecipe(recipe);updateSequence();status.textContent='Recipe loaded. Review the orange TCP path, then press Start.';}
function updateSequence(){document.querySelectorAll('.step').forEach((el,i)=>{el.classList.toggle('active',playing&&i===step);el.classList.toggle('done',i<step);});}
function reset(){playing=false;step=0;segmentFrom=null;[0,-.7,1.4,0,-.7,0].forEach((v,i)=>pivots[i].rotation.z=v);syncWorld();updateSequence();status.textContent='Reset to the visual home pose.';}
function animate(now){requestAnimationFrame(animate);const dt=Math.min(.05,(now-lastTime)/1000);lastTime=now;
  if(playing&&recipe){if(step>=recipe.path.length){playing=false;status.textContent='Sequence preview complete.';updateSequence();}
    else{const target=recipe.path[step].pose;const goal=new THREE.Vector3(...target.slice(0,3));const q=poseQuaternion(target.slice(3));if(!segmentFrom){segmentFrom=syncWorld().clone();segmentProgress=0;}
      const dist=segmentFrom.distanceTo(goal);segmentProgress=Math.min(1,segmentProgress+dt*(speed/Math.max(dist,1))*rate);const desired=segmentFrom.clone().lerp(goal,segmentProgress);
      const ok=ik(desired,q);syncWorld();if(!ok)status.textContent=`Approximate visual IK at ${recipe.path[step].name}; route playback continues.`;if(segmentProgress>=1){step++;segmentFrom=null;updateSequence();status.textContent=step<recipe.path.length?`Moving to ${recipe.path[step].name}…`:'Sequence preview complete.';}
    }}
  controls.update();renderer.render(scene,camera);
}
function resize(){const w=root.clientWidth,h=root.clientHeight;camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}
window.addEventListener('resize',resize);resize();[0,-.7,1.4,0,-.7,0].forEach((v,i)=>pivots[i].rotation.z=v);syncWorld();requestAnimationFrame(animate);
document.querySelector('#file').addEventListener('change',async e=>{try{setRecipe(JSON.parse(await e.target.files[0].text()));}catch(err){status.textContent=`Could not load recipe: ${err.message}`;}});
document.querySelector('#demo').addEventListener('click',()=>setRecipe({selected_plane:'Demo workplane',planes:[{name:'Demo workplane',reference:'robot_base',origin:[350,-100,250],basis:[[1,0,0],[0,1,0],[0,0,1]]}],xyz:{orientation_reference:[180,0,0]},task:{rows:3,columns:2,points:[[0,0,0],[35,0,0],[0,35,0],[35,35,0],[0,70,0],[35,70,0]],destination:[430,160,300,180,0,0],waypoints:[[430,0,360,180,0,0]]}}));
document.querySelector('#start').addEventListener('click',()=>{if(!recipe)return status.textContent='Load or create a recipe first.';if(step>=recipe.path.length)step=0;playing=true;segmentFrom=null;updateSequence();status.textContent=`Moving to ${recipe.path[step].name}…`;});
document.querySelector('#pause').addEventListener('click',()=>{playing=false;status.textContent='Paused. Press Start to continue.';});
document.querySelector('#reset').addEventListener('click',reset);
document.querySelector('#rate').addEventListener('input',e=>{rate=Number(e.target.value);document.querySelector('#rateValue').textContent=`${rate.toFixed(1)}×`;});
document.querySelector('#speed').addEventListener('input',e=>{speed=Number(e.target.value);document.querySelector('#speedValue').textContent=`${speed} mm/s`;});
const loading=document.querySelector('#loading');loading.style.display='none';
document.querySelector('#demo').click();
