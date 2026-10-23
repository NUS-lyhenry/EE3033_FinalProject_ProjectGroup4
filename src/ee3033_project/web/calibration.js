'use strict';
const $=id=>document.getElementById(id);
let state=null,online=false,busy=false,box=null,freezeId=null,drag=null,imageBusy=false;
let lastImage=null,rowsKey='',lastReason='',loadedFreezeId=null;
const canvas=$('camera'),ctx=canvas.getContext('2d');
const n=id=>Number($(id).value);
if(new URLSearchParams(location.search).get('embedded')==='1')document.body.classList.add('embedded');
function message(text,error=false){$('feedback').textContent=text;$('feedback').classList.toggle('error',error);}
async function post(name,body={}){
  if(!online||!state||state.read_only)throw Error('Reconnect to the calibration service to use controls');
  const r=await fetch('/api/calibration/'+name,{method:'POST',headers:{'Content-Type':'application/json','X-Project-Token':state.token},body:JSON.stringify({generation:state.generation,...body}),keepalive:name==='stop'});
  const data=await r.json();if(!r.ok||data.success===false)throw Error(data.error||'Request rejected');return data;
}
async function action(name,body={}){busy=true;render();try{const result=await post(name,body);await poll();message(name==='freeze'?'Image frozen. Check the outline and enter measured floor offsets.':state.reason);return result;}catch(e){message(e.message,true);}finally{busy=false;render();}}
function setBox(next){box=next?.map(v=>Math.round(v));['boxLeft','boxTop','boxRight','boxBottom'].forEach((id,i)=>$(id).value=box?.[i]??'');draw();}
function draw(){
  ctx.clearRect(0,0,canvas.width,canvas.height);
  if(lastImage)ctx.drawImage(lastImage,0,0,canvas.width,canvas.height);
  if(box){ctx.strokeStyle=freezeId?'#f5cf79':'#69e7bd';ctx.lineWidth=2;ctx.strokeRect(box[0],box[1],box[2]-box[0],box[3]-box[1]);ctx.setLineDash([5,5]);ctx.beginPath();ctx.moveTo((box[0]+box[2])/2,box[1]);ctx.lineTo((box[0]+box[2])/2,box[3]);ctx.stroke();ctx.setLineDash([]);}
}
function render(){
  $('connection').textContent=online?'Connected':'Disconnected';
  if(!state)return;
  const writable=online&&!state.read_only&&!busy,idle=state.motion==='idle';
  const ready=writable&&idle&&state.ownership&&Object.values(state.health||{}).every(Boolean)&&(state.capture?state.capture.ready:state.stationary?.verified!==false);
  const automatic=state.automatic,autoActive=automatic&&['collecting','fitting'].includes(automatic.state);
  $('automaticPanel').hidden=!automatic;
  if(!automatic){$('manualControls').open=true;$('manualResults').open=true;}
  $('startAutomatic').disabled=!ready||!state.allow_motion||!state.detection_fresh||!automatic?.prerequisites?.ready||autoActive;
  ['autoHeight','autoDiameter','autoDistance'].forEach(id=>$(id).disabled=!writable||autoActive);
  $('autoStop').disabled=!online||!!state.read_only;
  $('autoPrerequisites').textContent=automatic?.prerequisites?.reason||'Automatic collection requires a saved map and localization.';
  $('autoProgress').textContent=automatic?`${automatic.reason}${automatic.collected!=null?' · '+automatic.collected+' / '+automatic.total+' views':''}`:'Waiting to start.';
  $('autoResult').hidden=automatic?.state!=='complete';
  if(automatic?.state==='complete')$('autoResult').textContent=`${state.simulated?'SIMULATION ONLY · Demo profile cannot be loaded by the robot.':'Saved for future missions.'} Floor checks: ${(state.profile?.validation||[]).map(v=>(v.error_m*100).toFixed(1)+' cm').join(' / ')}. ${state.saved||''}`;
  const demo=state.simulation_reference;
  $('preview').hidden=!state.simulated||!!demo;$('stop').disabled=!online||!!state.read_only;
  $('simulationNotice').hidden=!demo||!automatic;
  $('simulationControls').hidden=!demo;
  if(demo){$('simulationReference').textContent=`Position ${demo.scene} / ${demo.total} · Reference forward ${demo.distance.toFixed(3)} m · Left ${demo.lateral.toFixed(3)} m`;$('nextScene').disabled=!writable||!idle;$('fillReference').disabled=!writable||!idle;}
  $('setup').disabled=!ready;
  document.querySelectorAll('[data-move]').forEach(b=>b.disabled=!ready||!state.allow_motion||!state.detection_fresh);
  $('freeze').disabled=!ready||!state.setup||!state.detection_fresh;
  $('capture').disabled=!ready||!state.frozen||loadedFreezeId!==freezeId||!box||($('validation').checked&&!state.profile);
  $('fit').disabled=!ready||state.samples.length<6;
  $('validation').disabled=!state.profile||!idle||!!state.read_only;
  if(!state.profile)$('validation').checked=false;
  const validation=state.profile?.validation||[],passed=validation.filter(v=>v.passed).length;
  $('save').disabled=!ready||validation.length<2||validation.some(v=>!v.passed);
  $('motionReason').textContent=state.motion_reason+(idle&&state.capture?' · '+state.capture.reason:idle&&state.stationary&&!state.stationary.verified?' · '+state.stationary.reason:'');
  $('cameraTitle').textContent=state.frozen?'Frozen · check the bottle outline':'Live · find one bottle';
  $('resume').hidden=!state.frozen;$('resume').disabled=!writable;
  const preview=state.preview;
  $('imageHelp').textContent=state.frozen?'Frozen measurement image. Select Resume live camera to return to video.':preview?`${preview.fresh?'Live JPEG preview':'Waiting for fresh video'} · ${preview.age==null?'no frame yet':preview.age.toFixed(1)+' s old'} · detector ${Number.isFinite(state.inference_latency)?state.inference_latency.toFixed(2)+' s':'starting'}${state.detection_error?' · '+state.detection_error:''}`:automatic?'Views are captured automatically at each robot pose. Keep the bottle fixed.':'The overlay shows the detector box. Freeze a stable image before entering measurements.';
  $('detection').textContent=state.frozen?'Frozen':!state.detection_fresh?'Waiting for a fresh bottle detection':`${state.detected_count} bottle detected`;
  $('boxFields').hidden=!state.frozen;
  $('trainingCount').textContent=`${state.samples.length} / 6`;
  $('validationCount').textContent=`${passed} / 2`;
  $('fitError').textContent=state.profile?state.profile.quality.reprojection_rms_px.toFixed(1):'—';
  if(state.profile){const p=state.profile;$('fitDetails').textContent=`Focal lengths ${p.camera.k[0].toFixed(1)} / ${p.camera.k[4].toFixed(1)} px · pitch ${(p.pitch*180/Math.PI).toFixed(1)}° · yaw ${(p.yaw*180/Math.PI).toFixed(1)}°. Trained forward range ${p.range_m.map(v=>v.toFixed(2)).join('–')} m. Now check two new positions.`;}
  else $('fitDetails').textContent='The fit estimates focal lengths and camera pitch/yaw. It retains the principal point and lens distortion from CameraInfo when available.';
  $('cameraInfo').textContent=state.camera?`${state.camera.width} × ${state.camera.height} · ${state.camera.frame_id} · ${state.camera.source}`:'Waiting for camera geometry…';
  $('health').replaceChildren(...Object.entries({...state.health,exclusive_control:state.ownership}).map(([key,ok])=>{const e=document.createElement('span');e.textContent=`${ok?'●':'○'} ${key.replaceAll('_',' ')}`;e.className=ok?'ok':'';return e;}));
  const key=JSON.stringify([state.samples,validation,ready]);
  if(key!==rowsKey){rowsKey=key;$('sampleRows').replaceChildren();for(const [kind,items] of [['training',state.samples],['validation',validation]])items.forEach((s,index)=>{
    const row=document.createElement('tr');for(const text of [kind==='training'?'Train':'Check',s.distance.toFixed(2)+' m',s.lateral.toFixed(2)+' m',s.error_m==null?'—':`${s.error_m.toFixed(3)} m ${s.passed?'✓':'✕'}`]){const td=document.createElement('td');td.textContent=text;row.append(td);}
    if(s.error_m!=null)row.children[3].className=s.passed?'pass':'fail';
    const td=document.createElement('td'),button=document.createElement('button');button.textContent='Remove';button.setAttribute('aria-label',`Remove ${kind} sample ${index+1}`);button.disabled=!ready;button.onclick=()=>action('remove',{kind,index});td.append(button);row.append(td);$('sampleRows').append(row);
  });}
  $('savedPanel').hidden=!state.saved;
  if(state.saved){const quote=s=>"'"+s.replaceAll("'","'\\''")+"'";$('launchCommand').textContent=demo?'SIMULATION COMPLETE · This demo profile cannot be loaded by the robot.':`ros2 launch ee3033_project assessment.launch.py allow_motion:=true calibration_file:=${quote(state.saved)}`;$('savedPath').textContent=state.saved;$('savedPanel').querySelector('h3').textContent=demo?'Simulation profile saved':'Profile ready for the mission';$('savedPanel').querySelector('p').textContent=demo?'You completed the guided workflow using simulated measurements. Physical calibration requires a live camera and measured bottle.':'Stop this calibration launch with Ctrl+C. Start the assessment console with the saved profile:';$('copyCommand').hidden=!!demo;}
}
async function poll(){
  try{const r=await fetch('/api/calibration/state');if(!r.ok)throw Error('Calibration service unavailable');const next=await r.json();state=next;online=true;
    if(freezeId!==(next.frozen?.id||null)){freezeId=next.frozen?.id||null;setBox(next.frozen?.box||next.box);}
    else if(!freezeId)setBox(next.box);
    if(next.camera&&(canvas.width!==next.camera.width||canvas.height!==next.camera.height)){canvas.width=next.camera.width;canvas.height=next.camera.height;canvas.parentElement.style.aspectRatio=`${canvas.width}/${canvas.height}`;draw();}
    if(next.reason!==lastReason){lastReason=next.reason;if(!busy)message(next.reason);}
  }catch(e){online=false;message('Connection lost. Movement heartbeat has stopped; reconnect before using controls.',true);}
  render();
}
async function imageTick(){
  if(imageBusy||!online||document.hidden)return;
  if(freezeId&&loadedFreezeId===freezeId)return;
  imageBusy=true;const requestFreezeId=freezeId,controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),3000);let url;
  try{
    const response=await fetch('/api/calibration/image.jpg?t='+Date.now(),{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw Error('No fresh camera image');
    url=URL.createObjectURL(await response.blob());const frame=new Image();frame.src=url;await frame.decode();
    if(requestFreezeId===freezeId){lastImage=frame;loadedFreezeId=requestFreezeId;$('cameraEmpty').hidden=true;draw();render();}
  }catch(e){if(!freezeId)$('imageHelp').textContent='Waiting for fresh video · reconnecting automatically…';}
  finally{clearTimeout(timeout);if(url)URL.revokeObjectURL(url);imageBusy=false;}
}
$('automaticForm').onsubmit=e=>{e.preventDefault();action('automatic',{bottle_height:n('autoHeight'),bottle_diameter:n('autoDiameter'),distance:n('autoDistance')});};
$('autoStop').onclick=()=>action('stop');
$('setupForm').onsubmit=e=>{e.preventDefault();action('setup',{bottle_height:n('bottleHeight'),bottle_diameter:n('bottleDiameter'),camera_height:n('cameraHeight'),camera_x:n('cameraX'),camera_y:n('cameraY')});};
document.querySelectorAll('[data-move]').forEach(b=>b.onclick=()=>action('move',{mode:b.dataset.move}));
$('stop').onclick=()=>action('stop');$('freeze').onclick=()=>action('freeze');$('fit').onclick=()=>action('fit');$('save').onclick=()=>action('save');
$('resume').onclick=()=>action('unfreeze');
$('sampleForm').onsubmit=e=>{e.preventDefault();action('sample',{freeze_id:freezeId,box,distance:n('distance'),lateral:n('lateral'),validation:$('validation').checked});};
$('validation').onchange=render;
$('nextScene').onclick=()=>action('scene');
$('fillReference').onclick=()=>{const d=state?.simulation_reference;if(d){$('distance').value=d.distance.toFixed(3);$('lateral').value=d.lateral.toFixed(3);message('Reference measurements filled. Record the frozen image when ready.');}};
['boxLeft','boxTop','boxRight','boxBottom'].forEach(id=>$(id).oninput=()=>setBox(['boxLeft','boxTop','boxRight','boxBottom'].map(n)));
function point(event){const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(canvas.width,(event.clientX-r.left)*canvas.width/r.width)),Math.max(0,Math.min(canvas.height,(event.clientY-r.top)*canvas.height/r.height))];}
canvas.onpointerdown=e=>{if(!freezeId||busy||state?.read_only)return;drag=point(e);canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{if(!drag)return;const p=point(e);setBox([Math.min(drag[0],p[0]),Math.min(drag[1],p[1]),Math.max(drag[0],p[0]),Math.max(drag[1],p[1])]);};
canvas.onpointerup=()=>{drag=null;};canvas.onpointercancel=()=>{drag=null;};
$('copyCommand').onclick=async()=>{try{await navigator.clipboard.writeText($('launchCommand').textContent);message('Launch command copied.');}catch{message('Select and copy the launch command above.');}};
function leave(){if(state?.motion!=='idle')post('stop').catch(()=>{});}
document.addEventListener('visibilitychange',()=>{if(document.hidden)leave();});window.addEventListener('pagehide',leave);window.addEventListener('blur',leave);
setInterval(()=>{if(online&&!document.hidden&&document.hasFocus()&&state?.motion!=='idle')post('heartbeat').catch(()=>{});},200);
setInterval(poll,400);setInterval(imageTick,125);poll();
