'use strict';
const $=id=>document.getElementById(id),canvas=$('camera'),ctx=canvas.getContext('2d');
let state=null,online=false,busy=false,imageBusy=false,lastImage=null,lastReason='';
if(new URLSearchParams(location.search).get('embedded')==='1')document.body.classList.add('embedded');
function message(text,error=false){$('feedback').textContent=text;$('feedback').classList.toggle('error',error);}
async function post(name,body={}){
  if(!online||!state||state.read_only)throw Error('Reconnect to use calibration controls');
  const r=await fetch('/api/calibration/'+name,{method:'POST',headers:{'Content-Type':'application/json','X-Project-Token':state.token},body:JSON.stringify({generation:state.generation,...body}),keepalive:name==='stop'});
  const data=await r.json();if(!r.ok)throw Error(data.error||'Request rejected');return data;
}
async function action(name,body={}){busy=true;render();try{await post(name,body);await poll();}catch(e){message(e.message,true);}finally{busy=false;render();}}
function draw(){ctx.clearRect(0,0,canvas.width,canvas.height);if(lastImage)ctx.drawImage(lastImage,0,0,canvas.width,canvas.height);ctx.strokeStyle='#69e7bd';ctx.lineWidth=2;for(const d of state?.bottle_detections||[]){const b=d.box;ctx.strokeRect(b[0],b[1],b[2]-b[0],b[3]-b[1]);}}
function render(){
  $('connection').textContent=online?'Connected':'Disconnected';if(!state)return;
  const a=state.automatic||{},active=['collecting','fitting'].includes(a.state),writable=online&&!state.read_only&&!busy;
  $('startAutomatic').disabled=!writable||active||!state.allow_motion||!state.capture?.ready||!a.prerequisites?.ready;
  ['autoHeight','autoDiameter'].forEach(id=>$(id).disabled=!writable||active);
  ['stop','autoStop'].forEach(id=>$(id).disabled=!online||!!state.read_only);
  $('targetPoint').textContent=state.target?`Fixed bottle base: x ${state.target.xy[0].toFixed(3)} m, y ${state.target.xy[1].toFixed(3)} m. Place it here before Start.`:'Select the bottle base position on the main control map.';
  $('autoPrerequisites').textContent=a.prerequisites?.reason||'Waiting for the map-point service';
  $('autoProgress').textContent=`${a.reason||state.reason}${a.collected!=null?' · '+a.collected+' / '+a.total+' views':''}`;
  $('pitchHint').textContent=`Starting pitch estimate: ${a.pitch_hint_deg??14}° downward. Actual pitch and mounting height are fitted from the collected views.`;
  $('simulationNotice').hidden=!state.simulated;
  $('simulationTarget').hidden=!state.simulated||!!state.read_only;
  $('simSetTarget').disabled=!writable||active;
  $('autoResult').hidden=a.state!=='complete';
  if(a.state==='complete')$('autoResult').textContent=`${state.simulated?'SIMULATION ONLY.':'Saved for future missions.'} Pitch ${(state.profile.pitch*180/Math.PI).toFixed(1)}° downward; height ${(state.profile.setup.camera_height*100).toFixed(1)} cm. Floor checks: ${state.profile.validation.map(v=>(v.error_m*100).toFixed(1)+' cm').join(' / ')}. ${state.saved||''}`;
  $('detection').textContent=state.detected_count>1?'Ambiguous proposals · capture waits':state.detection_fresh?'Bottle detected':'Waiting for complete bottle view';
  $('imageHelp').textContent=`${state.preview?.fresh?'Live JPEG preview':'Waiting for fresh video'} · detector ${Number.isFinite(state.inference_latency)?state.inference_latency.toFixed(2)+' s':'starting'}${state.detection_error?' · '+state.detection_error:''}`;
  $('health').replaceChildren(...Object.entries({...state.health,exclusive_control:state.ownership}).map(([key,ok])=>{const e=document.createElement('span');e.textContent=`${ok?'●':'○'} ${key}`;e.className=ok?'ok':'';return e;}));
}
async function poll(){try{const r=await fetch('/api/calibration/state',{cache:'no-store'});if(!r.ok)throw Error('Service unavailable');state=await r.json();online=true;if(state.camera){canvas.width=state.camera.width;canvas.height=state.camera.height;}if(state.reason!==lastReason){lastReason=state.reason;if(!busy)message(state.reason);}draw();}catch(e){online=false;message('Connection lost. Motion heartbeat stopped.',true);}render();}
async function imageTick(){if(imageBusy||!online||document.hidden)return;imageBusy=true;const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),2500);let url;try{const r=await fetch('/api/calibration/image.jpg?t='+Date.now(),{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error('No frame');url=URL.createObjectURL(await r.blob());const image=new Image();image.src=url;await image.decode();lastImage=image;$('cameraEmpty').hidden=true;draw();}catch{}finally{clearTimeout(timeout);if(url)URL.revokeObjectURL(url);imageBusy=false;}}
$('simSetTarget').onclick=()=>action('target',{xy:[Number($('simX').value),Number($('simY').value)]});
$('automaticForm').onsubmit=e=>{e.preventDefault();action('start',{bottle_height:Number($('autoHeight').value),bottle_diameter:Number($('autoDiameter').value)});};
$('stop').onclick=$('autoStop').onclick=()=>action('stop');
function leave(){if(state?.motion==='auto')post('stop').catch(()=>{});}
document.addEventListener('visibilitychange',()=>{if(document.hidden)leave();});window.addEventListener('pagehide',leave);window.addEventListener('blur',leave);
setInterval(()=>{if(online&&!document.hidden&&document.hasFocus()&&state?.motion==='auto')post('heartbeat').catch(()=>{});},200);
setInterval(poll,400);setInterval(imageTick,125);poll();
