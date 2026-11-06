'use strict';
const $=id=>document.getElementById(id);
const ACTIVE=new Set(['PLANNING','SEARCHING','OBSERVING','CANCELING','RETURNING','TURNING','VERIFYING','VERIFYING_HOME','MAPPING','SAVING','CALIBRATING','ACQUIRING','LOCALIZING','RECOVERING','RESETTING']);
const fmt=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';
const duration=s=>{s=Math.max(0,Math.floor(s||0));return `${String(Math.floor(s/60)).padStart(2,'0')}:${String(s%60).padStart(2,'0')}`;};
let snapshot=null,online=false,busy=0,mapData=null,mapImage=null,mapRevision=-1,drag=null,driveTimer=null,driveRunning=false;
let recoveryBusy=false;
let maps=[],gap=null,trial=null,paramTime=0,trialDirty=false,parametersBusy=false,loadedGeneration=null,settingsGeneration=null;
let clearanceDraft={};
let gridDraft={},gridSettingsBusy=false;
let modelLabels=null,labelKey='',labelsBusy=false;
let scanFilter=null,scanFilterDirty=false,scanFilterBusy=false,scanFilterError=null,scanFilterRequest=0;
let slamSettings=null,slamSettingsDirty=false,slamSettingsBusy=false,slamSettingsError=null,slamSettingsRequest=0;
let waypointDraft=null;
let lastSavedMapId=null;
let measuring=false,measurement=null;
let simulationDraft=null,simulationRevision=null;
const PAGES={overview:['Robot overview','Live map, camera and robot status.'],mapping:['Map the environment','Explore automatically, drive manually, or choose a mapping waypoint.'],mission:['Set up the mission','Choose a supported target, establish the start pose, then search and return.'],navigation:['Navigation settings','Configure speed, clearance and grid settings while stopped.'],results:['Trial results','Inspect reported coordinates, return verification and saved trial evidence.']};
PAGES.workflow=['Map, find & return','Build the map, confirm it, set the start, then run the autonomous mission.'];
let currentPage='workflow',workflowSerial=0,visionDirty=false;
const visionFields={camera_height:['visionHeight',1],camera_pitch:['visionPitch',180/Math.PI],camera_yaw:['visionYaw',180/Math.PI],camera_x:['visionX',1],camera_y:['visionY',1],horizontal_fov:['visionFov',180/Math.PI],camera_min_range:['visionMin',1],camera_max_range:['visionMax',1]};
function validCoordinate(target){return !!target&&[target.x,target.y].every(Number.isFinite);}
function targetReported(m){return validCoordinate(m.target)&&m.target_reported!==false;}
function sourceFrameCount(target){
  if(Array.isArray(target?.source_frames))return new Set(target.source_frames.filter(Number.isFinite)).size;
  return [target?.frames,target?.samples,target?.confirmation_frames].find(value=>Number.isInteger(value)&&value>0)??null;
}
function displayTarget(s){
  const m=s.mission||{};
  if(validCoordinate(m.target))return {...m.target,label:m.target.class,
    sigma:m.target.sigma??Math.sqrt(m.target.variance),samples:sourceFrameCount(m.target),
    stable:m.target.measurement_stable??m.target.stable??false,
    confirmed:targetReported(m),accepted:true,display_only:!targetReported(m)};
  const fused=s.target_estimate,preview=s.target_preview;
  const valid=t=>validCoordinate(t)&&Number.isFinite(t.sigma)&&t.sigma>0;
  if(valid(fused)&&!fused.stale)return {...fused,confirmed:false,display_only:true};
  if(valid(preview)&&!preview.stale)return {...preview,confirmed:false,display_only:true};
  const retained=[fused,preview].filter(valid).sort((a,b)=>(b.stamp??0)-(a.stamp??0))[0];
  return retained?{...retained,confirmed:false,display_only:true}:null;
}
function renderResultEvidence(s,estimate,distance){
  const m=s.mission||{},coordinate=validCoordinate(m.target)?m.target:null,reported=targetReported(m);
  const ended=['COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','STOPPED','FAILED'].includes(m.state);
  const detected=!!coordinate||m.target_detected===true||m.target_ever_detected===true;
  const candidate=(s.perception?.detected_candidates??0)>0&&s.perception?.frame_age<=1.5;
  $('resultDetection').textContent=detected?'Target detected':candidate?'Candidate visible':ended?'Target not detected':'Not detected yet';
  const approximate=['best_available','approximate'].includes(coordinate?.quality);
  $('resultTarget').textContent=coordinate?`(${fmt(coordinate.x)}, ${fmt(coordinate.y)}) m${approximate?' · approximate estimate':''}`:'No coordinate reported';
  $('resultCoordinateStatus').textContent=reported?(approximate?'Reported · approximate':'Reported')
    :coordinate?'Accepted estimate · not reported':estimate?'Preview only · not reported':'No coordinate reported';
  const frames=sourceFrameCount(coordinate),sigma=coordinate?.sigma??(Number.isFinite(coordinate?.variance)?Math.sqrt(coordinate.variance):null);
  if(coordinate)$('resultCoordinateStatus').textContent+=`${frames!=null?' · '+frames+' source exposure'+(frames===1?'':'s'):''}${Number.isFinite(sigma)?' · σ '+fmt(sigma)+' m':''}${Number.isFinite(coordinate.age)?' · '+fmt(coordinate.age,1)+' s old':''}${coordinate.stale?' · retained record':''}`;
  const positionConfidence=coordinate?.position_confidence,confidenceRadius=coordinate?.confidence_radius_m??.2;
  $('resultPositionConfidence').textContent=reported&&Number.isFinite(positionConfidence)&&positionConfidence>=0&&positionConfidence<=1
    ?`${Math.round(positionConfidence*100)}% within ${fmt(confidenceRadius)} m${coordinate.stale?' · stale':''}${Number.isFinite(coordinate.age)?' · '+fmt(coordinate.age,1)+' s old':''}`:'Unavailable';
  $('resultReturn').textContent=m.return_verified?`Verified · ${fmt(distance)} m`:m.state==='RETURNING'?'Returning…':m.state==='VERIFYING'?'Checking home position…':ended?'Return not verified':'Pending';
  $('resultStop').textContent=m.return_verified?'Verified position hold':m.state==='VERIFYING'?`Checking ${fmt(m.stationary_check?.duration??0,1)} / ${fmt(m.stationary_required_seconds??.5,1)} s…`:ended?'Not verified':'Pending';
  const estimatedOutcome=approximate&&Number.isFinite(positionConfidence)&&positionConfidence>=0&&positionConfidence<=1
    ?`Complete · estimated confidence ${Math.round(positionConfidence*100)}% within ${fmt(confidenceRadius)} m`:'Complete';
  $('resultOutcome').textContent=m.state==='COMPLETE'&&m.return_verified&&reported?estimatedOutcome
    :m.state==='FOUND_UNLOCALIZED'?'Target found · position unavailable':m.state==='SEARCH_INCOMPLETE'?'Search incomplete'
    :m.state==='NOT_FOUND'?'Search completed · target not found':m.state==='FAILED'?'Failed'
    :m.state==='STOPPED'?(m.internal_error||m.safety_error||m.safety_errors?.length?'Stopped · safety error':'Stopped'):'Trial not complete';
  $('resultReason').textContent=m.internal_error||m.termination_reason||(m.state==='VERIFYING'&&m.stationary_check?.reason)||m.reason||'Completion requires a reported coordinate and verified return.';
  if(coordinate&&approximate)$('resultReason').textContent+=` · Approximate estimate: uncertainty σ ${fmt(sigma)} m; preferred ${fmt(coordinate.preferred_sigma)} m.`;
  else if(Number.isFinite(sigma))$('resultReason').textContent+=` · Position σ ${fmt(sigma)} m.`;
}
function renderVisualDetails(s,estimate){
  const p=s.perception||{},details=s.perception_details||{},d=details.selected||{},last=details.last_detection;
  const motion=d.motion_check||p.motion_check||{},attitude=d.attitude||p.attitude||{};
  const mmse=details.diagnostic?.mmse||{},count=sourceFrameCount(estimate)??mmse.batch_frames??0;
  const single=estimate?.source==='bottom_inverse_single'||estimate?.single_frame===true||mmse.required_frames===1;
  const raw=details.raw_count??p.raw_detected_candidates??p.detected_candidates;
  const eligible=details.in_range_count??p.detected_candidates;
  const fresh=p.ready&&Number.isFinite(p.frame_age)&&p.frame_age<=1.5;
  const limits=s.mission_options||{},near=limits.bottle_min_range??p.camera_model?.min_range??.35,far=limits.bottle_max_range??p.camera_model?.max_range??.95;
  $('visionLimits').textContent=`Active range ${fmt(near*100,0)}–${fmt(far*100,0)} cm · detector threshold ${fmt((p.detector_confidence_threshold??.60)*100,0)}%`;
  const state=estimate?.confirmed?'Reported target':estimate?.accepted?'Accepted estimate':estimate?.stale?'Last known estimate':estimate?.display_only?'Unconfirmed preview':raw>0&&eligible===0?'Outside range':raw>0?'Target visible':fresh?'No target this frame':'Waiting for camera';
  setPill('visionState',state,estimate?.confirmed?'good':estimate||raw>0?'warn':'');
  $('visionMapPosition').textContent=estimate?`(${fmt(estimate.x)}, ${fmt(estimate.y)}) m`:'No map coordinate';
  $('visionMapQuality').textContent=estimate?.confirmed?'Reported target':estimate?.accepted?'Accepted estimate · not reported':estimate?.stale?`Last seen ${fmt(estimate.age,1)} s ago · retained on map`:estimate?.display_only?'Single-frame preview · not confirmed':estimate?'Map-fused estimate · not confirmed':s.pose?'Waiting for a valid coordinate observation':'Set the robot position and heading first';
  $('visionSigma').textContent=estimate?`σ ≈ ${fmt(estimate.sigma*100,1)} cm`:'—';
  $('visionFrames').textContent=single?`${count} source exposure${count===1?'':'s'} · single-frame estimate`:mmse.required_frames?`${mmse.batch_frames??count} / ${mmse.required_frames} valid frames${mmse.updated?' · fusion updated':''}`:`${count} source exposure${count===1?'':'s'}`;
  $('visionConfidence').textContent=Number.isFinite(d.confidence)?`${fmt(d.confidence*100,0)}%`:raw===0?'No target':last?`${fmt(last.confidence*100,0)}% · last seen`:'—';
  $('visionCounts').textContent=raw!=null?`${raw} YOLO box${raw===1?'':'es'} · ${eligible??0} in range · ${p.localized_candidates??0} usable`:'No fresh detection frame';
  $('visionRange').textContent=Number.isFinite(d.range_m)?`${fmt(d.range_m*100,1)} cm`:d.range_state==='too_far'?`>${fmt(far*100,0)} cm`:d.range_state==='below_selected_minimum'?`<${fmt(near*100,0)} cm`:'—';
  $('visionRangeKind').textContent=d.ranging_mode==='near_guess'?'Near-field guess · 12–35 cm prior':d.range_state==='too_far'?'Outside range · direction only':Number.isFinite(d.range_m)?'Original-image bottom-edge estimate':'No range observation';
  const box=d.box;
  $('visionBottom').textContent=box?`${fmt(box[3],1)} px`:'—';
  $('visionBox').textContent=box?`Box ${fmt(box[2]-box[0],1)} × ${fmt(box[3]-box[1],1)} px · original ${d.image_width??640} × ${d.image_height??480}`:'Original image coordinates; not inference pixels';
  const error=d.center_error_px??(box?(box[0]+box[2])/2-(d.image_width??640)/2:null);
  $('visionCenter').textContent=Number.isFinite(error)?`${error>0?'+':''}${fmt(error,1)} px`:'—';
  const centerLimit=32*(d.image_width??640)/640;
  $('visionAlignment').textContent=Number.isFinite(error)?Math.abs(error)<=centerLimit?`Within preferred ±${fmt(centerLimit,0)} px`:'Off-center · bounded correction; uncertainty retained':'Prefer ±32 px at 640 px width; bounded correction';
  $('visionMotion').textContent=motion.samples?'Motion recorded':'Waiting for motion data';
  $('visionMotionDetail').textContent=Number.isFinite(motion.linear_mean)?`${fmt(motion.linear_mean,3)} m/s · yaw ${fmt((motion.angular_mean??0)*180/Math.PI,1)}°/s · span ${fmt((motion.position_span??0)*100,1)} cm`:motion.reason||'Waiting for odometry';
  $('visionPosture').textContent=attitude.valid?'Stable':'Unverified';
  $('visionPostureDetail').textContent=attitude.span?`Roll / pitch span ${attitude.span.map(v=>fmt(v*180/Math.PI,1)+'°').join(' / ')} · no angle correction`:attitude.reason?'Conservative uncertainty · '+attitude.reason:'Stability telemetry is unavailable';
  $('visionTiming').textContent=Number.isFinite(p.frame_age)?`${fmt(p.frame_age,2)} s · ${fmt(p.latency*1000,0)} ms`:'—';
  $('visionFreshness').textContent=raw>0&&details.age!=null?`Detection update ${fmt(details.age,2)} s ago`:details.last_seen_age!=null?`Target last seen ${fmt(details.last_seen_age,1)} s ago`:fresh?'Camera live · no target visible':'Frame unavailable or stale';
  $('visionRobotPose').textContent=s.pose?`(${fmt(s.pose[0])}, ${fmt(s.pose[1])}) m`:'Pose unavailable';
  const covariance=s.amcl_covariance;
  $('visionRobotUncertainty').textContent=covariance?`Position σ ${fmt(Math.sqrt(Math.max(covariance[0],covariance[1]))*100,1)} cm · heading σ ${fmt(Math.sqrt(covariance[2])*180/Math.PI,1)}°`:s.pose?'Capture-time transform available':'Map coordinates need robot localization';
  const reasons=[];
  if(!online)reasons.push('Connection lost; displayed values are retained observations.');
  else if(!fresh)reasons.push('Waiting for a fresh camera frame.');
  if(!s.pose)reasons.push('Set the robot position and heading to place the estimate on the map.');
  if(raw===0)reasons.push('No target above the detector confidence threshold in this frame.');
  if(raw>0&&eligible===0)reasons.push('Visible target is outside the selected detection range; it cannot count as found.');
  if(d.projection_error)reasons.push(d.projection_error);
  if(d.ranging_mode==='near_guess')reasons.push('Bottom is close or cropped; the map shows a conservative near-field guess.');
  if(estimate?.display_only&&!estimate?.accepted)reasons.push('The map marker is a provisional visual estimate; a preview is not a mission coordinate report.');
  if(estimate?.prior_conflict)reasons.push('Map and observation disagree; the estimate is shown but cannot be accepted.');
  if(!reasons.length)reasons.push(estimate?.confirmed?'Target coordinates reported.':estimate?.accepted?'Valid coordinate accepted; bounded centering may still be in progress.':'Waiting for a valid timestamped coordinate observation.');
  $('visionReason').textContent=[...new Set(reasons)].join(' ');
  $('visionReason').classList.toggle('good',!!estimate?.confirmed);
}
const SIMULATION_CAMERA_FORM="<section id=\"simulationCameraPanel\" class=\"panel\"><div class=\"panel-body\"><h3 id=\"cameraMountTitle\">Camera mounting · one-time setup</h3><p id=\"simulationGeometryHelp\" class=\"notice amber\" hidden></p><p id=\"cameraMountHelp\" class=\"muted\">Zero height uses the installed camera TF. Zero FOV uses CameraInfo. Fill measured values only; no bottle collection is required. Angles below are degrees.</p><div class=\"form-grid\">\n<label>Camera height (m)<input id=\"visionHeight\" type=\"number\" min=\"0\" max=\"1.5\" step=\"0.001\" value=\"0\"></label><label>Downward pitch (°)<input id=\"visionPitch\" type=\"number\" min=\"-60\" max=\"60\" step=\"0.5\" value=\"0\"></label><label>Camera yaw (°)<input id=\"visionYaw\" type=\"number\" min=\"-180\" max=\"180\" step=\"0.5\" value=\"0\"></label>\n<label>Forward mount offset (m)<input id=\"visionX\" type=\"number\" min=\"-0.5\" max=\"0.5\" step=\"0.001\" value=\"0\"></label><label>Left mount offset (m)<input id=\"visionY\" type=\"number\" min=\"-0.5\" max=\"0.5\" step=\"0.001\" value=\"0\"></label><label>Explicit horizontal FOV (°; 0 = CameraInfo)<input id=\"visionFov\" type=\"number\" min=\"0\" max=\"160\" step=\"0.5\" value=\"0\"></label>\n<label>Minimum usable range (m)<input id=\"visionMin\" type=\"number\" min=\"0.2\" max=\"1\" step=\"0.05\" value=\"0.35\"></label><label>Maximum usable range (m)<input id=\"visionMax\" type=\"number\" min=\"0.5\" max=\"3\" step=\"0.1\" value=\"2\"></label>\n</div><button id=\"saveVision\" class=\"secondary\">Save camera geometry</button><p id=\"cameraMountSavedHint\" class=\"muted\">Saved settings apply the next time the mission loads.</p></div></section>";
function renderSimulationMount(s){
  if(!physicalCameraMode()){$('simulationCameraPanel')?.remove();return;}
  if(!$('simulationCameraPanel')){
    $('toolsBody').insertAdjacentHTML('beforeend',SIMULATION_CAMERA_FORM);
    Object.values(visionFields).forEach(([id])=>$(id).oninput=()=>{visionDirty=true;});
    $('saveVision').onclick=()=>{
      if(!simulationEditable())return;
      const values=Object.fromEntries(Object.entries(visionFields).map(([name,[id,scale]])=>[name,Number($(id).value)/scale]));
      action('simulation_camera',values).then(response=>{visionDirty=false;if(response.camera)snapshot.simulation_camera=response.camera;render();}).catch(()=>{});
    };
  }
  const mount=s.simulation_camera;
  $('saveVision').disabled=!simulationEditable();$('saveVision').textContent='Apply simulated camera';
  $('cameraMountTitle').textContent='Simulated camera · physical setup';
  $('cameraMountHelp').textContent='Adjust the simulated camera mount and lens. These controls affect the isolated simulation only.';
  $('cameraMountSavedHint').textContent='Apply while the simulation is stopped. No real robot camera settings are changed.';
  $('simulationGeometryHelp').hidden=false;
  $('simulationGeometryHelp').textContent=`Current simulated camera: ${fmt(mount.camera_height,2)} m high, ${fmt(mount.camera_pitch*180/Math.PI,1)}° downward. These controls move the simulated camera itself.`;
  $('visionHeight').min='0.05';
  if(!visionDirty)Object.entries(visionFields).forEach(([name,[id,scale]])=>$(id).value=fmt(mount[name]*scale,3));
}
function arrangeWorkflow(){
  document.querySelector('.sidebar').hidden=true;
  const tools=$('advancedTools'),body=$('toolsBody');
  ['waypointControls','mappingControls','advanced'].forEach(id=>body.append($(id)));
  body.append($('localizationMethod').closest('.field'));
  body.append(document.querySelector('.event-panel'));
  const grid=document.querySelector('.workspace-grid'),stages=document.querySelector('.stage-grid');
  grid.before(stages,$('mapLibrary'));
  $('localizationMethod').value='hybrid';
  const legacy=document.createElement('details'),summary=document.createElement('summary');
  legacy.id='legacyCalibration';summary.textContent='Legacy camera calibration · optional';
  legacy.append(summary,document.querySelector('section[data-pages="calibration"]'));body.append(legacy);
  tools.addEventListener('toggle',()=>{if(!tools.open)releaseDrive();});
}
arrangeWorkflow();
const settingsPanels=[
  ['trialPanel','settingsSpeed'],['clearancePanel','settingsClearance'],
  ['gridSettingsPanel','settingsGrids'],['slamSettingsPanel','settingsMapping'],
  ['scanFilterPanel','settingsScan'],['navigationRecoveryPanel','settingsRecovery']
].map(([id,slot])=>{
  const panel=$(id),home=document.createComment(id);
  panel.before(home);
  return {panel,home,slot:$(slot)};
});

function renderPages(){
  const route=location.hash.replace(/^#\/?/,''),previous=currentPage;currentPage=Object.hasOwn(PAGES,route)?route:'workflow';
  const settingsPage=currentPage==='navigation';
  document.body.dataset.consolePage=currentPage;
  if(previous!==currentPage){
    $('advancedTools').open=currentPage!=='workflow';
    if(currentPage==='calibration')$('legacyCalibration').open=true;
    for(const {panel,home,slot} of settingsPanels){
      if(settingsPage)slot.append(panel);
      else home.after(panel);
    }
    if(settingsPage){
      $('clearancePanel').querySelector('details').open=true;
      $('gridSettingsPanel').querySelector('details').open=true;
    }
  }
  document.querySelectorAll('[data-pages]').forEach(e=>e.hidden=currentPage==='workflow'?false:!e.dataset.pages.split(' ').includes(currentPage));
  $('settingsLayout').hidden=!settingsPage;
  if(currentPage==='workflow'){$('mappingPanel').hidden=snapshot?.phase==='mission';$('legacyCalibration').hidden=true;}
  else $('legacyCalibration').hidden=true;
  $('backWorkflow').hidden=currentPage==='workflow';
  document.querySelectorAll('[data-page-link]').forEach(e=>{if(e.dataset.pageLink===currentPage)e.setAttribute('aria-current','page');else e.removeAttribute('aria-current');});
  $('pageTitle').textContent=PAGES[currentPage][0];$('pageDescription').textContent=PAGES[currentPage][1];
  document.querySelector('.workspace-grid').classList.toggle('mapping-view',currentPage==='mapping');
  document.title=PAGES[currentPage][0]+' · Group 4 Assessment console';
  document.querySelector('.stepper').hidden=currentPage==='calibration';
}
function showPage(page){location.hash='/'+page;render();}
window.addEventListener('hashchange',()=>{releaseDrive();render();window.scrollTo(0,0);});
const trialFields=[['max_linear','trialLinear','m/s'],['max_angular','trialAngular','rad/s'],['lookahead_dist','trialLookahead','m'],['corner_lookahead_dist','trialCornerLookahead','m'],['alignment_angle','trialAlignment','rad'],['goal_timeout','trialTimeout','s']];
const clearanceFields=[...document.querySelectorAll('[data-clearance]')];
const gridFields=[...document.querySelectorAll('[data-grid-setting]')];
function trialValue(key,value,unit){return fmt(value,key==='goal_timeout'?0:2)+' '+unit+(key==='alignment_angle'?` (${fmt(value*180/Math.PI,1)}°)`:'');}
function feedback(message,error=false){$('feedback').textContent=message;$('feedback').classList.toggle('error',error);}
async function get(path){const r=await fetch('/api/'+path);const d=await r.json();if(!r.ok)throw Error(d.error||'Request failed');return d;}
async function send(path,body={}){
  if(!online||!snapshot)throw Error('Reconnect to the robot console before sending controls');
  const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Project-Token':snapshot.token,'X-Control-Generation':String(snapshot.control_generation??0)},body:JSON.stringify(body)});
  const data=await r.json();if(!r.ok||data.success===false)throw Error(data.error||data.message||'Command rejected');return data;
}
async function action(path,body={}){releaseDrive();busy++;render();try{const d=await send(path,body);if(d.message)feedback(d.message);return d;}catch(e){feedback(e.message,true);throw e;}finally{busy--;render();}}
function simulationEditable(){return setupIdle()&&snapshot?.simulated===true&&snapshot?.preview_backend==='main-ros'&&snapshot?.simulation_can_apply===true;}
function physicalCameraMode(){return snapshot?.simulated===true&&snapshot?.preview_backend==='main-ros'&&!!snapshot?.simulation_camera;}
function sceneTransform(scene){
  const [xmin,ymin,xmax,ymax]=scene.bounds,c=$('simulationScene'),padding=24;
  const scale=Math.min((c.width-2*padding)/(xmax-xmin),(c.height-2*padding)/(ymax-ymin));
  const left=(c.width-scale*(xmax-xmin))/2,top=(c.height-scale*(ymax-ymin))/2;
  return {point:([x,y])=>[left+(x-xmin)*scale,top+(ymax-y)*scale],
    world:([x,y])=>[xmin+(x-left)/scale,ymax-(y-top)/scale]};
}
function drawSimulation(){
  const scene=snapshot?.simulation_scenario;if(!scene)return;
  const c=$('simulationScene'),ctx=c.getContext('2d'),t=sceneTransform(scene);
  ctx.clearRect(0,0,c.width,c.height);ctx.fillStyle='#edf3f2';ctx.fillRect(0,0,c.width,c.height);
  ctx.strokeStyle='#243d45';ctx.lineWidth=5;ctx.lineCap='round';
  for(const wall of scene.walls){ctx.beginPath();ctx.moveTo(...t.point(wall[0]));ctx.lineTo(...t.point(wall[1]));ctx.stroke();}
  const dot=(p,color,r,label)=>{const [x,y]=t.point(p);ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();ctx.font='13px sans-serif';ctx.fillText(label,x+10,y-10);};
  dot(scene.start,'#2583ae',7,'Start');dot(scene.target,'#bd7b16',8,'Cup');
  if(simulationDraft){const [x,y]=t.point(simulationDraft);ctx.strokeStyle='#bd7b16';ctx.lineWidth=2;ctx.setLineDash([4,3]);ctx.strokeRect(x-10,y-10,20,20);ctx.setLineDash([]);}
}
function renderSimulation(){
  const s=snapshot,scene=s?.simulation_scenario;
  $('simulationPanel').hidden=!(s?.simulated===true&&s?.preview_backend==='main-ros'&&scene);
  if(!scene)return;
  const editable=simulationEditable();
  for(const id of ['simulationRandomMap','simulationRandomTarget','simulationPlaceTarget','simulationSeed','simulationTargetX','simulationTargetY'])$(id).disabled=!editable;
  if(simulationRevision!==scene.revision){
    simulationRevision=scene.revision;simulationDraft=null;
    $('simulationTargetX').value=fmt(scene.target[0],3);$('simulationTargetY').value=fmt(scene.target[1],3);
  }
  $('simulationSeedStatus').textContent=scene.seed==null?'Default scene':`Seed ${scene.seed}`;
  $('simulationSceneStatus').textContent=!editable?(s.simulation_scenario_reason||'Stop the current run before editing the scene.'):
    simulationDraft?'Target selected. Press Place target to apply it.':'Click the scene or enter coordinates, then press Place target.';
  $('simulationScene').style.cursor=editable?'crosshair':'default';drawSimulation();
}
async function editSimulation(kind){
  if(!simulationEditable())return;
  try{
    const body={action:kind};
    if(kind==='set_target'){
      body.x=Number($('simulationTargetX').value);body.y=Number($('simulationTargetY').value);
      if(!$('simulationTargetX').value.trim()||!$('simulationTargetY').value.trim()||!Number.isFinite(body.x)||!Number.isFinite(body.y))throw Error('Enter finite target coordinates.');
    }else if($('simulationSeed').value.trim()){
      body.seed=Number($('simulationSeed').value);
      if(!Number.isInteger(body.seed)||body.seed<0||body.seed>4294967295)throw Error('Use a whole-number seed from 0 to 4294967295.');
    }
    const response=await action('simulation_scenario',body);
    if(response.scenario){snapshot.simulation_scenario=response.scenario;simulationRevision=null;simulationDraft=null;renderSimulation();}
  }catch(e){feedback(e.message,true);}
}
$('simulationRandomMap').onclick=()=>editSimulation('random_map');
$('simulationRandomTarget').onclick=()=>editSimulation('random_target');
$('simulationPlaceTarget').onclick=()=>editSimulation('set_target');
for(const id of ['simulationTargetX','simulationTargetY'])$(id).oninput=()=>{
  const x=Number($('simulationTargetX').value),y=Number($('simulationTargetY').value);
  simulationDraft=Number.isFinite(x)&&Number.isFinite(y)?[x,y]:null;drawSimulation();
};
$('simulationScene').onclick=e=>{
  if(!simulationEditable())return;
  const c=$('simulationScene'),rect=c.getBoundingClientRect();
  simulationDraft=sceneTransform(snapshot.simulation_scenario).world([(e.clientX-rect.left)*c.width/rect.width,(e.clientY-rect.top)*c.height/rect.height]);
  $('simulationTargetX').value=fmt(simulationDraft[0],3);$('simulationTargetY').value=fmt(simulationDraft[1],3);renderSimulation();
};
function chosenMap(){return maps.find(m=>m.id===$('savedMaps').value);}
function currentSavedMap(){const path=snapshot&&Object.hasOwn(snapshot,'saved_map')?snapshot.saved_map:snapshot?.mission?.saved_map;return maps.find(m=>m.path===path);}
function running(){return snapshot?.stage?.managed?snapshot.stage.running:true;}
function setupIdle(){return online&&!snapshot?.read_only&&!busy&&!snapshot?.stage?.transitioning&&!ACTIVE.has(snapshot?.mission?.state);}
function ready(repeat=false){return online&&(repeat||!snapshot?.assessment?.interrupted&&snapshot?.assessment?.finished==null)&&running()&&snapshot?.health?.Mission?.ok&&snapshot?.mission?.ready!==false&&!snapshot?.control_fault&&!busy&&!snapshot?.stage?.transitioning;}
function missionReadinessHelp(reason){
  if(reason.includes('attitude_reference_missing'))return 'The controller attitude reference is missing. Keep the robot still, choose Capture controller reference under Advanced, then reload the mission and set the start pose again.';
  if(reason.includes('attitude_pivot_missing'))return 'The measured wheel pivot is missing. Save it under Advanced → Attitude localization reference, capture a controller reference, then reload the mission.';
  if(reason.includes('attitude_missing_or_stale'))return 'Fresh controller attitude data is unavailable. Check the robot connection and wait for live attitude readings.';
  if(reason.includes('No AMCL pose'))return 'Set the assigned start pose on the map and wait for localization.';
  if(reason.includes('Localization uncertainty'))return 'Localization uncertainty is too high. Check the robot position and heading on the map, then set the start pose again.';
  if(reason.includes('Nav2 navigation costmap'))return 'The navigation map is still loading. Wait for fresh navigation data.';
  if(reason.includes('No saved map'))return 'No map has been received. Load a reviewed saved map for the mission.';
  if(reason.includes('Lidar stream'))return 'Fresh lidar readings are unavailable. Check the lidar connection.';
  if(reason.includes('Odometry'))return 'Fresh odometry is unavailable. Check the robot connection.';
  if(reason.includes('YOLO stream'))return 'Target detection is not ready. Wait for the camera and detector, and check the selected target class.';
  if(reason.includes('Map-to-base TF'))return 'The robot position on the map is unavailable. Check localization and set the start pose again.';
  return reason||'The robot is not ready. Check the readiness indicators below.';
}
function renderMissionStartReason({active,loaded,managed,labelReady,settingsChanged,validSettings}){
  const s=snapshot,m=s.mission||{},button=$('startMission'),quote=$('missionStartReason');
  const reasons=[];
  if(!online)reasons.push('Disconnected from the robot. Reconnect to continue.');
  else if(active)reasons.push('A task is already running. Wait for it to finish, or choose Stop robot.');
  else if(busy||s.stage?.transitioning)reasons.push('Applying changes. Wait for the current operation to finish.');
  else{
    if(s.phase!=='mission')reasons.push('Confirm a reviewed map and load the mission stage first.');
    else if(!loaded)reasons.push('The mission stage is not running. Reload the map and mission stage.');
    if(!s.allow_motion)reasons.push('Motion is disabled in this console. Open the robot control console.');
    if(s.assessment?.interrupted||s.assessment?.finished!=null)reasons.push('This trial has ended or was interrupted. Reload the map and set the start pose for a new attempt.');
    if(s.control_fault)reasons.push(s.control_fault);
    if(loaded&&!s.health?.Mission?.ok)reasons.push('Fresh mission status is unavailable. Check the robot connection.');
    if(m.ready===false)reasons.push(missionReadinessHelp(m.readiness_reason||''));
    if(!labelReady)reasons.push(modelLabels?.ready?'The selected target is not supported by the detector. Choose a supported target.':'Detector labels are still loading. Wait for the target list.');
    if(settingsChanged)reasons.push('Mission settings have changed. Choose Apply settings & reload mission, then set the start pose again.');
    if(!validSettings)reasons.push('Check the return tolerance, preferred precision and acquisition time. Enter values within the displayed limits.');
    if(managed&&!s.initial_pose_sent&&!m.readiness_reason?.includes('No AMCL pose'))reasons.push('Set the assigned start pose on the map before starting.');
  }
  quote.hidden=button.hidden||!button.disabled;
  quote.replaceChildren(...[...new Set(reasons)].map(reason=>{const p=document.createElement('p');p.textContent=reason;return p;}));
  if(!quote.hidden&&!reasons.length)quote.textContent='Waiting for the robot to become ready. Check the readiness indicators below.';
}
function setPill(id,text,kind=''){const e=$(id);e.textContent=text;e.className='pill '+kind;}
function render(){
  renderPages();
  if(!snapshot)return;
  const s=snapshot,m=s.mission||{},active=ACTIVE.has(m.state),mapping=s.phase==='mapping',idle=setupIdle(),loaded=running(),managed=!!s.stage?.managed;
  $('previewNotice').hidden=!s.simulated;
  $('previewNotice').textContent=s.preview_backend==='main-ros'?`ALGORITHM SIMULATION · No robot is connected. ${s.preview_hint||''}`:'SIMULATED PREVIEW · No robot is connected. Values illustrate the operator workflow.';
  renderSimulation();
  $('stop').disabled=!!s.read_only;
  setPill('connection',online?(s.preview_interactive?'Simulated connection':'Connected'):'Disconnected',online?'good':'bad');
  $('connectionBanner').hidden=online;
  const calibrating=s.phase==='calibration';
  $('phase').textContent=calibrating?'Preparation · Camera setup':mapping?'Stage 1 · Mapping':'Stage 2 · Autonomous';
  const calibration=s.camera_calibration;
  setPill('calibrationBadge',calibration?.ready?'Saved calibration ready':'Not calibrated',calibration?.ready?'good':'');
  $('calibrationSummary').textContent=calibration?.message||'Calibrate once, save the validated profile, and subsequent missions will use it automatically.';
  $('calibrationProfile').textContent=calibration?.path||'';
  const sameCalibrationMap=calibrating&&loaded&&(!chosenMap()||chosenMap().id===s.selected_map?.id);
  $('loadCalibration').textContent=sameCalibrationMap?'Camera setup loaded':calibration?.ready?'Load map & recalibrate':'Load map & camera setup';
  const selectedCalibrationMap=chosenMap(),currentMap=currentSavedMap();
  const mappingNeedsSave=mapping&&loaded&&(!currentMap||selectedCalibrationMap?.id!==currentMap.id);
  $('loadCalibration').disabled=!idle||!managed||sameCalibrationMap||mappingNeedsSave||!selectedCalibrationMap?.reviewed;
  $('calibrationMapChoice').textContent=mappingNeedsSave?'Save the current mapping session and select that new map before calibration.':selectedCalibrationMap?`Selected map: ${selectedCalibrationMap.id}${selectedCalibrationMap.saved_utc?' · saved '+new Date(selectedCalibrationMap.saved_utc).toLocaleString():''}${selectedCalibrationMap.reviewed?'':' · review required'}`:'Select a saved map for automatic calibration.';
  $('calSetPose').disabled=!calibrating||!idle||!loaded||!s.health?.Map?.ok;
  $('calSetBottle').disabled=$('calSetPose').disabled||!s.calibration_live?.ready;
  ['calBottleX','calBottleY','calMapTool'].forEach(id=>$(id).disabled=$('calSetBottle').disabled);
  const bottle=s.calibration_live?.target;
  $('calBottleStatus').textContent=bottle?`Bottle base marker: ${bottle.xy.map(v=>fmt(v)).join(', ')} m. Place the bottle here and keep it fixed. Planned reverse extent: ${fmt(s.calibration_live?.plan?.reverse_extent_m)} m.`:'Select a free map point 0.40–1.50 m from the robot, with clear floor behind the robot for collection.';
  ['calPoseX','calPoseY','calPoseYaw'].forEach(id=>$(id).disabled=$('calSetPose').disabled);
  $('calPoseStatus').textContent=s.calibration_live?.automatic?.prerequisites?.reason||'Load a reviewed map with camera setup first.';
  const frame=$('calibrationFrame'),showCalibration=currentPage==='calibration'&&calibrating&&s.calibration_live?.ready;
  frame.hidden=!showCalibration;
  if(showCalibration&&!frame.hasAttribute('src'))frame.src=`http://${location.hostname}:${s.calibration_port||8780}/?embedded=1`;
  if(!showCalibration&&frame.hasAttribute('src'))frame.removeAttribute('src');
  setPill('battery',s.battery?.voltage!=null?`Battery ${fmt(s.battery.voltage)} V${s.battery.fresh?'':' · stale'}`:'Battery —',s.battery?.fresh?'good':'warn');
  $('battery').title=s.battery?.age!=null?`Measured voltage · ${s.battery.age} s old`:'Waiting for battery telemetry';
  setPill('motion',s.preview_interactive?'Simulated motion only':s.allow_motion?'Motion enabled':'Motion disabled',s.allow_motion?'good':'');
  $('state').textContent=m.state||'WAITING';$('elapsed').textContent=duration(m.elapsed_seconds)+' stage elapsed';
  const missionTiming=s.phase==='mission';
  $('totalElapsed').textContent=duration(missionTiming?m.elapsed_seconds:0);
  $('timerNote').textContent=missionTiming?'Search, localization & return · mapping excluded':'Starts when search begins · mapping excluded';
  const estimate=displayTarget(s),shownTarget=estimate;
  $('target').textContent=shownTarget?`${fmt(shownTarget.x)}, ${fmt(shownTarget.y)} m`:'—, — m';
  $('confidence').textContent=m.target?`${m.target.class||s.mission_options?.target_class||'Target'} · ${m.target.confidence!=null?Math.round(m.target.confidence*100)+'% detector confidence':'detected'}${Number.isFinite(m.target.variance)?' · position σ '+fmt(Math.sqrt(m.target.variance))+' m':''}`:estimate?`Estimated ${estimate.label} · position σ ${fmt(estimate.sigma)} m · ${estimate.samples} observations${estimate.stale?' · last seen '+fmt(estimate.age,1)+' s ago':''}`:'Waiting for estimated or confirmed coordinates';
  $('targetEstimateStatus').textContent=estimate?`Estimated ${estimate.label}: (${fmt(estimate.x)}, ${fmt(estimate.y)}) m · σ ${fmt(estimate.sigma)} m · ${estimate.stale?'last seen '+fmt(estimate.age,1)+' s ago':'updating'} · ${estimate.confirmed?'reported':estimate.accepted?'accepted estimate; not reported':estimate.display_only?'single-frame preview; not confirmed':estimate.prior_conflict?'map / observation conflict; cannot accept':estimate.stable?'stable MMSE estimate':'collecting stable MMSE observations'}`:s.perception?.detected_candidates?'Target visible; position unavailable.':'No estimated target position yet.';
  $('measureMap').disabled=!mapData;
  updateMeasurementStatus();
  if(m.target?.quality==='best_available')$('confidence').textContent+=' · Best stable estimate; preferred precision not reached';
  else if(m.target?.quality==='approximate')$('confidence').textContent+=' · Approximate position estimate';
  const distance=m.return_distance??(s.pose&&m.home?Math.hypot(s.pose[0]-m.home[0],s.pose[1]-m.home[1]):null);
  $('distance').textContent=distance!=null?`${fmt(distance)} m`:'— m';
  $('returnStatus').textContent=m.return_verified?'Home position verified':m.home?`Required tolerance ${fmt(m.return_tolerance??s.mission_options?.return_tolerance??.2)} m`:'Home is recorded at mission Start';
  $('coverage').textContent=m.viewpoints_total?`${Math.round((m.observed_fraction||0)*100)}%`:'—';
  const coverageDetail=m.search_completeness;
  if($('searchLegend'))$('searchLegend').hidden=!m.search_regions||mapping;
  $('coverage').title=coverageDetail?`Actual timestamped camera footprints; observed does not mean empty. ${coverageDetail.missing_cells} unobserved · ${coverageDetail.blocked_cells} blocked · ${coverageDetail.candidate_uncovered_cells} without a viewing position${coverageDetail.map_partial?' · partial map':''}`:'';
  $('waypoints').textContent=m.viewpoints_total?`${m.viewpoints_completed||0} / ${m.viewpoints_total} viewpoints · ${m.failed_viewpoints?.length||0} failed`:'No search plan yet';
  $('robotPose').textContent=s.pose?`Robot (${fmt(s.pose[0])}, ${fmt(s.pose[1])}) · ${fmt(s.pose[2]*180/Math.PI,0)}°`:'Pose unavailable';
  if($('navigationStatus')){const n=s.navigation||{};$('navigationStatus').textContent=n.stage?`${({IDLE:'Ready',ACCEPTED:'Preparing navigation',DISPATCHING:'Planning route',ALIGNING:'Turning toward route',TRACKING:'Following route',FINAL_ALIGN:'Adjusting final heading',STOPPING:'Stopping',REFRESHING:'Checking another route',CANCELING:'Stopping before recovery',REVERSING:'Backing up safely',CANCELED:'Navigation stopped',SUCCEEDED:'Arrived',FAILED:'Navigation failed'})[n.stage]||'Navigation'} · ${n.reason||''}${n.recovery_count?` · Recovery ${n.recovery_count}/2`:''}${n.failure_reason?` · ${n.failure_reason}`:''}`:'';}
  const fault=s.control_fault||s.workflow_fault||s.stage?.fault;
  $('faultBanner').hidden=!fault;$('faultBanner').textContent=fault||'';
  $('reason').textContent=s.stage?.transitioning?'Changing stage; waiting for shutdown and fresh services…':m.readiness_reason&&m.state==='WAITING'?m.readiness_reason:m.reason||'Load a stage to begin.';
  $('health').replaceChildren(...(calibrating?['Lidar','Odometry']:mapping?['Map','Lidar','Odometry','Mission']:['Map','Lidar','Odometry','AMCL','YOLO','Mission']).map(name=>{const h=s.health?.[name],e=document.createElement('span');e.className='check '+(h?.ok?'ok':'bad');e.textContent=`${h?.ok?'●':'○'} ${name}`;e.title=h?`Last received ${h.age} s ago`:'No fresh status';return e;}));
  $('loadMapping').hidden=!managed||currentPage==='workflow';$('loadMapping').disabled=!idle;
  $('start').hidden=!mapping;$('start').textContent='Start automatic mapping';
  $('start').disabled=!mapping||active||!s.allow_motion||!ready();
  if(currentPage==='workflow'&&managed&&!loaded)$('start').disabled=!idle||!s.allow_motion;
  $('saveMap').disabled=!mapping||!loaded||!idle||!s.health?.Map?.ok;
  $('mappingControls').hidden=!mapping;
  $('waypointControls').hidden=!mapping;
  const waypointAllowed=mapping&&loaded&&idle&&s.allow_motion&&ready()&&s.health?.Map?.ok;
  ['waypointX','waypointY','waypointYaw'].forEach(id=>$(id).disabled=!waypointAllowed);
  $('goWaypoint').disabled=!waypointAllowed||!waypointDraft;
  const selected=waypointDraft||(m.mapping_mode==='waypoint'?m.waypoint:null);
  $('waypointStatus').textContent=selected?`${waypointDraft?'Selected waypoint':m.waypoint_verified?'Verified waypoint':'Requested waypoint'} (${fmt(selected[0])}, ${fmt(selected[1])}) m · facing ${fmt(selected[2]*180/Math.PI,0)}°`:'Select a position and final facing.';
  $('mapTitle').textContent=(currentPage==='mapping'||currentPage==='workflow'&&$('advancedTools').open)&&mapping?'Map · drag to set waypoint and facing':(currentPage==='mission'||currentPage==='workflow')&&!mapping?'Map · drag to set initial pose':'On-site map';
  document.querySelectorAll('[data-drive]').forEach(b=>b.disabled=!mapping||!loaded||!idle||!s.allow_motion||!!fault||!s.health?.Lidar?.ok);
  const labelReady=modelLabels?.ready===true&&modelLabels.labels.includes($('targetClass').value);
  const targetChanged=s.phase==='mission'&&$('targetClass').value!==s.mission_options?.target_class;
  const mmseReady=s.mission_options?.preferred_target_sigma!=null||m.preferred_target_sigma!=null;
  const abReady=s.mission_options?.enable_ab_maneuver!=null,acquisitionReady=s.mission_options?.acquisition_seconds!=null;
  const abChanged=abReady&&$('enableAbManeuver').checked!==s.mission_options.enable_ab_maneuver;
  const acquisitionChanged=acquisitionReady&&Number($('acquisitionSeconds').value)!==s.mission_options.acquisition_seconds;
  const rangeChanged=$('targetClass').value==='bottle'&&(Number($('bottleRangeMin').value)/100!==(s.mission_options?.bottle_min_range??.35)||Number($('bottleRangeMax').value)/100!==(s.mission_options?.bottle_max_range??.95));
  const settingsChanged=rangeChanged||targetChanged||s.phase==='mission'&&(mmseReady&&Number($('preferredSigma').value)!==(s.mission_options?.preferred_target_sigma??.1)||Number($('returnTolerance').value)!==(s.mission_options?.return_tolerance??.2)||abChanged||acquisitionChanged);
  const rangeValid=$('targetClass').value!=='bottle'||($('bottleRangeMin').checkValidity()&&$('bottleRangeMax').checkValidity()&&Number($('bottleRangeMin').value)<Number($('bottleRangeMax').value));
  const validSettings=rangeValid&&$('preferredSigma').checkValidity()&&$('preferredSigma').value!==''&&$('returnTolerance').checkValidity()&&$('returnTolerance').value!==''&&$('acquisitionSeconds').checkValidity()&&$('acquisitionSeconds').value!=='';
  const searchEnded=['COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','STOPPED','FAILED'].includes(m.state);
  $('loadMission').hidden=!managed||(currentPage==='workflow'&&!settingsChanged&&!searchEnded);
  $('loadMission').textContent=settingsChanged?'Apply settings & reload mission':searchEnded?'Reload map & set start pose':'Load selected map for mission';
  $('loadMission').disabled=!idle||!validSettings||!labelReady||!chosenMap()?.reviewed;
  const canReuseSavedMap=managed&&chosenMap()?.reviewed&&chosenMap()?.id!==currentSavedMap()?.id;
  $('useSavedMap').hidden=!mapping||!canReuseSavedMap;
  $('useSavedMap').disabled=!idle||!labelReady||!validSettings||!canReuseSavedMap;
  const liveMapNeedsSave=mapping&&loaded&&!currentSavedMap();
  const canConfirmLiveMap=currentPage==='workflow'&&liveMapNeedsSave&&s.health?.Map?.ok;
  $('reviewMap').disabled=!mapping||!idle||!(canConfirmLiveMap||chosenMap()&&(!loaded||chosenMap().id===currentSavedMap()?.id));
  if(currentPage==='workflow'){$('reviewMap').disabled||=!labelReady||!validSettings;$('reviewMap').textContent='Confirm map & prepare mission';$('mapLibrary').hidden=!mapping;$('setup').hidden=mapping;$('startMission').hidden=mapping;}
  else{$('reviewMap').textContent='Mark map reviewed';$('setup').hidden=false;$('startMission').hidden=false;}
  $('savedMaps').disabled=!idle;$('refreshMaps').disabled=!online||!!busy;
  const entry=chosenMap();setPill('mapQuality',entry?(entry.reviewed?'Reviewed · ':'')+entry.quality:'Unverified',entry?.reviewed?'good':'warn');
  $('savedMap').textContent=entry?.path||s.saved_map||m.saved_map||'Every save uses a new directory.';
  $('mapReviewNote').textContent=entry?.quality==='partial'?'Partial map: review accessible and missing regions before use.':'Review records your inspection. Partial and unverified maps remain labelled.';
  if(liveMapNeedsSave)$('mapReviewNote').textContent='Inspect the live map above. Confirm saves this mapping session as a new snapshot, records your review, and prepares the mission. Manual maps remain labelled as snapshots.';
  const poseAllowed=s.phase==='mission'&&idle&&loaded&&(!managed||!m.home||m.started_stamp===0)&&m.state==='WAITING';
  $('setPose').disabled=!poseAllowed||!s.health?.Map?.ok;
  ['poseX','poseY','poseYaw'].forEach(id=>$(id).disabled=!poseAllowed);
  $('poseStatus').textContent=mapping?'Load stage 2 first':s.initial_pose_sent?(s.health?.AMCL?.ok?'Pose sent · localization received':'Pose sent · awaiting localization'):'Set the assigned start before Start';
  ['targetClass','localizationMethod','returnTolerance','preferredSigma'].forEach(id=>$(id).disabled=!idle);
  $('acquisitionSeconds').disabled=!idle||!acquisitionReady;
  $('enableAbManeuver').disabled=!idle||!abReady;
  $('missionAdvanced').title=!abReady||!acquisitionReady?'Robot software update required for advanced mission options':'';
  $('preferredSigma').disabled=!idle||!mmseReady;
  $('preferredSigmaHelp').textContent=mmseReady?'Soft goal, not guaranteed error. A valid approximate coordinate retains its actual uncertainty.':'Robot software update required to apply MMSE precision settings.';
  $('targetClass').disabled=!idle||!modelLabels?.ready;
  $('modelLabelsStatus').textContent=!modelLabels?.ready?(modelLabels?.error||'Reading labels from the configured model…'):targetChanged?'Apply target & reload mission, then set the initial pose again.':`${modelLabels.model} · ${modelLabels.labels.length} supported labels`;
  $('projectionHelp').textContent=$('targetClass').value==='bottle'?'Bottle ranging uses the fixed bottom-edge model; camera-angle calibration is retired.':'Coordinate settings apply when the mission loads.';
  $('startMission').disabled=s.phase!=='mission'||active||!s.allow_motion||!ready()||!labelReady||settingsChanged||!validSettings||(managed&&!s.initial_pose_sent);
  $('retrySearch').hidden=s.phase!=='mission'||!searchEnded;
  $('retrySearch').disabled=!idle||!loaded||!managed||!s.allow_motion||!ready(true)||!labelReady||!s.initial_pose_sent||settingsChanged||!validSettings||!!fault;
  if(s.phase==='mission'&&searchEnded)$('startMission').hidden=true;
  renderMissionStartReason({active,loaded,managed,labelReady,settingsChanged,validSettings});
  $('localizationProgress').hidden=s.phase!=='mission'||!m.target_detected;
  const sites=m.bearing_progress?.sites||[];
  const viewProgress=sites.map((v,i)=>`${'ABC'[i]||'View'}: ${v.ready?'ready':`${v.frames||0}/4 frames`}`).join(' · ');
  const acquisitionElapsed=Number.isFinite(m.localization_elapsed_seconds)?fmt(m.localization_elapsed_seconds,1):'—';
  const acquisitionBudget=m.localization_budget_seconds??m.acquisition_seconds??s.mission_options?.acquisition_seconds??90;
  $('localizationProgress').textContent=`Target acquisition: ${acquisitionElapsed} / ${fmt(acquisitionBudget,0)} s wall time (includes travel) · work ${fmt(m.localization_work_seconds??m.localization_active_seconds??0,1)} s · travel ${fmt(m.localization_travel_seconds??0,1)} s${m.localization_stage?' · '+m.localization_stage:''}${viewProgress?' · '+viewProgress:''}`;
  $('newTrial').disabled=!idle||!managed;$('exportResult').disabled=!online;
  setPill('mappingBadge',mapping?(active?'In progress':loaded?'Ready':'Setup'):entry?.reviewed?'Map reviewed':'Saved map',mapping&&loaded?'good':'');
  setPill('missionBadge',mapping?'Awaiting map':m.state==='COMPLETE'?'Complete':active?'In progress':'Setup',m.state==='COMPLETE'?'good':'');
  const currentStep=mapping?0:m.state==='WAITING'?1:!targetReported(m)?2:3;
  ['stepMap','stepPose','stepSearch','stepReturn'].forEach((id,i)=>{$(id).className=i===currentStep?'current':i<currentStep?'done':'';});
  if(m.state==='COMPLETE'&&m.return_verified)$('stepReturn').className='done';
  renderResultEvidence(s,estimate,distance);
  const p=s.perception;
  if(currentPage==='workflow'){
    document.querySelector('.metrics').hidden=mapping;
    document.querySelector('.camera-panel').hidden=mapping;
    document.querySelector('.workspace-grid').classList.toggle('mapping-view',mapping);
    $('missionPanel').hidden=mapping&&!currentSavedMap()&&!canConfirmLiveMap&&!chosenMap()?.reviewed;
    $('mapLibrary').hidden=!mapping||!maps.length&&!liveMapNeedsSave;
    $('mapLibrary').querySelector('h2').textContent=liveMapNeedsSave?'Finish manual mapping':currentSavedMap()?'Review this trial map':'Use a saved map';
    $('resultsPanel').hidden=!m.target&&!['COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','STOPPED','FAILED'].includes(m.state);
  }
  $('detectorSummary').textContent=p?`${p.target_class} · ${p.detected_candidates??'—'} visible · ${p.localized_candidates??0} localized${p.highest_confidence!=null?' · '+Math.round(p.highest_confidence*100)+'%':''}`:'No mission detector status';
  $('methodSummary').textContent=p?`${p.localization_method==='bottom_inverse'?'Calibrated bottle bottom · timestamped single exposure':p.localization_method==='hybrid'?'Floor + bearing intersection':p.localization_method==='ground_plane'?'Floor-plane projection':'Camera + lidar'} · ${fmt(p.latency,2)} s inference latency`:'Coordinate method is selected before loading stage 2.';
  if(p?.camera_model?.rotation){const f=p.camera_model.rotation;const pitch=Math.atan2(-f[2][2],Math.hypot(f[0][2],f[1][2]))*180/Math.PI;$('methodSummary').textContent+=` · pitch ${fmt(pitch,1)}° · ${p.camera_model.source}`;}
  if(p?.geometry_error)$('methodSummary').textContent=p.geometry_error;
  $('gateReason').textContent=s.velocity_gate?`Motion: ${s.velocity_gate.reason.replaceAll('_',' ')} · requested ${s.velocity_gate.requested?.map(v=>fmt(v)).join(', ')} · output ${s.velocity_gate.output?.map(v=>fmt(v)).join(', ')}`:'Motion diagnostics appear after a stage loads.';
  const recovery=s.recovery,reverseButton=$('reverseRecoveryToggle');
  const reverseValue=recovery?.live_value ?? recovery?.value;
  reverseButton.textContent=typeof reverseValue==='boolean'?`Recovery reverse · ${reverseValue?'ON':'OFF'}`:'Recovery reverse · unavailable';
  reverseButton.setAttribute('aria-pressed',String(reverseValue===true));
  reverseButton.disabled=!online||!!busy||recoveryBusy||!idle||recovery?.editable!==true;
  $('reverseRecoveryStatus').textContent=recoveryBusy?'Saving and verifying…':recovery?.fault||(!recovery?'Robot software update required.':recovery.available?(recovery.synchronized?'Live setting verified · saved across stage reloads.':'Saved and live settings differ; reapply while stopped.'):'Saved for the next stage.');
  const home=m.home,canReturn=Array.isArray(home)&&home.length===3&&home.every(Number.isFinite);
  const researchCancel=s.phase==='mission'&&m.state==='CANCELING'&&m.pending_action==='RESEARCH';
  $('returnHome').disabled=!online||!!s.read_only||!!busy||!loaded||!s.return_home_supported||!canReturn||!Array.isArray(s.pose)||!s.health?.Mission?.ok||s.stage?.transitioning||!s.allow_motion||!!s.control_fault||!!m.navigation_lock_reason||!!s.navigation?.locked||(m.state==='CANCELING'&&!researchCancel)||['SAVING','RETURNING','VERIFYING','VERIFYING_HOME'].includes(m.state);
  $('returnHome').title=!canReturn?'Home is recorded when a task starts':!Array.isArray(s.pose)?'Set the robot current position on the map first':m.state==='RETURNING'?'Already returning to home':'Cancel the current task, then navigate to its recorded home';
  ['profileFast','profileOriginal'].forEach(id=>$(id).disabled=!idle||!loaded||!s.health?.Mission?.ok||(s.phase==='mission'&&m.state!=='WAITING'));
  const physical=physicalCameraMode();
  const calibratedBottle=!physical&&($('targetClass').value?.toLowerCase()==='bottle'||p?.localization_method==='bottom_inverse');
  $('bottleRangePanel').hidden=!calibratedBottle;
  ['bottleRangeMin','bottleRangeMax'].forEach(id=>$(id).disabled=!idle||!!s.read_only);
  $('legacyCalibration').hidden=true;
  $('localizationMethod').closest('.field').hidden=calibratedBottle;
  if(calibratedBottle)$('projectionHelp').textContent='Calibrated bottle ranging with bounded refinement. Close cropped views can provide conservative approximate positions.';
  if(calibratedBottle)$('methodSummary').textContent='Bottle: timestamped single exposure · optional ±32 px centering at 640 px width, up to 2.5 s. Normal motion is allowed; approximate coordinates retain their actual uncertainty.';
  renderSimulationMount(s);
  $('cameraSource').textContent=s.preview_interactive&&s.camera_source?'Simulated camera':s.camera_source==='project'?'Mission detector':s.camera_source==='lab'?'Lab display detector':'No stream';
  $('camera').hidden=!s.camera_source;$('cameraEmpty').hidden=!!s.camera_source;
  $('events').replaceChildren(...(s.events?.length?s.events:[{time:'',state:'WAITING',reason:'Waiting for the controller.'}]).map(e=>{const li=document.createElement('li'),t=document.createElement('time'),st=document.createElement('strong'),r=document.createElement('small');t.textContent=e.time||'';st.textContent=e.state;r.textContent=e.reason||'';li.append(t,st,r);return li;}));
  $('stageHelp').textContent=currentPage==='workflow'?'Automatic mapping returns and saves a new map. After manual mapping, stop driving and confirm the live map to save it and prepare the mission.':managed?'Load mapping, then Start. The saved map can be loaded for stage 2 here.':'Standalone stage: UI stage switching requires assessment.launch.py.';
  $('mappingProgress').hidden=!mapping;
  $('mappingProgress').textContent=`${m.viewpoints_completed||0} goals reached · ${m.failed_viewpoints?.length||0} failed areas skipped · Bounded navigation recovery`;
  renderVisualDetails(s,estimate);
  renderParameters();draw();
}
function renderParameters(){
  renderScanFilter();
  renderSlamSettings();
  renderGridSettings();
  const idle=setupIdle(),fresh=Date.now()-paramTime<8000,mapping=snapshot?.phase==='mapping';
  renderClearance(idle&&fresh&&(mapping||snapshot?.mission?.state==='WAITING'));
  $('trialPanel').hidden=snapshot?.phase==='calibration';
  trialFields.forEach(([key,id,unit])=>{
    const input=$(id),limits=trial?.limits?.[key];
    if(limits){
      input.min=limits.min;input.max=limits.max;input.step=limits.step;
      if(!trialDirty)input.value=trial.values[key];
    }
    input.disabled=!idle||!fresh||!trial||parametersBusy;
    $(id+'Value').textContent=trialValue(key,Number(input.value),unit);
    const live = trial ? trialValue(key,trial.values[key],unit) : 'unavailable';
    const fallback = trial?.defaults?.[key];
    $(id+'Live').textContent = 'Live: '+live+(Number.isFinite(fallback)?' · Default: '+trialValue(key,fallback,unit):'');
  });
  $('applyTrial').disabled=!idle||!fresh||!trial||!trialDirty||parametersBusy;
  $('loadTrialDefaults').disabled=!idle||!fresh||!trial?.defaults||parametersBusy;
  $('trialStatus').textContent=parametersBusy?'Applying and verifying…':trialDirty?'Unapplied changes. Select Apply to save.':trial?.fault||'Applied values are saved for future stages. Adjust while stopped.';
  if(trial?.protection)$('trialProtection').textContent=`Obstacle stop ${fmt(trial.protection.stop_distance)} m · turn clearance ${fmt(trial.protection.rotation_clearance)} m`;
}
function renderGridSettings(){
  const data=snapshot?.grid_settings, editable=setupIdle()&&data?.editable&&!parametersBusy&&!gridSettingsBusy;
  const dirty=Object.keys(gridDraft).length>0;
  const effective={
    mapping_resolution:data?.map_resolution,
    path_search_spacing:data?.effective_path_resolution,
    viewpoint_spacing:data?.active_phase==='mission'?data.active.viewpoint_spacing:null,
    coverage_spacing:data?.active_phase==='mission'?data.active.coverage_spacing:null
  };
  const distance=value=>Number.isFinite(value)?`${Number(value.toFixed(5))} m`:'unavailable';
  for(const input of gridFields){
    const key=input.dataset.gridSetting, limits=data?.limits[key];
    input.min=limits?.min??'';input.max=limits?.max??'';
    if(!Object.hasOwn(gridDraft,key))input.value=data?.selected[key]??'';
    input.disabled=!editable;
    $(input.id+'Live').textContent=`Active: ${distance(effective[key])} · Default: ${distance(data?.defaults[key])}`+
      (limits?` · Range: ${limits.min}–${limits.max} m`:'');
  }
  $('applyGridSettings').disabled=!editable||!dirty||gridFields.some(input=>!input.checkValidity());
  $('gridSettingsStatus').textContent=gridSettingsBusy?'Saving…':dirty?'Unapplied changes. Save to use them on the next stage load.':
    data?'Saved values apply on the next stage load. Rebuild the map to change its SLAM cell size.':'Load settings from the console.';
}
function renderClearance(idle){
  const tuning=gap?.tuning, editable=idle&&!!tuning&&!parametersBusy;
  const requested=tuning?.requested??{...tuning?.defaults,...tuning?.overrides};
  const dirty=Object.keys(clearanceDraft).length>0;
  const distance=value=>Number.isFinite(value)?`${Number(value.toFixed(6))} m`:'unavailable';
  $('clearancePanel').hidden=snapshot?.phase==='calibration';
  for(const input of clearanceFields){
    const key=input.dataset.clearance, limits=tuning?.limits[key];
    input.min=limits?.min??'';input.max=limits?.max??'';
    if(!Object.hasOwn(clearanceDraft,key))input.value=requested[key]??'';
    input.disabled=!editable;
    $(input.id+'Live').textContent=`Live: ${distance(tuning?.live[key])} · Default: ${distance(tuning?.defaults[key])}`+
      (limits?` · Range: ${limits.min}–${limits.max} m`:'');
  }
  $('applyClearance').disabled=!editable||!dirty||clearanceFields.some(input=>!input.checkValidity());
  $('resetClearance').disabled=!editable||(!dirty&&!Object.keys(requested).some(key=>requested[key]!==tuning?.defaults[key]));
  $('clearanceStatus').textContent=parametersBusy?'Applying and verifying…':dirty?'Unapplied changes. Clear a field to use its default.':
    tuning?(gap.fault||'Applied values are saved for future stages.'):'Load a stage to read the live values.';
}
function renderSlamSettings(){
  const data=slamSettings||snapshot?.slam_settings;
  const selected=data?.selected,live=data?.live;
  const editable=setupIdle()&&!parametersBusy&&!slamSettingsBusy&&data?.editable===true;
  const fields=[['occupancy_threshold','slamOccupancyThreshold'],['minimum_pass_through','slamMinimumPassThrough']];
  fields.forEach(([key,id])=>{
    const input=$(id);if(!slamSettingsDirty&&!slamSettingsBusy&&Number.isFinite(selected?.[key]))input.value=String(selected[key]);
    input.disabled=!editable;
  });
  const valid=fields.every(([,id])=>$(id).value.trim()&&$(id).validity.valid);
  $('saveSlamSettings').disabled=!editable||!slamSettingsDirty||!valid;
  $('slamSettingsSelected').textContent=selected?`Saved: threshold ${fmt(selected.occupancy_threshold,2)} · minimum passes ${selected.minimum_pass_through}`:'Saved: unverified';
  $('slamSettingsLive').textContent=slamSettingsError?'Active mapper: unverified':data?.available&&live?
    `Active mapper readback: threshold ${fmt(live.occupancy_threshold,2)} · minimum passes ${live.minimum_pass_through}`:'Active mapper: no verified readback';
  $('slamSettingsStatus').textContent=slamSettingsBusy?'Saving for the next mapping build…':slamSettingsError||data?.readback_error||
    (slamSettingsDirty?'Unsaved changes — press Save for next map.':data?.message||'Reading mapping settings.');
}
async function pollSlamSettings(){
  if(online&&!snapshot?.read_only&&!busy&&!slamSettingsBusy&&!snapshot?.stage?.transitioning){
    const generation=snapshot.control_generation,request=++slamSettingsRequest;
    try{const data=await get('slam_settings');if(generation===snapshot?.control_generation&&request===slamSettingsRequest){slamSettings=data;slamSettingsError=null;}}
    catch(e){if(generation===snapshot?.control_generation&&request===slamSettingsRequest)slamSettingsError=e.message;}
    renderSlamSettings();
  }
  setTimeout(pollSlamSettings,4000);
}
['slamOccupancyThreshold','slamMinimumPassThrough'].forEach(id=>$(id).oninput=()=>{slamSettingsDirty=true;renderSlamSettings();});
$('saveSlamSettings').onclick=async()=>{
  if($('saveSlamSettings').disabled)return;
  const generation=snapshot.control_generation,request=++slamSettingsRequest;
  slamSettingsBusy=true;slamSettingsError=null;renderSlamSettings();
  try{
    const saved=await action('slam_settings',{occupancy_threshold:Number($('slamOccupancyThreshold').value),minimum_pass_through:Number($('slamMinimumPassThrough').value)});
    if(generation===snapshot?.control_generation&&request===slamSettingsRequest){slamSettings=saved;slamSettingsDirty=false;slamSettingsError=null;}
  }catch(e){if(generation===snapshot?.control_generation&&request===slamSettingsRequest)slamSettingsError=e.message;}
  finally{slamSettingsBusy=false;render();}
};
function renderScanFilter(){
  const data=scanFilter||snapshot?.scan_filter;
  const saved=Number.isFinite(data?.value)?data.value:(Number.isFinite(data?.saved)?data.saved:null);
  const editable=setupIdle()&&!parametersBusy&&!scanFilterBusy&&data?.editable===true;
  const input=$('scanFilterRadius');
  if(!scanFilterDirty&&!scanFilterBusy&&saved!==null)input.value=String(Math.round(saved*1000)/10);
  input.disabled=!editable;
  $('saveScanFilter').disabled=!editable||!scanFilterDirty||!input.value.trim()||!input.validity.valid;
  $('scanFilterSaved').textContent=saved===null?'Saved: unverified':`Saved: ${fmt(saved*100,1)} cm`;
  const live=scanFilter?.live_value;
  $('scanFilterLive').textContent=scanFilterError?'Live: unverified':Number.isFinite(live)?`Live: ${fmt(live*100,1)} cm · ${scanFilter.synchronized?'readback matches saved':'readback differs from saved'}`:live===null&&!running()?'Live: no stage loaded':'Live: unavailable';
  $('scanFilterStatus').textContent=scanFilterBusy?'Saving and checking readback…':scanFilterError||data?.fault||(scanFilterDirty?'Unsaved change — press Save.':data?.message||(data?(data.live_value===null&&!running()?'Saved for next stage.':data.synchronized?'Saved and live readback match.':'Waiting for live readback.'):'Reading filter settings.'));
}
async function pollScanFilter(){
  if(online&&!busy&&!scanFilterBusy&&!snapshot?.stage?.transitioning){
    const generation=snapshot.control_generation,request=++scanFilterRequest;
    try{const data=await get('scan_filter');if(generation===snapshot?.control_generation&&request===scanFilterRequest){scanFilter=data;scanFilterError=null;}}
    catch(e){if(generation===snapshot?.control_generation&&request===scanFilterRequest)scanFilterError=e.message;}
    renderScanFilter();
  }
  setTimeout(pollScanFilter,4000);
}
$('scanFilterRadius').oninput=()=>{scanFilterDirty=true;renderScanFilter();};
$('saveScanFilter').onclick=async()=>{
  const input=$('scanFilterRadius');if(input.disabled||!input.value.trim()||!input.validity.valid)return;
  const generation=snapshot.control_generation,request=++scanFilterRequest,radius=Number(input.value)/100;
  scanFilterBusy=true;scanFilterError=null;renderScanFilter();
  try{
    await action('scan_filter',{radius});
    const actual=await get('scan_filter');
    if(generation===snapshot?.control_generation&&request===scanFilterRequest){scanFilter=actual;scanFilterDirty=false;scanFilterError=null;}
  }catch(e){if(generation===snapshot?.control_generation&&request===scanFilterRequest)scanFilterError=e.message;}
  finally{scanFilterBusy=false;render();}
};
async function refreshMaps(preferred){
  const d=await get('maps');maps=d.maps||[];
  const current=$('savedMaps').value,saved=currentSavedMap();
  const previousSaveIndex=maps.findIndex(m=>m.id===lastSavedMapId);
  const newSave=!!saved&&saved.id!==lastSavedMapId&&(previousSaveIndex<0||maps.indexOf(saved)<previousSaveIndex);
  if(newSave)lastSavedMapId=saved.id;
  // A newly saved map wins once. Later polling preserves the operator's choice
  // instead of restoring the map loaded by an earlier stage.
  const selected=[preferred,newSave?saved.id:null,current,saved?.id,snapshot?.selected_map?.id].find(id=>maps.some(m=>m.id===id));
  const options=maps.map(m=>{const o=document.createElement('option');o.value=m.id;o.textContent=`${m.reviewed?'Reviewed':'Unreviewed'} · ${m.quality} · ${m.id.split('/')[0]}${m.saved_utc?' · '+new Date(m.saved_utc).toLocaleString():''}`;return o;});
  if(!options.length){const o=document.createElement('option');o.value='';o.textContent='No saved maps';options.push(o);}
  $('savedMaps').replaceChildren(...options);if(selected)$('savedMaps').value=selected;render();
}
async function loadStage(phase){const body={phase};if(phase==='mission'){Object.assign(body,{map_id:$('savedMaps').value,target_class:$('targetClass').value.trim().toLowerCase(),localization_method:$('localizationMethod').value,return_tolerance:Number($('returnTolerance').value),preferred_target_sigma:Number($('preferredSigma').value)});if(body.target_class==='bottle')Object.assign(body,{bottle_min_range:Number($('bottleRangeMin').value)/100,bottle_max_range:Number($('bottleRangeMax').value)/100});if(snapshot?.mission_options?.enable_ab_maneuver!=null&&snapshot?.mission_options?.acquisition_seconds!=null)Object.assign(body,{enable_ab_maneuver:$('enableAbManeuver').checked,acquisition_seconds:Number($('acquisitionSeconds').value)});}const workflow=currentPage==='workflow';await action('stage',body);if(!workflow)showPage(phase);}
$('loadMapping').onclick=()=>loadStage('mapping').catch(()=>{});$('loadMission').onclick=()=>loadStage('mission').catch(()=>{});
$('useSavedMap').onclick=()=>loadStage('mission').catch(()=>{});
$('start').onclick=async()=>{const serial=++workflowSerial;try{
  if(currentPage==='workflow'&&snapshot?.stage?.managed&&!running()){
    await loadStage('mapping');const deadline=Date.now()+30000;
    while(!ready()){if(serial!==workflowSerial||!online)throw Error('Mapping start canceled');if(Date.now()>deadline)throw Error('Mapping is still preparing; inspect readiness and press Start again');await new Promise(r=>setTimeout(r,150));}
  }
  if(serial===workflowSerial)await action('start');
}catch(e){feedback(e.message,true);}};$('startMission').onclick=()=>action('start').catch(()=>{});
$('returnHome').onclick=async()=>{workflowSerial++;releaseDrive();try{await action('return_home');}catch(_){} };
$('stop').onclick=async()=>{workflowSerial++;releaseDrive();try{const d=await send('stop');feedback(d.message||'Stop requested');}catch(e){feedback(e.message,true);}};
async function saveCurrentMap(){
  const d=await action('save_map');
  // The save response is authoritative before the next state poll arrives.
  snapshot.saved_map=d.path||d.map?.path;
  await refreshMaps(d.map?.id||maps.find(m=>m.path===d.path)?.id);
  return d.map?.id||currentSavedMap()?.id;
}
$('saveMap').onclick=()=>saveCurrentMap().catch(()=>{});
$('refreshMaps').onclick=()=>refreshMaps().catch(e=>feedback(e.message,true));$('savedMaps').onchange=render;
$('reviewMap').onclick=async()=>{
  const serial=workflowSerial,workflow=currentPage==='workflow';
  busy++;releaseDrive();render();
  try{
    let mapId=$('savedMaps').value;
    if(workflow&&snapshot?.phase==='mapping'&&running()&&!currentSavedMap())mapId=await saveCurrentMap();
    if(serial!==workflowSerial)return;
    if(!mapId)throw Error('Save the current map before preparing the mission.');
    await action('review_map',{map_id:mapId});
    if(serial!==workflowSerial)return;
    await refreshMaps(mapId);
    if(workflow&&serial===workflowSerial)await loadStage('mission');
  }catch(e){feedback(e.message,true);}finally{busy--;render();}
};
$('calSetBottle').onclick=()=>action('calibration_target',{x:Number($('calBottleX').value),y:Number($('calBottleY').value),map_id:snapshot.selected_map?.id}).catch(()=>{});
$('calSetPose').onclick=()=>action('initial_pose',{x:Number($('calPoseX').value),y:Number($('calPoseY').value),yaw:Number($('calPoseYaw').value)*Math.PI/180}).catch(()=>{});
$('setPose').onclick=()=>action('initial_pose',{x:Number($('poseX').value),y:Number($('poseY').value),yaw:Number($('poseYaw').value)*Math.PI/180}).catch(()=>{});
function readWaypointDraft(){const fields=['waypointX','waypointY','waypointYaw'].map(id=>$(id).value);const values=fields.map(Number);waypointDraft=fields.every(v=>v!=='')&&values.every(Number.isFinite)?[values[0],values[1],values[2]*Math.PI/180]:null;render();}
['waypointX','waypointY','waypointYaw'].forEach(id=>$(id).oninput=readWaypointDraft);
$('goWaypoint').onclick=()=>{if(waypointDraft)action('map_waypoint',{x:waypointDraft[0],y:waypointDraft[1],yaw:waypointDraft[2]}).then(()=>{waypointDraft=null;render();}).catch(()=>{});};
$('retrySearch').onclick=()=>action('retry_search').catch(()=>{});
$('newTrial').onclick=()=>action('new_trial').then(()=>showPage('workflow')).catch(()=>{});$('localizationMethod').onchange=render;
$('loadCalibration').onclick=()=>action('stage',{phase:'calibration',map_id:chosenMap()?.reviewed?chosenMap().id:undefined}).then(()=>showPage('calibration')).catch(()=>{});
$('targetClass').onchange=render;
['preferredSigma','returnTolerance','acquisitionSeconds','bottleRangeMin','bottleRangeMax'].forEach(id=>$(id).oninput=render);$('enableAbManeuver').onchange=render;
$('exportResult').onclick=async()=>{try{const d=await get('result'),url=URL.createObjectURL(new Blob([JSON.stringify(d,null,2)+'\n'],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=(d.assessment?.id||'assessment')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(e){feedback(e.message,true);}};
$('reverseRecoveryToggle').onclick=async()=>{
  const data=snapshot?.recovery,value=data?.live_value ?? data?.value;
  if(typeof value!=='boolean')return;
  recoveryBusy=true;render();
  try{
    const saved=await action('recovery',{enabled:!value});
    const actual=await get('recovery');
    if(actual.value!==!value||(actual.available&&actual.live_value!==!value))throw Error('Recovery reverse readback mismatch');
    snapshot.recovery=actual;feedback(saved.message||'Recovery reverse saved');
  }catch(e){feedback(e.message,true);}finally{recoveryBusy=false;render();}
};
trialFields.forEach(([,id])=>$(id).oninput=()=>{trialDirty=true;renderParameters();});
$('loadTrialDefaults').onclick=()=>{
  for(const [key,id] of trialFields)$(id).value=trial.defaults[key];
  trialDirty=true;
  renderParameters();
};
$('applyTrial').onclick=async()=>{parametersBusy=true;try{trial=await action('trial',Object.fromEntries(trialFields.map(([k,id])=>[k,Number($(id).value)])));trialDirty=false;paramTime=Date.now();}catch(_){}finally{parametersBusy=false;render();}};
for(const [id,profile] of [['profileOriginal','original'],['profileFast','fast']])$(id).onclick=async()=>{parametersBusy=true;try{trial=await action('navigation',{profile});trialDirty=false;paramTime=Date.now();trialFields.forEach(([name,field])=>$(field).value=trial.values[name]);$('navigationProfileStatus').textContent=`Applied and verified: ${profile} · ${fmt(trial.values.max_linear)} m/s · ${fmt(trial.values.max_angular)} rad/s`;}catch(_){}finally{parametersBusy=false;render();}};
for(const input of clearanceFields)input.oninput=()=>{
  clearanceDraft[input.dataset.clearance]=input.value;
  renderParameters();
};
for(const input of gridFields)input.oninput=()=>{
  gridDraft[input.dataset.gridSetting]=input.value;
  renderGridSettings();
};
$('applyGridSettings').onclick=async()=>{
  gridSettingsBusy=true;
  try{
    const tuning=Object.fromEntries(Object.entries(gridDraft).map(([key,value])=>[key,Number(value)]));
    snapshot.grid_settings=await action('grid_settings',tuning);
    gridDraft={};
  }catch(_){}finally{gridSettingsBusy=false;render();}
};
async function applyClearance(tuning){
  parametersBusy=true;
  try{
    gap=await action('gap',{tuning});
    clearanceDraft={};
    paramTime=Date.now();
    trial=await get('trial');
  }catch(_){}finally{parametersBusy=false;render();}
}
$('applyClearance').onclick=()=>{
  const tuning={...(gap.tuning.requested??gap.tuning.overrides)};
  for(const [key,value] of Object.entries(clearanceDraft)){
    if(value==='')delete tuning[key];
    else tuning[key]=Number(value);
  }
  applyClearance(tuning);
};
$('resetClearance').onclick=()=>applyClearance({});
async function pollParameters(){
  if(online&&snapshot?.health?.Mission?.ok&&running()&&!busy&&!parametersBusy&&!snapshot.stage?.transitioning&&snapshot.phase!=='calibration'){const generation=snapshot.control_generation;try{const g=await get('gap'),t=await get('trial');if(generation===snapshot.control_generation){gap=g;trial=t;paramTime=Date.now();if(t)trialFields.forEach(([k,id])=>{if(!trialDirty)$(id).value=t.values[k];});$('navigationProfileStatus').textContent=`Live limits: ${fmt(t.values.max_linear)} m/s · ${fmt(t.values.max_angular)} rad/s. Fast is a trial profile pending physical validation.`;}}catch(_){paramTime=0;}renderParameters();}
  setTimeout(pollParameters,4000);
}
async function poll(){
  try{const s=await get('state');snapshot=s;online=true;
    if(loadedGeneration!==s.control_generation){loadedGeneration=s.control_generation;gap=trial=null;scanFilter=null;scanFilterDirty=false;scanFilterError=null;scanFilterRequest++;slamSettings=null;slamSettingsDirty=false;slamSettingsError=null;slamSettingsRequest++;paramTime=0;trialDirty=false;clearanceDraft={};gridDraft={};waypointDraft=null;['waypointX','waypointY','waypointYaw'].forEach(id=>$(id).value='');releaseDrive();}
    if(settingsGeneration!==s.stage?.phase){settingsGeneration=s.stage?.phase;if(s.mission_options){$('targetClass').value=s.mission_options.target_class;$('localizationMethod').value=s.mission_options.localization_method;$('returnTolerance').value=s.mission_options.return_tolerance;$('preferredSigma').value=s.mission_options.preferred_target_sigma??.23;$('enableAbManeuver').checked=s.mission_options.enable_ab_maneuver===true;$('acquisitionSeconds').value=String(s.mission_options.acquisition_seconds??90);$('bottleRangeMin').value=String(100*(s.mission_options.bottle_min_range??.35));$('bottleRangeMax').value=String(100*(s.mission_options.bottle_max_range??.95));}}
    if(s.map_revision!==mapRevision){mapRevision=s.map_revision;renderMap(await get('map'));}
    render();
  }catch(e){online=false;releaseDrive();if(snapshot)render();else{$('connection').textContent='Disconnected';$('connectionBanner').hidden=false;}feedback(e.message,true);}
  setTimeout(poll,500);
}
async function pollMaps(){if(online&&!busy)try{await refreshMaps();}catch(_){}setTimeout(pollMaps,5000);}
async function pollLabels(){
  if(online&&!labelsBusy){labelsBusy=true;try{
    const data=await get('labels');modelLabels=data;
    const key=JSON.stringify([data.model,data.labels]);
    if(key!==labelKey){labelKey=key;const current=$('targetClass').value||snapshot?.mission_options?.target_class||'bottle';
      const placeholder=document.createElement('option');placeholder.value='';placeholder.textContent=data.ready?'Select a target class':'Model labels unavailable';
      const options=(data.labels||[]).map(label=>{const option=document.createElement('option');option.value=label;option.textContent=label;return option;});
      $('targetClass').replaceChildren(placeholder,...options);
      if(data.labels?.includes(current))$('targetClass').value=current;
    }
  }catch(e){modelLabels={ready:false,labels:[],error:e.message};}finally{labelsBusy=false;render();}}
  setTimeout(pollLabels,5000);
}
function cameraTick(){if(online&&snapshot?.camera_source)$('camera').src='/api/image.jpg?t='+Date.now();setTimeout(cameraTick,800);}
function releaseDrive(){if(driveTimer)clearInterval(driveTimer);driveTimer=null;if(driveRunning){driveRunning=false;send('teleop',{linear:0,angular:0}).catch(()=>{});}}
for(const button of document.querySelectorAll('[data-drive]')){button.onpointerdown=e=>{if(button.disabled)return;e.preventDefault();releaseDrive();button.setPointerCapture(e.pointerId);const [linear,angular]=button.dataset.drive.split(',').map(Number);driveRunning=true;const tick=()=>send('teleop',{linear,angular}).catch(e=>{releaseDrive();feedback(e.message,true);});tick();driveTimer=setInterval(tick,100);};button.onpointerup=button.onpointercancel=button.onlostpointercapture=releaseDrive;}
window.addEventListener('blur',releaseDrive);document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseDrive();});
function point(x,y){const [ox,oy,a]=mapData.origin,dx=x-ox,dy=y-oy;return[(Math.cos(a)*dx+Math.sin(a)*dy)/mapData.resolution,mapData.height-(-Math.sin(a)*dx+Math.cos(a)*dy)/mapData.resolution];}
function worldPoint(x,y){const [ox,oy,a]=mapData.origin,dx=x*mapData.resolution,dy=(mapData.height-y)*mapData.resolution;return[ox+Math.cos(a)*dx-Math.sin(a)*dy,oy+Math.sin(a)*dx+Math.cos(a)*dy];}
function updateMeasurementStatus(){
  const value=drag?.mode==='measure'?{start:worldPoint(...drag.start),end:worldPoint(...drag.end)}:measurement;
  $('measureMap').setAttribute('aria-pressed',String(measuring));
  $('measureMap').textContent=measuring?'Done measuring':'Measure distance';
  $('clearMeasurement').disabled=!measurement&&!drag;
  $('map').parentElement.classList.toggle('measuring',measuring);
  const distance=value?.end?Math.hypot(value.end[0]-value.start[0],value.end[1]-value.start[1]):null;
  $('measurementStatus').textContent=distance!=null?`Distance: ${fmt(distance)} m · map resolution ${fmt(mapData.resolution)} m/cell`:measuring?(measurement?'Select the second point.':'Click two points, or drag, to measure distance.'):'Measure distances on the saved map.';
  if(distance!=null)$('measurementStatus').dataset.distance=String(distance);else delete $('measurementStatus').dataset.distance;
}
$('measureMap').onclick=()=>{measuring=!measuring;drag=null;updateMeasurementStatus();draw();};
$('clearMeasurement').onclick=()=>{measurement=null;drag=null;updateMeasurementStatus();draw();};
function draw(){if(!mapData||!mapImage||!snapshot)return;const canvas=$('map'),ctx=canvas.getContext('2d'),wrap=canvas.parentElement;if(wrap.clientWidth<1||wrap.clientHeight<1)return;const scale=Math.min((wrap.clientWidth-28)/mapData.width,(wrap.clientHeight-28)/mapData.height);canvas.width=mapData.width*3;canvas.height=mapData.height*3;canvas.style.width=mapData.width*scale+'px';canvas.style.height=mapData.height*scale+'px';ctx.scale(3,3);ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImage,0,0);
  const regions=snapshot.mission?.search_regions;
  if(snapshot.phase==='mission'&&regions&&regions.width===mapData.width&&regions.height===mapData.height&&regions.resolution===mapData.resolution&&regions.origin.every((v,i)=>Math.abs(v-mapData.origin[i])<1e-6)){
    for(const [kind,color] of [['observed','rgba(64,170,115,.45)'],['missing','rgba(245,186,65,.65)'],['blocked','rgba(225,74,74,.75)']]){
      ctx.fillStyle=color;for(const cell of regions[kind]||[]){const c=cell%regions.width,r=Math.floor(cell/regions.width);ctx.fillRect(c-.5,mapData.height-r-1.5,2,2);}
    }
  }

  function line(points,color,dash=[]){if(!points?.length)return;ctx.beginPath();ctx.setLineDash(dash);points.forEach((p,i)=>{const [x,y]=point(...p);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.strokeStyle=color;ctx.lineWidth=.8;ctx.stroke();ctx.setLineDash([]);}
  function dot(p,color,r=2){if(!p)return;const[x,y]=point(...p);ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();}
  line(snapshot.route,'#bca6d8',[1.5,2]);line(snapshot.travelled_path,'#408ee0');line(snapshot.path,'#52cfaa');line(snapshot.local_path,'#d79727',[1,1]);const m=snapshot.mission;dot(m.home,'#ce92df',2.3);if(m.target)dot([m.target.x,m.target.y],'#ecba59',2.8);
  const estimate=displayTarget(snapshot);
  delete canvas.dataset.targetEstimate;delete canvas.dataset.targetBearingOnly;
  if(estimate){
    const [rawX,rawY]=point(estimate.x,estimate.y),pad=8/scale;
    const x=Math.max(pad,Math.min(rawX,mapData.width-pad)),y=Math.max(pad,Math.min(rawY,mapData.height-pad));
    const outside=rawX<0||rawX>mapData.width||rawY<0||rawY>mapData.height;
    const radius=Math.min(estimate.sigma/mapData.resolution,2*Math.hypot(mapData.width,mapData.height));
    const stale=estimate.stale||!online;
    canvas.dataset.targetEstimate=JSON.stringify({x:estimate.x,y:estimate.y,sigma:estimate.sigma,displayOnly:!!estimate.display_only,stale,outside});
    ctx.save();ctx.globalAlpha=stale ? .45 : 1;
    if(!outside){ctx.beginPath();ctx.arc(x,y,radius,0,Math.PI*2);ctx.fillStyle='rgba(222,151,45,.13)';ctx.fill();ctx.setLineDash([5/scale,4/scale]);ctx.strokeStyle='#b57117';ctx.lineWidth=1.4/scale;ctx.stroke();}
    ctx.setLineDash([]);ctx.beginPath();ctx.arc(x,y,6/scale,0,Math.PI*2);ctx.fillStyle=estimate.confirmed?'#d29124':'#fff';ctx.fill();ctx.strokeStyle='#a95d09';ctx.lineWidth=2/scale;ctx.stroke();
    ctx.beginPath();ctx.moveTo(x-9/scale,y);ctx.lineTo(x+9/scale,y);ctx.moveTo(x,y-9/scale);ctx.lineTo(x,y+9/scale);ctx.lineWidth=1.4/scale;ctx.stroke();
    ctx.font=`${12/scale}px sans-serif`;
    const label=`${outside?'Off-map ':''}${estimate.confirmed?'Accepted':stale?'Last seen':estimate.display_only?'Preview':'Estimate'} ${estimate.label} (${fmt(estimate.x)}, ${fmt(estimate.y)}) · σ ${fmt(estimate.sigma)} m`;
    const textWidth=ctx.measureText(label).width,tx=Math.max(2/scale,Math.min(x+12/scale,mapData.width-textWidth-4/scale)),ty=Math.max(16/scale,Math.min(y-12/scale,mapData.height-4/scale));
    ctx.fillStyle='rgba(255,255,255,.94)';ctx.fillRect(tx-3/scale,ty-13/scale,textWidth+6/scale,17/scale);
    ctx.fillStyle='#754306';ctx.fillText(label,tx,ty);ctx.restore();
  }
  const currentDetection=snapshot.perception_details?.selected;
  if(currentDetection&&Number.isFinite(currentDetection.bearing)&&currentDetection.origin?.every(Number.isFinite)&&!Number.isFinite(currentDetection.x)){
    const distance=snapshot.mission_options?.bottle_max_range??.95;
    const origin=currentDetection.origin,end=[origin[0]+distance*Math.cos(currentDetection.bearing),origin[1]+distance*Math.sin(currentDetection.bearing)];
    line([origin,end],'#b57117',[2,2]);canvas.dataset.targetBearingOnly='true';
    const [x,y]=point(...end);ctx.save();ctx.font=`${11/scale}px sans-serif`;ctx.fillStyle='#855211';
    ctx.fillText(currentDetection.search_eligible===false?'Outside range · direction only':'Direction only · no map position',Math.max(0,Math.min(x,mapData.width-45)),Math.max(5,y));ctx.restore();
  }
  if(currentPage==='calibration'){const c=snapshot.calibration_live;if(c?.plan)line([snapshot.pose,...c.plan.goals.map(g=>g.pose)].filter(Boolean),'#ecba59',[2,2]);if(c?.target){dot(c.target.xy,'#ffba5a',2.8);const [x,y]=point(...c.target.xy);ctx.fillStyle='#50340b';ctx.font='3px sans-serif';ctx.fillText('Bottle',x+3,y-3);}}
  const waypoint=waypointDraft||(m.mapping_mode==='waypoint'?m.waypoint:null);
  if(waypoint){const[x,y]=point(...waypoint),a=-(waypoint[2]-mapData.origin[2]);ctx.save();ctx.translate(x,y);ctx.rotate(a);ctx.beginPath();ctx.arc(0,0,1.6,0,Math.PI*2);ctx.moveTo(0,0);ctx.lineTo(7,0);ctx.lineTo(4,-1.5);ctx.moveTo(7,0);ctx.lineTo(4,1.5);ctx.strokeStyle='#ecba59';ctx.lineWidth=.7;ctx.stroke();ctx.restore();}
  if(snapshot.pose){const[x,y]=point(...snapshot.pose);ctx.save();ctx.translate(x,y);ctx.rotate(-(snapshot.pose[2]-mapData.origin[2]));const radius=(snapshot.footprint_radius??.15)/mapData.resolution;ctx.beginPath();ctx.arc(0,0,radius,0,Math.PI*2);ctx.fillStyle='rgba(93,186,236,.18)';ctx.fill();ctx.strokeStyle='#2583bd';ctx.lineWidth=.5;ctx.stroke();const body=snapshot.robot_geometry;ctx.fillStyle='#5dbaec';if(body?.length&&body?.width){const length=body.length/mapData.resolution,width=body.width/mapData.resolution;ctx.fillRect(-length/2,-width/2,length,width);ctx.strokeStyle='#176c9e';ctx.lineWidth=.6;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(length/2,0);ctx.stroke();}else{ctx.beginPath();ctx.moveTo(radius,0);ctx.lineTo(-radius*.55,-radius*.55);ctx.lineTo(-radius*.55,radius*.55);ctx.closePath();ctx.fill();}ctx.restore();}
  const measured=drag?.mode==='measure'?{start:worldPoint(...drag.start),end:worldPoint(...drag.end)}:measurement;
  if(measured){dot(measured.start,'#9a5a10',1.6);if(measured.end){line([measured.start,measured.end],'#9a5a10',[1,1]);dot(measured.end,'#9a5a10',1.6);const a=point(...measured.start),b=point(...measured.end);ctx.fillStyle='#583308';ctx.font='3.3px sans-serif';ctx.fillText(`${fmt(Math.hypot(measured.end[0]-measured.start[0],measured.end[1]-measured.start[1]))} m`,(a[0]+b[0])/2+2,(a[1]+b[1])/2-2);}}
  if(drag&&drag.mode!=='measure'){ctx.beginPath();ctx.moveTo(...drag.start);ctx.lineTo(...drag.end);ctx.strokeStyle='#ecba59';ctx.stroke();}
  updateMeasurementStatus();
}
function renderMap(m){measurement=null;drag=null;mapData=m;mapImage=null;$('mapEmpty').hidden=!!m;if(!m){$('map').getContext('2d').clearRect(0,0,$('map').width,$('map').height);return;}$('mapScale').textContent=`${fmt(m.width*m.resolution,1)} × ${fmt(m.height*m.resolution,1)} m · ${fmt(m.resolution)} m/cell`;mapImage=document.createElement('canvas');mapImage.width=m.width;mapImage.height=m.height;const ctx=mapImage.getContext('2d'),pixels=ctx.createImageData(m.width,m.height);for(let r=0;r<m.height;r++)for(let c=0;c<m.width;c++){const v=m.cells[r*m.width+c],i=((m.height-1-r)*m.width+c)*4;pixels.data.set([...(v<0?[48,65,75]:v>=50?[14,29,36]:[196,210,211]),255],i);}ctx.putImageData(pixels,0,0);draw();}
function canvasPosition(e){const r=$('map').getBoundingClientRect();return[(e.clientX-r.left)/r.width*mapData.width,(e.clientY-r.top)/r.height*mapData.height];}
$('map').onpointerdown=e=>{const mode=measuring?'measure':(currentPage==='mapping'||currentPage==='workflow'&&$('advancedTools').open)&&!$('waypointX').disabled?'waypoint':currentPage==='calibration'&&!$('calSetBottle').disabled?($('calMapTool').value==='bottle'?'calBottle':'calPose'):(currentPage==='mission'||currentPage==='workflow')&&!$('setPose').disabled?'pose':null;if(!mapData||!mode)return;e.preventDefault();$('map').setPointerCapture(e.pointerId);const p=canvasPosition(e);drag={start:mode==='measure'&&measurement&&!measurement.end?point(...measurement.start):p,end:p,press:p,continued:mode==='measure'&&!!measurement&&!measurement.end,mode};draw();};
$('map').onpointermove=e=>{if(drag){drag.end=canvasPosition(e);draw();}};
$('map').onpointerup=()=>{if(!drag)return;if(drag.mode==='measure'){const moved=Math.hypot(drag.end[0]-drag.press[0],drag.end[1]-drag.press[1])*$('map').getBoundingClientRect().width/mapData.width;measurement={start:worldPoint(...drag.start),end:moved>4||drag.continued?worldPoint(...drag.end):null};drag=null;updateMeasurementStatus();draw();return;}const a=worldPoint(...drag.start),b=worldPoint(...drag.end),yaw=Math.atan2(b[1]-a[1],b[0]-a[0])*180/Math.PI,mode=drag.mode;drag=null;if(mode==='waypoint'){$('waypointX').value=a[0].toFixed(2);$('waypointY').value=a[1].toFixed(2);$('waypointYaw').value=yaw.toFixed(0);readWaypointDraft();feedback('Waypoint and final facing selected. Press Go & map to send it.');}else if(mode==='calBottle'){$('calBottleX').value=a[0].toFixed(3);$('calBottleY').value=a[1].toFixed(3);action('calibration_target',{x:a[0],y:a[1],map_id:snapshot.selected_map?.id}).catch(()=>{});}else if(mode==='calPose'){$('calPoseX').value=a[0].toFixed(2);$('calPoseY').value=a[1].toFixed(2);$('calPoseYaw').value=yaw.toFixed(0);feedback('Robot pose selected. Press Set robot pose on calibration map to send it.');}else{$('poseX').value=a[0].toFixed(2);$('poseY').value=a[1].toFixed(2);$('poseYaw').value=yaw.toFixed(0);feedback('Start pose selected. Press Set initial pose to send it.');}draw();};
$('map').onpointercancel=()=>{drag=null;draw();};window.addEventListener('resize',draw);
poll();setTimeout(pollMaps,600);setTimeout(pollParameters,1000);setTimeout(pollScanFilter,1000);setTimeout(pollSlamSettings,1100);setTimeout(pollLabels,500);cameraTick();
