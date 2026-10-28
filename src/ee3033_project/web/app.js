'use strict';
const $=id=>document.getElementById(id);
const ACTIVE=new Set(['PLANNING','SEARCHING','OBSERVING','CANCELING','RETURNING','TURNING','VERIFYING','VERIFYING_HOME','MAPPING','SAVING','CALIBRATING','ACQUIRING','LOCALIZING','RECOVERING']);
const fmt=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';
const duration=s=>{s=Math.max(0,Math.floor(s||0));return `${String(Math.floor(s/60)).padStart(2,'0')}:${String(s%60).padStart(2,'0')}`;};
let snapshot=null,online=false,busy=0,mapData=null,mapImage=null,mapRevision=-1,drag=null,driveTimer=null,driveRunning=false;
let maps=[],gap=null,trial=null,paramTime=0,trialDirty=false,parametersBusy=false,loadedGeneration=null,settingsGeneration=null;
let modelLabels=null,labelKey='',labelsBusy=false;
let waypointDraft=null;
let coordinateMethodChosen=false,lastSavedMapId=null;
let measuring=false,measurement=null;
let simulationDraft=null,simulationRevision=null;
const PAGES={overview:['Robot overview','Live map, camera and robot status.'],mapping:['Map the environment','Explore automatically, drive manually, or choose a mapping waypoint.'],mission:['Set up the mission','Choose a supported target, establish the start pose, then search and return.'],navigation:['Navigation settings','Select an audacity level and configure navigation while stopped.'],results:['Trial results','Inspect reported coordinates, return verification and saved trial evidence.']};
PAGES.workflow=['Map, find & return','Build the map, confirm it, set the start, then run the autonomous mission.'];
let currentPage='workflow',workflowSerial=0,visionDirty=false;
const visionFields={camera_height:['visionHeight',1],camera_pitch:['visionPitch',180/Math.PI],camera_yaw:['visionYaw',180/Math.PI],camera_x:['visionX',1],camera_y:['visionY',1],horizontal_fov:['visionFov',180/Math.PI],camera_min_range:['visionMin',1],camera_max_range:['visionMax',1]};
function arrangeWorkflow(){
  document.querySelector('.sidebar').hidden=true;
  document.querySelector('.page-tabs').hidden=true;
  const tools=$('advancedTools'),body=$('toolsBody');
  ['waypointControls','mappingControls','audacityPanel','advanced'].forEach(id=>body.append($(id)));
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
PAGES.calibration=['Camera setup','Calibrate once, then reuse the saved profile automatically for future missions.'];
function renderPages(){
  const route=location.hash.replace(/^#\/?/,''),previous=currentPage;currentPage=Object.hasOwn(PAGES,route)?route:'workflow';
  if(previous!==currentPage){$('advancedTools').open=currentPage!=='workflow';if(currentPage==='calibration')$('legacyCalibration').open=true;}
  document.querySelectorAll('[data-pages]').forEach(e=>e.hidden=currentPage==='workflow'?false:!e.dataset.pages.split(' ').includes(currentPage));
  if(currentPage==='workflow'){$('mappingPanel').hidden=snapshot?.phase==='mission';$('legacyCalibration').hidden=true;}
  else $('legacyCalibration').hidden=false;
  $('backWorkflow').hidden=currentPage==='workflow';
  document.querySelectorAll('[data-page-link]').forEach(e=>{if(e.dataset.pageLink===currentPage)e.setAttribute('aria-current','page');else e.removeAttribute('aria-current');});
  $('pageTitle').textContent=PAGES[currentPage][0];$('pageDescription').textContent=PAGES[currentPage][1];
  document.querySelector('.workspace-grid').classList.toggle('mapping-view',currentPage==='mapping');
  document.title=PAGES[currentPage][0]+' · Group 4 Assessment console';
  document.querySelector('.stepper').hidden=currentPage==='calibration';
}
function showPage(page){location.hash='/'+page;render();}
window.addEventListener('hashchange',()=>{releaseDrive();render();window.scrollTo(0,0);});
const trialFields=[['max_linear','trialLinear','m/s'],['max_angular','trialAngular','rad/s'],['goal_timeout','trialTimeout','s']];
function feedback(message,error=false){$('feedback').textContent=message;$('feedback').classList.toggle('error',error);}
async function get(path){const r=await fetch('/api/'+path);const d=await r.json();if(!r.ok)throw Error(d.error||'Request failed');return d;}
async function send(path,body={}){
  if(!online||!snapshot)throw Error('Reconnect to the robot console before sending controls');
  const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Project-Token':snapshot.token,'X-Control-Generation':String(snapshot.control_generation??0)},body:JSON.stringify(body)});
  const data=await r.json();if(!r.ok||data.success===false)throw Error(data.error||data.message||'Command rejected');return data;
}
async function action(path,body={}){releaseDrive();busy++;render();try{const d=await send(path,body);if(d.message)feedback(d.message);return d;}catch(e){feedback(e.message,true);throw e;}finally{busy--;render();}}
function simulationEditable(){return setupIdle()&&snapshot?.simulated===true&&snapshot?.preview_backend==='main-ros'&&snapshot?.simulation_can_apply===true;}
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
function currentSavedMap(){return maps.find(m=>m.path===(snapshot?.saved_map||snapshot?.mission?.saved_map));}
function running(){return snapshot?.stage?.managed?snapshot.stage.running:true;}
function setupIdle(){return online&&!snapshot?.read_only&&!busy&&!snapshot?.stage?.transitioning&&!ACTIVE.has(snapshot?.mission?.state);}
function ready(repeat=false){return online&&(repeat||!snapshot?.assessment?.interrupted&&snapshot?.assessment?.finished==null)&&running()&&snapshot?.health?.Mission?.ok&&snapshot?.mission?.ready!==false&&!snapshot?.control_fault&&!busy&&!snapshot?.stage?.transitioning;}
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
  if(currentPage==='workflow'&&s.phase==='mapping')$('localizationMethod').value='hybrid';
  if(calibration?.ready&&s.phase!=='mission'&&!coordinateMethodChosen)$('localizationMethod').value='hybrid';
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
  const a=s.assessment||{};$('totalElapsed').textContent=duration(a.elapsed_seconds);
  $('timerNote').textContent=a.interrupted?'Interrupted · timing invalid':a.finished!=null?'Finished':a.running?(a.timing_valid?'Both stages · running':'Search only · mapping not timed'):'Starts when mapping loads';
  const estimate=m.target?{...m.target,label:m.target.class,sigma:m.target.sigma??Math.sqrt(m.target.variance),confirmed:true}:s.target_estimate,shownTarget=estimate;
  $('target').textContent=shownTarget?`${fmt(shownTarget.x)}, ${fmt(shownTarget.y)} m`:'—, — m';
  $('confidence').textContent=m.target?`${m.target.class||s.mission_options?.target_class||'Target'} · ${m.target.confidence!=null?Math.round(m.target.confidence*100)+'% confidence':'confirmed'}${Number.isFinite(m.target.variance)?' · σ '+fmt(Math.sqrt(m.target.variance))+' m':''}`:estimate?`Estimated ${estimate.label} · σ ${fmt(estimate.sigma)} m · ${estimate.samples} observations${estimate.stale?' · last seen '+fmt(estimate.age,1)+' s ago':''}`:'Waiting for estimated or confirmed coordinates';
  $('targetEstimateStatus').textContent=estimate?`Estimated ${estimate.label}: (${fmt(estimate.x)}, ${fmt(estimate.y)}) m · σ ${fmt(estimate.sigma)} m · ${estimate.stale?'last seen '+fmt(estimate.age,1)+' s ago':'updating'} · ${estimate.confirmed?'accepted':estimate.prior_conflict?'map / observation conflict; cannot accept':estimate.stable?'stable MMSE estimate':'collecting stable MMSE observations'}`:s.perception?.detected_candidates?'Target visible; position unavailable.':'No estimated target position yet.';
  $('measureMap').disabled=!mapData;
  updateMeasurementStatus();
  if(m.target?.quality==='best_available')$('confidence').textContent+=' · Best stable estimate; preferred precision not reached';
  const distance=m.return_distance??(s.pose&&m.home?Math.hypot(s.pose[0]-m.home[0],s.pose[1]-m.home[1]):null);
  $('distance').textContent=distance!=null?`${fmt(distance)} m`:'— m';
  $('returnStatus').textContent=m.return_verified?'Home position verified':m.home?`Required tolerance ${fmt(m.return_tolerance??s.mission_options?.return_tolerance??.2)} m`:'Home is recorded at mission Start';
  $('coverage').textContent=m.viewpoints_total?`${Math.round((m.observed_fraction||0)*100)}%`:'—';
  const coverageDetail=m.search_completeness;
  if($('searchLegend'))$('searchLegend').hidden=!m.search_regions||mapping;
  $('coverage').title=coverageDetail?`${coverageDetail.missing_cells} unobserved · ${coverageDetail.blocked_cells} blocked · ${coverageDetail.candidate_uncovered_cells} without a viewing position${coverageDetail.map_partial?' · partial map':''}`:'';
  $('waypoints').textContent=m.viewpoints_total?`${m.viewpoints_completed||0} / ${m.viewpoints_total} viewpoints · ${m.failed_viewpoints?.length||0} failed`:'No search plan yet';
  $('robotPose').textContent=s.pose?`Robot (${fmt(s.pose[0])}, ${fmt(s.pose[1])}) · ${fmt(s.pose[2]*180/Math.PI,0)}°`:'Pose unavailable';
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
  const settingsChanged=targetChanged||s.phase==='mission'&&(mmseReady&&Number($('preferredSigma').value)!==(s.mission_options?.preferred_target_sigma??.1)||Number($('returnTolerance').value)!==(s.mission_options?.return_tolerance??.2));
  const validSettings=$('preferredSigma').checkValidity()&&$('preferredSigma').value!==''&&$('returnTolerance').checkValidity()&&$('returnTolerance').value!=='';
  const searchEnded=['COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','STOPPED','FAILED'].includes(m.state);
  $('loadMission').hidden=!managed||(currentPage==='workflow'&&!settingsChanged&&!searchEnded);
  $('loadMission').textContent=settingsChanged?'Apply settings & reload mission':searchEnded?'Reload map & set start pose':'Load selected map for mission';
  $('loadMission').disabled=!idle||!validSettings||!labelReady||!chosenMap()?.reviewed;
  const canReuseSavedMap=managed&&chosenMap()?.reviewed&&chosenMap()?.id!==currentSavedMap()?.id;
  $('useSavedMap').hidden=!mapping||!canReuseSavedMap;
  $('useSavedMap').disabled=!idle||!labelReady||!validSettings||!canReuseSavedMap;
  $('reviewMap').disabled=!mapping||!idle||!chosenMap()||(loaded&&chosenMap().id!==currentSavedMap()?.id);
  if(currentPage==='workflow'){$('reviewMap').disabled||=!labelReady||!validSettings;$('reviewMap').textContent='Confirm map & prepare mission';$('mapLibrary').hidden=!mapping;$('setup').hidden=mapping;$('startMission').hidden=mapping;}
  else{$('reviewMap').textContent='Mark map reviewed';$('setup').hidden=false;$('startMission').hidden=false;}
  $('savedMaps').disabled=!idle;$('refreshMaps').disabled=!online||!!busy;
  const entry=chosenMap();setPill('mapQuality',entry?(entry.reviewed?'Reviewed · ':'')+entry.quality:'Unverified',entry?.reviewed?'good':'warn');
  $('savedMap').textContent=entry?.path||s.saved_map||m.saved_map||'Every save uses a new directory.';
  $('mapReviewNote').textContent=entry?.quality==='partial'?'Partial map: review accessible and missing regions before use.':'Review records your inspection. Partial and unverified maps remain labelled.';
  const poseAllowed=s.phase==='mission'&&idle&&loaded&&(!managed||!m.home)&&m.state==='WAITING';
  $('setPose').disabled=!poseAllowed||!s.health?.Map?.ok;
  ['poseX','poseY','poseYaw'].forEach(id=>$(id).disabled=!poseAllowed);
  $('poseStatus').textContent=mapping?'Load stage 2 first':s.initial_pose_sent?(s.health?.AMCL?.ok?'Pose sent · localization received':'Pose sent · awaiting localization'):'Set the assigned start before Start';
  ['targetClass','localizationMethod','returnTolerance','preferredSigma'].forEach(id=>$(id).disabled=!idle);
  $('preferredSigma').disabled=!idle||!mmseReady;
  $('preferredSigmaHelp').textContent=mmseReady?'Soft goal, not guaranteed error. After bounded refinement, the best stable estimate can still be accepted.':'Robot software update required to apply MMSE precision settings.';
  $('targetClass').disabled=!idle||!modelLabels?.ready;
  $('modelLabelsStatus').textContent=!modelLabels?.ready?(modelLabels?.error||'Reading labels from the configured model…'):targetChanged?'Apply target & reload mission, then set the initial pose again.':`${modelLabels.model} · ${modelLabels.labels.length} supported labels`;
  $('projectionHelp').textContent=['ground_plane','hybrid'].includes($('localizationMethod').value)?'Camera-only floor localization. Mount settings are under Advanced; no bottle collection is required.':'Legacy camera + lidar localization.';
  $('startMission').disabled=s.phase!=='mission'||active||!s.allow_motion||!ready()||!labelReady||settingsChanged||!validSettings||(managed&&!s.initial_pose_sent);
  $('retrySearch').hidden=s.phase!=='mission'||!searchEnded;
  $('retrySearch').disabled=!idle||!loaded||!managed||!s.allow_motion||!ready(true)||!labelReady||!s.initial_pose_sent||settingsChanged||!validSettings||!!fault;
  if(s.phase==='mission'&&searchEnded)$('startMission').hidden=true;
  $('localizationProgress').hidden=s.phase!=='mission'||!m.target_detected;
  const sites=m.bearing_progress?.sites||[];
  const viewProgress=sites.map((v,i)=>`${'ABC'[i]||'View'}: ${v.ready?'ready':`${v.frames||0}/4 frames`}`).join(' · ');
  $('localizationProgress').textContent=`Precision work: ${fmt(m.localization_active_seconds??0,1)} / ${fmt(m.localization_budget_seconds??45,0)} s · Travel between views: ${fmt(m.localization_travel_seconds??0,1)} s${m.localization_stage?' · '+m.localization_stage:''}${viewProgress?' · '+viewProgress:''}`;
  $('newTrial').disabled=!idle||!managed;$('exportResult').disabled=!online;
  setPill('mappingBadge',mapping?(active?'In progress':loaded?'Ready':'Setup'):entry?.reviewed?'Map reviewed':'Saved map',mapping&&loaded?'good':'');
  setPill('missionBadge',mapping?'Awaiting map':m.state==='COMPLETE'?'Complete':active?'In progress':'Setup',m.state==='COMPLETE'?'good':'');
  const currentStep=mapping?0:m.state==='WAITING'?1:!m.target?2:3;
  ['stepMap','stepPose','stepSearch','stepReturn'].forEach((id,i)=>{$(id).className=i===currentStep?'current':i<currentStep?'done':'';});
  if(m.state==='COMPLETE'&&m.return_verified)$('stepReturn').className='done';
  $('resultTarget').textContent=m.target?`(${fmt(m.target.x)}, ${fmt(m.target.y)}) m`:'Awaiting target';
  if(m.target?.quality==='best_available')$('resultTarget').textContent+=' · approximate estimate';
  $('resultReturn').textContent=m.return_verified?`Verified · ${fmt(distance)} m`:m.state==='RETURNING'?'Returning…':'Pending';
  $('resultStop').textContent=m.return_verified?'Verified stationary':m.state==='VERIFYING'?`Checking ${fmt(m.stationary_check?.duration??0,1)} / ${fmt(m.stationary_required_seconds??2,1)} s…`:m.state==='FAILED'&&m.stationary_check?'Not verified':'Pending';
  $('resultOutcome').textContent=m.state==='COMPLETE'&&m.return_verified?'Complete':m.state==='FOUND_UNLOCALIZED'?'Target found · position unavailable':m.state==='SEARCH_INCOMPLETE'?'Search incomplete':m.state==='NOT_FOUND'?'Search completed · target not found':m.state==='FAILED'?'Failed':m.state==='STOPPED'?'Stopped':'Trial not complete';
  $('resultReason').textContent=(m.state==='VERIFYING'&&m.stationary_check?.reason)||m.reason||'Completion requires a target report, return within tolerance, and stationary odometry.';
  if(m.target?.quality==='best_available')$('resultReason').textContent+=` · Best stable estimate: uncertainty σ ${fmt(m.target.sigma??Math.sqrt(m.target.variance))} m; preferred ${fmt(m.target.preferred_sigma)} m.`;
  const p=s.perception;
  if(currentPage==='workflow'){
    document.querySelector('.metrics').hidden=mapping;
    document.querySelector('.camera-panel').hidden=mapping;
    document.querySelector('.workspace-grid').classList.toggle('mapping-view',mapping);
    $('missionPanel').hidden=mapping&&!currentSavedMap()&&!chosenMap()?.reviewed;
    $('mapLibrary').hidden=!mapping||!maps.length;
    $('mapLibrary').querySelector('h2').textContent=currentSavedMap()?'Review this trial map':'Use a saved map';
    $('resultsPanel').hidden=!m.target&&!['COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','STOPPED','FAILED'].includes(m.state);
  }
  $('detectorSummary').textContent=p?`${p.target_class} · ${p.detected_candidates??'—'} visible · ${p.localized_candidates??0} localized${p.highest_confidence!=null?' · '+Math.round(p.highest_confidence*100)+'%':''}`:'No mission detector status';
  $('methodSummary').textContent=p?`${p.localization_method==='hybrid'?'Floor + bearing intersection':p.localization_method==='ground_plane'?'Floor-plane projection':'Camera + lidar'} · ${fmt(p.latency,2)} s inference latency`:'Coordinate method is selected before loading stage 2.';
  if(p?.camera_model){const f=p.camera_model.rotation;const pitch=Math.atan2(-f[2][2],Math.hypot(f[0][2],f[1][2]))*180/Math.PI;$('methodSummary').textContent+=` · pitch ${fmt(pitch,1)}° · ${p.camera_model.source}`;}
  if(p?.geometry_error)$('methodSummary').textContent=p.geometry_error;
  $('gateReason').textContent=s.velocity_gate?`Motion: ${s.velocity_gate.reason.replaceAll('_',' ')} · requested ${s.velocity_gate.requested?.map(v=>fmt(v)).join(', ')} · output ${s.velocity_gate.output?.map(v=>fmt(v)).join(', ')}`:'Motion diagnostics appear after a stage loads.';
  ['profileFast','profileOriginal'].forEach(id=>$(id).disabled=!idle||!loaded||!s.health?.Mission?.ok||(s.phase==='mission'&&m.state!=='WAITING'));
  $('saveVision').disabled=!idle;
  if(!visionDirty)$('visionPitchOverride').checked=s.vision_settings?.use_pitch_override===true;
  if(s.vision_settings&&!visionDirty)Object.entries(visionFields).forEach(([name,[id,scale]])=>$(id).value=fmt(s.vision_settings[name]*scale,3));
  $('cameraSource').textContent=s.preview_interactive&&s.camera_source?'Simulated camera':s.camera_source==='project'?'Mission detector':s.camera_source==='lab'?'Lab display detector':'No stream';
  $('camera').hidden=!s.camera_source;$('cameraEmpty').hidden=!!s.camera_source;
  $('events').replaceChildren(...(s.events?.length?s.events:[{time:'',state:'WAITING',reason:'Waiting for the controller.'}]).map(e=>{const li=document.createElement('li'),t=document.createElement('time'),st=document.createElement('strong'),r=document.createElement('small');t.textContent=e.time||'';st.textContent=e.state;r.textContent=e.reason||'';li.append(t,st,r);return li;}));
  $('stageHelp').textContent=currentPage==='workflow'?'Automatic mapping returns and saves a new map. Inspect it, then confirm to prepare the mission.':managed?'Load mapping, then Start. The saved map can be loaded for stage 2 here.':'Standalone stage: UI stage switching requires assessment.launch.py.';
  $('mappingProgress').hidden=!mapping;
  $('mappingProgress').textContent=`${m.viewpoints_completed||0} goals reached · ${m.deferred_frontiers||0} deferred · ${m.frontier_retry_round?'Final retry pass':'Explore other areas before one final retry pass'}`;
  renderParameters();draw();
}
function renderParameters(){
  const idle=setupIdle(),fresh=Date.now()-paramTime<8000,mapping=snapshot?.phase==='mapping';
  const policyReady=idle&&fresh&&!!gap&&!parametersBusy&&(mapping||snapshot?.mission?.state==='WAITING');
  document.querySelectorAll('[data-audacity]').forEach(button=>{
    const level=button.dataset.audacity,option=gap?.audacity_options?.find(p=>p.id===level);
    button.disabled=!policyReady||!option;
    button.setAttribute('aria-pressed',String(fresh&&gap?.audacity===level&&gap?.synchronized===true));
    button.querySelector('small').textContent=option?`${fmt(option.clearance*100,0)} cm clearance · ~${fmt(option.nominal_min_opening*100,0)} cm opening`:'Waiting for live settings';
    button.querySelector('em').textContent=option?`Turn stop ${fmt(option.turn_stop_distance*100,0)} cm · ${level==='cautious'?'original turning':level==='balanced'?'gentle turning':'firm turning'}`:'';
  });
  const label=gap?.audacity_options?.find(p=>p.id===gap?.audacity)?.label;
  setPill('audacityLive',fresh&&label?label:'Unverified',fresh&&gap?.synchronized?'good':'warn');
  $('audacityStatus').textContent=parametersBusy?'Applying and verifying navigation settings…':!fresh?'Load a stage and wait for live settings.':gap?.fault||`Live: ${label||'unsynchronized'}. Applies to automatic mapping and mission navigation. Configure while stopped${mapping?'.':', before mission Start.'}`;
  $('gapToggle').disabled=!idle||!fresh||!gap||parametersBusy||(!mapping&&snapshot?.mission?.state!=='WAITING');
  $('gapToggle').setAttribute('aria-pressed',String(gap?.enabled===true));
  $('gapToggle').textContent=gap?.enabled===true?'Narrow-gap blocking: ON · Disable':gap?.enabled===false?'Narrow-gap blocking: OFF · Enable':'Synchronize gap policy: Enable';
  if(gap&&fresh){const n=gap.nodes,g=n?.['global_costmap/global_costmap'],l=n?.['local_costmap/local_costmap'],stage=n?.[mapping?'project_mapper':'project_mission'];$('gapStatus').textContent=gap.fault||`Verified radius: global ${fmt(g?.robot_radius)} m / local ${fmt(l?.robot_radius)} m. Mapping/search clearance ${fmt(stage?.clearance)} m.`;}
  $('trialPanel').hidden=snapshot?.phase==='calibration';
  trialFields.forEach(([key,id,unit])=>{$(id).disabled=!idle||!fresh||!trial||parametersBusy;$(id+'Value').textContent=fmt(Number($(id).value),key==='goal_timeout'?0:2)+' '+unit;$(id+'Live').textContent='Live: '+(trial?fmt(trial.values[key],key==='goal_timeout'?0:2)+' '+unit:'unavailable');});
  $('applyTrial').disabled=!idle||!fresh||!trial||!trialDirty||parametersBusy;
  $('trialStatus').textContent=parametersBusy?'Applying and verifying…':trialDirty?'Unapplied changes — press Apply.':trial?.fault||'Temporary until restart. Adjust only while stopped.';
  if(trial?.protection)$('trialProtection').textContent=`Obstacle stop ${fmt(trial.protection.stop_distance)} m · turn clearance ${fmt(trial.protection.rotation_clearance)} m`;
}
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
async function loadStage(phase){const body={phase};if(phase==='mission')Object.assign(body,{map_id:$('savedMaps').value,target_class:$('targetClass').value.trim().toLowerCase(),localization_method:$('localizationMethod').value,return_tolerance:Number($('returnTolerance').value),preferred_target_sigma:Number($('preferredSigma').value)});const workflow=currentPage==='workflow';await action('stage',body);if(!workflow)showPage(phase);}
$('loadMapping').onclick=()=>loadStage('mapping').catch(()=>{});$('loadMission').onclick=()=>loadStage('mission').catch(()=>{});
$('useSavedMap').onclick=()=>loadStage('mission').catch(()=>{});
$('start').onclick=async()=>{const serial=++workflowSerial;try{
  if(currentPage==='workflow'&&snapshot?.stage?.managed&&!running()){
    await loadStage('mapping');const deadline=Date.now()+30000;
    while(!ready()){if(serial!==workflowSerial||!online)throw Error('Mapping start canceled');if(Date.now()>deadline)throw Error('Mapping is still preparing; inspect readiness and press Start again');await new Promise(r=>setTimeout(r,150));}
  }
  if(serial===workflowSerial)await action('start');
}catch(e){feedback(e.message,true);}};$('startMission').onclick=()=>action('start').catch(()=>{});
$('stop').onclick=async()=>{workflowSerial++;releaseDrive();try{const d=await send('stop');feedback(d.message||'Stop requested');}catch(e){feedback(e.message,true);}};
$('saveMap').onclick=async()=>{try{const d=await action('save_map');await refreshMaps(d.map?.id||maps.find(m=>m.path===d.path)?.id);}catch(_){}};
$('refreshMaps').onclick=()=>refreshMaps().catch(e=>feedback(e.message,true));$('savedMaps').onchange=render;
$('reviewMap').onclick=async()=>{const serial=workflowSerial;try{await action('review_map',{map_id:$('savedMaps').value});await refreshMaps();if(currentPage==='workflow'&&serial===workflowSerial)await loadStage('mission');}catch(_){}};
$('calSetBottle').onclick=()=>action('calibration_target',{x:Number($('calBottleX').value),y:Number($('calBottleY').value),map_id:snapshot.selected_map?.id}).catch(()=>{});
$('calSetPose').onclick=()=>action('initial_pose',{x:Number($('calPoseX').value),y:Number($('calPoseY').value),yaw:Number($('calPoseYaw').value)*Math.PI/180}).catch(()=>{});
$('setPose').onclick=()=>action('initial_pose',{x:Number($('poseX').value),y:Number($('poseY').value),yaw:Number($('poseYaw').value)*Math.PI/180}).catch(()=>{});
function readWaypointDraft(){const fields=['waypointX','waypointY','waypointYaw'].map(id=>$(id).value);const values=fields.map(Number);waypointDraft=fields.every(v=>v!=='')&&values.every(Number.isFinite)?[values[0],values[1],values[2]*Math.PI/180]:null;render();}
['waypointX','waypointY','waypointYaw'].forEach(id=>$(id).oninput=readWaypointDraft);
$('goWaypoint').onclick=()=>{if(waypointDraft)action('map_waypoint',{x:waypointDraft[0],y:waypointDraft[1],yaw:waypointDraft[2]}).then(()=>{waypointDraft=null;render();}).catch(()=>{});};
$('retrySearch').onclick=()=>action('retry_search').catch(()=>{});
$('newTrial').onclick=()=>action('new_trial').then(()=>showPage('workflow')).catch(()=>{});$('localizationMethod').onchange=()=>{coordinateMethodChosen=true;render();};
$('loadCalibration').onclick=()=>action('stage',{phase:'calibration',map_id:chosenMap()?.reviewed?chosenMap().id:undefined}).then(()=>showPage('calibration')).catch(()=>{});
$('targetClass').onchange=render;
['preferredSigma','returnTolerance'].forEach(id=>$(id).oninput=render);
$('exportResult').onclick=async()=>{try{const d=await get('result'),url=URL.createObjectURL(new Blob([JSON.stringify(d,null,2)+'\n'],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=(d.assessment?.id||'assessment')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(e){feedback(e.message,true);}};
trialFields.forEach(([,id])=>$(id).oninput=()=>{trialDirty=true;renderParameters();});
$('applyTrial').onclick=async()=>{parametersBusy=true;try{trial=await action('trial',Object.fromEntries(trialFields.map(([k,id])=>[k,Number($(id).value)])));trialDirty=false;paramTime=Date.now();}catch(_){}finally{parametersBusy=false;render();}};
Object.values(visionFields).forEach(([id])=>$(id).oninput=()=>{visionDirty=true;});
$('visionPitchOverride').onchange=()=>{visionDirty=true;};
$('saveVision').onclick=()=>action('vision',{...Object.fromEntries(Object.entries(visionFields).map(([name,[id,scale]])=>[name,Number($(id).value)/scale])),use_pitch_override:$('visionPitchOverride').checked}).then(()=>{visionDirty=false;}).catch(()=>{});
for(const [id,profile] of [['profileOriginal','original'],['profileFast','fast']])$(id).onclick=async()=>{parametersBusy=true;try{trial=await action('navigation',{profile});trialDirty=false;paramTime=Date.now();trialFields.forEach(([name,field])=>$(field).value=trial.values[name]);$('navigationProfileStatus').textContent=`Applied and verified: ${profile} · ${fmt(trial.values.max_linear)} m/s · ${fmt(trial.values.max_angular)} rad/s`;}catch(_){}finally{parametersBusy=false;render();}};
$('gapToggle').onclick=async()=>{parametersBusy=true;try{gap=await action('gap',{enabled:gap.enabled!==true});paramTime=Date.now();}catch(_){}finally{parametersBusy=false;render();}};
document.querySelectorAll('[data-audacity]').forEach(button=>button.onclick=async()=>{
  parametersBusy=true;
  try{gap=await action('gap',{audacity:button.dataset.audacity,enabled:gap.enabled===true});paramTime=Date.now();}
  catch(_){paramTime=0;}
  finally{parametersBusy=false;render();}
});
async function pollParameters(){
  if(online&&snapshot?.health?.Mission?.ok&&running()&&!busy&&!parametersBusy&&!snapshot.stage?.transitioning&&snapshot.phase!=='calibration'){const generation=snapshot.control_generation;try{const g=await get('gap'),t=await get('trial');if(generation===snapshot.control_generation){gap=g;trial=t;paramTime=Date.now();if(t)trialFields.forEach(([k,id])=>{if(!trialDirty)$(id).value=t.values[k];});$('navigationProfileStatus').textContent=`Live limits: ${fmt(t.values.max_linear)} m/s · ${fmt(t.values.max_angular)} rad/s. Fast is a trial profile pending physical validation.`;}}catch(e){paramTime=0;$('gapStatus').textContent=e.message;}renderParameters();}
  setTimeout(pollParameters,4000);
}
async function poll(){
  try{const s=await get('state');snapshot=s;online=true;
    if(loadedGeneration!==s.control_generation){loadedGeneration=s.control_generation;gap=trial=null;paramTime=0;trialDirty=false;waypointDraft=null;['waypointX','waypointY','waypointYaw'].forEach(id=>$(id).value='');releaseDrive();}
    if(settingsGeneration!==s.stage?.phase){settingsGeneration=s.stage?.phase;if(s.mission_options){$('targetClass').value=s.mission_options.target_class;$('localizationMethod').value=s.mission_options.localization_method;$('returnTolerance').value=s.mission_options.return_tolerance;$('preferredSigma').value=s.mission_options.preferred_target_sigma??.1;}}
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
    if(key!==labelKey){labelKey=key;const current=$('targetClass').value||snapshot?.mission_options?.target_class||'cup';
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
  line(snapshot.route,'#bca6d8',[1.5,2]);line(snapshot.path,'#52cfaa');const m=snapshot.mission;dot(m.home,'#ce92df',2.3);if(m.target)dot([m.target.x,m.target.y],'#ecba59',2.8);
  const estimate=m.target?{...m.target,label:m.target.class,sigma:m.target.sigma??Math.sqrt(m.target.variance),confirmed:true}:snapshot.target_estimate;
  if(estimate){const[x,y]=point(estimate.x,estimate.y),radius=Math.min(estimate.sigma/mapData.resolution,2*Math.hypot(mapData.width,mapData.height));ctx.save();ctx.globalAlpha=estimate.stale ? .4 : 1;ctx.beginPath();ctx.arc(x,y,radius,0,Math.PI*2);ctx.fillStyle='rgba(222,151,45,.12)';ctx.fill();ctx.setLineDash([1.4,1.4]);ctx.strokeStyle='#c17a21';ctx.lineWidth=.7;ctx.stroke();ctx.setLineDash([]);ctx.beginPath();ctx.arc(x,y,2.8,0,Math.PI*2);ctx.stroke();ctx.fillStyle='#80500d';ctx.font='3px sans-serif';ctx.fillText(`${estimate.confirmed?'Accepted':'Est.'} ${estimate.label} · σ ${fmt(estimate.sigma)} m`,Math.max(1,Math.min(x+4,mapData.width-38)),Math.max(4,y-4));ctx.restore();}
  if(currentPage==='calibration'){const c=snapshot.calibration_live;if(c?.plan)line([snapshot.pose,...c.plan.goals.map(g=>g.pose)].filter(Boolean),'#ecba59',[2,2]);if(c?.target){dot(c.target.xy,'#ffba5a',2.8);const [x,y]=point(...c.target.xy);ctx.fillStyle='#50340b';ctx.font='3px sans-serif';ctx.fillText('Bottle',x+3,y-3);}}
  const waypoint=waypointDraft||(m.mapping_mode==='waypoint'?m.waypoint:null);
  if(waypoint){const[x,y]=point(...waypoint),a=-(waypoint[2]-mapData.origin[2]);ctx.save();ctx.translate(x,y);ctx.rotate(a);ctx.beginPath();ctx.arc(0,0,1.6,0,Math.PI*2);ctx.moveTo(0,0);ctx.lineTo(7,0);ctx.lineTo(4,-1.5);ctx.moveTo(7,0);ctx.lineTo(4,1.5);ctx.strokeStyle='#ecba59';ctx.lineWidth=.7;ctx.stroke();ctx.restore();}
  if(snapshot.pose){const[x,y]=point(...snapshot.pose);ctx.save();ctx.translate(x,y);ctx.rotate(-(snapshot.pose[2]-mapData.origin[2]));ctx.beginPath();ctx.moveTo(4,0);ctx.lineTo(-2.5,-2);ctx.lineTo(-2.5,2);ctx.closePath();ctx.fillStyle='#5dbaec';ctx.fill();ctx.restore();}
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
poll();setTimeout(pollMaps,600);setTimeout(pollParameters,1000);setTimeout(pollLabels,500);cameraTick();
