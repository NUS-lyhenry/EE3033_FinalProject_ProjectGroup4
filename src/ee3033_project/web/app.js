'use strict';
const $=id=>document.getElementById(id);
const ACTIVE=new Set(['PLANNING','SEARCHING','OBSERVING','CANCELING','RETURNING','TURNING','VERIFYING','MAPPING','SAVING','CALIBRATING','ACQUIRING','LOCALIZING']);
const fmt=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';
const duration=s=>{s=Math.max(0,Math.floor(s||0));return `${String(Math.floor(s/60)).padStart(2,'0')}:${String(s%60).padStart(2,'0')}`;};
let snapshot=null,online=false,busy=0,mapData=null,mapImage=null,mapRevision=-1,drag=null,driveTimer=null,driveRunning=false;
let maps=[],gap=null,trial=null,paramTime=0,trialDirty=false,parametersBusy=false,loadedGeneration=null,settingsGeneration=null;
let modelLabels=null,labelKey='',labelsBusy=false;
let waypointDraft=null;
let coordinateMethodChosen=false,lastSavedMapId=null;
const PAGES={overview:['Robot overview','Live map, camera and robot status.'],mapping:['Map the environment','Explore automatically, drive manually, or choose a mapping waypoint.'],mission:['Set up the mission','Choose a supported target, establish the start pose, then search and return.'],navigation:['Navigation settings','Select an audacity level and configure navigation while stopped.'],results:['Trial results','Inspect reported coordinates, return verification and saved trial evidence.']};
PAGES.workflow=['Map, find & return','Build the map, confirm it, set the start, then run the autonomous mission.'];
let currentPage='workflow',workflowSerial=0,visionDirty=false;
const visionFields={camera_height:['visionHeight',1],camera_pitch:['visionPitch',180/Math.PI],camera_yaw:['visionYaw',180/Math.PI],camera_x:['visionX',1],camera_y:['visionY',1],horizontal_fov:['visionFov',180/Math.PI],camera_min_range:['visionMin',1],camera_max_range:['visionMax',1]};
function arrangeWorkflow(){
  document.querySelector('.sidebar').hidden=true;
  document.querySelector('.page-tabs').hidden=true;
  const tools=$('advancedTools'),body=$('toolsBody');
  ['waypointControls','mappingControls','audacityPanel','advanced'].forEach(id=>body.append($(id)));
  body.append($('mapLibrary').querySelector('.field'));
  body.append($('localizationMethod').closest('.field'));
  body.append(document.querySelector('.event-panel'));
  const grid=document.querySelector('.workspace-grid'),stages=document.querySelector('.stage-grid');
  grid.before(stages,$('mapLibrary'));
  $('localizationMethod').value='ground_plane';
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
function chosenMap(){return maps.find(m=>m.id===$('savedMaps').value);}
function currentSavedMap(){return maps.find(m=>m.path===(snapshot?.saved_map||snapshot?.mission?.saved_map));}
function running(){return snapshot?.stage?.managed?snapshot.stage.running:true;}
function setupIdle(){return online&&!snapshot?.read_only&&!busy&&!snapshot?.stage?.transitioning&&!ACTIVE.has(snapshot?.mission?.state);}
function ready(){return online&&!snapshot?.assessment?.interrupted&&snapshot?.assessment?.finished==null&&running()&&snapshot?.health?.Mission?.ok&&snapshot?.mission?.ready!==false&&!snapshot?.control_fault&&!busy&&!snapshot?.stage?.transitioning;}
function setPill(id,text,kind=''){const e=$(id);e.textContent=text;e.className='pill '+kind;}
function render(){
  renderPages();
  if(!snapshot)return;
  const s=snapshot,m=s.mission||{},active=ACTIVE.has(m.state),mapping=s.phase==='mapping',idle=setupIdle(),loaded=running(),managed=!!s.stage?.managed;
  $('previewNotice').hidden=!s.simulated;
  $('previewNotice').textContent=s.preview_backend==='main-ros'?`算法仿真 · 未连接机器人。${s.preview_hint||''}`:'SIMULATED PREVIEW · No robot is connected. Values illustrate the operator workflow.';
  $('stop').disabled=!!s.read_only;
  setPill('connection',online?(s.preview_interactive?'模拟连接':'Connected'):'Disconnected',online?'good':'bad');
  $('connectionBanner').hidden=online;
  const calibrating=s.phase==='calibration';
  $('phase').textContent=calibrating?'Preparation · Camera setup':mapping?'Stage 1 · Mapping':'Stage 2 · Autonomous';
  const calibration=s.camera_calibration;
  if(currentPage==='workflow')$('localizationMethod').value='ground_plane';
  if(calibration?.ready&&s.phase!=='mission'&&!coordinateMethodChosen)$('localizationMethod').value='ground_plane';
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
  setPill('motion',s.preview_interactive?'仅模拟运动':s.allow_motion?'Motion enabled':'Motion disabled',s.allow_motion?'good':'');
  $('state').textContent=m.state||'WAITING';$('elapsed').textContent=duration(m.elapsed_seconds)+' stage elapsed';
  const a=s.assessment||{};$('totalElapsed').textContent=duration(a.elapsed_seconds);
  $('timerNote').textContent=a.interrupted?'Interrupted · timing invalid':a.finished!=null?'Finished':a.running?(a.timing_valid?'Both stages · running':'Search only · mapping not timed'):'Starts when mapping loads';
  $('target').textContent=m.target?`${fmt(m.target.x)}, ${fmt(m.target.y)} m`:'—, — m';
  $('confidence').textContent=m.target?`${m.target.class||s.mission_options?.target_class||'Target'} · ${m.target.confidence!=null?Math.round(m.target.confidence*100)+'% confidence':'confirmed'}${Number.isFinite(m.target.variance)?' · σ '+fmt(Math.sqrt(m.target.variance))+' m':''}`:'Waiting for confirmed coordinates';
  if(m.target?.quality==='best_available')$('confidence').textContent+=' · 最佳稳定估计，位置精度未达期望';
  const distance=m.return_distance??(s.pose&&m.home?Math.hypot(s.pose[0]-m.home[0],s.pose[1]-m.home[1]):null);
  $('distance').textContent=distance!=null?`${fmt(distance)} m`:'— m';
  $('returnStatus').textContent=m.return_verified?'Home position verified':m.home?`Required tolerance ${fmt(m.return_tolerance??s.mission_options?.return_tolerance??.2)} m`:'Home is recorded at mission Start';
  $('coverage').textContent=m.viewpoints_total?`${Math.round((m.observed_fraction||0)*100)}%`:'—';
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
  $('loadMission').hidden=!managed||currentPage==='workflow';$('loadMission').disabled=!idle||!labelReady||!chosenMap()?.reviewed||!(s.assessment?.maps||[]).includes(chosenMap()?.id)||s.assessment?.finished!=null;
  $('reviewMap').disabled=!mapping||!idle||!chosenMap()||(loaded&&chosenMap().id!==currentSavedMap()?.id);
  if(currentPage==='workflow'){$('reviewMap').disabled||=!labelReady;$('reviewMap').textContent='Confirm map & prepare mission';$('mapLibrary').hidden=!mapping;$('setup').hidden=mapping;$('startMission').hidden=mapping;}
  else{$('reviewMap').textContent='Mark map reviewed';$('setup').hidden=false;$('startMission').hidden=false;}
  $('savedMaps').disabled=!idle;$('refreshMaps').disabled=!online||!!busy;
  const entry=chosenMap();setPill('mapQuality',entry?(entry.reviewed?'Reviewed · ':'')+entry.quality:'Unverified',entry?.reviewed?'good':'warn');
  $('savedMap').textContent=entry?.path||s.saved_map||m.saved_map||'Every save uses a new directory.';
  $('mapReviewNote').textContent=entry?.quality==='partial'?'Partial map: review accessible and missing regions before use.':'Review records your inspection. Partial and unverified maps remain labelled.';
  const poseAllowed=s.phase==='mission'&&idle&&loaded&&(!managed||!m.home)&&m.state==='WAITING';
  $('setPose').disabled=!poseAllowed||!s.health?.Map?.ok;
  ['poseX','poseY','poseYaw'].forEach(id=>$(id).disabled=!poseAllowed);
  $('poseStatus').textContent=mapping?'Load stage 2 first':s.initial_pose_sent?(s.health?.AMCL?.ok?'Pose sent · localization received':'Pose sent · awaiting localization'):'Set the assigned start before Start';
  ['targetClass','localizationMethod','returnTolerance'].forEach(id=>$(id).disabled=!idle);
  $('targetClass').disabled=!idle||!modelLabels?.ready;
  const targetChanged=!mapping&&$('targetClass').value!==s.mission_options?.target_class;
  $('modelLabelsStatus').textContent=!modelLabels?.ready?(modelLabels?.error||'Reading labels from the configured model…'):targetChanged?'Reload the mission to apply the selected target class.':`${modelLabels.model} · ${modelLabels.labels.length} supported labels`;
  $('projectionHelp').textContent=$('localizationMethod').value==='ground_plane'?'Camera-only floor localization. Mount settings are under Advanced; no bottle collection is required.':'Legacy camera + lidar localization.';
  $('startMission').disabled=s.phase!=='mission'||active||!s.allow_motion||!ready()||!labelReady||targetChanged||(managed&&!s.initial_pose_sent);
  $('newTrial').disabled=!idle||!managed;$('exportResult').disabled=!online;
  setPill('mappingBadge',mapping?(active?'In progress':loaded?'Ready':'Setup'):entry?.reviewed?'Map reviewed':'Saved map',mapping&&loaded?'good':'');
  setPill('missionBadge',mapping?'Awaiting map':m.state==='COMPLETE'?'Complete':active?'In progress':'Setup',m.state==='COMPLETE'?'good':'');
  const currentStep=mapping?0:m.state==='WAITING'?1:!m.target?2:3;
  ['stepMap','stepPose','stepSearch','stepReturn'].forEach((id,i)=>{$(id).className=i===currentStep?'current':i<currentStep?'done':'';});
  if(m.state==='COMPLETE'&&m.return_verified)$('stepReturn').className='done';
  $('resultTarget').textContent=m.target?`(${fmt(m.target.x)}, ${fmt(m.target.y)}) m`:'Awaiting target';
  if(m.target?.quality==='best_available')$('resultTarget').textContent+=' · 近似估计';
  $('resultReturn').textContent=m.return_verified?`Verified · ${fmt(distance)} m`:m.state==='RETURNING'?'Returning…':'Pending';
  $('resultStop').textContent=m.return_verified?'Verified stationary':m.state==='VERIFYING'?`Checking ${fmt(m.stationary_check?.duration??0,1)} / ${fmt(m.stationary_required_seconds??2,1)} s…`:m.state==='FAILED'&&m.stationary_check?'Not verified':'Pending';
  $('resultOutcome').textContent=m.state==='COMPLETE'&&m.return_verified?'Complete':m.state==='NOT_FOUND'?'Target not found':m.state==='FAILED'?'Failed':m.state==='STOPPED'?'Stopped':'Trial not complete';
  $('resultReason').textContent=(m.state==='VERIFYING'&&m.stationary_check?.reason)||m.reason||'Completion requires a target report, return within tolerance, and stationary odometry.';
  if(m.target?.quality==='best_available')$('resultReason').textContent+=` · 使用最佳稳定估计：不确定度 σ ${fmt(m.target.sigma??Math.sqrt(m.target.variance))} m，期望 ${fmt(m.target.preferred_sigma)} m。`;
  const p=s.perception;
  if(currentPage==='workflow'){
    document.querySelector('.metrics').hidden=mapping;
    document.querySelector('.camera-panel').hidden=mapping;
    document.querySelector('.workspace-grid').classList.toggle('mapping-view',mapping);
    $('missionPanel').hidden=mapping&&!currentSavedMap();
    $('mapLibrary').hidden=!mapping||!currentSavedMap();
    $('mapLibrary').querySelector('h2').textContent='Review this trial map';
    $('resultsPanel').hidden=!m.target&&!['COMPLETE','NOT_FOUND','STOPPED','FAILED'].includes(m.state);
  }
  $('detectorSummary').textContent=p?`${p.target_class} · ${p.detected_candidates??'—'} visible · ${p.localized_candidates??0} localized${p.highest_confidence!=null?' · '+Math.round(p.highest_confidence*100)+'%':''}`:'No mission detector status';
  $('methodSummary').textContent=p?`${p.localization_method==='ground_plane'?'Floor-plane projection':'Camera + lidar'} · ${fmt(p.latency,2)} s inference latency`:'Coordinate method is selected before loading stage 2.';
  if(p?.geometry_error)$('methodSummary').textContent=p.geometry_error;
  $('gateReason').textContent=s.velocity_gate?`Motion: ${s.velocity_gate.reason.replaceAll('_',' ')} · requested ${s.velocity_gate.requested?.map(v=>fmt(v)).join(', ')} · output ${s.velocity_gate.output?.map(v=>fmt(v)).join(', ')}`:'Motion diagnostics appear after a stage loads.';
  ['profileFast','profileOriginal'].forEach(id=>$(id).disabled=!idle||!loaded||!s.health?.Mission?.ok||(s.phase==='mission'&&m.state!=='WAITING'));
  $('saveVision').disabled=!idle;
  if(s.vision_settings&&!visionDirty)Object.entries(visionFields).forEach(([name,[id,scale]])=>$(id).value=fmt(s.vision_settings[name]*scale,3));
  $('cameraSource').textContent=s.preview_interactive&&s.camera_source?'模拟相机':s.camera_source==='project'?'Mission detector':s.camera_source==='lab'?'Lab display detector':'No stream';
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
async function loadStage(phase){const body={phase};if(phase==='mission')Object.assign(body,{map_id:$('savedMaps').value,target_class:$('targetClass').value.trim().toLowerCase(),localization_method:currentPage==='workflow'?'ground_plane':$('localizationMethod').value,return_tolerance:Number($('returnTolerance').value)});const workflow=currentPage==='workflow';await action('stage',body);if(!workflow)showPage(phase);}
$('loadMapping').onclick=()=>loadStage('mapping').catch(()=>{});$('loadMission').onclick=()=>loadStage('mission').catch(()=>{});
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
$('newTrial').onclick=()=>action('new_trial').then(()=>showPage('workflow')).catch(()=>{});$('localizationMethod').onchange=()=>{coordinateMethodChosen=true;render();};
$('loadCalibration').onclick=()=>action('stage',{phase:'calibration',map_id:chosenMap()?.reviewed?chosenMap().id:undefined}).then(()=>showPage('calibration')).catch(()=>{});
$('targetClass').onchange=render;
$('exportResult').onclick=async()=>{try{const d=await get('result'),url=URL.createObjectURL(new Blob([JSON.stringify(d,null,2)+'\n'],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=(d.assessment?.id||'assessment')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(e){feedback(e.message,true);}};
trialFields.forEach(([,id])=>$(id).oninput=()=>{trialDirty=true;renderParameters();});
$('applyTrial').onclick=async()=>{parametersBusy=true;try{trial=await action('trial',Object.fromEntries(trialFields.map(([k,id])=>[k,Number($(id).value)])));trialDirty=false;paramTime=Date.now();}catch(_){}finally{parametersBusy=false;render();}};
Object.values(visionFields).forEach(([id])=>$(id).oninput=()=>{visionDirty=true;});
$('saveVision').onclick=()=>action('vision',Object.fromEntries(Object.entries(visionFields).map(([name,[id,scale]])=>[name,Number($(id).value)/scale]))).then(()=>{visionDirty=false;}).catch(()=>{});
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
    if(settingsGeneration!==s.stage?.phase){settingsGeneration=s.stage?.phase;if(s.mission_options){$('targetClass').value=s.mission_options.target_class;$('localizationMethod').value=s.mission_options.localization_method;$('returnTolerance').value=s.mission_options.return_tolerance;}}
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
function draw(){if(!mapData||!mapImage||!snapshot)return;const canvas=$('map'),ctx=canvas.getContext('2d'),wrap=canvas.parentElement;if(wrap.clientWidth<1||wrap.clientHeight<1)return;const scale=Math.min((wrap.clientWidth-28)/mapData.width,(wrap.clientHeight-28)/mapData.height);canvas.width=mapData.width*3;canvas.height=mapData.height*3;canvas.style.width=mapData.width*scale+'px';canvas.style.height=mapData.height*scale+'px';ctx.scale(3,3);ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImage,0,0);
  function line(points,color,dash=[]){if(!points?.length)return;ctx.beginPath();ctx.setLineDash(dash);points.forEach((p,i)=>{const [x,y]=point(...p);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.strokeStyle=color;ctx.lineWidth=.8;ctx.stroke();ctx.setLineDash([]);}
  function dot(p,color,r=2){if(!p)return;const[x,y]=point(...p);ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();}
  line(snapshot.route,'#bca6d8',[1.5,2]);line(snapshot.path,'#52cfaa');const m=snapshot.mission;dot(m.home,'#ce92df',2.3);if(m.target)dot([m.target.x,m.target.y],'#ecba59',2.8);
  if(currentPage==='calibration'){const c=snapshot.calibration_live;if(c?.plan)line([snapshot.pose,...c.plan.goals.map(g=>g.pose)].filter(Boolean),'#ecba59',[2,2]);if(c?.target){dot(c.target.xy,'#ffba5a',2.8);const [x,y]=point(...c.target.xy);ctx.fillStyle='#50340b';ctx.font='3px sans-serif';ctx.fillText('Bottle',x+3,y-3);}}
  const waypoint=waypointDraft||(m.mapping_mode==='waypoint'?m.waypoint:null);
  if(waypoint){const[x,y]=point(...waypoint),a=-(waypoint[2]-mapData.origin[2]);ctx.save();ctx.translate(x,y);ctx.rotate(a);ctx.beginPath();ctx.arc(0,0,1.6,0,Math.PI*2);ctx.moveTo(0,0);ctx.lineTo(7,0);ctx.lineTo(4,-1.5);ctx.moveTo(7,0);ctx.lineTo(4,1.5);ctx.strokeStyle='#ecba59';ctx.lineWidth=.7;ctx.stroke();ctx.restore();}
  if(snapshot.pose){const[x,y]=point(...snapshot.pose);ctx.save();ctx.translate(x,y);ctx.rotate(-(snapshot.pose[2]-mapData.origin[2]));ctx.beginPath();ctx.moveTo(4,0);ctx.lineTo(-2.5,-2);ctx.lineTo(-2.5,2);ctx.closePath();ctx.fillStyle='#5dbaec';ctx.fill();ctx.restore();}
  if(drag){ctx.beginPath();ctx.moveTo(...drag.start);ctx.lineTo(...drag.end);ctx.strokeStyle='#ecba59';ctx.stroke();}
}
function renderMap(m){mapData=m;mapImage=null;$('mapEmpty').hidden=!!m;if(!m){$('map').getContext('2d').clearRect(0,0,$('map').width,$('map').height);return;}$('mapScale').textContent=`${fmt(m.width*m.resolution,1)} × ${fmt(m.height*m.resolution,1)} m · ${fmt(m.resolution)} m/cell`;mapImage=document.createElement('canvas');mapImage.width=m.width;mapImage.height=m.height;const ctx=mapImage.getContext('2d'),pixels=ctx.createImageData(m.width,m.height);for(let r=0;r<m.height;r++)for(let c=0;c<m.width;c++){const v=m.cells[r*m.width+c],i=((m.height-1-r)*m.width+c)*4;pixels.data.set([...(v<0?[48,65,75]:v>=50?[14,29,36]:[196,210,211]),255],i);}ctx.putImageData(pixels,0,0);draw();}
function canvasPosition(e){const r=$('map').getBoundingClientRect();return[(e.clientX-r.left)/r.width*mapData.width,(e.clientY-r.top)/r.height*mapData.height];}
$('map').onpointerdown=e=>{const mode=(currentPage==='mapping'||currentPage==='workflow'&&$('advancedTools').open)&&!$('waypointX').disabled?'waypoint':currentPage==='calibration'&&!$('calSetBottle').disabled?($('calMapTool').value==='bottle'?'calBottle':'calPose'):(currentPage==='mission'||currentPage==='workflow')&&!$('setPose').disabled?'pose':null;if(!mapData||!mode)return;e.preventDefault();$('map').setPointerCapture(e.pointerId);const p=canvasPosition(e);drag={start:p,end:p,mode};draw();};
$('map').onpointermove=e=>{if(drag){drag.end=canvasPosition(e);draw();}};
$('map').onpointerup=()=>{if(!drag)return;const a=worldPoint(...drag.start),b=worldPoint(...drag.end),yaw=Math.atan2(b[1]-a[1],b[0]-a[0])*180/Math.PI,mode=drag.mode;drag=null;if(mode==='waypoint'){$('waypointX').value=a[0].toFixed(2);$('waypointY').value=a[1].toFixed(2);$('waypointYaw').value=yaw.toFixed(0);readWaypointDraft();feedback('Waypoint and final facing selected. Press Go & map to send it.');}else if(mode==='calBottle'){$('calBottleX').value=a[0].toFixed(3);$('calBottleY').value=a[1].toFixed(3);action('calibration_target',{x:a[0],y:a[1],map_id:snapshot.selected_map?.id}).catch(()=>{});}else if(mode==='calPose'){$('calPoseX').value=a[0].toFixed(2);$('calPoseY').value=a[1].toFixed(2);$('calPoseYaw').value=yaw.toFixed(0);feedback('Robot pose selected. Press Set robot pose on calibration map to send it.');}else{$('poseX').value=a[0].toFixed(2);$('poseY').value=a[1].toFixed(2);$('poseYaw').value=yaw.toFixed(0);feedback('Start pose selected. Press Set initial pose to send it.');}draw();};
$('map').onpointercancel=()=>{drag=null;draw();};window.addEventListener('resize',draw);
poll();setTimeout(pollMaps,600);setTimeout(pollParameters,1000);setTimeout(pollLabels,500);cameraTick();
