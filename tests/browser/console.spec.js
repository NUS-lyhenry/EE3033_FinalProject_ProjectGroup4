const {test,expect}=require('@playwright/test');
async function view(page,name){await page.evaluate(p=>location.hash='/'+p,name);await expect(page.locator('.page-tabs [data-page-link="'+name+'"]')).toHaveAttribute('aria-current','page');}

test('live uncertain estimates update without becoming reported targets',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  f.state.target_estimate={x:2,y:1.5,sigma:.65,label:'bottle',samples:5,age:.2,stale:false,confirmed:false};
  await expect(page.locator('#targetEstimateStatus')).toContainText('Estimated bottle: (2.00, 1.50)');
  await expect(page.locator('#confidence')).toContainText('Estimated bottle · position σ 0.65 m');
  await expect(page.locator('#resultTarget')).toHaveText('Awaiting target');
  f.state.target_estimate={...f.state.target_estimate,x:2.3,sigma:.4,samples:8};
  await expect(page.locator('#targetEstimateStatus')).toContainText('(2.30, 1.50)');
  f.state.target_estimate.stale=true;f.state.target_estimate.age=4;
  await expect(page.locator('#targetEstimateStatus')).toContainText('last seen 4.0 s ago');
  expect(f.calls).toEqual([]);
});

test('map ruler supports two clicks and dragging without pose commands',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  await expect(page.locator('#measureMap')).toBeEnabled();
  const initialPose=await page.locator('#poseX').inputValue();
  await page.getByRole('button',{name:'Measure distance',exact:true}).click();
  const rect=await page.locator('#map').boundingBox();
  await page.mouse.click(rect.x+rect.width*.2,rect.y+rect.height*.3);
  await expect(page.locator('#measurementStatus')).toContainText('Select the second point');
  await page.mouse.click(rect.x+rect.width*.7,rect.y+rect.height*.3);
  await expect(page.locator('#measurementStatus')).toContainText('Distance: 3.00 m');
  await page.getByRole('button',{name:'Clear measurement',exact:true}).click();
  await page.mouse.move(rect.x+rect.width*.2,rect.y+rect.height*.3);await page.mouse.down();
  await page.mouse.move(rect.x+rect.width*.2,rect.y+rect.height*.8);await page.mouse.up();
  await expect(page.locator('#measurementStatus')).toContainText('Distance: 2.00 m');
  await expect(page.locator('#poseX')).toHaveValue(initialPose);
  expect(f.calls).toEqual([]);
  await page.getByRole('button',{name:'Done measuring',exact:true}).click();
  await expect(page.locator('#measureMap')).toHaveAttribute('aria-pressed','false');
});

async function fixture(page,{phase='mapping',active=false}={}){
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  const state={simulated:true,read_only:false,phase,allow_motion:true,token:'test-only',control_generation:0,
    mission:{phase,state:active?(phase==='mapping'?'MAPPING':'SEARCHING'):'WAITING',reason:'SIMULATED UI TEST · no robot motion',home:null,target:null,ready:true,return_verified:false},
    stage:{managed:true,running:true,phase,transitioning:false},assessment:{elapsed_seconds:65,started:10,finished:null,running:true,timing_valid:true,includes_mapping:true,maps:[]},
    mission_options:{target_class:'cup',localization_method:'lidar',return_tolerance:.2,preferred_target_sigma:.1,enable_ab_maneuver:false,acquisition_seconds:90},
    map_revision:1,pose:[1,1,0],route:[],path:[],health:Object.fromEntries(['Map','Lidar','Odometry','AMCL','YOLO','Mission'].map(k=>[k,{ok:true,age:.1}])),
    events:[],camera_source:null,battery:{voltage:11.87,fresh:true,age:.1},control_fault:null,initial_pose_sent:false};
  let maps=[],gap=false,audacity='bold',trial={max_linear:.12,max_angular:.4,goal_timeout:60};const calls=[];
  let attitude={success:true,mode:'fixed',topic:'/project/body_attitude',pivot:[0,0,.033],reference:null,sample:{age:.1,frame:'base_link'},ready:true,editable:true,reason:'',requires_reload:false};
  const cells=Array.from({length:2400},(_,i)=>i<60||i>=2340||i%60===0||i%60===59?100:0);
  const map={width:60,height:40,resolution:.1,origin:[0,0,0],cells};
  let stageDelay=null;
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname,method=route.request().method();
    const body=method==='POST'?route.request().postDataJSON():{};
    if(method==='POST'){
      expect(route.request().headers()['x-project-token']).toBe('test-only');
      calls.push({path,body});
      if(path!=='/api/stop')expect(route.request().headers()['x-control-generation']).toBe(String(state.control_generation));
    }
    let json={success:true};
    if(path==='/api/state')json=state;
    else if(path==='/api/map')json=map;
    else if(path==='/api/maps')json={maps};
    else if(path==='/api/labels')json={ready:true,model:'SIMULATED detector',labels:['book','bottle','cup'],error:null};
    else if(path==='/api/save_map'){
      const entry={id:'onsite_simulated/map.yaml',path:'/SIMULATED/maps/onsite_simulated/map.yaml',quality:'snapshot',reviewed:false,resolution:.1};
      maps=[entry,...maps.filter(m=>m.id!==entry.id)];state.assessment.maps.push(entry.id);state.saved_map=entry.path;
      json={success:true,path:entry.path,map:entry,message:'SIMULATED map saved'};
    }else if(path==='/api/review_map'){maps.find(m=>m.id===body.map_id).reviewed=true;json={success:true,map:maps[0]};}
    else if(path==='/api/stage'){
      if(stageDelay){await stageDelay();json={success:true,message:'SIMULATED transition stopped'};}
      else{
        state.phase=body.phase;state.stage.phase=body.phase;state.stage.running=true;state.control_generation++;
        gap=false;audacity='bold';
        state.mission={phase:body.phase,state:'WAITING',reason:'SIMULATED stage ready',home:null,target:null,ready:body.phase==='mapping',return_verified:false};
        state.initial_pose_sent=false;
        if(body.phase==='calibration')state.selected_map=maps.find(m=>m.id===body.map_id);
        if(body.phase==='mission'){state.selected_map=maps[0];state.mission_options={target_class:body.target_class,localization_method:body.localization_method,return_tolerance:body.return_tolerance,preferred_target_sigma:body.preferred_target_sigma,enable_ab_maneuver:body.enable_ab_maneuver,acquisition_seconds:body.acquisition_seconds};state.health.AMCL.ok=false;}
      }
    }else if(path==='/api/calibration_target'){state.calibration_live.target={xy:[body.x,body.y]};state.calibration_live.plan={goals:[],reverse_extent_m:.8};}
    else if(path==='/api/initial_pose'){state.initial_pose_sent=true;state.health.AMCL.ok=true;state.mission.ready=true;}
    else if(path==='/api/start'){state.mission.state=state.phase==='mapping'?'MAPPING':'SEARCHING';state.mission.home=[1,1,0];}
    else if(path==='/api/retry_search'){
      state.mission={...state.mission,state:'SEARCHING',home:[1,1,0],target:null,target_detected:false};
      state.target_estimate=null;state.route=[];state.path=[];state.control_generation++;
      state.assessment={...state.assessment,finished:null,includes_mapping:false,timing_valid:false,elapsed_seconds:0};
    }
    else if(path==='/api/map_waypoint'){state.mission.state='MAPPING';state.mission.mapping_mode='waypoint';state.mission.waypoint=[body.x,body.y,body.yaw];state.mission.waypoint_verified=false;}
    else if(path==='/api/stop'){state.mission.state='STOPPED';state.control_generation++;}
    else if(path==='/api/result')json={simulated:true,assessment:state.assessment,mission:state.mission,selected_map:state.selected_map};
    else if(path==='/api/gap'){
      if(method==='POST'){if('enabled' in body)gap=body.enabled;if('audacity' in body)audacity=body.audacity;}
      const p={robot_radius:gap?.18:.14,footprint_padding:.01,resolution:.05};
      const options=[['cautious','Cautious',.22,.16],['balanced','Balanced',.18,.15],['bold','Bold',.14,.14]].map(([id,label,c,t])=>({id,label,clearance:c+(gap?.04:0),turn_stop_distance:t,nominal_min_opening:2*Math.max(c+(gap?.04:0),p.robot_radius+.01)}));
      json={success:true,enabled:gap,audacity,audacity_options:options,synchronized:true,nodes:{'global_costmap/global_costmap':p,'local_costmap/local_costmap':p,[state.phase==='mapping'?'project_mapper':'project_mission']:{clearance:options.find(o=>o.id===audacity).clearance}}};
    }else if(path==='/api/trial'){
      if(method==='POST')trial=body;
      json={success:true,values:trial,synchronized:true,protection:{stop_distance:.22,rotation_clearance:{cautious:.16,balanced:.15,bold:.14}[audacity]}};
    }else if(path==='/api/navigation'){
      trial={max_linear:body.profile==='fast'?.16:.12,max_angular:body.profile==='fast'?.6:.4,goal_timeout:60};
      json={success:true,values:trial,synchronized:true,profile:body.profile};
    }else if(path==='/api/vision'){
      state.vision_settings=body;json={success:true,message:'Camera geometry saved'};
    }else if(path==='/api/attitude'){
      if(method==='POST'&&body.action==='configure')attitude={...attitude,...body,reason:'Settings saved for next mission load',requires_reload:true};
      if(method==='POST'&&body.action==='reference')attitude={...attitude,reference:{valid:true,roll:0.5,pitch:1.2,samples:10,span:[.1,.1]},reason:'Controller reference saved',requires_reload:true};
      json=attitude;
    }else if(path==='/api/simulation_camera'){
      state.simulation_camera={...body,revision:(state.simulation_camera.revision||0)+1};
      json={success:true,camera:state.simulation_camera,message:'Physical simulated camera updated'};
    }else if(path==='/api/simulation_scenario'){
      const scene=state.simulation_scenario;
      scene.revision++;
      if(body.action==='set_target')scene.target=[body.x,body.y,0];
      if(body.seed!=null)scene.seed=body.seed;
      json={success:true,scenario:scene,message:'Simulation scene updated'};
    }else if(path==='/api/teleop')json={success:true};
    else return route.fulfill({status:404,json:{error:'Unknown test endpoint'}});
    return route.fulfill({json:JSON.parse(JSON.stringify(json))});
  });
  await page.goto('/#/'+(phase==='mapping'?'mapping':'mission'));await expect(page.locator('#connection')).toHaveText('Connected');
  return{state,map,calls,errors,setMaps:values=>{maps=values;},setStageDelay:fn=>stageDelay=fn};
}

test('preview is explicitly simulated and cannot command robot motion',async({page,request})=>{
  await page.goto('/');await expect(page.locator('#previewNotice')).toBeVisible();
  await expect(page.locator('#start')).toBeDisabled();await expect(page.locator('#loadMapping')).toBeDisabled();
  const response=await request.post('/api/start',{data:{}});expect(response.status()).toBe(403);
});

test('Search again reuses the loaded map after a completed search without rebuilding',async({page})=>{
  const f=await fixture(page,{phase:'mission'});await view(page,'workflow');
  const entry={id:'room/map.yaml',path:'/SIMULATED/maps/room/map.yaml',reviewed:true,quality:'mapped'};
  f.setMaps([entry]);f.state.selected_map=entry;f.state.initial_pose_sent=true;
  f.state.mission={...f.state.mission,state:'FOUND_UNLOCALIZED',target_detected:true,home:[1,1,0]};
  f.state.assessment.finished=70;f.state.assessment.maps=[entry.id];
  await expect(page.getByRole('button',{name:'Search again',exact:true})).toBeVisible();
  await expect(page.locator('#retrySearch')).toBeEnabled();
  await expect(page.locator('#startMission')).toBeHidden();
  await page.locator('#retrySearch').click();
  await expect(page.locator('#state')).toHaveText('SEARCHING');
  await expect(page.locator('#retrySearch')).toBeHidden();
  expect(f.calls.map(c=>c.path)).toEqual(['/api/retry_search']);
  expect(f.state.selected_map).toEqual(entry);expect(f.state.initial_pose_sent).toBe(true);
  expect(f.state.assessment.includes_mapping).toBe(false);
  expect(f.errors).toEqual([]);
});

test('Search again waits for localization and is unavailable during recovery or transition',async({page})=>{
  const f=await fixture(page,{phase:'mission'});await view(page,'workflow');
  f.state.initial_pose_sent=true;f.state.mission.state='FAILED';
  f.state.mission.ready=false;
  await expect(page.locator('#retrySearch')).toBeVisible();
  await expect(page.locator('#retrySearch')).toBeDisabled();
  await expect(page.locator('#loadMission')).toHaveText('Reload map & set start pose');
  f.state.mission.ready=true;f.state.stage.transitioning=true;
  await expect(page.locator('#retrySearch')).toBeDisabled();
  f.state.stage.transitioning=false;
  await expect(page.locator('#retrySearch')).toBeEnabled();
  f.state.mission.state='RECOVERING';
  await expect(page.locator('#retrySearch')).toBeHidden();
  await expect(page.locator('#newTrial')).toBeDisabled();
  await expect(page.locator('#loadMapping')).toBeDisabled();
  expect(f.calls).toEqual([]);expect(f.errors).toEqual([]);
});

test('workflow can load a reviewed saved map after restart without rebuilding or starting motion',async({page})=>{
  const f=await fixture(page);await view(page,'workflow');
  const entry={id:'previous_room/map.yaml',path:'/SIMULATED/maps/previous_room/map.yaml',reviewed:true,quality:'mapped'};
  f.setMaps([entry]);f.state.assessment.finished=70;f.state.stage.running=false;
  await page.locator('#refreshMaps').click();
  await expect(page.locator('#mapLibrary')).toBeVisible();
  await expect(page.locator('#savedMaps')).toBeVisible();
  await expect(page.getByRole('button',{name:'Use saved map',exact:true})).toBeEnabled();
  await page.locator('#useSavedMap').click();
  await expect(page.locator('#phase')).toHaveText('Stage 2 · Autonomous');
  await expect(page.locator('#setup')).toBeVisible();
  await expect(page.locator('#startMission')).toBeDisabled();
  expect(f.calls.filter(c=>c.path==='/api/stage').map(c=>c.body.map_id)).toEqual([entry.id]);
  expect(f.calls.some(c=>['/api/new_trial','/api/start'].includes(c.path))).toBe(false);
  expect(f.errors).toEqual([]);
});

test('localization work and waypoint travel use separate displayed timers',async({page})=>{
  const f=await fixture(page,{phase:'mission'});await view(page,'workflow');
  f.state.mission={...f.state.mission,state:'LOCALIZING',target_detected:true,
    localization_elapsed_seconds:68.2,localization_work_seconds:8.2,localization_active_seconds:8.2,
    localization_travel_seconds:61.4,acquisition_seconds:90,
    localization_stage:'B: collecting settled bearing frames',
    bearing_progress:{sites:[{ready:true,frames:4},{ready:false,frames:2}]}};
  await expect(page.locator('#localizationProgress')).toContainText('68.2 / 90 s wall time (includes travel)');
  await expect(page.locator('#localizationProgress')).toContainText('work 8.2 s · travel 61.4 s');
  await expect(page.locator('#localizationProgress')).toContainText('A: ready · B: 2/4 frames');
  expect(f.errors).toEqual([]);
});

test('camera setup is a preparation page and shows saved profile reuse',async({page})=>{
  const f=await fixture(page);f.state.stage.running=false;
  await page.route('http://127.0.0.1:8780/**',route=>route.fulfill({contentType:'text/html',body:'Camera setup fixture'}));
  f.setMaps([{id:'room/map.yaml',path:'/maps/room/map.yaml',reviewed:true}]);await page.locator('#refreshMaps').click();
  await view(page,'calibration');await expect(page.locator('#loadCalibration')).toBeEnabled();
  await page.locator('#loadCalibration').click();
  expect(f.calls.find(c=>c.path==='/api/stage').body).toEqual({phase:'calibration',map_id:'room/map.yaml'});
  f.state.calibration_live={ready:true,motion:'idle'};
  f.state.camera_calibration={ready:true,path:'/SIMULATED/camera_checked.json',message:'Saved calibration will be used automatically when the mission loads.'};
  await expect(page.locator('#calibrationFrame')).toBeVisible();
  await expect(page.locator('#calibrationBadge')).toHaveText('Saved calibration ready');
  await expect(page.locator('#calibrationSummary')).toContainText('automatically');
  await expect(page.locator('#startMission')).toBeDisabled();
  await expect(page.locator('#stop')).toBeEnabled();
  expect(f.errors).toEqual([]);
});

test('mapping reports excluded failures and bounded navigation recovery',async({page})=>{
  const f=await fixture(page,{active:true});
  Object.assign(f.state.mission,{viewpoints_completed:2,failed_viewpoints:[[1,1]]});
  await expect(page.locator('#mappingProgress')).toContainText('2 goals reached · 1 failed areas skipped');
  await expect(page.locator('#mappingProgress')).toContainText('Bounded navigation recovery');
  await expect(page.locator('#mappingProgress')).not.toContainText('retry pass');
  expect(f.errors).toEqual([]);
});

test('map distinguishes global route, local prediction, travelled path and recovery state',async({page})=>{
  const f=await fixture(page,{active:true});
  Object.assign(f.state,{path:[[1,1],[2,1]],local_path:[[1,1],[1.1,1.05]],
    travelled_path:[[.7,1],[1,1]],footprint_radius:.116801,
    robot_geometry:{profile:'measured',length:.115,width:.18},
    navigation:{stage:'REVERSING',reason:'Known rear corridor verified',recovery_count:1}});
  await expect(page.locator('#navigationStatus')).toContainText('Backing up safely');
  await expect(page.locator('#navigationStatus')).toContainText('Recovery 1/2');
  for(const name of ['Global path','Local prediction','Travelled']){
    await expect(page.locator('.legend').getByText(name,{exact:true})).toBeVisible();
  }
  await expect(page.locator('#stop')).toBeEnabled();
  expect(f.errors).toEqual([]);
});

test('complete basic workflow: map save review, stage transition, assigned pose, search, target and verified return',async({page})=>{
  const f=await fixture(page);
  await view(page,'navigation');
  await expect(page.locator('[data-audacity="bold"]')).toHaveAttribute('aria-pressed','true');
  await page.locator('[data-audacity="cautious"]').click();
  await expect(page.locator('[data-audacity="cautious"]')).toHaveAttribute('aria-pressed','true');
  await view(page,'mapping');
  await expect(page.locator('#totalElapsed')).toHaveText('01:05');
  await page.locator('#saveMap').click();await expect(page.locator('#savedMaps')).toContainText('onsite_simulated');
  await expect(page.locator('#loadMission')).toBeDisabled();
  await page.locator('#reviewMap').click();await expect(page.locator('#mapQuality')).toContainText('Reviewed');
  await view(page,'mission');
  await page.locator('#localizationMethod').selectOption('ground_plane');
  await page.locator('#returnTolerance').fill('0.15');
  await page.locator('#loadMission').click();
  await expect(page.locator('#phase')).toHaveText('Stage 2 · Autonomous');
  await view(page,'navigation');
  await expect(page.locator('[data-audacity="bold"]')).toHaveAttribute('aria-pressed','true');
  await view(page,'mission');
  const load=f.calls.find(c=>c.path==='/api/stage');expect(load.body).toEqual({phase:'mission',map_id:'onsite_simulated/map.yaml',target_class:'cup',localization_method:'ground_plane',return_tolerance:.15,preferred_target_sigma:.1,enable_ab_maneuver:false,acquisition_seconds:90});
  await expect(page.locator('#mappingControls')).toBeHidden();await expect(page.locator('#startMission')).toBeDisabled();
  await page.locator('#poseX').fill('1');await page.locator('#poseY').fill('1');await page.locator('#poseYaw').fill('90');
  await page.locator('#setPose').click();await expect(page.locator('#startMission')).toBeEnabled();
  const pose=f.calls.find(c=>c.path==='/api/initial_pose');expect(pose.body.yaw).toBeCloseTo(Math.PI/2);
  await page.locator('#startMission').click();await expect(page.locator('#state')).toHaveText('SEARCHING');
  await expect(page.locator('#setPose')).toBeDisabled();await expect(page.locator('#loadMapping')).toBeDisabled();
  f.state.mission={...f.state.mission,state:'RETURNING',target:{x:2.1,y:1.8,class:'cup',confidence:.88,variance:.01},return_distance:.46,return_tolerance:.15,viewpoints_total:6,viewpoints_completed:4,observed_fraction:.67};
  await expect(page.locator('#target')).toHaveText('2.10, 1.80 m');await expect(page.locator('#confidence')).toContainText('88%');
  await expect(page.locator('#resultStop')).toHaveText('Pending');await expect(page.locator('#coverage')).toHaveText('67%');
  f.state.mission.state='VERIFYING';await expect(page.locator('#resultStop')).toContainText('Checking');
  f.state.mission={...f.state.mission,state:'COMPLETE',return_verified:true,return_distance:.08,reason:'SIMULATED home distance and stationary hold verified'};
  f.state.assessment={...f.state.assessment,elapsed_seconds:195,finished:205,running:false};
  await expect(page.locator('#resultOutcome')).toHaveText('Complete');await expect(page.locator('#resultReturn')).toHaveText('Verified · 0.08 m');
  await expect(page.locator('#resultStop')).toHaveText('Verified stationary');await expect(page.locator('#totalElapsed')).toHaveText('03:15');
  await view(page,'results');
  const download=page.waitForEvent('download');await page.locator('#exportResult').click();expect((await download).suggestedFilename()).toBe('assessment.json');
  expect(f.errors).toEqual([]);await page.evaluate(()=>window.scrollTo({top:0,behavior:'instant'}));await page.screenshot({path:'test-results/simulated-basic-workflow.png',fullPage:true});
});

test('advanced mission options submit the opt-in maneuver and acquisition window',async({page})=>{
  const f=await fixture(page);await view(page,'workflow');
  await page.locator('#saveMap').click();await page.locator('#reviewMap').click();
  await page.locator('#missionAdvanced').evaluate(el=>el.open=true);
  await page.locator('#enableAbManeuver').check();await page.locator('#acquisitionSeconds').fill('120');
  await expect(page.locator('#loadMission')).toContainText('Apply settings');
  await page.locator('#loadMission').click();
  const body=f.calls.filter(c=>c.path==='/api/stage').at(-1).body;
  expect(body.enable_ab_maneuver).toBe(true);expect(body.acquisition_seconds).toBe(120);
  expect(f.state.mission_options.enable_ab_maneuver).toBe(true);
  expect(f.errors).toEqual([]);
});

test('position confidence stays separate from detector confidence and reports estimate freshness',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  f.state.mission={...f.state.mission,state:'COMPLETE',return_verified:true,target:{x:2,y:1,class:'cup',confidence:.84,
    position_confidence:.91,confidence_radius_m:.2,sigma:.12,quality:'approximate',stale:true,age:2.5,preferred_sigma:.1},reason:'Returned inside assigned tolerance'};
  await expect(page.locator('#confidence')).toContainText('84% detector confidence');
  await expect(page.locator('#resultPositionConfidence')).toHaveText('91% within 0.20 m · stale · 2.5 s old');
  await expect(page.locator('#resultTarget')).toContainText('approximate estimate');
  await expect(page.locator('#resultOutcome')).toHaveText('Complete · estimated confidence 91% within 0.20 m');
  await expect(page.locator('#resultReason')).toContainText('Returned inside assigned tolerance');
  expect(f.errors).toEqual([]);
});

test('attitude settings save a controller mode and capture a reference without submitting angles',async({page})=>{
  const f=await fixture(page,{phase:'mission'});await view(page,'navigation');
  await page.locator('#attitudeMode').selectOption('controller');
  await expect(page.locator('#attitudeX')).toHaveValue('-0.01');
  await page.locator('#saveAttitude').click();
  await expect.poll(()=>f.calls.some(c=>c.path==='/api/attitude'&&c.body.action==='configure')).toBe(true);
  const configure=f.calls.find(c=>c.path==='/api/attitude'&&c.body.action==='configure');
  expect(configure.body.pivot[0]).toBe(-.01);
  await page.locator('#referenceAttitude').click();
  await expect.poll(()=>f.calls.some(c=>c.path==='/api/attitude'&&c.body.action==='reference')).toBe(true);
  expect(f.calls.find(c=>c.path==='/api/attitude'&&c.body.action==='reference').body).toEqual({action:'reference'});
  await expect(page.locator('#attitudeStatus')).toContainText('Reference captured');
  expect(f.errors).toEqual([]);
});

for(const phase of ['mapping','mission'])test(`active ${phase} locks setup and manual intervention`,async({page})=>{
  await fixture(page,{phase,active:true});
  await expect(page.locator('#loadMapping')).toBeDisabled();await expect(page.locator('#loadMission')).toBeDisabled();
  await expect(page.locator('#saveMap')).toBeDisabled();await expect(page.locator('#setPose')).toBeDisabled();
  await expect(page.locator('#targetClass')).toBeDisabled();await expect(page.locator('#stop')).toBeEnabled();
  await view(page,'navigation');await expect(page.locator('#gapToggle')).toBeDisabled();
  for(const level of ['cautious','balanced','bold'])await expect(page.locator(`[data-audacity="${level}"]`)).toBeDisabled();
  if(phase==='mapping'){await view(page,'mapping');await page.locator('#mappingControls > summary').click();await expect(page.getByLabel('Drive forward')).toBeDisabled();}
});

test('unverified home success and absent targets never look like a complete mission',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  f.state.mission={...f.state.mission,state:'COMPLETE',target:{x:1,y:2,class:'cup'},return_verified:false};
  await expect(page.locator('#state')).toHaveText('COMPLETE');await expect(page.locator('#resultOutcome')).toHaveText('Trial not complete');
  f.state.mission={...f.state.mission,state:'NOT_FOUND',target:null,return_verified:true,return_distance:.05};
  await expect(page.locator('#resultOutcome')).toHaveText('Search completed · target not found');await expect(page.locator('#resultTarget')).toHaveText('Awaiting target');
});

test('target selector contains only model labels and locks mission setup until labels arrive',async({page})=>{
  const f=await fixture(page);await view(page,'mission');
  await expect(page.locator('#targetClass')).toHaveJSProperty('tagName','SELECT');
  await expect(page.locator('#targetClass')).toBeEnabled();
  await expect(page.locator('#targetClass option')).toHaveText(['Select a target class','book','bottle','cup']);
  await expect(page.locator('#targetClass')).toHaveValue('cup');
  await page.locator('#targetClass').selectOption('bottle');
  await expect(page.locator('#targetClass')).toHaveValue('bottle');
  await page.route('**/api/labels',r=>r.fulfill({json:{ready:false,labels:[],model:'SIMULATED detector',error:'Model labels unavailable'}}));
  await expect(page.locator('#targetClass')).toBeDisabled();
  await expect(page.locator('#loadMission')).toBeDisabled();
  expect(f.errors).toEqual([]);
});

test('stale telemetry and disconnection disable driving and mark unavailable data',async({page})=>{
  const f=await fixture(page);f.state.health.Mission.ok=false;f.state.battery.fresh=false;f.state.battery.age=6;
  await expect(page.locator('#start')).toBeDisabled();await expect(page.locator('#battery')).toHaveText('Battery 11.87 V · stale');
  await page.route('**/api/state',r=>r.abort());await expect(page.locator('#connectionBanner')).toBeVisible();
  await expect(page.locator('#setPose')).toBeDisabled();await expect(page.locator('#loadMapping')).toBeDisabled();
});

test('optional gap reserve and trial settings retain verified readback',async({page})=>{
  const f=await fixture(page);await view(page,'navigation');
  await expect(page.locator('#gapToggle')).toBeEnabled();await page.locator('#gapToggle').click();
  await expect(page.locator('#gapToggle')).toHaveAttribute('aria-pressed','true');
  await expect(page.locator('#gapStatus')).toContainText('clearance 0.18 m');
  await expect(page.locator('#trialLinear')).toBeEnabled();await page.locator('#trialLinear').focus();await page.keyboard.press('End');
  await expect(page.locator('#trialLinearValue')).toHaveText('0.18 m/s');await page.locator('#applyTrial').click();
  await expect(page.locator('#trialLinearLive')).toHaveText('Live: 0.18 m/s');
  expect(f.calls.find(c=>c.path==='/api/trial').body.max_linear).toBe(.18);
});

for(const phase of ['mapping','mission'])test(`three audacity levels use verified readback and retain gap reserve in ${phase}`,async({page})=>{
  const f=await fixture(page,{phase});await view(page,'navigation');
  const cautious=page.getByRole('button',{name:/1 · Cautious/}),balanced=page.getByRole('button',{name:/2 · Balanced/}),bold=page.getByRole('button',{name:/3 · Bold/});
  await expect(bold).toHaveAttribute('aria-pressed','true');
  await expect(cautious).toContainText('22 cm clearance · ~44 cm opening');
  await balanced.click();await expect(balanced).toHaveAttribute('aria-pressed','true');
  await expect(balanced).toContainText('18 cm clearance · ~36 cm opening');
  await expect(balanced).toContainText('Turn stop 15 cm · gentle turning');
  await bold.click();await expect(bold).toHaveAttribute('aria-pressed','true');
  await expect(bold).toContainText('14 cm clearance · ~30 cm opening');
  await expect(bold).toContainText('Turn stop 14 cm · firm turning');
  expect(f.calls.filter(c=>c.path==='/api/gap').map(c=>c.body)).toEqual([{audacity:'balanced',enabled:false},{audacity:'bold',enabled:false}]);
  await page.locator('#gapToggle').click();
  await expect(bold).toHaveAttribute('aria-pressed','true');await expect(bold).toContainText('18 cm clearance · ~38 cm opening');
  await cautious.click();await expect(cautious).toContainText('26 cm clearance · ~52 cm opening');
  await expect(page.locator('#gapToggle')).toHaveAttribute('aria-pressed','true');
  expect(f.calls.some(c=>c.path==='/api/start')).toBe(false);
  expect(f.errors).toEqual([]);
  if(phase==='mapping')await page.screenshot({path:'test-results/simulated-audacity-levels.png',fullPage:true});
});

test('a rejected audacity change never appears applied',async({page})=>{
  const f=await fixture(page);await view(page,'navigation');
  const cautious=page.locator('[data-audacity="cautious"]'),bold=page.locator('[data-audacity="bold"]');
  await expect(bold).toHaveAttribute('aria-pressed','true');
  await page.route('**/api/gap',route=>route.request().method()==='POST'?route.fulfill({status:503,json:{error:'Parameter update rejected; original values restored'}}):route.fallback());
  await cautious.click();await expect(page.locator('#feedback')).toContainText('original values restored');
  await expect(cautious).toHaveAttribute('aria-pressed','false');
  await expect(page.locator('#audacityLive')).toHaveText('Unverified');
  expect(f.errors).toEqual([]);
});

test('mapping reload restores Bold after choosing a different level',async({page})=>{
  const f=await fixture(page);await view(page,'navigation');
  await page.locator('[data-audacity="cautious"]').click();
  await expect(page.locator('[data-audacity="cautious"]')).toHaveAttribute('aria-pressed','true');
  await view(page,'mapping');await page.locator('#loadMapping').click();
  await expect(page.locator('#loadMapping')).toBeEnabled();
  await view(page,'navigation');
  await expect(page.locator('[data-audacity="bold"]')).toHaveAttribute('aria-pressed','true');
  expect(f.errors).toEqual([]);
});

test('stationary window progress and failure keep the detected target visible',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  Object.assign(f.state.mission,{state:'VERIFYING',target:{x:.38,y:-.53,class:'bottle'},
    stationary_required_seconds:2,stationary_check:{duration:1.4,reason:'Collecting a complete stationary window'}});
  await view(page,'results');
  await expect(page.locator('#resultStop')).toHaveText('Checking 1.4 / 2.0 s…');
  await expect(page.locator('#resultReason')).toContainText('Collecting a complete stationary window');
  Object.assign(f.state.mission,{state:'FAILED',reason:'Return verification failed: Position is still drifting'});
  await expect(page.locator('#resultStop')).toHaveText('Not verified');
  await expect(page.locator('#resultTarget')).toHaveText('(0.38, -0.53) m');
  await expect(page.locator('#resultReason')).toContainText('Position is still drifting');
  expect(f.errors).toEqual([]);
});

test('Stop remains available while a stage request is pending',async({page})=>{
  const f=await fixture(page);let release;f.setStageDelay(()=>new Promise(r=>release=r));
  await page.locator('#loadMapping').click();await expect(page.locator('#loadMapping')).toBeDisabled();
  await page.locator('#stop').click();await expect(page.locator('#state')).toHaveText('STOPPED');
  expect(f.calls.map(c=>c.path)).toContain('/api/stop');release();
  await expect(page.locator('#loadMapping')).toBeEnabled();
});

test('subpages separate functions and retain Stop and direct links',async({page})=>{
  const f=await fixture(page);
  for(const [name,panel] of [['overview','.workspace-grid'],['mapping','#mappingPanel'],['mission','#missionPanel'],['navigation','#audacityPanel'],['results','#resultsPanel']]){
    await view(page,name);await expect(page.locator(panel)).toBeVisible();await expect(page.locator('#stop')).toBeVisible();
    if(name!=='mapping')await expect(page.locator('#mappingPanel')).toBeHidden();
    if(name!=='mission')await expect(page.locator('#missionPanel')).toBeHidden();
    if(name!=='navigation')await expect(page.locator('#audacityPanel')).toBeHidden();
  }
  await page.reload();await expect(page.locator('#resultsPanel')).toBeVisible();
  expect(f.errors).toEqual([]);
});

test('mapping waypoint drag sets position and final facing, then locks controls until stopped',async({page})=>{
  const f=await fixture(page);
  f.map.cells[20*60+30]=-1;f.state.map_revision++;
  await expect(page.locator('#waypointX')).toBeEnabled();
  await page.locator('#map').scrollIntoViewIfNeeded();const box=await page.locator('#map').boundingBox();
  await page.mouse.move(box.x+box.width*.5,box.y+box.height*.5);await page.mouse.down();
  await page.mouse.move(box.x+box.width*.5,box.y+box.height*.3);await page.mouse.up();
  await expect(page.locator('#waypointYaw')).toHaveValue('90');
  await expect(page.locator('#goWaypoint')).toBeEnabled();await page.locator('#goWaypoint').click();
  const goal=f.calls.find(c=>c.path==='/api/map_waypoint');expect(goal.body.x).toBeCloseTo(3,1);expect(goal.body.y).toBeCloseTo(2,1);expect(goal.body.yaw).toBeCloseTo(Math.PI/2);
  await expect(page.locator('#goWaypoint')).toBeDisabled();await expect(page.locator('#waypointYaw')).toBeDisabled();
  f.state.mission.state='TURNING';await expect(page.locator('#state')).toHaveText('TURNING');
  await expect(page.locator('#newTrial')).toBeDisabled();
  await page.locator('#stop').click();await expect(page.locator('#state')).toHaveText('STOPPED');
  expect(f.calls.filter(c=>c.path==='/api/map_waypoint')).toHaveLength(1);expect(f.errors).toEqual([]);
});

test('mission drag sets initial pose and cannot send a mapping waypoint',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  await expect(page.locator('#waypointControls')).toBeHidden();
  const box=await page.locator('#map').boundingBox();
  await page.mouse.move(box.x+box.width*.5,box.y+box.height*.5);await page.mouse.down();
  await page.mouse.move(box.x+box.width*.7,box.y+box.height*.5);await page.mouse.up();
  await expect(page.locator('#poseYaw')).toHaveValue('0');await page.locator('#setPose').click();
  expect(f.calls.some(c=>c.path==='/api/initial_pose')).toBe(true);
  expect(f.calls.some(c=>c.path==='/api/map_waypoint')).toBe(false);expect(f.errors).toEqual([]);
});

test('mobile layout keeps required controls on screen without horizontal overflow',async({page})=>{
  await page.setViewportSize({width:390,height:844});const f=await fixture(page);
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await view(page,'mission');
  await expect(page.locator('#stop')).toBeInViewport();await expect(page.getByLabel('Detector target class')).toBeVisible();
  for(const name of ['overview','mapping','mission','navigation','results']){await view(page,name);expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);await expect(page.locator('#battery')).toBeVisible();await expect(page.locator('#connection')).toBeVisible();await expect(page.locator('#stop')).toBeInViewport();}
  expect(f.errors).toEqual([]);await page.evaluate(()=>window.scrollTo({top:0,behavior:'instant'}));await page.screenshot({path:'test-results/simulated-mobile-console.png',fullPage:true});
});

test('camera setup forwards the reviewed map and can establish localization while idle',async({page})=>{
  const f=await fixture(page);
  await page.locator('#saveMap').click();await expect(page.locator('#reviewMap')).toBeEnabled();
  await page.locator('#reviewMap').click();await view(page,'calibration');
  await page.locator('#loadCalibration').click();
  expect(f.calls.find(c=>c.path==='/api/stage').body).toEqual({phase:'calibration',map_id:'onsite_simulated/map.yaml'});
  await page.getByText('Establish localization if needed',{exact:true}).click();
  await expect(page.locator('#calSetPose')).toBeEnabled();
  await page.locator('#calPoseX').fill('1.2');await page.locator('#calPoseY').fill('1.1');await page.locator('#calPoseYaw').fill('90');
  await page.locator('#calSetPose').click();
  expect(f.calls.find(c=>c.path==='/api/initial_pose').body).toEqual({x:1.2,y:1.1,yaw:Math.PI/2});
  f.state.mission.state='CALIBRATING';
  await expect(page.locator('#calSetPose')).toBeDisabled();expect(f.errors).toEqual([]);
});


test('newly saved map stays selected over the map loaded by an earlier stage',async({page})=>{
  const f=await fixture(page);
  const old={id:'onsite_old/map.yaml',path:'/SIMULATED/maps/onsite_old/map.yaml',reviewed:true,quality:'mapped'};
  f.setMaps([old]);f.state.selected_map=old;
  await page.locator('#refreshMaps').click();
  await expect(page.locator('#savedMaps')).toHaveValue(old.id);
  await expect(page.locator('#reviewMap')).toBeDisabled();
  await page.locator('#saveMap').click();
  await expect(page.locator('#savedMaps')).toHaveValue('onsite_simulated/map.yaml');
  await page.locator('#refreshMaps').click();
  await expect(page.locator('#savedMaps')).toHaveValue('onsite_simulated/map.yaml');
  await page.locator('#reviewMap').click();await view(page,'calibration');
  await expect(page.locator('#calibrationMapChoice')).toContainText('onsite_simulated/map.yaml');
  await page.locator('#loadCalibration').click();
  expect(f.calls.find(c=>c.path==='/api/stage').body.map_id).toBe('onsite_simulated/map.yaml');
  expect(f.errors).toEqual([]);
});

test('unsaved live mapping cannot silently load or review an old calibration map',async({page})=>{
  const f=await fixture(page);f.setMaps([{id:'old/map.yaml',path:'/old/map.yaml',reviewed:true,quality:'mapped'}]);
  await page.locator('#refreshMaps').click();await expect(page.locator('#reviewMap')).toBeDisabled();
  await view(page,'calibration');await expect(page.locator('#loadCalibration')).toBeDisabled();
  await expect(page.locator('#calibrationMapChoice')).toContainText('Save the current mapping session');
  expect(f.calls.some(c=>c.path==='/api/stage')).toBe(false);
});


test('camera map tool sends bottle coordinates on the currently loaded map',async({page})=>{
  const f=await fixture(page);f.state.phase='calibration';f.state.stage.phase='calibration';
  f.state.selected_map={id:'current/map.yaml',path:'/maps/current/map.yaml'};
  f.state.calibration_live={ready:true,motion:'idle'};
  await page.route('http://127.0.0.1:8780/**',r=>r.fulfill({contentType:'text/html',body:'Camera fixture'}));
  await view(page,'calibration');await expect(page.locator('#calSetBottle')).toBeEnabled();
  await expect(page.locator('#calMapTool')).toHaveValue('bottle');
  const b=await page.locator('#map').boundingBox();await page.mouse.click(b.x+b.width*.4,b.y+b.height*.5);
  await expect.poll(()=>f.calls.filter(c=>c.path==='/api/calibration_target').length).toBe(1);
  const call=f.calls.find(c=>c.path==='/api/calibration_target');
  expect(call.body.map_id).toBe('current/map.yaml');expect(call.body.x).toBeCloseTo(2.4,1);expect(call.body.y).toBeCloseTo(2,1);
  await expect(page.locator('#calBottleStatus')).toContainText('Bottle base marker');
  expect(f.calls.some(c=>c.path==='/api/initial_pose')).toBe(false);expect(f.errors).toEqual([]);
});

test('single-page workflow hides optional tools and prepares a visual mission after map confirmation',async({page})=>{
  const f=await fixture(page);await view(page,'workflow');
  await expect(page.locator('#advancedTools')).not.toHaveAttribute('open','');
  await expect(page.locator('#loadMapping')).toBeHidden();await expect(page.locator('#localizationMethod')).toBeHidden();
  await expect(page.locator('#profileFast')).toBeHidden();await expect(page.locator('#mappingControls')).toBeHidden();
  await expect(page.locator('#targetClass')).toBeHidden();
  await page.locator('#saveMap').click();await expect(page.locator('#targetClass')).toBeVisible();await page.locator('#reviewMap').click();
  await expect(page.locator('#phase')).toHaveText('Stage 2 · Autonomous');
  expect(f.calls.find(c=>c.path==='/api/stage').body.localization_method).toBe('hybrid');
  await expect(page.locator('#setup')).toBeVisible();await page.locator('#setPose').click();
  await page.locator('#startMission').click();await expect(page.locator('#state')).toHaveText('SEARCHING');
  f.state.mission.state='ACQUIRING';await expect(page.locator('#startMission')).toBeDisabled();
  f.state.mission.state='LOCALIZING';await expect(page.locator('#setPose')).toBeDisabled();
  await page.locator('#stop').click();await expect(page.locator('#state')).toHaveText('STOPPED');
  expect(f.errors).toEqual([]);
});

test('default workflow can apply a changed target without starting the mission',async({page})=>{
  const f=await fixture(page);await view(page,'workflow');
  await page.locator('#saveMap').click();await page.locator('#reviewMap').click();
  await expect(page.locator('#phase')).toHaveText('Stage 2 · Autonomous');
  await page.locator('#setPose').click();await expect(page.locator('#startMission')).toBeEnabled();
  await expect(page.locator('#loadMission')).toBeHidden();
  await page.locator('#targetClass').selectOption('bottle');
  await expect(page.locator('#startMission')).toBeDisabled();
  const apply=page.getByRole('button',{name:'Apply settings & reload mission',exact:true});
  await expect(apply).toBeVisible();await expect(apply).toBeEnabled();
  await expect(page.locator('#modelLabelsStatus')).toContainText('set the initial pose again');
  f.state.mission.state='SEARCHING';await expect(apply).toBeDisabled();
  f.state.mission.state='WAITING';await expect(apply).toBeEnabled();
  await apply.click();
  await expect(page.locator('#loadMission')).toBeHidden();
  expect(f.calls.filter(c=>c.path==='/api/stage').at(-1).body).toEqual({
    phase:'mission',map_id:'onsite_simulated/map.yaml',target_class:'bottle',
    localization_method:'hybrid',return_tolerance:.2,preferred_target_sigma:.1,enable_ab_maneuver:false,acquisition_seconds:90});
  expect(new URL(page.url()).hash).toBe('#/workflow');
  await expect(page.locator('#startMission')).toBeDisabled();
  await page.locator('#setPose').click();await expect(page.locator('#startMission')).toBeEnabled();
  expect(f.calls.some(c=>c.path==='/api/start')).toBe(false);expect(f.errors).toEqual([]);
});

test('advanced camera settings and fast profile use explicit values and verified responses',async({page})=>{
  const f=await fixture(page);await view(page,'workflow');
  await page.locator('#advancedTools > summary').click();
  await page.locator('#visionHeight').fill('0.22');await page.locator('#visionPitch').fill('10');
  await page.locator('#saveVision').click();
  await expect(page.locator('#feedback')).toHaveText('Camera geometry saved');
  const settings=f.calls.find(c=>c.path==='/api/vision').body;
  expect(settings.camera_height).toBe(.22);expect(settings.camera_pitch).toBeCloseTo(Math.PI/18);
  await page.locator('#profileFast').click();
  await expect(page.locator('#navigationProfileStatus')).toContainText('Applied and verified: fast');
  expect(f.calls.find(c=>c.path==='/api/navigation').body).toEqual({profile:'fast'});
  expect(f.errors).toEqual([]);
});

test('single-page mobile workflow keeps Stop visible and has no horizontal overflow',async({page})=>{
  await page.setViewportSize({width:390,height:844});const f=await fixture(page);await view(page,'workflow');
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await expect(page.locator('#stop')).toBeInViewport();
  await page.screenshot({path:'test-results/workflow-mobile.png',fullPage:true});
  expect(f.errors).toEqual([]);
});

test('single-page desktop workflow has a compact primary path',async({page})=>{
  const f=await fixture(page);await view(page,'workflow');
  await page.screenshot({path:'test-results/workflow-desktop.png',fullPage:true});
  await expect(page.locator('.sidebar')).toBeHidden();expect(f.errors).toEqual([]);
});

test('best available target keeps uncertainty visible after returning successfully',async({page})=>{
  const f=await fixture(page,{phase:'mission'});await view(page,'workflow');
  Object.assign(f.state.mission,{state:'COMPLETE',home:[1,1,0],return_verified:true,return_distance:0,
    target:{x:2.1,y:1.8,class:'cup',confidence:.9,variance:.0625,sigma:.25,preferred_sigma:.1,quality:'best_available'}});
  await expect(page.locator('#confidence')).toContainText('σ 0.25 m');
  await expect(page.locator('#confidence')).toContainText('preferred precision not reached');
  await expect(page.locator('#resultTarget')).toContainText('approximate estimate');
  await expect(page.locator('#resultReason')).toContainText('preferred 0.10 m');
  await expect(page.locator('#resultOutcome')).toHaveText('Complete');
  expect(f.errors).toEqual([]);
});

test('preferred MMSE precision is applied through a reload and locked during search',async({page})=>{
  const f=await fixture(page);await view(page,'workflow');
  await page.locator('#saveMap').click();await page.locator('#reviewMap').click();
  await expect(page.locator('#phase')).toHaveText('Stage 2 · Autonomous');
  await page.locator('#setPose').click();await expect(page.locator('#startMission')).toBeEnabled();
  await expect(page.getByLabel('Preferred precision (σ, m)')).toHaveValue('0.1');
  await page.getByLabel('Preferred precision (σ, m)').fill('0.25');
  await expect(page.locator('#startMission')).toBeDisabled();
  const apply=page.getByRole('button',{name:'Apply settings & reload mission',exact:true});
  await expect(apply).toBeEnabled();
  await apply.click();await expect(apply).toBeHidden();
  expect(f.calls.filter(c=>c.path==='/api/stage').at(-1).body.preferred_target_sigma).toBe(.25);
  await expect(page.locator('#preferredSigma')).toHaveValue('0.25');
  await page.locator('#setPose').click();await page.locator('#startMission').click();
  await expect(page.locator('#preferredSigma')).toBeDisabled();
  expect(f.errors).toEqual([]);
});

test('accepted map display uses the reported posterior even while a new message is pending',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  f.state.target_estimate={x:7,y:8,sigma:.8,label:'bottle',samples:7,age:0,stale:false};
  f.state.mission.target={x:2.1,y:1.8,class:'bottle',sigma:.15,variance:.0225,quality:'best_available'};
  await expect(page.locator('#target')).toHaveText('2.10, 1.80 m');
  await expect(page.locator('#targetEstimateStatus')).toContainText('Estimated bottle: (2.10, 1.80) m · σ 0.15 m');
  await expect(page.locator('#targetEstimateStatus')).toContainText('accepted');
  expect(f.calls).toEqual([]);
});

test('older robot software cannot silently ignore the new precision setting',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  delete f.state.mission_options.preferred_target_sigma;
  await expect(page.locator('#preferredSigma')).toBeDisabled();
  await expect(page.locator('#preferredSigmaHelp')).toContainText('Robot software update required');
  expect(f.calls).toEqual([]);
});


test('found without coordinates and incomplete search have distinct return outcomes',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  f.state.mission={...f.state.mission,state:'FOUND_UNLOCALIZED',target_detected:true,target:null,return_verified:true,reason:'No safe observation remains'};
  await expect(page.locator('#resultOutcome')).toHaveText('Target found · position unavailable');
  await expect(page.locator('#resultReturn')).toContainText('Verified');
  f.state.mission={...f.state.mission,state:'SEARCH_INCOMPLETE',target_detected:false,reason:'Wall-side region remains blocked'};
  await expect(page.locator('#resultOutcome')).toHaveText('Search incomplete');
  await expect(page.locator('#resultReason')).toContainText('Wall-side region');
  f.state.mission={...f.state.mission,state:'STOPPED',internal_error:'Safety stop: navigation fault'};
  await expect(page.locator('#resultOutcome')).toHaveText('Stopped · safety error');
  await expect(page.locator('#resultReason')).toContainText('Safety stop');
});

test('TF pitch override is saved as a boolean with radians',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  await page.goto('/#/workflow');await page.locator('#advancedTools').evaluate(el=>el.open=true);
  await page.locator('#visionPitch').fill('28.6');
  await page.locator('#visionPitchOverride').check();
  await page.locator('#saveVision').click();
  await expect.poll(()=>f.calls.some(c=>c.path==='/api/vision')).toBe(true);
  const body=f.calls.find(c=>c.path==='/api/vision').body;
  expect(body.use_pitch_override).toBe(true);expect(body.camera_pitch).toBeCloseTo(28.6*Math.PI/180,8);
});

function enableScenario(f){
  Object.assign(f.state,{preview_backend:'main-ros',simulation_can_apply:true,
    simulation_camera:{camera_height:.22,camera_pitch:10*Math.PI/180,camera_yaw:0,camera_x:.05,camera_y:0,horizontal_fov:0,camera_min_range:.35,camera_max_range:2,revision:0},
    simulation_scenario:{revision:1,seed:42,bounds:[0,0,6,4],start:[1,1,0],target:[4,2,0],
      walls:[[[0,0],[6,0]],[[6,0],[6,4]],[[6,4],[0,4]],[[0,4],[0,0]],[[3,1],[3,3]]]}});
}

test('simulation scene supports seeded random maps and click target placement',async({page})=>{
  const f=await fixture(page);enableScenario(f);await view(page,'workflow');
  await expect(page.locator('#simulationPanel')).toBeVisible();
  await expect(page.locator('#simulationGeometryHelp')).toContainText('0.22 m high, 10.0° downward');
  await expect(page.locator('#simulationGeometryHelp')).toContainText('move the simulated camera itself');
  await page.locator('#simulationSeed').fill('123');
  await page.getByRole('button',{name:'Generate random map',exact:true}).click();
  await expect.poll(()=>f.calls.find(c=>c.path==='/api/simulation_scenario')?.body).toEqual({action:'random_map',seed:123});
  await page.getByRole('button',{name:'Random target',exact:true}).click();
  await expect.poll(()=>f.calls.filter(c=>c.path==='/api/simulation_scenario').at(-1)?.body).toEqual({action:'random_target',seed:123});
  const scene=page.locator('#simulationScene'),box=await scene.boundingBox();
  await scene.click({position:{x:box.width/2,y:box.height/2}});
  const x=Number(await page.locator('#simulationTargetX').inputValue()),y=Number(await page.locator('#simulationTargetY').inputValue());
  expect(x).toBeCloseTo(3,1);expect(y).toBeCloseTo(2,1);
  await page.getByRole('button',{name:'Place target',exact:true}).click();
  await expect.poll(()=>f.calls.filter(c=>c.path==='/api/simulation_scenario').at(-1)?.body).toEqual({action:'set_target',x,y});
  expect(f.errors).toEqual([]);
});

test('simulation editor stays hidden on hardware and locked during a run',async({page})=>{
  const f=await fixture(page);enableScenario(f);f.state.simulated=false;
  await expect(page.locator('#simulationPanel')).toBeHidden();
  f.state.simulated=true;f.state.mission.state='MAPPING';f.state.simulation_can_apply=false;
  await expect(page.locator('#simulationPanel')).toBeVisible();
  await expect(page.locator('#simulationRandomMap')).toBeDisabled();
  await expect(page.locator('#simulationPlaceTarget')).toBeDisabled();
  await page.locator('#simulationScene').click();
  expect(f.calls.filter(c=>c.path==='/api/simulation_scenario')).toEqual([]);
});

test('simulation target coordinate entry validates input and shows server rejection',async({page})=>{
  const f=await fixture(page);enableScenario(f);
  await expect(page.locator('#simulationPlaceTarget')).toBeEnabled();
  await page.locator('#simulationTargetX').fill('');
  await page.getByRole('button',{name:'Place target',exact:true}).click();
  await expect(page.locator('#feedback')).toContainText('Enter finite target coordinates');
  expect(f.calls.filter(c=>c.path==='/api/simulation_scenario')).toEqual([]);
  await page.route('**/api/simulation_scenario',route=>route.fulfill({status:400,json:{error:'Target is too close to a wall'}}));
  await page.locator('#simulationTargetX').fill('3');
  await page.getByRole('button',{name:'Place target',exact:true}).click();
  await expect(page.locator('#feedback')).toContainText('too close to a wall');
  await expect(page.locator('#simulationPlaceTarget')).toBeEnabled();
});

test('simulation editor fits a mobile screen and retains Stop',async({page})=>{
  await page.setViewportSize({width:390,height:844});
  const f=await fixture(page);enableScenario(f);await view(page,'workflow');
  await expect(page.locator('#simulationPanel')).toBeVisible();
  await expect.poll(()=>page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await page.locator('#simulationPlaceTarget').scrollIntoViewIfNeeded();
  await expect(page.locator('#stop')).toBeInViewport();
  expect(f.errors).toEqual([]);
});

test('simulated camera form changes physical mount and lens through its isolated API',async({page})=>{
  const f=await fixture(page);enableScenario(f);await view(page,'workflow');
  await page.locator('#advancedTools > summary').click();
  await expect(page.locator('#visionHeight')).toHaveValue('0.220');
  await expect(page.locator('#visionPitch')).toHaveValue('10.000');
  await expect(page.locator('#visionPitchOverride')).toBeHidden();
  await page.locator('#visionHeight').fill('0.31');
  await page.locator('#visionPitch').fill('28.6');
  await page.locator('#visionFov').fill('60');
  await page.getByRole('button',{name:'Apply simulated camera',exact:true}).click();
  await expect.poll(()=>f.calls.some(c=>c.path==='/api/simulation_camera')).toBe(true);
  const body=f.calls.find(c=>c.path==='/api/simulation_camera').body;
  expect(body.camera_height).toBe(.31);expect(body.camera_pitch).toBeCloseTo(28.6*Math.PI/180,8);
  expect(body.horizontal_fov).toBeCloseTo(Math.PI/3,8);expect(body).not.toHaveProperty('use_pitch_override');
  expect(f.calls.filter(c=>c.path==='/api/vision')).toEqual([]);
  await expect(page.locator('#simulationGeometryHelp')).toContainText('0.31 m high, 28.6° downward');
  f.state.mission.state='SEARCHING';f.state.simulation_can_apply=false;
  await expect(page.locator('#saveVision')).toBeDisabled();
  expect(f.errors).toEqual([]);
});
