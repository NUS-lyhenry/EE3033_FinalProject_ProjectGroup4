const {test,expect}=require('@playwright/test');

async function fixture(page,{phase='mapping',active=false}={}){
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  const state={simulated:true,read_only:false,phase,allow_motion:true,token:'test-only',control_generation:0,
    mission:{phase,state:active?(phase==='mapping'?'MAPPING':'SEARCHING'):'WAITING',reason:'SIMULATED UI TEST · no robot motion',home:null,target:null,ready:true,return_verified:false},
    stage:{managed:true,running:true,phase,transitioning:false},assessment:{elapsed_seconds:65,started:10,finished:null,running:true,timing_valid:true,includes_mapping:true,maps:[]},
    mission_options:{target_class:'cup',localization_method:'lidar',return_tolerance:.2},
    map_revision:1,pose:[1,1,0],route:[],path:[],health:Object.fromEntries(['Map','Lidar','Odometry','AMCL','YOLO','Mission'].map(k=>[k,{ok:true,age:.1}])),
    events:[],camera_source:null,battery:{voltage:11.87,fresh:true,age:.1},control_fault:null,initial_pose_sent:false};
  let maps=[],gap=false,trial={max_linear:.12,max_angular:.4,goal_timeout:60};const calls=[];
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
    else if(path==='/api/save_map'){
      const entry={id:'onsite_simulated/map.yaml',path:'/SIMULATED/maps/onsite_simulated/map.yaml',quality:'snapshot',reviewed:false,resolution:.1};
      maps=[entry];state.assessment.maps.push(entry.id);state.saved_map=entry.path;
      json={success:true,path:entry.path,map:entry,message:'SIMULATED map saved'};
    }else if(path==='/api/review_map'){maps.find(m=>m.id===body.map_id).reviewed=true;json={success:true,map:maps[0]};}
    else if(path==='/api/stage'){
      if(stageDelay){await stageDelay();json={success:true,message:'SIMULATED transition stopped'};}
      else{
        state.phase=body.phase;state.stage.phase=body.phase;state.stage.running=true;state.control_generation++;
        state.mission={phase:body.phase,state:'WAITING',reason:'SIMULATED stage ready',home:null,target:null,ready:body.phase==='mapping',return_verified:false};
        state.initial_pose_sent=false;
        if(body.phase==='mission'){state.selected_map=maps[0];state.mission_options={target_class:body.target_class,localization_method:body.localization_method,return_tolerance:body.return_tolerance};state.health.AMCL.ok=false;}
      }
    }else if(path==='/api/initial_pose'){state.initial_pose_sent=true;state.health.AMCL.ok=true;state.mission.ready=true;}
    else if(path==='/api/start'){state.mission.state=state.phase==='mapping'?'MAPPING':'SEARCHING';state.mission.home=[1,1,0];}
    else if(path==='/api/stop'){state.mission.state='STOPPED';state.control_generation++;}
    else if(path==='/api/result')json={simulated:true,assessment:state.assessment,mission:state.mission,selected_map:state.selected_map};
    else if(path==='/api/gap'){
      if(method==='POST')gap=body.enabled;
      const p={robot_radius:gap?.18:.14,footprint_padding:.01,resolution:.05};
      json={success:true,enabled:gap,synchronized:true,nodes:{'global_costmap/global_costmap':p,'local_costmap/local_costmap':p,[state.phase==='mapping'?'project_mapper':'project_mission']:{clearance:gap?.26:.22}}};
    }else if(path==='/api/trial'){
      if(method==='POST')trial=body;
      json={success:true,values:trial,synchronized:true,protection:{stop_distance:.22,rotation_clearance:.16}};
    }else if(path==='/api/teleop')json={success:true};
    else return route.fulfill({status:404,json:{error:'Unknown test endpoint'}});
    return route.fulfill({json:JSON.parse(JSON.stringify(json))});
  });
  await page.goto('/');await expect(page.locator('#connection')).toHaveText('Connected');
  return{state,calls,errors,setStageDelay:fn=>stageDelay=fn};
}

test('preview is explicitly simulated and cannot command robot motion',async({page,request})=>{
  await page.goto('/');await expect(page.locator('#previewNotice')).toBeVisible();
  await expect(page.locator('#start')).toBeDisabled();await expect(page.locator('#loadMapping')).toBeDisabled();
  const response=await request.post('/api/start',{data:{}});expect(response.status()).toBe(403);
});

test('complete basic workflow: map save review, stage transition, assigned pose, search, target and verified return',async({page})=>{
  const f=await fixture(page);
  await expect(page.locator('#totalElapsed')).toHaveText('01:05');
  await page.locator('#saveMap').click();await expect(page.locator('#savedMaps')).toContainText('onsite_simulated');
  await expect(page.locator('#loadMission')).toBeDisabled();
  await page.locator('#reviewMap').click();await expect(page.locator('#mapQuality')).toContainText('Reviewed');
  await page.locator('#localizationMethod').selectOption('ground_plane');
  await page.locator('#returnTolerance').fill('0.15');
  await page.locator('#loadMission').click();
  await expect(page.locator('#phase')).toHaveText('Stage 2 · Autonomous');
  const load=f.calls.find(c=>c.path==='/api/stage');expect(load.body).toEqual({phase:'mission',map_id:'onsite_simulated/map.yaml',target_class:'cup',localization_method:'ground_plane',return_tolerance:.15});
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
  const download=page.waitForEvent('download');await page.locator('#exportResult').click();expect((await download).suggestedFilename()).toBe('assessment.json');
  expect(f.errors).toEqual([]);await page.evaluate(()=>window.scrollTo({top:0,behavior:'instant'}));await page.screenshot({path:'test-results/simulated-basic-workflow.png',fullPage:true});
});

for(const phase of ['mapping','mission'])test(`active ${phase} locks setup and manual intervention`,async({page})=>{
  await fixture(page,{phase,active:true});
  await expect(page.locator('#loadMapping')).toBeDisabled();await expect(page.locator('#loadMission')).toBeDisabled();
  await expect(page.locator('#saveMap')).toBeDisabled();await expect(page.locator('#setPose')).toBeDisabled();
  await expect(page.locator('#targetClass')).toBeDisabled();await expect(page.locator('#stop')).toBeEnabled();
  await page.locator('#advanced > summary').click();await expect(page.locator('#gapToggle')).toBeDisabled();
  if(phase==='mapping'){await page.locator('#mappingControls > summary').click();await expect(page.getByLabel('Drive forward')).toBeDisabled();}
});

test('unverified home success and absent targets never look like a complete mission',async({page})=>{
  const f=await fixture(page,{phase:'mission'});
  f.state.mission={...f.state.mission,state:'COMPLETE',target:{x:1,y:2,class:'cup'},return_verified:false};
  await expect(page.locator('#state')).toHaveText('COMPLETE');await expect(page.locator('#resultOutcome')).toHaveText('Trial not complete');
  f.state.mission={...f.state.mission,state:'NOT_FOUND',target:null,return_verified:true,return_distance:.05};
  await expect(page.locator('#resultOutcome')).toHaveText('Target not found');await expect(page.locator('#resultTarget')).toHaveText('Awaiting target');
});

test('stale telemetry and disconnection disable driving and mark unavailable data',async({page})=>{
  const f=await fixture(page);f.state.health.Mission.ok=false;f.state.battery.fresh=false;f.state.battery.age=6;
  await expect(page.locator('#start')).toBeDisabled();await expect(page.locator('#battery')).toHaveText('Battery 11.87 V · stale');
  await page.route('**/api/state',r=>r.abort());await expect(page.locator('#connectionBanner')).toBeVisible();
  await expect(page.locator('#setPose')).toBeDisabled();await expect(page.locator('#loadMapping')).toBeDisabled();
});

test('optional gap reserve and trial settings retain verified readback',async({page})=>{
  const f=await fixture(page);await page.locator('#advanced > summary').click();
  await expect(page.locator('#gapToggle')).toBeEnabled();await page.locator('#gapToggle').click();
  await expect(page.locator('#gapToggle')).toHaveAttribute('aria-pressed','true');
  await expect(page.locator('#gapStatus')).toContainText('clearance 0.26 m');
  await expect(page.locator('#trialLinear')).toBeEnabled();await page.locator('#trialLinear').focus();await page.keyboard.press('End');
  await expect(page.locator('#trialLinearValue')).toHaveText('0.18 m/s');await page.locator('#applyTrial').click();
  await expect(page.locator('#trialLinearLive')).toHaveText('Live: 0.18 m/s');
  expect(f.calls.find(c=>c.path==='/api/trial').body.max_linear).toBe(.18);
});

test('Stop remains available while a stage request is pending',async({page})=>{
  const f=await fixture(page);let release;f.setStageDelay(()=>new Promise(r=>release=r));
  await page.locator('#loadMapping').click();await expect(page.locator('#loadMapping')).toBeDisabled();
  await page.locator('#stop').click();await expect(page.locator('#state')).toHaveText('STOPPED');
  expect(f.calls.map(c=>c.path)).toContain('/api/stop');release();
  await expect(page.locator('#loadMapping')).toBeEnabled();
});

test('mobile layout keeps required controls on screen without horizontal overflow',async({page})=>{
  await page.setViewportSize({width:390,height:844});const f=await fixture(page);
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await expect(page.locator('#stop')).toBeInViewport();await expect(page.getByLabel('Detector target class')).toBeVisible();
  expect(f.errors).toEqual([]);await page.evaluate(()=>window.scrollTo({top:0,behavior:'instant'}));await page.screenshot({path:'test-results/simulated-mobile-console.png',fullPage:true});
});
