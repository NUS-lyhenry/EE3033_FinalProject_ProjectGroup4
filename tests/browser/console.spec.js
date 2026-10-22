const {test,expect}=require('@playwright/test');
async function view(page,name){await page.evaluate(p=>location.hash='/'+p,name);await expect(page.locator('.page-tabs [data-page-link="'+name+'"]')).toHaveAttribute('aria-current','page');}

async function fixture(page,{phase='mapping',active=false}={}){
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  const state={simulated:true,read_only:false,phase,allow_motion:true,token:'test-only',control_generation:0,
    mission:{phase,state:active?(phase==='mapping'?'MAPPING':'SEARCHING'):'WAITING',reason:'SIMULATED UI TEST · no robot motion',home:null,target:null,ready:true,return_verified:false},
    stage:{managed:true,running:true,phase,transitioning:false},assessment:{elapsed_seconds:65,started:10,finished:null,running:true,timing_valid:true,includes_mapping:true,maps:[]},
    mission_options:{target_class:'cup',localization_method:'lidar',return_tolerance:.2},
    map_revision:1,pose:[1,1,0],route:[],path:[],health:Object.fromEntries(['Map','Lidar','Odometry','AMCL','YOLO','Mission'].map(k=>[k,{ok:true,age:.1}])),
    events:[],camera_source:null,battery:{voltage:11.87,fresh:true,age:.1},control_fault:null,initial_pose_sent:false};
  let maps=[],gap=false,audacity='cautious',trial={max_linear:.12,max_angular:.4,goal_timeout:60};const calls=[];
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
      maps=[entry];state.assessment.maps.push(entry.id);state.saved_map=entry.path;
      json={success:true,path:entry.path,map:entry,message:'SIMULATED map saved'};
    }else if(path==='/api/review_map'){maps.find(m=>m.id===body.map_id).reviewed=true;json={success:true,map:maps[0]};}
    else if(path==='/api/stage'){
      if(stageDelay){await stageDelay();json={success:true,message:'SIMULATED transition stopped'};}
      else{
        state.phase=body.phase;state.stage.phase=body.phase;state.stage.running=true;state.control_generation++;
        gap=false;audacity='cautious';
        state.mission={phase:body.phase,state:'WAITING',reason:'SIMULATED stage ready',home:null,target:null,ready:body.phase==='mapping',return_verified:false};
        state.initial_pose_sent=false;
        if(body.phase==='mission'){state.selected_map=maps[0];state.mission_options={target_class:body.target_class,localization_method:body.localization_method,return_tolerance:body.return_tolerance};state.health.AMCL.ok=false;}
      }
    }else if(path==='/api/initial_pose'){state.initial_pose_sent=true;state.health.AMCL.ok=true;state.mission.ready=true;}
    else if(path==='/api/start'){state.mission.state=state.phase==='mapping'?'MAPPING':'SEARCHING';state.mission.home=[1,1,0];}
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
      json={success:true,values:trial,synchronized:true,protection:{stop_distance:.22,rotation_clearance:.16}};
    }else if(path==='/api/teleop')json={success:true};
    else return route.fulfill({status:404,json:{error:'Unknown test endpoint'}});
    return route.fulfill({json:JSON.parse(JSON.stringify(json))});
  });
  await page.goto('/#/'+(phase==='mapping'?'mapping':'mission'));await expect(page.locator('#connection')).toHaveText('Connected');
  return{state,map,calls,errors,setStageDelay:fn=>stageDelay=fn};
}

test('preview is explicitly simulated and cannot command robot motion',async({page,request})=>{
  await page.goto('/');await expect(page.locator('#previewNotice')).toBeVisible();
  await expect(page.locator('#start')).toBeDisabled();await expect(page.locator('#loadMapping')).toBeDisabled();
  const response=await request.post('/api/start',{data:{}});expect(response.status()).toBe(403);
});

test('mapping shows deferred areas and the final retry pass',async({page})=>{
  const f=await fixture(page,{active:true});
  Object.assign(f.state.mission,{viewpoints_completed:2,deferred_frontiers:1,frontier_retry_round:0});
  await expect(page.locator('#mappingProgress')).toContainText('2 goals reached · 1 deferred');
  await expect(page.locator('#mappingProgress')).toContainText('Explore other areas');
  f.state.mission.frontier_retry_round=1;
  await expect(page.locator('#mappingProgress')).toContainText('Final retry pass');
  expect(f.errors).toEqual([]);
});

test('complete basic workflow: map save review, stage transition, assigned pose, search, target and verified return',async({page})=>{
  const f=await fixture(page);
  await expect(page.locator('#totalElapsed')).toHaveText('01:05');
  await page.locator('#saveMap').click();await expect(page.locator('#savedMaps')).toContainText('onsite_simulated');
  await expect(page.locator('#loadMission')).toBeDisabled();
  await page.locator('#reviewMap').click();await expect(page.locator('#mapQuality')).toContainText('Reviewed');
  await view(page,'mission');
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
  await view(page,'results');
  const download=page.waitForEvent('download');await page.locator('#exportResult').click();expect((await download).suggestedFilename()).toBe('assessment.json');
  expect(f.errors).toEqual([]);await page.evaluate(()=>window.scrollTo({top:0,behavior:'instant'}));await page.screenshot({path:'test-results/simulated-basic-workflow.png',fullPage:true});
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
  await expect(page.locator('#resultOutcome')).toHaveText('Target not found');await expect(page.locator('#resultTarget')).toHaveText('Awaiting target');
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
  await expect(page.locator('#gapStatus')).toContainText('clearance 0.26 m');
  await expect(page.locator('#trialLinear')).toBeEnabled();await page.locator('#trialLinear').focus();await page.keyboard.press('End');
  await expect(page.locator('#trialLinearValue')).toHaveText('0.18 m/s');await page.locator('#applyTrial').click();
  await expect(page.locator('#trialLinearLive')).toHaveText('Live: 0.18 m/s');
  expect(f.calls.find(c=>c.path==='/api/trial').body.max_linear).toBe(.18);
});

for(const phase of ['mapping','mission'])test(`three audacity levels use verified readback and retain gap reserve in ${phase}`,async({page})=>{
  const f=await fixture(page,{phase});await view(page,'navigation');
  const cautious=page.getByRole('button',{name:/1 · Cautious/}),balanced=page.getByRole('button',{name:/2 · Balanced/}),bold=page.getByRole('button',{name:/3 · Bold/});
  await expect(cautious).toHaveAttribute('aria-pressed','true');
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
  await expect(cautious).toHaveAttribute('aria-pressed','true');
  await page.route('**/api/gap',route=>route.request().method()==='POST'?route.fulfill({status:503,json:{error:'Parameter update rejected; original values restored'}}):route.fallback());
  await bold.click();await expect(page.locator('#feedback')).toContainText('original values restored');
  await expect(bold).toHaveAttribute('aria-pressed','false');
  await expect(page.locator('#audacityLive')).toHaveText('Unverified');
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
  const box=await page.locator('#map').boundingBox();
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
