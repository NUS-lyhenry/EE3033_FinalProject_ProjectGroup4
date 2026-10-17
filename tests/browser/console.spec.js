const {test,expect}=require('@playwright/test');

test('live console connects and displays camera with motion disabled',async({page})=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');
  await expect(page.locator('#connection')).toHaveText('Connected');
  await expect(page.locator('#start')).toBeDisabled();
  await expect(page.locator('#motion')).toHaveText('Motion disabled');
  await expect(page.locator('#camera')).toHaveJSProperty('naturalWidth',640,{timeout:15000});
  expect(errors).toEqual([]);
  await page.screenshot({path:'test-results/live-console.png',fullPage:true});
});

test('active mapping locks manual controls and map saving',async({page})=>{
  await page.route('**/api/state',r=>r.fulfill({json:{
    phase:'mapping',allow_motion:true,token:'test-only',pose:[1,1,0],
    mission:{state:'MAPPING',reason:'SIMULATED UI TEST — no robot motion',
             elapsed_seconds:65,home:[1,1,0],target:null,viewpoints_total:0},
    map_revision:0,route:[],path:[],health:{},events:[],camera_source:null,
  }}));
  await page.route('**/api/map',r=>r.fulfill({json:null}));
  await page.goto('/');
  await expect(page.locator('#state')).toHaveText('MAPPING');
  await expect(page.locator('#start')).toBeDisabled();
  await expect(page.locator('#saveMap')).toBeDisabled();
  await expect(page.locator('[aria-label="Drive forward"]')).toBeDisabled();
  await expect(page.locator('#setup')).toBeHidden();
  await expect(page.locator('#stop')).toBeEnabled();
});

test('mission results, confidence and map render without browser errors',async({page})=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.route('**/api/state',r=>r.fulfill({json:{
    phase:'mission',allow_motion:true,token:'test-only',pose:[1,1,0],
    mission:{state:'COMPLETE',reason:'SIMULATED UI TEST — home verified',elapsed_seconds:95,
             home:[1,1,0],target:{x:2.1,y:1.8,class:'cup',confidence:.88,variance:.01},
             observed_fraction:.67,viewpoints_completed:4,viewpoints_total:6,failed_viewpoints:[]},
    map_revision:1,route:[[1,1],[2,1],[2,2]],path:[],
    health:{Map:{ok:true,age:1},Lidar:{ok:true,age:.1},Mission:{ok:true,age:.1}},
    events:[{time:'12:00:00',state:'COMPLETE',reason:'SIMULATED UI TEST'}],camera_source:null,
  }}));
  const cells=Array.from({length:1600},(_,i)=>i<40||i>=1560||i%40===0||i%40===39?100:0);
  await page.route('**/api/map',r=>r.fulfill({json:{width:40,height:40,resolution:.1,
                                                               origin:[0,0,0],cells}}));
  await page.goto('/');
  await expect(page.locator('#target')).toHaveText('2.10, 1.80 m');
  await expect(page.locator('#confidence')).toContainText('88%');
  await expect(page.locator('#distance')).toHaveText('0.00 m');
  await expect(page.locator('#coverage')).toHaveText('67%');
  await expect(page.locator('#mapEmpty')).toBeHidden();
  expect(errors).toEqual([]);
  await page.screenshot({path:'test-results/simulated-results.png',fullPage:true});
});
