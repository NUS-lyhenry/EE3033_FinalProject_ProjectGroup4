const {test,expect}=require('@playwright/test');
async function fixture(page){
  const errors=[],calls=[];page.on('pageerror',e=>errors.push(e.message));
  const state={token:'test',generation:0,allow_motion:true,read_only:false,motion:'idle',reason:'Ready',ownership:true,
    health:{camera:true,odometry:true,lidar:true,output:true,ownership:true},capture:{ready:true},
    automatic:{state:'idle',reason:'Ready',prerequisites:{ready:true,reason:'Map point ready'}},
    target:{xy:[.9,0]},camera:{width:640,height:480},preview:{fresh:true},detected_count:1,detection_fresh:true,
    bottle_detections:[{box:[295,100,345,350]}]};
  await page.route('**/api/calibration/**',async route=>{
    const path=new URL(route.request().url()).pathname.split('/').pop();
    if(path==='state')return route.fulfill({json:state});
    if(path==='image.jpg')return route.fulfill({contentType:'image/svg+xml',body:'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480"><rect width="640" height="480" fill="#234"/></svg>'});
    const body=route.request().postDataJSON();expect(route.request().headers()['x-project-token']).toBe('test');calls.push({path,body});
    if(path==='start'){state.motion='auto';state.automatic.state='collecting';}
    if(path==='stop'){state.motion='idle';state.automatic.state='stopped';state.generation++;}
    return route.fulfill({json:{success:true}});
  });
  await page.goto('/calibration');await expect(page.locator('#connection')).toHaveText('Connected');
  return {state,calls,errors};
}
test('preview is read-only and describes the map-point workflow',async({page,request})=>{
  await page.goto('/calibration');await expect(page.locator('#simulationNotice')).toBeVisible();
  await expect(page.getByRole('heading',{name:'Calibrate from a fixed map point.'})).toBeVisible();
  await expect(page.locator('#startAutomatic')).toBeDisabled();
  expect((await request.post('/api/calibration/start',{data:{}})).status()).toBe(403);
});
test('Start sends only bottle dimensions; Stop remains available',async({page})=>{
  const f=await fixture(page);await expect(page.locator('#targetPoint')).toContainText('0.900');
  await page.locator('#autoHeight').fill('0.215');await page.locator('#autoDiameter').fill('0.065');
  await page.locator('#startAutomatic').click();
  expect(f.calls.find(c=>c.path==='start').body).toEqual({generation:0,bottle_height:.215,bottle_diameter:.065});
  await expect(page.locator('#startAutomatic')).toBeDisabled();await expect(page.locator('#autoStop')).toBeEnabled();
  await page.locator('#autoStop').click();await expect(page.locator('#startAutomatic')).toBeEnabled();
  expect(f.errors).toEqual([]);await expect(page.locator('#autoDistance')).toHaveCount(0);await expect(page.locator('#manualControls')).toHaveCount(0);
});
test('missing map pose or unsettled command output blocks Start',async({page})=>{
  const f=await fixture(page);f.state.automatic.prerequisites={ready:false,reason:'Establish localization'};
  await expect(page.locator('#startAutomatic')).toBeDisabled();await expect(page.locator('#autoPrerequisites')).toHaveText('Establish localization');
  f.state.automatic.prerequisites.ready=true;f.state.capture.ready=false;
  await expect(page.locator('#startAutomatic')).toBeDisabled();
});
test('detector ambiguity delays capture without claiming a second physical bottle',async({page})=>{
  const f=await fixture(page);f.state.detected_count=2;f.state.detection_fresh=false;
  await expect(page.locator('#detection')).toHaveText('Ambiguous proposals · capture waits');
  await expect(page.locator('#startAutomatic')).toBeEnabled();expect(f.errors).toEqual([]);
});
test('successful fit shows actual pitch, height and checks',async({page})=>{
  const f=await fixture(page);f.state.automatic.state='complete';f.state.saved='/tmp/camera.json';
  f.state.profile={pitch:14.2*Math.PI/180,setup:{camera_height:.2655},validation:[{error_m:.02},{error_m:.03}]};
  await expect(page.locator('#autoResult')).toContainText('14.2° downward');await expect(page.locator('#autoResult')).toContainText('2.0 cm / 3.0 cm');
  expect(f.errors).toEqual([]);
});
test('mobile layout keeps Stop and camera available without overflow',async({page})=>{
  await page.setViewportSize({width:390,height:844});const f=await fixture(page);
  await expect(page.locator('#stop')).toBeVisible();await expect(page.locator('#camera')).toBeVisible();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);expect(f.errors).toEqual([]);
});
