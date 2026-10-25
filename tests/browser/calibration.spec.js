const {test,expect}=require('@playwright/test');

async function fixture(page){
  const errors=[],calls=[];page.on('pageerror',e=>errors.push(e.message));
  const camera={width:640,height:480,frame_id:'camera',source:'SIMULATED CameraInfo',k:[520,0,320,0,520,240,0,0,1],d:[0,0,0,0,0]};
  const state={token:'calibration-test',generation:0,allow_motion:true,read_only:false,simulated:true,
    motion:'idle',motion_reason:'Stopped',reason:'SIMULATED calibration fixture',
    health:{camera:true,odometry:true,lidar:true,output:true,ownership:true},ownership:true,
    camera,setup:null,samples:[],profile:null,saved:null,detected_count:1,
    box:[295,140,345,350],detection_fresh:true,frozen:null};
  let reject=null;
  await page.route('**/api/calibration/**',async route=>{
    const path=new URL(route.request().url()).pathname.split('/').pop();
    if(path==='image.jpg')return route.fulfill({contentType:'image/svg+xml',body:'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480"><rect width="640" height="480" fill="#233c42"/><rect x="295" y="140" width="50" height="210" rx="15" fill="#7ca591"/></svg>'});
    if(path==='state')return route.fulfill({json:state});
    const body=route.request().postDataJSON();expect(route.request().headers()['x-project-token']).toBe(state.token);calls.push({path,body});
    if(reject===path)return route.fulfill({status:400,json:{error:'SIMULATED measurement rejected'}});
    if(path==='setup'){state.setup=body;state.samples=[];state.profile=null;}
    if(path==='freeze')state.frozen={id:'frozen-'+calls.length,box:state.box};
    if(path==='unfreeze')state.frozen=null;
    if(path==='sample'){
      const sample={distance:body.distance,lateral:body.lateral,box:body.box};
      if(body.validation)state.profile.validation.push({...sample,error_m:.025,passed:true});else{state.samples.push(sample);state.profile=null;}
      state.frozen=null;
    }
    if(path==='fit')state.profile={camera,quality:{reprojection_rms_px:1.5},pitch:.12,yaw:.02,range_m:[.6,1.5],validation:[]};
    if(path==='save')state.saved='/tmp/calibration/camera_checked.json';
    if(path==='scene')state.simulation_reference={distance:.8,lateral:.1,scene:2,total:8};
    if(path==='move'){state.motion=body.mode;state.frozen=null;}
    if(path==='stop'){state.motion='idle';state.generation++;}
    if(path==='remove'){
      if(body.kind==='training'){state.samples.splice(body.index,1);state.profile=null;}
      else state.profile.validation.splice(body.index,1);
    }
    return route.fulfill({json:{success:true}});
  });
  await page.goto('/calibration');await expect(page.locator('#connection')).toHaveText('Connected');
  return {state,calls,errors,reject:path=>reject=path};
}

test('calibration preview is read-only and shows measurement instructions',async({page,request})=>{
  await page.goto('/calibration');await expect(page.locator('#preview')).toBeVisible();
  await expect(page.getByRole('heading',{name:'Give the camera a sense of distance.'})).toBeVisible();
  await expect(page.locator('#setup')).toBeDisabled();await expect(page.locator('#forward')).toBeDisabled();
  await expect(page.locator('#save')).toBeDisabled();
  expect((await request.post('/api/calibration/move',{data:{mode:'forward'}})).status()).toBe(403);
});

test('guided setup, freeze, train, fit, independently validate and save',async({page})=>{
  const f=await fixture(page);
  await page.locator('#cameraHeight').fill('0.21');await page.locator('#setup').click();
  expect(f.calls.find(c=>c.path==='setup').body.camera_height).toBe(.21);
  for(let i=0;i<6;i++){
    await page.locator('#freeze').click();await expect(page.locator('#cameraTitle')).toContainText('Frozen');
    await page.locator('#distance').fill(String(.6+.15*i));
    await page.locator('#lateral').fill(i%2?'.15':'-.15');
    await expect(page.locator('#capture')).toBeEnabled();await page.locator('#capture').click();
    await expect(page.locator('#trainingCount')).toHaveText(`${i+1} / 6`);
  }
  await page.locator('#fit').click();await expect(page.locator('#fitError')).toHaveText('1.5');
  await expect(page.locator('#save')).toBeDisabled();await page.locator('#validation').check();
  for(let i=0;i<2;i++){
    await page.locator('#freeze').click();await page.locator('#distance').fill(String(.85+i*.25));
    await expect(page.locator('#capture')).toBeEnabled();await page.locator('#capture').click();
    await expect(page.locator('#validationCount')).toHaveText(`${i+1} / 2`);
  }
  await page.locator('#save').click();await expect(page.locator('#savedPanel')).toBeVisible();
  await expect(page.locator('#launchCommand')).toContainText("calibration_file:='/tmp/calibration/camera_checked.json'");
  expect(f.calls.filter(c=>c.path==='sample'&&c.body.validation)).toHaveLength(2);
  expect(f.errors).toEqual([]);
});

test('frozen bottle box supports pointer and keyboard corrections',async({page})=>{
  const f=await fixture(page);await page.locator('#setup').click();await page.locator('#freeze').click();
  await expect(page.locator('#capture')).toBeEnabled();await page.locator('#camera').scrollIntoViewIfNeeded();const b=await page.locator('#camera').boundingBox();
  await page.mouse.move(b.x+b.width*.4,b.y+b.height*.25);await page.mouse.down();
  await page.mouse.move(b.x+b.width*.6,b.y+b.height*.8);await page.mouse.up();
  await page.locator('#boxBottom').fill('380');await page.locator('#capture').click();
  const sample=f.calls.find(c=>c.path==='sample');expect(sample.body.box).toEqual([256,120,384,380]);
  expect(sample.body.freeze_id).toMatch(/^frozen-/);expect(f.errors).toEqual([]);
});

test('Stop stays available during a movement and sends the current generation',async({page})=>{
  const f=await fixture(page);await page.locator('#forward').click();
  await expect(page.locator('#forward')).toBeDisabled();await expect(page.locator('#setup')).toBeDisabled();
  await expect(page.locator('#stop')).toBeEnabled();await page.locator('#stop').click();
  await expect(page.locator('#forward')).toBeEnabled();expect(f.calls.find(c=>c.path==='stop').body.generation).toBe(0);
});

test('stale detection and sensor ownership failure lock movements',async({page})=>{
  const f=await fixture(page);f.state.detection_fresh=false;
  await expect(page.locator('#center')).toBeDisabled();await expect(page.locator('#freeze')).toBeDisabled();
  f.state.detection_fresh=true;f.state.ownership=false;
  await expect(page.locator('#backward')).toBeDisabled();f.state.ownership=true;f.state.health.odometry=false;
  await expect(page.locator('#forward')).toBeDisabled();
});

test('failed validation locks save and can be removed for a new measurement',async({page})=>{
  const f=await fixture(page);f.state.profile={camera:f.state.camera,quality:{reprojection_rms_px:1},pitch:0,yaw:0,range_m:[.6,1.5],validation:[{distance:.9,lateral:0,error_m:.2,passed:false},{distance:1.1,lateral:0,error_m:.02,passed:true},{distance:1.3,lateral:0,error_m:.03,passed:true}]};
  await expect(page.locator('#validationCount')).toHaveText('2 / 2');await expect(page.locator('#save')).toBeDisabled();
  await page.getByRole('button',{name:'Remove validation sample 1',exact:true}).click();
  await expect(page.locator('#save')).toBeEnabled();
});

test('measurement rejection remains visible instead of disappearing on polling',async({page})=>{
  const f=await fixture(page);await page.locator('#setup').click();await page.locator('#freeze').click();
  await expect(page.locator('#capture')).toBeEnabled();f.reject('sample');await page.locator('#capture').click();
  await expect(page.locator('#feedback')).toContainText('measurement rejected');
  await page.waitForTimeout(900);await expect(page.locator('#feedback')).toContainText('measurement rejected');
});

test('mobile layout keeps camera and Stop visible without horizontal overflow',async({page})=>{
  await page.setViewportSize({width:390,height:844});const f=await fixture(page);
  await expect(page.locator('#stop')).toBeVisible();await expect(page.locator('#camera')).toBeVisible();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await page.screenshot({path:'test-results/calibration-mobile.png',fullPage:true});expect(f.errors).toEqual([]);
});

test('clickable simulation exposes reference controls and labels saved profiles as simulation only',async({page})=>{
  const f=await fixture(page);f.state.simulation_reference={distance:.65,lateral:-.09,scene:1,total:8};
  await expect(page.locator('#simulationControls')).toBeVisible();
  await page.locator('#fillReference').click();await expect(page.locator('#distance')).toHaveValue('0.650');
  await expect(page.locator('#lateral')).toHaveValue('-0.090');
  await page.locator('#nextScene').click();await expect(page.locator('#simulationReference')).toContainText('Position 2 / 8');
  f.state.profile={camera:f.state.camera,quality:{reprojection_rms_px:1},pitch:0,yaw:0,range_m:[.6,1.5],validation:[{distance:.9,lateral:0,error_m:.02,passed:true},{distance:1.2,lateral:0,error_m:.02,passed:true}]};
  await expect(page.locator('#save')).toBeEnabled();await page.locator('#save').click();
  await expect(page.locator('#launchCommand')).toContainText('SIMULATION COMPLETE');
  await expect(page.locator('#copyCommand')).toBeHidden();
  await page.screenshot({path:'test-results/calibration-desktop.png',fullPage:true});expect(f.errors).toEqual([]);
});

test('frozen measurement can resume live camera without resetting samples',async({page})=>{
  const f=await fixture(page);await page.locator('#setup').click();await page.locator('#freeze').click();
  await expect(page.locator('#capture')).toBeEnabled();await expect(page.locator('#resume')).toBeVisible();
  await page.locator('#resume').click();await expect(page.locator('#cameraTitle')).toContainText('Live');
  await expect(page.locator('#resume')).toBeHidden();expect(f.calls.some(c=>c.path==='unfreeze')).toBe(true);
});

test('automatic setup takes only bottle dimensions and one distance, then shows saved checks',async({page})=>{
  const f=await fixture(page);
  f.state.automatic={state:'idle',reason:'Ready',collected:0,total:18,prerequisites:{ready:true,reason:'Map localization ready'}};
  await expect(page.locator('#automaticPanel')).toBeVisible();
  await page.locator('#autoHeight').fill('0.23');await page.locator('#autoDiameter').fill('0.06');await page.locator('#autoDistance').fill('0.95');
  await page.locator('#startAutomatic').click();
  const request=f.calls.find(c=>c.path==='automatic');
  expect(request.body).toEqual({generation:0,bottle_height:.23,bottle_diameter:.06,distance:.95});
  f.state.automatic.state='collecting';f.state.motion='auto';
  await expect(page.locator('#startAutomatic')).toBeDisabled();await expect(page.locator('#autoStop')).toBeEnabled();
  f.state.automatic={...f.state.automatic,state:'complete',reason:'Passed and saved',collected:18};f.state.motion='idle';f.state.saved='/tmp/demo_profile.json';
  f.state.profile={camera:f.state.camera,quality:{reprojection_rms_px:.5},pitch:.1,yaw:0,range_m:[.9,1.6],validation:[{distance:1.2,lateral:0,error_m:.02,passed:true},{distance:1.4,lateral:0,error_m:.03,passed:true}]};
  await expect(page.locator('#autoResult')).toContainText('2.0 cm / 3.0 cm');
  await expect(page.locator('#autoResult')).toContainText('Fitted camera pitch: 5.7° downward');expect(f.errors).toEqual([]);
});
