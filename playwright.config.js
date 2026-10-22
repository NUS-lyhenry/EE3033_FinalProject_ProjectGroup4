const {defineConfig}=require('@playwright/test');
module.exports=defineConfig({
  testDir:'./tests/browser',
  use:{baseURL:process.env.GROUP4_UI_URL||'http://127.0.0.1:8790',
       viewport:{width:1440,height:1100},headless:true},
  webServer:process.env.GROUP4_UI_URL?undefined:{
    command:'python3 scripts/preview_ui.py --port 8790',
    url:'http://127.0.0.1:8790',reuseExistingServer:!process.env.CI},
  workers:1,
});
