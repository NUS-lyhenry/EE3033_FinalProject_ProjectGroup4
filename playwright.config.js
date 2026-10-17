const {defineConfig}=require('@playwright/test');
module.exports=defineConfig({
  testDir:'./tests/browser',
  use:{baseURL:process.env.GROUP4_UI_URL||'http://127.0.0.1:8765',
       viewport:{width:1440,height:1100},headless:true},
  workers:1,
});
