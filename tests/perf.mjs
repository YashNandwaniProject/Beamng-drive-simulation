import { spawn } from 'node:child_process';
import puppeteer from 'puppeteer';
const PORT = 5204;
const server = spawn(process.execPath, ['serve.mjs'], { env: { ...process.env, PORT: String(PORT) }, stdio: 'ignore' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
await sleep(700);
const browser = await puppeteer.launch({ headless: 'shell', protocolTimeout: 240000,
  args: ['--no-sandbox', '--enable-unsafe-swiftshader', '--use-gl=angle', '--mute-audio'] });
const page = await browser.newPage();
await page.setViewport({ width: 1280, height: 760 });
page.on('pageerror', (e) => console.log('[PAGEERROR]', e.message));
await page.goto('http://localhost:' + PORT + '/?audio=off', { waitUntil: 'load' });
await page.waitForFunction('window.sim && window.sim.running === true', { timeout: 180000 });
for (const id of ['bmw-m3-gtr', 'lamborghini-revuelto']) {
  await page.evaluate((v) => { window.sim.garage.select(v); window.sim.garage.confirm(); }, id);
  await page.waitForFunction((v) => window.sim.vehicle && window.sim.vehicle.loaded && window.sim.vehicle.id === v,
    { timeout: 240000 }, id);
  await sleep(4000);
  const r = await page.evaluate(() => {
    const s = window.sim, v = s.vehicle;
    let meshes = 0, shadowCasters = 0, tris = 0;
    v.object3D.traverse(o => { if (o.isMesh) { meshes++; if (o.castShadow) shadowCasters++;
      if (o.geometry && o.geometry.index) tris += o.geometry.index.count / 3;
      else if (o.geometry && o.geometry.attributes.position) tris += o.geometry.attributes.position.count / 3; } });
    return { id: v.id, fps: +s.debug.fps.toFixed(1), calls: s.renderer.info.render.calls,
      tris: s.renderer.info.render.triangles, meshes, shadowCasters, carTris: Math.round(tris),
      programs: s.renderer.info.programs.length, geometries: s.renderer.info.memory.geometries,
      textures: s.renderer.info.memory.textures, substeps: s.substeps };
  });
  console.log(JSON.stringify(r));
}
await browser.close(); server.kill();
