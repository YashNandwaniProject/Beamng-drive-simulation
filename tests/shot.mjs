/**
 * tests/shot.mjs - quick visual capture of both cars.
 *   node tests/shot.mjs
 */
import { spawn } from 'node:child_process';
import { mkdirSync } from 'node:fs';
import puppeteer from 'puppeteer';

const PORT = 5201;
const server = spawn(process.execPath, ['serve.mjs'],
  { env: { ...process.env, PORT: String(PORT) }, stdio: 'ignore' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
await sleep(700);

const browser = await puppeteer.launch({ headless: 'shell', protocolTimeout: 240000,
  args: ['--no-sandbox', '--enable-unsafe-swiftshader', '--use-gl=angle', '--mute-audio'] });
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 900 });
page.on('console', (m) => { if (m.type() === 'error') console.log('[err]', m.text()); });
await page.goto(`http://localhost:${PORT}/?audio=off`, { waitUntil: 'load' });
await page.waitForFunction('window.sim && window.sim.running === true', { timeout: 180000 });
mkdirSync('tests/screenshots', { recursive: true });

await page.evaluate(() => window.sim.garage.select('lamborghini-revuelto'));
await sleep(600);
await page.screenshot({ path: 'tests/screenshots/g-garage.png' });

for (const id of ['lamborghini-revuelto', 'bmw-m3-gtr']) {
  await page.evaluate((v) => {
    if (!window.sim.garage.visible) window.sim._openGarage();
    window.sim.garage.select(v); window.sim.garage.confirm();
  }, id);
  await page.waitForFunction((v) => window.sim.vehicle
    && window.sim.vehicle.loaded && window.sim.vehicle.id === v && !window.sim.swapping,
  { timeout: 300000 }, id);
  await sleep(2500);
  const tag = id.split('-')[0];
  await page.screenshot({ path: `tests/screenshots/g-${tag}-spawn.png` });
  await page.keyboard.down('w');
  await sleep(5000);
  await page.screenshot({ path: `tests/screenshots/g-${tag}-drive.png` });
  await page.keyboard.down('d');
  await sleep(1500);
  await page.screenshot({ path: `tests/screenshots/g-${tag}-corner.png` });
  await page.keyboard.up('d'); await page.keyboard.up('w');
  await page.keyboard.press('c'); await page.keyboard.press('c');
  await sleep(1500);
  await page.screenshot({ path: `tests/screenshots/g-${tag}-hood.png` });
  await page.keyboard.press('c'); await page.keyboard.press('c');
  await sleep(800);
  await page.keyboard.press('r');
  await sleep(700);
}
console.log('done');
await browser.close(); server.kill();
