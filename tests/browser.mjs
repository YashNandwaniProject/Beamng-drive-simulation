/**
 * tests/browser.mjs
 * ---------------------------------------------------------------------------
 * End-to-end check in a real headless browser.
 *
 * The head-less suites (run.mjs, vehicles.mjs) prove the simulation is right.
 * This one proves the whole thing actually boots: that the garage works, that
 * both GLBs load and rig correctly, that the exact named objects are found,
 * that the hierarchies survive, that the interior steering wheel animates on
 * the car that should have one and does NOT on the car that must not, and that
 * two browser tabs can see each other's cars.
 *
 *   node tests/browser.mjs [--screenshots]
 * ---------------------------------------------------------------------------
 */

import { spawn } from 'node:child_process';
import { mkdirSync } from 'node:fs';
import puppeteer from 'puppeteer';

const PORT = 5199;
const SHOTS = process.argv.includes('--screenshots');
const SHOT_DIR = 'tests/screenshots';

let passed = 0;
let failed = 0;
const failures = [];

function check(name, cond, detail = '') {
  console.log(`   ${cond ? 'PASS' : 'FAIL'}  ${name.padEnd(54)} ${detail}`);
  if (cond) passed++; else { failed++; failures.push(name); }
  return cond;
}

function section(title) {
  console.log(`\n=== ${title} ${'='.repeat(Math.max(0, 62 - title.length))}`);
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * Poll a page for a condition with short, independent evaluates. A single
 * long-lived waitForFunction can outlive the CDP protocol timeout when the
 * renderer is busy; a retrying poll simply tries again.
 */
/**
 * Evaluate in a page, retrying on a CDP timeout. Same reasoning as `at`.
 */
const safeEval = async (target, fn, arg) => {
  let err;
  for (let i = 0; i < 3; i += 1) {
    try {
      return arg === undefined ? await target.evaluate(fn) : await target.evaluate(fn, arg);
    } catch (e) { err = e; await sleep(1000); }
  }
  throw err;
};

const poll = async (target, expr, timeoutMs) => {
  const t0 = Date.now();
  let last = 'never evaluated';
  while (Date.now() - t0 < timeoutMs) {
    try {
      if (await target.evaluate(`!!(${expr})`)) return true;
      last = 'false';
    } catch (e) { last = String(e).split(String.fromCharCode(10))[0]; }
    await sleep(500);
  }
  poll.lastReason = last;
  return false;
};

/** Hold the throttle until the car reaches `kmh`, then lift. */
const accelTo = async (kmh, cap = 60000) => {
  await page.keyboard.down('w');
  const wall = Date.now();
  let now = await at();
  while (now.speed < kmh && Date.now() - wall < cap) {
    await sleep(120);
    now = await at();
  }
  await page.keyboard.up('w');
  return now;
};

/**
 * Wait for the SIMULATION to advance `seconds`, not the wall clock. Under
 * software WebGL the render loop is slow enough that the substep cap makes the
 * simulation run behind real time, so a plain sleep is not a fixed amount of
 * driving and every distance/speed assertion downstream drifts.
 */
const driveFor = async (seconds, cap = 90000) => {
  const t0 = (await at()).simSeconds;
  const wall = Date.now();
  for (;;) {
    await sleep(120);
    const now = await at();
    if (now.simSeconds - t0 >= seconds || Date.now() - wall > cap) return now;
  }
};

/** The exact objects the two briefs pin down. */
const EXPECTED = {
  'bmw-m3-gtr': {
    wheels: {
      FL: 'Object_221', FR: 'Object_95', RL: 'Object_236', RR: 'Object_206',
    },
    steeringWheel: null,          // must NOT be animated
    drive: 'rwd',
  },
  'lamborghini-revuelto': {
    wheels: {
      FL: 'Wheel_001-material.001',
      FR: 'Wheel_001-material',
      RL: 'Wheel_001-material.003',
      RR: 'Wheel_001-material.002',
    },
    steeringWheel: 'MAT_CarbonFiberMatte-material.006',
    drive: 'awd',
  },
};

/** three.js strips these characters from node names when it imports a glTF. */
const sanitize = (n) => n.replace(/\s/g, '_').replace(/[[\].:/]/g, '');

/* ------------------------------------------------------------------------ */

const server = spawn(process.execPath, ['serve.mjs'], {
  env: { ...process.env, PORT: String(PORT) },
  stdio: 'ignore',
});
await sleep(700);

const browser = await puppeteer.launch({
  headless: 'shell',
  // Software WebGL is slow; the default 30 s protocol timeout is not enough.
  // Not enormous either: a wedged renderer is retried by `at`, and a very long
  // timeout only delays that retry.
  protocolTimeout: 90000,
  args: [
    '--no-sandbox',
    '--enable-unsafe-swiftshader',
    '--use-gl=angle',
    '--mute-audio',
    '--window-size=960,600',
    // Two tabs run at once in the multiplayer section; without these Chrome
    // throttles whichever one is not in front and the second never finishes
    // booting inside the timeout.
    '--disable-backgrounding-occluded-windows',
    '--disable-renderer-backgrounding',
    '--disable-background-timer-throttling',
  ],
});

const page = await browser.newPage();
await page.setViewport({ width: 960, height: 600 });

const consoleLines = [];
const pageErrors = [];
page.on('console', (m) => consoleLines.push(`[${m.type()}] ${m.text()}`));
page.on('pageerror', (e) => pageErrors.push(String(e)));
page.on('requestfailed', (r) => pageErrors.push(`request failed: ${r.url()}`));

/** Snapshot of the live simulation. */
const readState = () => page.evaluate(() => {
  const s = window.sim;
  const v = s.vehicle;
  if (!v) return null;
  const p = v.physics;
  return {
    id: v.id,
    speed: p.speedKmh,
    rpm: p.rpm,
    gear: p.gear,
    x: p.body.position.x,
    z: p.body.position.z,
    contacts: p.contactCount,
    spinFL: p.wheelFL.spinVelocity,
    spinRL: p.wheelRL.spinVelocity,
    angleFL: v.wheelRigs.FL.spin.rotation.z,
    steerPivotFL: v.wheelRigs.FL.steer.rotation.y,
    steerPivotRL: v.wheelRigs.RL.steer.rotation.y,
    steerAngle: p.steering.angle,
    hubFL: v.wheelRigs.FL.hub.position.y,
    deflFL: v.wheelRigs.FL.uniforms.uDeflection.value,
    tyreRadiusLocalFL: v.wheelRigs.FL.uniforms.uRadius.value,
    swAngle: v.steeringWheelPivot ? v.steeringWheelDegrees : null,
    swQuat: v.steeringWheelPivot
      ? v.steeringWheelPivot.quaternion.toArray().map((n) => +n.toFixed(6)) : null,
    eTorqueFL: p.wheelFL.electricTorque,
    gLong: p.gForce.x,
    gLat: p.gForce.z,
    roll: p.rollAngle,
    yawRate: p.yawRate,
    surface: p.wheelFL.surface.name,
    camera: s.cameraSystem.mode,
    substeps: s.substeps,
    simSeconds: s.simSeconds,
    fps: s.debug.fps,
    calls: s.renderer.info.render.calls,
  };
});

/**
 * Read the simulation state, retrying if the renderer was busy enough that the
 * CDP call timed out. Under software WebGL that happens occasionally and it is
 * a property of the CI machine, not of the game.
 */
const at = async () => {
  let err;
  for (let i = 0; i < 3; i += 1) {
    try {
      return await readState();
    } catch (e) { err = e; await sleep(1000); }
  }
  throw err;
};

/** Pick a car in the garage and wait for it to be driveable. */
async function drive(id) {
  await safeEval(page, (v) => {
    if (!window.sim.garage.visible) window.sim._openGarage();
    window.sim.garage.select(v);
    window.sim.garage.confirm();
  }, id);
  if (!await poll(page, `window.sim.vehicle && window.sim.vehicle.loaded
    && window.sim.vehicle.id === ${JSON.stringify(id)} && !window.sim.swapping`, 600000)) {
    throw new Error(`the ${id} never finished loading (${poll.lastReason})`);
  }
  await sleep(2200);
}

try {
  section('Boot and garage');
  await page.goto(`http://localhost:${PORT}/?audio=off&reflections=low`,
    { waitUntil: 'load', timeout: 60000 });
  if (!await poll(page, 'window.sim && window.sim.running === true', 300000)) {
    throw new Error('the page never started its loop');
  }
  check('the page boots and the loop starts', true);

  const errs = consoleLines.filter((l) => l.startsWith('[error]'));
  check('no console errors on boot', errs.length === 0, errs.slice(0, 2).join(' | '));
  check('no uncaught page errors', pageErrors.length === 0, pageErrors.slice(0, 2).join(' | '));

  const garage = await safeEval(page, () => ({
    visible: window.sim.garage.visible,
    cards: [...document.querySelectorAll('.garage-card[data-vehicle]')]
      .map((c) => c.dataset.vehicle),
    metrics: document.querySelectorAll('.cmp-row').length,
    bars: document.querySelectorAll('.cmp-bar').length,
  }));
  check('the garage opens first', garage.visible);
  check('both cars are listed', garage.cards.length === 2
    && garage.cards.includes('bmw-m3-gtr')
    && garage.cards.includes('lamborghini-revuelto'), garage.cards.join(', '));
  check('the comparison has a bar per car per metric',
    garage.bars === garage.metrics * 2, `${garage.metrics} metrics, ${garage.bars} bars`);

  // The comparison must be driven by real spec values, not placeholders.
  const cmp = await safeEval(page, () => {
    const rows = {};
    for (const row of document.querySelectorAll('.cmp-row')) {
      const label = row.querySelector('label').textContent.trim();
      rows[label] = [...row.querySelectorAll('.cmp-bar')].map((b) => ({
        v: b.dataset.vehicle, text: b.querySelector('span').textContent,
      }));
    }
    return rows;
  });
  const power = cmp.Power || [];
  const bmwPower = parseFloat((power.find((p) => p.v === 'bmw-m3-gtr') || {}).text);
  const lamboPower = parseFloat((power.find((p) => p.v === 'lamborghini-revuelto') || {}).text);
  check('the comparison shows different power for the two cars',
    Number.isFinite(bmwPower) && Number.isFinite(lamboPower) && lamboPower > bmwPower * 2,
    `BMW ${bmwPower} PS vs Revuelto ${lamboPower} PS`);
  if (SHOTS) {
    mkdirSync(SHOT_DIR, { recursive: true });
    await safeEval(page, () => window.sim.garage.select('lamborghini-revuelto'));
    await sleep(400);
    await page.screenshot({ path: `${SHOT_DIR}/10-garage.png` });
  }

  /* ==================================================================== */
  /* Per-vehicle rig checks                                               */
  /* ==================================================================== */

  const interiorSnapshots = {};

  for (const id of ['bmw-m3-gtr', 'lamborghini-revuelto']) {
    const want = EXPECTED[id];
    section(`${id} — model, rig and hierarchy`);
    await drive(id);

    const rig = await safeEval(page, () => {
      const v = window.sim.vehicle;
      const out = { rigged: v.riggedObjects.slice(), wheels: {} };
      for (const id2 of ['FL', 'FR', 'RL', 'RR']) {
        const r = v.wheelRigs[id2];
        const chain = [];
        let o = r.tyre;
        while (o && chain.length < 8) { chain.push(o.name); o = o.parent; }
        out.wheels[id2] = {
          tyre: r.tyre.name,
          kids: r.tyre.children.length,
          kidNames: r.tyre.children.map((c) => c.name),
          chain,
          hub: r.hub.position.toArray().map((n) => +n.toFixed(4)),
          radius: r.wheel.radius,
        };
      }
      out.hasSteeringPivot = !!v.steeringWheelPivot;
      out.steeringObject = v.steeringWheelObject ? v.steeringWheelObject.name : null;
      out.drive = v.spec.drive;
      out.mergeStats = v._mergeStats || null;
      return out;
    });

    for (const corner of ['FL', 'FR', 'RL', 'RR']) {
      const w = rig.wheels[corner];
      const expected = want.wheels[corner];
      check(`${corner} uses ${expected}`,
        w.tyre === expected || w.tyre === sanitize(expected),
        w.tyre === expected ? w.tyre : `${w.tyre}  (three.js sanitised form)`);
      check(`${corner} keeps its child objects`, w.kids >= 1,
        `${w.kids}: ${w.kidNames.join(', ')}`);
      check(`${corner} sits under Spin < Camber < Steer < Hub`,
        w.chain[1] === `Spin_${corner}` && w.chain[2] === `Camber_${corner}`
        && w.chain[3] === `Steer_${corner}` && w.chain[4] === `Hub_${corner}`,
        w.chain.slice(0, 5).join(' < '));
    }

    check('front wheels are ahead of the rears in the body frame',
      rig.wheels.FL.hub[0] > 0 && rig.wheels.RL.hub[0] < 0,
      `FL.x ${rig.wheels.FL.hub[0]}, RL.x ${rig.wheels.RL.hub[0]}`);
    check('left wheels are at -Z, right at +Z',
      rig.wheels.FL.hub[2] < 0 && rig.wheels.FR.hub[2] > 0,
      `FL.z ${rig.wheels.FL.hub[2]}, FR.z ${rig.wheels.FR.hub[2]}`);
    check(`drivetrain layout is ${want.drive}`, rig.drive === want.drive, rig.drive);

    if (want.steeringWheel) {
      check('the interior steering wheel object was found',
        !!rig.steeringObject, rig.steeringObject || 'MISSING');
      check('its pivot was built', rig.hasSteeringPivot);
      check('the steering object is the one specified',
        rig.steeringObject === want.steeringWheel
        || rig.steeringObject === sanitize(want.steeringWheel),
        rig.steeringObject);
    } else {
      check('NO interior steering wheel is rigged', !rig.hasSteeringPivot);
    }

    if (id === 'lamborghini-revuelto') {
      check('the staggered axles use different radii',
        Math.abs(rig.wheels.FL.radius - rig.wheels.RL.radius) > 0.015,
        `front ${rig.wheels.FL.radius} m, rear ${rig.wheels.RL.radius} m`);
      check('the body was batched for performance',
        rig.mergeStats && rig.mergeStats.batches > 0,
        rig.mergeStats
          ? `${rig.mergeStats.meshesRemoved} meshes -> ${rig.mergeStats.batches} draw calls`
          : 'not merged');
    }

    /* --- interior transform snapshot, for the "did it move" test --------- */
    interiorSnapshots[id] = await safeEval(page, () => {
      const v = window.sim.vehicle;
      const out = [];
      v.bodyModel.traverse((o) => {
        const mats = o.material
          ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
        if (mats.some((m) => m && /INTERIOR|Alcantara|Pelle/i.test(m.name || ''))) {
          out.push(JSON.stringify({
            n: o.name,
            q: o.quaternion.toArray().map((x) => +x.toFixed(6)),
            p: o.position.toArray().map((x) => +x.toFixed(6)),
          }));
        }
      });
      return out;
    });

    /* --- driving --------------------------------------------------------- */
    section(`${id} — driving`);
    const idle = await at();
    check('all four wheels on the ground at spawn', idle.contacts === 4, `${idle.contacts}/4`);
    check('the engine is idling', idle.rpm > 650 && idle.rpm < 1100,
      `${idle.rpm.toFixed(0)} rpm`);
    const squash = idle.deflFL / idle.tyreRadiusLocalFL;
    check('the tyres are squashed by the car weight',
      squash > 0.004 && squash < 0.12,
      `${(squash * 100).toFixed(2)} % of the tyre radius`);
    if (SHOTS) await page.screenshot({ path: `${SHOT_DIR}/11-${id}-spawn.png` });

    // Accelerate.
    await page.keyboard.down('w');
    const fast = await driveFor(4.5);
    await page.keyboard.up('w');
    check('the car accelerates', fast.speed > 40, `${fast.speed.toFixed(1)} km/h`);
    check('it actually moved', Math.hypot(fast.x - idle.x, fast.z - idle.z) > 25,
      `${Math.hypot(fast.x - idle.x, fast.z - idle.z).toFixed(1)} m`);
    check('the gearbox shifted up', fast.gear !== '1', `gear ${fast.gear}`);
    check('the wheels are turning', fast.spinFL > 5, `${fast.spinFL.toFixed(1)} rad/s`);
    check('the visual spin angle tracks the physics', Math.abs(fast.angleFL) > 1,
      `${fast.angleFL.toFixed(2)} rad`);
    check('positive longitudinal G under power', fast.gLong > 0.05,
      `${fast.gLong.toFixed(2)} G`);

    if (id === 'lamborghini-revuelto') {
      check('the front axle is driven electrically', fast.eTorqueFL > 10,
        `${fast.eTorqueFL.toFixed(0)} N.m at the front wheel`);
    } else {
      check('the front wheels receive no drive torque', fast.eTorqueFL === 0);
    }
    if (SHOTS) await page.screenshot({ path: `${SHOT_DIR}/12-${id}-driving.png` });

    // Steer, and watch the interior wheel. Reset first so the car starts from
    // a known point in the middle of the road: the acceleration run above ends
    // wherever the machine's frame rate happened to leave it, and a hard lock
    // from an unknown position can put the car in the scenery.
    await page.keyboard.press('r');
    await sleep(700);
    await accelTo(55);
    await page.keyboard.down('w');
    await page.keyboard.down('d');
    const right = await driveFor(1.0);
    await page.keyboard.up('d');
    await page.keyboard.down('a');
    const left = await driveFor(1.6);
    await page.keyboard.up('a');
    await page.keyboard.up('w');

    check('the front wheels steer', Math.abs(right.steerPivotFL) > 0.02,
      `pivot ${(right.steerPivotFL * 180 / Math.PI).toFixed(1)} deg`);
    check('the rear wheels do NOT steer',
      Math.abs(right.steerPivotRL - idle.steerPivotRL) < 1e-9,
      `held at ${(idle.steerPivotRL * 180 / Math.PI).toFixed(3)} deg of static toe`);
    check('the steer pivot is the inverse of the physics angle',
      Math.sign(right.steerPivotFL) === -Math.sign(right.steerAngle),
      `physics ${(right.steerAngle * 180 / Math.PI).toFixed(1)} deg`);
    check('the car yaws into the corner', Math.abs(right.yawRate) > 0.05,
      `${(right.yawRate * 180 / Math.PI).toFixed(1)} deg/s`);

    section(`${id} — interior steering wheel`);
    if (want.steeringWheel) {
      check('it rotates when steering right', Math.abs(right.swAngle) > 5,
        `${right.swAngle.toFixed(1)} deg`);
      check('it rotates the OTHER way when steering left',
        Math.sign(left.swAngle) === -Math.sign(right.swAngle)
        && Math.abs(left.swAngle) > 5,
        `right ${right.swAngle.toFixed(1)} deg, left ${left.swAngle.toFixed(1)} deg`);
      check('it turns further than the road wheels (steering ratio)',
        Math.abs(right.swAngle) > Math.abs(right.steerAngle * 180 / Math.PI) * 5,
        `${right.swAngle.toFixed(0)} deg wheel vs `
        + `${(right.steerAngle * 180 / Math.PI).toFixed(1)} deg road`);
      check('steering LEFT turns it anticlockwise from the driving seat',
        left.swAngle > 0,
        `left -> ${left.swAngle.toFixed(1)} deg about the column axis`);
      check('the pivot quaternion actually changes',
        JSON.stringify(right.swQuat) !== JSON.stringify(left.swQuat));
    } else {
      check('there is no interior steering wheel animation', right.swAngle === null);
    }

    // Whatever the car, its INTERIOR MESHES must not have moved.
    const after = await safeEval(page, () => {
      const v = window.sim.vehicle;
      const out = [];
      v.bodyModel.traverse((o) => {
        const mats = o.material
          ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
        if (mats.some((m) => m && /INTERIOR|Alcantara|Pelle/i.test(m.name || ''))) {
          out.push(JSON.stringify({
            n: o.name,
            q: o.quaternion.toArray().map((x) => +x.toFixed(6)),
            p: o.position.toArray().map((x) => +x.toFixed(6)),
          }));
        }
      });
      return out;
    });
    check('no interior mesh moved during hard driving',
      JSON.stringify(after) === JSON.stringify(interiorSnapshots[id]),
      `${after.length} interior meshes compared`);

    // Braking, again from a reset so the stop happens on the tarmac.
    await page.keyboard.press('r');
    await sleep(700);
    await accelTo(95);
    /**
     * The flat spot must stay at the BOTTOM of every wheel while it spins.
     *
     * Sampled from inside `syncVisuals`, because that is the only instant at
     * which the shader's contact direction and the wheel's transform describe
     * the same moment - between frames the wheel has already turned, at 150
     * km/h by more than a whole revolution.
     *
     * This is the guard on the mirrored rear right (`Wheel_001-material.002`
     * on the Lamborghini), whose flat spot used to ride round with the tyre.
     */
    await safeEval(page, () => {
      const v = window.sim.vehicle;
      const inner = v.syncVisuals.bind(v);
      const worst = {};
      let frames = 0;
      window.__flat = { worst, frames: 0, restore: () => { v.syncVisuals = inner; } };
      v.syncVisuals = (alpha, dt) => {
        inner(alpha, dt);
        const p = v.physics;
        for (const id of ['FL', 'FR', 'RL', 'RR']) {
          const rig = v.wheelRigs[id];
          const w = p[`wheel${id}`];
          /**
           * Only when a flat spot is actually being DRAWN. The shader's own
           * guard is `uDeflection > 1e-5`; below that the uniform is left at
           * its last value and there is no deformation to misplace, so
           * including those frames would measure a direction nothing uses.
           */
          if (!w.hasContact || rig.uniforms.uDeflection.value <= 1e-5) continue;
          rig.tyre.updateWorldMatrix(true, false);
          const e = rig.tyre.matrixWorld.elements;
          const toWorld = (v) => {
            const x = e[0] * v.x + e[4] * v.y + e[8] * v.z;
            const y = e[1] * v.x + e[5] * v.y + e[9] * v.z;
            const z = e[2] * v.x + e[6] * v.y + e[10] * v.z;
            const l = Math.hypot(x, y, z) || 1;
            return { x: x / l, y: y / l, z: z / l };
          };
          const dir = toWorld(rig.uniforms.uContactDir.value);
          const axis = toWorld(rig.uniforms.uAxis.value);
          const n = w.contactNormal;
          const nl = Math.hypot(n.x, n.y, n.z) || 1;
          /**
           * Compare IN THE WHEEL PLANE. The shader's direction has its axial
           * component removed on purpose, so measuring it against the raw road
           * normal would just report the wheel's camber - a couple of degrees
           * that are meant to be there.
           */
          let rx = -n.x / nl; let ry = -n.y / nl; let rz = -n.z / nl;
          const ax = rx * axis.x + ry * axis.y + rz * axis.z;
          rx -= axis.x * ax; ry -= axis.y * ax; rz -= axis.z * ax;
          const rl = Math.hypot(rx, ry, rz) || 1;
          const dot = (dir.x * rx + dir.y * ry + dir.z * rz) / rl;
          const deg = Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
          worst[id] = Math.max(worst[id] || 0, deg);
        }
        frames++;
        window.__flat.frames = frames;
      };
    });
    // Wait OUTSIDE the page. A three-second evaluate is a three-second CDP
    // call, and a busy renderer turns that into a protocol timeout.
    await sleep(3000);
    const flat = await safeEval(page, () => {
      window.__flat.restore();
      return { worst: window.__flat.worst, frames: window.__flat.frames };
    });
    const worstWheel = Object.entries(flat.worst)
      .sort((a, b) => b[1] - a[1])[0] || ['none', 0];
    check('the flat spot stays at the road on every wheel',
      flat.frames > 5 && worstWheel[1] < 0.5,
      `worst ${worstWheel[0]} ${worstWheel[1].toFixed(3)} deg over ${flat.frames} frames`);

    const beforeBrake = await at();
    await safeEval(page, () => {
      const p = window.sim.vehicle.physics;
      const v = window.sim.vehicle;
      window.__rec = { maxFront: -1, minRear: 1, minG: 0, abs: false, err: 0, n: 0 };
      window.__id = setInterval(() => {
        const r = window.__rec;
        const fl = p.wheelFL;
        r.maxFront = Math.max(r.maxFront, fl.suspension.displacement);
        r.minRear = Math.min(r.minRear, p.wheelRL.suspension.displacement);
        r.minG = Math.min(r.minG, p.gForce.x);
        if (p.brakes.anyAbsActive) r.abs = true;
        const expect = v.wheelRigs.FL.restHubY
          + fl.suspension.displacement - fl.tire.deflection;
        r.err = Math.max(r.err, Math.abs(v.wheelRigs.FL.hub.position.y - expect));
        r.n++;
      }, 16);
    });
    await page.keyboard.down('s');
    const braking = await driveFor(1.5);
    await page.keyboard.up('s');
    const rec = await safeEval(page, () => { clearInterval(window.__id); return window.__rec; });

    check('braking slows the car', braking.speed < beforeBrake.speed * 0.75,
      `${beforeBrake.speed.toFixed(1)} -> ${braking.speed.toFixed(1)} km/h`);
    check('negative longitudinal G under braking', rec.minG < -0.4,
      `${rec.minG.toFixed(2)} G`);
    check('the front suspension compresses', rec.maxFront > 0.003,
      `+${(rec.maxFront * 1000).toFixed(1)} mm`);
    check('the rear suspension extends', rec.minRear < -0.003,
      `${(rec.minRear * 1000).toFixed(1)} mm`);
    check('ABS modulated during the stop', rec.abs);
    check('the visual wheel exactly follows the physics', rec.err < 1e-9,
      `max error ${rec.err.toExponential(2)} m over ${rec.n} samples`);
    if (SHOTS) await page.screenshot({ path: `${SHOT_DIR}/13-${id}-braking.png` });

    // Reset for the next car.
    await page.keyboard.press('r');
    await sleep(600);
  }

  /* ==================================================================== */
  section('Cameras, reset, pause and telemetry');
  const modes = [];
  for (let i = 0; i < 5; i++) {
    await page.keyboard.press('c');
    await sleep(420);
    modes.push((await at()).camera);
  }
  check('C cycles through all five camera modes', new Set(modes).size === 5,
    modes.join(' -> '));
  check('one of them is the in-car view', modes.includes('interior'),
    modes.join(' -> '));

  // The cockpit camera must actually be INSIDE the car, not near it.
  await safeEval(page, () => { window.sim.cameraSystem.setMode('interior'); });
  await sleep(900);
  const eye = await safeEval(page, () => {
    const s = window.sim;
    const c = s.cameraSystem.camera.position;
    const b = s.vehicle.physics.body.position;
    return {
      dist: Math.hypot(c.x - b.x, c.y - b.y, c.z - b.z),
      above: c.y - s.terrain.quickHeight(c.x, c.z),
      fov: s.cameraSystem.camera.fov,
    };
  });
  check('the cockpit eye sits inside the cabin', eye.dist < 1.2,
    `${eye.dist.toFixed(2)} m from the centre of mass`);
  check('it is at head height above the road', eye.above > 0.75 && eye.above < 1.5,
    `${eye.above.toFixed(2)} m`);
  check('it uses the narrower cockpit field of view', eye.fov < 62,
    `${eye.fov.toFixed(1)} deg`);

  await page.keyboard.press('h');
  await sleep(400);
  check('H opens the telemetry overlay', await safeEval(page, 
    () => window.sim.debug.enabled
      && document.getElementById('debug').style.display !== 'none',
  ));
  // The overlay fills in on its next update tick, which is a frame or two away
  // and can be several hundred ms under software WebGL.
  let dbg = '';
  for (let i = 0; i < 60 && !dbg.trim(); i += 1) {
    dbg = await safeEval(page, () => document.getElementById('debug').textContent);
    if (!dbg.trim()) await sleep(200);
  }
  check('telemetry names the current vehicle', dbg.includes('Lamborghini'),
    dbg.slice(0, 60).replace(/\s+/g, ' '));
  check('telemetry shows the electric axle for the hybrid',
    dbg.includes('e-axle'), '');
  if (SHOTS) await page.screenshot({ path: `${SHOT_DIR}/14-telemetry.png` });
  await page.keyboard.press('h');
  await sleep(200);

  await page.keyboard.press('Escape');
  await sleep(350);
  check('Esc pauses', await safeEval(page, () => window.sim.paused === true));
  await page.keyboard.press('Escape');
  await sleep(250);
  check('Esc resumes', await safeEval(page, () => window.sim.paused === false));

  /* ==================================================================== */
  section('Frame budget');
  await sleep(1800);
  const perf = await at();
  console.log(`       ${perf.fps.toFixed(1)} fps, ${perf.calls} draw calls, `
    + `${perf.substeps} physics substeps/frame (software WebGL in CI)`);
  check('the render loop runs', perf.fps > 8, `${perf.fps.toFixed(1)} fps`);
  check('draw calls stay reasonable for the heavier car', perf.calls < 700,
    `${perf.calls}`);
  check('physics substeps stay within budget',
    perf.substeps > 0 && perf.substeps <= 24, `${perf.substeps}`);

  /* ==================================================================== */
  section('Reflections');
  const refl = await safeEval(page, () => {
    const r = window.sim.reflections;
    const v = window.sim.vehicle;
    const road = window.sim.environment.roadMesh.material;
    return {
      level: r.level.name,
      size: r.level.size,
      captures: r.captures,
      ms: r.lastCaptureMs,
      roadEnv: !!road.envMap,
      roadIsProbe: road.envMap === r.texture,
      roadClearcoat: road.clearcoat,
      bodyCount: v.reflectiveMaterials.length,
      bodyOnProbe: v.reflectiveMaterials.filter((m) => m.material.envMap === r.texture).length,
      paintPhysical: v.reflectiveMaterials
        .filter((m) => /CARPAINT|CARSKIN/i.test(m.material.name || ''))
        .filter((m) => m.material.isMeshPhysicalMaterial && m.material.clearcoat > 0).length,
    };
  });
  check('the probe is capturing the scene', refl.captures > 0,
    `${refl.captures} captures at ${refl.size} px, ${refl.ms.toFixed(1)} ms each`);
  check('the road reflects the probe, not just the sky', refl.roadIsProbe);
  check('the asphalt has a clear coat to reflect in', refl.roadClearcoat > 0,
    `clearcoat ${refl.roadClearcoat}`);
  check('the Lamborghini body samples the probe',
    refl.bodyCount > 0 && refl.bodyOnProbe === refl.bodyCount,
    `${refl.bodyOnProbe}/${refl.bodyCount} materials`);
  check('its paint is clear-coated', refl.paintPhysical > 0,
    `${refl.paintPhysical} clear-coated paint material(s)`);

  // Off must be genuinely off, and must put the sky IBL back rather than
  // leaving the panels with no environment at all.
  const off = await safeEval(page, () => {
    const r = window.sim.reflections;
    while (r.level.name !== 'Off') r.cycle();
    const v = window.sim.vehicle;
    return {
      enabled: r.enabled,
      textureIsNull: r.texture === null,
      bodyHasEnv: v.reflectiveMaterials.filter((m) => m.material.envMap).length,
      bodyCount: v.reflectiveMaterials.length,
    };
  });
  check('V can switch reflections off', off.enabled === false && off.textureIsNull);
  check('switching off restores the sky environment', off.bodyHasEnv === 0,
    `${off.bodyHasEnv}/${off.bodyCount} still hold a probe map`);
  await safeEval(page, () => { window.sim.reflections.setLevel(1); });

  /* ==================================================================== */
  section('Multiplayer — two tabs in one session');
  const page2 = await browser.newPage();
  await page2.setViewport({ width: 900, height: 600 });
  const p2errors = [];
  page2.on('pageerror', (e) => p2errors.push(String(e)));
  // Two SwiftShader contexts on one machine starve each other badly enough
  // that the second tab's boot can block its renderer past the CDP protocol
  // timeout. Shrink the first tab's framebuffer while the second loads, then
  // give it back. This is a CI resource concession, nothing to do with the game.
  await page.setViewport({ width: 320, height: 200 });
  await page2.goto(`http://localhost:${PORT}/?audio=off&reflections=off`,
    { waitUntil: 'load', timeout: 120000 });
  const booted = await poll(page2, 'window.sim && window.sim.running === true', 600000);
  if (!booted) throw new Error('the second tab never finished booting');

  // Second player takes the BMW; the first is already in the Lamborghini.
  await safeEval(page2, () => {
    window.sim.garage.select('bmw-m3-gtr');
    window.sim.garage.confirm();
  });
  const swapped = await poll(page2,
    'window.sim.vehicle && window.sim.vehicle.loaded && !window.sim.swapping', 600000);
  await page.setViewport({ width: 960, height: 600 });   // give tab one its pixels back
  if (!swapped) throw new Error('the second tab never finished swapping vehicle');
  await sleep(4000);

  const net1 = await safeEval(page, () => ({
    connected: window.sim.network.connected,
    peers: window.sim.network.peerCount,
    remote: window.sim.remotePlayers.count,
    remoteIds: [...window.sim.remotePlayers.entries.values()]
      .filter((e) => e.ready).map((e) => e.vehicle.id),
  }));
  const net2 = await safeEval(page2, () => ({
    connected: window.sim.network.connected,
    peers: window.sim.network.peerCount,
    remote: window.sim.remotePlayers.count,
    remoteIds: [...window.sim.remotePlayers.entries.values()]
      .filter((e) => e.ready).map((e) => e.vehicle.id),
  }));

  check('tab 1 is connected', net1.connected);
  check('tab 2 is connected', net2.connected);
  check('tab 1 sees the other player', net1.peers >= 1, `${net1.peers} peer(s)`);
  check('tab 2 sees the other player', net2.peers >= 1, `${net2.peers} peer(s)`);
  check('tab 1 spawned the remote car', net1.remote >= 1,
    net1.remoteIds.join(', ') || 'none');
  check('tab 2 spawned the remote car', net2.remote >= 1,
    net2.remoteIds.join(', ') || 'none');
  check('each tab loaded the OTHER car, chosen by vehicle id',
    net1.remoteIds.includes('bmw-m3-gtr')
    && net2.remoteIds.includes('lamborghini-revuelto'),
    `tab1 sees ${net1.remoteIds.join(',')}; tab2 sees ${net2.remoteIds.join(',')}`);

  // Drive tab 2 and check tab 1's replica follows, wheels and all.
  const before = await safeEval(page, () => {
    const e = [...window.sim.remotePlayers.entries.values()].find((x) => x.ready);
    if (!e) return null;
    return {
      x: e.vehicle.object3D.position.x,
      z: e.vehicle.object3D.position.z,
      spin: e.vehicle.wheelRigs.FL.spin.rotation.z,
      steer: e.vehicle.wheelRigs.FL.steer.rotation.y,
      susp: e.vehicle.wheelRigs.FL.hub.position.y,
    };
  });
  await page2.keyboard.down('w');
  await page2.keyboard.down('d');
  await sleep(4000);
  /**
   * Sample the steer angle WHILE the other tab is still turning. Read after
   * the key is released it measures the rack recentring, not the steering -
   * which is a property of the test, not of the replication.
   */
  const whileSteering = await safeEval(page, () => {
    const e = [...window.sim.remotePlayers.entries.values()].find((x) => x.ready);
    return e ? e.vehicle.wheelRigs.FL.steer.rotation.y : null;
  });
  await page2.keyboard.up('d');
  await page2.keyboard.up('w');
  await sleep(600);
  const after2 = await safeEval(page, () => {
    const e = [...window.sim.remotePlayers.entries.values()].find((x) => x.ready);
    if (!e) return null;
    return {
      x: e.vehicle.object3D.position.x,
      z: e.vehicle.object3D.position.z,
      spin: e.vehicle.wheelRigs.FL.spin.rotation.z,
      steer: e.vehicle.wheelRigs.FL.steer.rotation.y,
      susp: e.vehicle.wheelRigs.FL.hub.position.y,
    };
  });

  check('the remote car MOVED on the other screen',
    before && after2 && Math.hypot(after2.x - before.x, after2.z - before.z) > 5,
    before && after2
      ? `${Math.hypot(after2.x - before.x, after2.z - before.z).toFixed(1)} m`
      : 'no remote entry');

  /**
   * And it moved to the RIGHT PLACE, which is the property that actually
   * matters and the one a distance threshold cannot check. Sampled once both
   * cars have stopped, so the fixed 0.12 s interpolation delay - which is
   * metres of legitimate offset at speed - is not being measured as error.
   */
  await sleep(3500);
  const truth = await safeEval(page2, () => {
    const b = window.sim.vehicle.physics.body.position;
    return { x: b.x, z: b.z, speed: window.sim.vehicle.physics.speedKmh };
  });
  const mirrored = await safeEval(page, () => {
    const e = [...window.sim.remotePlayers.entries.values()].find((x) => x.ready);
    return e ? { x: e.vehicle.object3D.position.x, z: e.vehicle.object3D.position.z } : null;
  });
  const agree = truth && mirrored
    ? Math.hypot(truth.x - mirrored.x, truth.z - mirrored.z) : Infinity;
  check('and it is where the other tab actually is', agree < 2.0,
    `${agree.toFixed(2)} m apart at ${truth ? truth.speed.toFixed(1) : '?'} km/h`);
  check('its wheels are ROLLING on the other screen',
    before && after2 && Math.abs(after2.spin - before.spin) > 5,
    before && after2 ? `${Math.abs(after2.spin - before.spin).toFixed(1)} rad` : '');
  check('its front wheels STEERED on the other screen',
    whileSteering !== null && Math.abs(whileSteering) > 0.05,
    whileSteering !== null
      ? `${(whileSteering * 180 / Math.PI).toFixed(1)} deg while the other tab held D`
      : 'no remote entry');
  check('and they recentred once it stopped',
    before && after2 && Math.abs(after2.steer) < Math.abs(whileSteering ?? 1),
    after2 ? `${(after2.steer * 180 / Math.PI).toFixed(1)} deg after release` : '');
  check('its suspension state replicated',
    before && after2 && Number.isFinite(after2.susp), `${after2 && after2.susp}`);
  check('tab 2 logged no page errors', p2errors.length === 0,
    p2errors.slice(0, 2).join(' | '));
  if (SHOTS) await page.screenshot({ path: `${SHOT_DIR}/15-multiplayer.png` });

  // Leaving must clean up.
  await page2.close();
  await sleep(6500);
  const afterLeave = await safeEval(page, () => ({
    peers: window.sim.network.peerCount,
    remote: window.sim.remotePlayers.count,
  }));
  check('the peer is dropped when its tab closes',
    afterLeave.peers === 0 && afterLeave.remote === 0,
    `${afterLeave.peers} peers, ${afterLeave.remote} cars`);

  const finalErrors = consoleLines.filter((l) => l.startsWith('[error]'));
  check('still no console errors at the end', finalErrors.length === 0,
    finalErrors.slice(0, 3).join(' | '));
} catch (err) {
  failed++;
  failures.push('harness');
  console.error('\n  HARNESS ERROR:', err);
  console.error('  console tail:\n   ', consoleLines.slice(-18).join('\n    '));
  console.error('  page errors:\n   ', pageErrors.slice(-8).join('\n    '));
} finally {
  await browser.close();
  server.kill();
}

console.log(`\n${'='.repeat(74)}`);
console.log(`  ${passed} passed, ${failed} failed`);
if (failed) {
  console.log('\n  Failures:');
  for (const f of failures) console.log(`   - ${f}`);
}
console.log(`${'='.repeat(74)}\n`);
process.exit(failed ? 1 : 0);
