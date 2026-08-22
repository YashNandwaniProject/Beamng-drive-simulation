/**
 * tests/vehicles.mjs
 * ---------------------------------------------------------------------------
 * Multi-vehicle regression suite.
 *
 * tests/run.mjs pins the BMW's behaviour. This one covers everything that came
 * in with the second car:
 *
 *   - the registry and vehicle identity used by multiplayer
 *   - the Lamborghini's measured geometry against the actual .glb
 *   - its physics: rest, launch, braking, cornering, reverse, handbrake, bumps
 *   - the AWD electric axle and the dual-clutch gearbox, which the BMW has
 *     neither of
 *   - that the two cars genuinely DO NOT drive alike
 *   - timestep independence for the new drivetrain paths
 *   - the network state codec round trip
 *
 *   node tests/vehicles.mjs
 * ---------------------------------------------------------------------------
 */

import { Terrain, SurfaceSample } from '../js/terrain.js';
import { VehiclePhysics } from '../js/vehiclePhysics.js';
import { Vec3 } from '../js/mathUtils.js';
import { SIM, SPAWN, SURFACES } from '../js/config.js';
import {
  getVehicleSpec, allVehicleSpecs, vehicleStats,
  VEHICLE_IDS, DEFAULT_VEHICLE_ID,
} from '../js/vehicles/registry.js';
import { captureVehicleState, applyVehicleState } from '../js/network.js';

const RAD2DEG = 180 / Math.PI;
const BMW = getVehicleSpec('bmw-m3-gtr');
const LAMBO = getVehicleSpec('lamborghini-revuelto');

let passed = 0;
let failed = 0;
const failures = [];

function check(name, value, min, max, unit = '') {
  const ok = Number.isFinite(value) && value >= min && value <= max;
  const v = typeof value === 'number' ? value.toFixed(3) : String(value);
  console.log(`   ${ok ? 'PASS' : 'FAIL'}  ${name.padEnd(46)} ${v.padStart(10)}${unit}`
    + `${ok ? '' : `   expected ${min}..${max}`}`);
  if (ok) passed++; else { failed++; failures.push(name); }
  return ok;
}

function checkTrue(name, cond, detail = '') {
  console.log(`   ${cond ? 'PASS' : 'FAIL'}  ${name.padEnd(46)} ${detail}`);
  if (cond) passed++; else { failed++; failures.push(name); }
  return cond;
}

function section(title) {
  console.log(`\n=== ${title} ${'='.repeat(Math.max(0, 62 - title.length))}`);
}

/* ------------------------------------------------------------------------ */
/* Fixtures                                                                  */
/* ------------------------------------------------------------------------ */

const circuit = new Terrain();

class Skidpad {
  constructor(s = SURFACES.ASPHALT) { this.surface = s; this.length = 1e6; }
  sample(x, z, o) {
    o.height = 0; o.normal.set(0, 1, 0); o.surface = this.surface;
    o.lateral = 0; o.distance = 0; o.onRoad = true; o.index = 0; return o;
  }
  quickHeight() { return 0; }
}

const NEUTRAL = { throttle: 0, brake: 0, steer: 0, handbrake: 0 };
const HANDBRAKE_ONLY = { throttle: 0, brake: 0, steer: 0, handbrake: 1 };

function makeCar(spec, where = 'skidpad', hz = SIM.physicsHz, opts = {}) {
  const terrain = where === 'circuit' ? circuit : new Skidpad(opts.surface);
  const v = new VehiclePhysics(terrain, spec);
  if (where === 'circuit') {
    const sp = circuit.spawnTransform(
      opts.distance ?? SPAWN.trackDistance, opts.lateral ?? SPAWN.lateralOffset,
    );
    v.reset(sp.position, sp.heading);
  } else {
    v.reset(new Vec3(0, 0, 0), 0);
  }
  v.__dt = 1 / hz;
  return v;
}

function run(car, seconds, inputFn, sampleFn) {
  const dt = car.__dt;
  const steps = Math.round(seconds / dt);
  for (let i = 0; i < steps; i++) {
    const t = i * dt;
    const input = typeof inputFn === 'function' ? inputFn(t, car) : (inputFn || NEUTRAL);
    car.step(dt, input);
    if (sampleFn) sampleFn(t + dt, car);
  }
}

function accelerateTo(car, speed, maxSeconds = 25) {
  run(car, maxSeconds, (t, c) => ({
    throttle: c.forwardSpeed < speed ? 1 : 0, brake: 0, steer: 0, handbrake: 0,
  }));
  return car;
}

function brakeToStop(car, sampleFn, maxSeconds = 12) {
  const dt = car.__dt;
  const p0 = car.body.position.clone();
  const input = { throttle: 0, brake: 1, steer: 0, handbrake: 0 };
  let t = 0;
  while (t < maxSeconds && car.speed > 0.3) {
    car.step(dt, input);
    t += dt;
    if (sampleFn) sampleFn(t, car);
  }
  return { distance: car.body.position.distanceTo(p0), time: t };
}

/** Measure a car's headline numbers on the flat skidpad. */
function profile(spec) {
  const accel = makeCar(spec);
  let t100 = null;
  let t60 = null;
  let maxG = 0;
  run(accel, 18, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    maxG = Math.max(maxG, c.gForce.x);
    if (t60 === null && c.speedKmh >= 60) t60 = t;
    if (t100 === null && c.speedKmh >= 100) t100 = t;
  });
  const v18 = accel.speedKmh;

  const brake = makeCar(spec);
  accelerateTo(brake, 27.8);
  run(brake, 0.4, NEUTRAL);
  const entry = brake.speed;
  let peakBrakeG = 0;
  const stop = brakeToStop(brake, (t, c) => {
    peakBrakeG = Math.max(peakBrakeG, -c.gForce.x);
  });

  const corner = makeCar(spec);
  accelerateTo(corner, 26);
  let maxLat = 0;
  let maxRoll = 0;
  run(corner, 4, { throttle: 0.30, brake: 0, steer: 0.45, handbrake: 0 }, (t, c) => {
    if (t > 1.2) {
      maxLat = Math.max(maxLat, Math.abs(c.gForce.z));
      maxRoll = Math.max(maxRoll, Math.abs(c.rollAngle));
    }
  });

  return {
    t60, t100, maxG, v18,
    entryKmh: entry * 3.6,
    brakeDist: stop.distance,
    peakBrakeG,
    lateralG: maxLat,
    rollDeg: maxRoll * RAD2DEG,
    rollGradient: (maxRoll * RAD2DEG) / Math.max(maxLat, 0.01),
  };
}

/* ========================================================================= */

section('1. Registry and vehicle identity');
{
  check('vehicles registered', VEHICLE_IDS.length, 2, 8);
  checkTrue('the Lamborghini is registered under its exact id',
    VEHICLE_IDS.includes('lamborghini-revuelto'), VEHICLE_IDS.join(', '));
  checkTrue('lookup by id returns the right spec',
    getVehicleSpec('lamborghini-revuelto').displayName === 'Lamborghini Revuelto');
  checkTrue('the default is still the pre-existing car',
    DEFAULT_VEHICLE_ID === 'bmw-m3-gtr', DEFAULT_VEHICLE_ID);
  checkTrue('every spec has a unique id',
    new Set(allVehicleSpecs().map((s) => s.id)).size === VEHICLE_IDS.length);
  checkTrue('an unknown id falls back rather than crashing',
    getVehicleSpec('does-not-exist').id === DEFAULT_VEHICLE_ID);
  checkTrue('identity is the id, not the display name',
    getVehicleSpec('lamborghini-revuelto') !== getVehicleSpec('Lamborghini Revuelto'));
}

section('2. The BMW is untouched by the refactor');
{
  check('mass', BMW.chassis.mass, 1720, 1720, ' kg');
  check('wheelbase', BMW.axle.wheelbase, 2.7315, 2.7315, ' m');
  check('CoM -> front axle', BMW.axle.a, 1.3110, 1.3110, ' m');
  check('CoM -> rear axle', BMW.axle.b, 1.4205, 1.4205, ' m');
  check('hub height (uniform, as before)', BMW.axle.hubY, -0.17180, -0.17179, ' m');
  check('ride height', BMW.rideHeight, 0.5008, 0.5008, ' m');
  check('front static load', BMW.suspension.front.staticLoad, 4385.5, 4386.5, ' N');
  check('front spring rate', BMW.suspension.front.springRate, 34000, 34000, ' N/m');
  check('peak friction', BMW.tyre.muNominal, 1.25, 1.25);
  check('steering lock', BMW.steering.maxAngle * RAD2DEG, 32.99, 33.01, ' deg');
  checkTrue('still rear-wheel drive', BMW.drive === 'rwd', BMW.drive);
  checkTrue('still a torque converter', !!BMW.converter && !BMW.clutch);
  checkTrue('has no electric axle', !BMW.electric);
  checkTrue('its interior steering wheel is still NOT animated',
    !BMW.model.steeringWheel);
}

section('3. Lamborghini geometry matches the .glb');
{
  const m = LAMBO.model;
  checkTrue('model is yawed 180 deg into the body frame',
    Math.abs(m.modelYaw - Math.PI) < 1e-9, `${m.modelYaw}`);
  check('wheelbase', LAMBO.axle.wheelbase, 2.780, 2.783, ' m');
  check('front track', LAMBO.axle.trackFront, 1.710, 1.714, ' m');
  check('rear track', LAMBO.axle.trackRear, 1.678, 1.682, ' m');
  check('front tyre radius', m.radiusFront, 0.318, 0.321, ' m');
  check('rear tyre radius', m.radiusRear, 0.343, 0.346, ' m');
  check('rear tyre width', m.widthRear, 0.33, 0.345, ' m');
  check('front weight fraction',
    (LAMBO.axle.b / LAMBO.axle.wheelbase) * 100, 43.5, 44.5, ' %');
  check('CoM height above the road', LAMBO.rideHeight, 0.42, 0.46, ' m');

  checkTrue('the front axle is ahead of the rear in the body frame',
    LAMBO.hubs.FL.x > 0 && LAMBO.hubs.RL.x < 0,
    `FL.x ${LAMBO.hubs.FL.x.toFixed(3)}, RL.x ${LAMBO.hubs.RL.x.toFixed(3)}`);
  checkTrue('left wheels are at -Z, right at +Z (body frame)',
    LAMBO.hubs.FL.z < 0 && LAMBO.hubs.RL.z < 0
    && LAMBO.hubs.FR.z > 0 && LAMBO.hubs.RR.z > 0,
    `FL.z ${LAMBO.hubs.FL.z.toFixed(3)}, FR.z ${LAMBO.hubs.FR.z.toFixed(3)}`);
  checkTrue('the staggered axles sit at different heights',
    Math.abs(LAMBO.hubs.FL.y - LAMBO.hubs.RL.y) > 0.005,
    `front ${LAMBO.hubs.FL.y.toFixed(4)}, rear ${LAMBO.hubs.RL.y.toFixed(4)}`);

  const wo = m.wheelObjects;
  checkTrue('the four wheel object names are exactly as specified',
    wo.FL === 'Wheel_001-material.001' && wo.FR === 'Wheel_001-material'
    && wo.RL === 'Wheel_001-material.003' && wo.RR === 'Wheel_001-material.002',
    `${wo.FL} / ${wo.FR} / ${wo.RL} / ${wo.RR}`);
  checkTrue('the steering wheel object is exactly as specified',
    m.steeringWheel.object === 'MAT_CarbonFiberMatte-material.006',
    m.steeringWheel.object);

  const ax = m.steeringWheel.axis;
  check('steering column rake', Math.atan2(ax.y, ax.x) * RAD2DEG, 18, 28, ' deg');
  check('steering column lateral tilt', Math.abs(ax.z), 0, 0.02);
}

section('4. Lamborghini at rest');
{
  const car = makeCar(LAMBO);
  run(car, 3.0, HANDBRAKE_ONLY);
  const loadSum = car.wheels.reduce((a, w) => a + w.normalForce, 0);
  const weight = LAMBO.chassis.mass * 9.80665;

  check('wheels in contact', car.contactCount, 4, 4);
  check('total normal force / weight', loadSum / weight, 0.97, 1.03);
  check('speed with the handbrake on', car.speed, 0, 0.05, ' m/s');
  check('front corner load', car.wheelFL.normalForce, 3800, 4400, ' N');
  check('rear corner load', car.wheelRL.normalForce, 4800, 5500, ' N');
  checkTrue('the rear axle carries more, as a mid-engined car should',
    car.wheelRL.normalForce > car.wheelFL.normalForce * 1.15,
    `front ${car.wheelFL.normalForce.toFixed(0)} N, rear ${car.wheelRL.normalForce.toFixed(0)} N`);
  check('body roll at rest', Math.abs(car.rollAngle * RAD2DEG), 0, 0.4, ' deg');
  check('idle rpm', car.rpm, 820, 1000, ' rpm');

  const hubAbove = car.wheelFL.hubWorld.y - car.wheelFL.contactPoint.y;
  check('FL hub height above contact', hubAbove,
    LAMBO.model.radiusFront - 0.05, LAMBO.model.radiusFront + 0.001, ' m');
  const hubAboveR = car.wheelRL.hubWorld.y - car.wheelRL.contactPoint.y;
  check('RL hub height above contact', hubAboveR,
    LAMBO.model.radiusRear - 0.05, LAMBO.model.radiusRear + 0.001, ' m');
  checkTrue('the two axles use genuinely different radii',
    Math.abs(hubAbove - hubAboveR) > 0.015,
    `front ${hubAbove.toFixed(3)} m vs rear ${hubAboveR.toFixed(3)} m`);
  check('tyre carcass deflection', car.wheelFL.tire.deflection * 1000, 8, 30, ' mm');
}

section('5. Lamborghini acceleration, braking and cornering');
const LP = profile(LAMBO);
{
  check('0-60 km/h', LP.t60 ?? 999, 1.6, 2.9, ' s');
  check('0-100 km/h', LP.t100 ?? 999, 2.6, 4.2, ' s');
  check('peak longitudinal G', LP.maxG, 1.05, 1.65, ' G');
  check('speed after 18 s', LP.v18, 250, 360, ' km/h');
  check('braking distance from 100 km/h', LP.brakeDist, 26, 38, ' m');
  check('peak braking G', LP.peakBrakeG, 1.15, 1.75, ' G');
  check('lateral G', LP.lateralG, 1.15, 1.70, ' G');
  check('body roll', LP.rollDeg, 1.2, 4.5, ' deg');
  check('roll gradient', LP.rollGradient, 0.8, 3.2, ' deg/G');
}

section('6. The two cars do NOT drive alike');
const BP = profile(BMW);
{
  console.log('                              BMW M3 GTR    Revuelto');
  const row = (label, a, b, unit = '', d = 2) => console.log(
    `       ${label.padEnd(24)}${a.toFixed(d).padStart(8)}${unit}`
    + `${b.toFixed(d).padStart(12)}${unit}`,
  );
  row('0-100 km/h', BP.t100 ?? 0, LP.t100 ?? 0, ' s', 2);
  row('peak long G', BP.maxG, LP.maxG, ' G', 2);
  row('speed @18 s', BP.v18, LP.v18, ' km/h', 0);
  row('braking 100-0', BP.brakeDist, LP.brakeDist, ' m', 1);
  row('lateral G', BP.lateralG, LP.lateralG, ' G', 2);
  row('body roll', BP.rollDeg, LP.rollDeg, ' deg', 2);
  row('roll gradient', BP.rollGradient, LP.rollGradient, ' deg/G', 2);

  checkTrue('the Lamborghini accelerates far harder',
    LP.t100 < BP.t100 * 0.65,
    `${LP.t100.toFixed(2)} s vs ${BP.t100.toFixed(2)} s`);
  checkTrue('it pulls more longitudinal G',
    LP.maxG > BP.maxG * 1.4, `${LP.maxG.toFixed(2)} vs ${BP.maxG.toFixed(2)}`);
  checkTrue('it stops shorter',
    LP.brakeDist < BP.brakeDist * 0.95,
    `${LP.brakeDist.toFixed(1)} m vs ${BP.brakeDist.toFixed(1)} m`);
  checkTrue('it corners harder',
    LP.lateralG > BP.lateralG * 1.08,
    `${LP.lateralG.toFixed(2)} G vs ${BP.lateralG.toFixed(2)} G`);
  checkTrue('it rolls markedly less for the grip it makes',
    LP.rollGradient < BP.rollGradient * 0.7,
    `${LP.rollGradient.toFixed(2)} vs ${BP.rollGradient.toFixed(2)} deg/G`);
  checkTrue('it reaches a much higher speed on the same run',
    LP.v18 > BP.v18 * 1.4, `${LP.v18.toFixed(0)} vs ${BP.v18.toFixed(0)} km/h`);
}

section('7. All-wheel drive and the electric front axle');
{
  const car = makeCar(LAMBO);
  checkTrue('drivetrain layout is AWD', LAMBO.drive === 'awd', LAMBO.drive);
  checkTrue('the engine drives the rear pair only',
    car.drivenWheels.length === 2 && car.drivenWheels.every((w) => !w.isFront));
  checkTrue('the electric axle drives the front pair only',
    car.electricWheels.length === 2 && car.electricWheels.every((w) => w.isFront));
  checkTrue('all four wheels are powered', car.poweredWheels.length === 4);

  // Torque from a standstill, which no geared drive can deliver.
  run(car, 0.10, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });
  check('front e-motor torque within 0.1 s of a standstill',
    car.wheelFL.electricTorque, 600, 1000, ' N.m');
  checkTrue('the front wheels are already pulling',
    car.wheelFL.longForce > 800, `${car.wheelFL.longForce.toFixed(0)} N`);

  // Constant-power fade at speed.
  const fast = makeCar(LAMBO);
  accelerateTo(fast, 75);
  run(fast, 0.4, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });
  checkTrue('e-motor torque fades at speed (constant power)',
    fast.wheelFL.electricTorque < 520,
    `${fast.wheelFL.electricTorque.toFixed(0)} N.m at `
    + `${fast.speedKmh.toFixed(0)} km/h`);

  // Regeneration on a closed throttle.
  const coast = makeCar(LAMBO);
  accelerateTo(coast, 25);
  run(coast, 0.5, NEUTRAL);
  checkTrue('regenerative braking on a closed throttle',
    coast.wheelFL.electricTorque < 0 && coast.electricAxle.regenerating,
    `${coast.wheelFL.electricTorque.toFixed(0)} N.m`);

  // The BMW must have none of this.
  const bmw = makeCar(BMW);
  run(bmw, 0.5, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });
  checkTrue('the BMW has no electric axle and no front drive',
    bmw.electricWheels.length === 0 && bmw.wheelFL.electricTorque === 0
    && bmw.poweredWheels.length === 2);
}

section('8. Dual clutch versus torque converter');
{
  checkTrue('the Lamborghini uses a dry dual clutch',
    LAMBO.transmission.coupling === 'clutch');
  checkTrue('the BMW still uses a torque converter',
    (BMW.transmission.coupling || 'converter') === 'converter');
  checkTrue('the DCT shifts much faster',
    LAMBO.transmission.shiftTime < BMW.transmission.shiftTime * 0.7,
    `${(LAMBO.transmission.shiftTime * 1000).toFixed(0)} ms vs `
    + `${(BMW.transmission.shiftTime * 1000).toFixed(0)} ms`);

  // A converter multiplies torque at stall. A clutch must not.
  const lambo = makeCar(LAMBO);
  run(lambo, 0.05, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });
  check('DCT torque multiplication at launch', lambo.drivetrain.torqueRatio, 1, 1);

  const bmw = makeCar(BMW);
  run(bmw, 0.05, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });
  checkTrue('the converter DOES multiply torque at launch',
    bmw.drivetrain.torqueRatio > 1.4,
    `x${bmw.drivetrain.torqueRatio.toFixed(2)}`);

  // The clutch must actually close and stay closed once rolling.
  const rolling = makeCar(LAMBO);
  accelerateTo(rolling, 40);
  run(rolling, 1.0, { throttle: 0.5, brake: 0, steer: 0, handbrake: 0 });
  checkTrue('the clutch is locked when cruising',
    rolling.drivetrain.locked && rolling.drivetrain.clutchEngagement > 0.98,
    `engagement ${rolling.drivetrain.clutchEngagement.toFixed(3)}`);
  check('gear while cruising', Number(rolling.gear), 2, 8);

  // The gearbox must climb through its gears rather than getting stuck.
  const sweep = makeCar(LAMBO);
  const gears = new Set();
  run(sweep, 16, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    gears.add(c.gear);
  });
  checkTrue('it works up through the gearbox',
    gears.size >= 5, `used gears: ${[...gears].join(', ')}`);
  checkTrue('it does not free-rev on the limiter in a low gear',
    Number(sweep.gear) >= 5, `finished in ${sweep.gear}`);
}

section('9. Lamborghini steering, reverse and handbrake');
{
  // Ackermann, on this car's own geometry.
  const car = makeCar(LAMBO);
  for (let i = 0; i < 400; i++) {
    car.steering.update(1 / 60, 1, 0, car.wheelFL, car.wheelFR);
  }
  const inner = Math.abs(car.wheelFR.steerAngle);
  const outer = Math.abs(car.wheelFL.steerAngle);
  check('inner wheel angle at full lock', inner * RAD2DEG, 26, 38, ' deg');
  checkTrue('the inner wheel still steers more than the outer',
    inner > outer + 0.005,
    `${(inner * RAD2DEG).toFixed(1)} vs ${(outer * RAD2DEG).toFixed(1)} deg`);
  checkTrue('less steering lock than the BMW, as a supercar has',
    LAMBO.steering.maxAngle < BMW.steering.maxAngle,
    `${(LAMBO.steering.maxAngle * RAD2DEG).toFixed(0)} vs `
    + `${(BMW.steering.maxAngle * RAD2DEG).toFixed(0)} deg`);
  checkTrue('a quicker rack than the BMW',
    LAMBO.steering.rackSpeed > BMW.steering.rackSpeed * 1.15);
  checkTrue('the rear wheels never steer',
    car.wheelRL.steerAngle === 0 && car.wheelRR.steerAngle === 0);

  // Reverse.
  const rev = makeCar(LAMBO);
  run(rev, 1.5, { throttle: 0, brake: 1, steer: 0, handbrake: 0 });
  checkTrue('reverse engages from a standstill', rev.gear === 'R', `gear ${rev.gear}`);
  run(rev, 3.0, { throttle: 0, brake: 1, steer: 0, handbrake: 0 });
  check('reverse speed', -rev.forwardSpeed * 3.6, 5, 90, ' km/h');
  checkTrue('all four wheels turn backwards',
    rev.wheels.every((w) => w.spinVelocity < 0));

  // Handbrake: rear axle only, and it must break the rear away.
  const hb = makeCar(LAMBO);
  accelerateTo(hb, 20);
  let rearStopped = false;
  let maxSideslip = 0;
  let rearLat = 1e9;
  let frontLat = 0;
  run(hb, 2.5, { throttle: 0.15, brake: 0, steer: 0.8, handbrake: 1 }, (t, c) => {
    if (Math.abs(c.wheelRL.spinVelocity) < 6 && c.speed > 4) rearStopped = true;
    maxSideslip = Math.max(maxSideslip, Math.abs(c.sideslip));
    if (t > 0.4 && c.speed > 4) {
      rearLat = Math.min(rearLat, Math.abs(c.wheelRL.latForce) + Math.abs(c.wheelRR.latForce));
      frontLat = Math.max(frontLat, Math.abs(c.wheelFL.latForce) + Math.abs(c.wheelFR.latForce));
    }
  });
  checkTrue('the handbrake stops the rear wheels while the car moves', rearStopped);
  check('sideslip reached', maxSideslip * RAD2DEG, 5, 120, ' deg');
  checkTrue('locking the rears destroys their lateral grip', rearLat < frontLat,
    `rear ${rearLat.toFixed(0)} N vs front ${frontLat.toFixed(0)} N`);
}

section('10. Lamborghini suspension and ground contact (circuit)');
{
  const car = makeCar(LAMBO, 'circuit', SIM.physicsHz,
    { distance: circuit.length * 0.78, lateral: 0 });
  run(car, 1.0, NEUTRAL);

  const _ss = new SurfaceSample();
  const laneKeep = (c) => {
    circuit.sample(c.body.position.x, c.body.position.z, _ss);
    const target = circuit.offsetPoint(_ss.distance + 16, 0);
    const dx = target.x - c.body.position.x;
    const dz = target.z - c.body.position.z;
    const f = c.body.forward;
    const r = c.body.right;
    return Math.max(-1, Math.min(1, Math.atan2(
      dx * r.x + dz * r.z, Math.max(1, dx * f.x + dz * f.z),
    ) * 1.5));
  };

  let lo = 1e9;
  let hi = -1e9;
  let maxVel = 0;
  let lostContact = 0;
  run(car, 8, (t, c) => ({
    throttle: c.speedKmh < 85 ? 0.4 : 0.08, brake: 0, steer: laneKeep(c), handbrake: 0,
  }), (t, c) => {
    const s = c.wheelFL.suspension;
    lo = Math.min(lo, s.length);
    hi = Math.max(hi, s.length);
    maxVel = Math.max(maxVel, Math.abs(s.velocity));
    if (c.contactCount < 4) lostContact++;
  });

  check('suspension travel used on the broken section', hi - lo, 0.008, 0.14, ' m');
  check('peak suspension velocity', maxVel, 0.05, 14, ' m/s');
  checkTrue('the wheels stay on the road', lostContact < 8 * SIM.physicsHz * 0.45,
    `${lostContact} samples airborne`);

  // A flat surface must produce far less movement - proof it comes from the road.
  const flat = makeCar(LAMBO);
  accelerateTo(flat, 24);
  let flo = 1e9;
  let fhi = -1e9;
  run(flat, 4, { throttle: 0.2, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    flo = Math.min(flo, c.wheelFL.suspension.length);
    fhi = Math.max(fhi, c.wheelFL.suspension.length);
  });
  checkTrue('a flat road produces almost no suspension movement',
    (fhi - flo) < (hi - lo) * 0.6,
    `flat ${((fhi - flo) * 1000).toFixed(1)} mm vs broken ${((hi - lo) * 1000).toFixed(1)} mm`);

  checkTrue('it is stiffer than the BMW',
    LAMBO.suspension.front.springRate > BMW.suspension.front.springRate * 1.6,
    `${LAMBO.suspension.front.springRate} vs ${BMW.suspension.front.springRate} N/m`);
  checkTrue('and has less travel',
    (LAMBO.suspension.front.maxCompression + LAMBO.suspension.front.maxDroop)
    < (BMW.suspension.front.maxCompression + BMW.suspension.front.maxDroop) * 0.8);
}

section('11. Lamborghini tyre pressure is not cosmetic');
{
  const results = {};
  for (const psi of [22, 34, 46]) {
    const car = makeCar(LAMBO);
    car.setTirePressures(psi);
    run(car, 2.0, HANDBRAKE_ONLY);
    const staticDeflection = car.wheelFL.tire.deflection;
    accelerateTo(car, 26);
    let maxLat = 0;
    run(car, 3.5, { throttle: 0.3, brake: 0, steer: 0.5, handbrake: 0 }, (t, c) => {
      if (t > 1.2) maxLat = Math.max(maxLat, Math.abs(c.gForce.z));
    });
    results[psi] = {
      lat: maxLat,
      slipAngle: car.wheelFL.tire.peakSlipAngle * RAD2DEG,
      deflection: staticDeflection,
      roll: car.wheelFL.tire.rollingResistance,
      radial: car.wheelFL.tire.radialStiffness,
    };
  }
  console.log('       psi   latG   peakSlipAngle  deflection  rollRes   radial');
  for (const psi of [22, 34, 46]) {
    const r = results[psi];
    console.log(`       ${String(psi).padStart(3)}  ${r.lat.toFixed(3)}      `
      + `${r.slipAngle.toFixed(2)} deg      ${(r.deflection * 1000).toFixed(1)} mm    `
      + `${r.roll.toFixed(4)}   ${(r.radial / 1000).toFixed(0)} kN/m`);
  }
  checkTrue('soft tyres need more slip angle',
    results[22].slipAngle > results[34].slipAngle * 1.1);
  checkTrue('soft tyres deform more under the same load',
    results[22].deflection > results[34].deflection * 1.15);
  checkTrue('carcass stiffness rises with pressure',
    results[46].radial > results[34].radial && results[34].radial > results[22].radial);
  checkTrue('nominal pressure gives the most grip',
    results[34].lat >= results[22].lat * 0.98 && results[34].lat >= results[46].lat * 0.98,
    `22 ${results[22].lat.toFixed(3)} | 34 ${results[34].lat.toFixed(3)} `
    + `| 46 ${results[46].lat.toFixed(3)}`);
  checkTrue('its low-profile tyre deflects less than the BMW road tyre',
    LAMBO.tyre.maxVisualDeflection < BMW.tyre.maxVisualDeflection);
}

section('12. Lamborghini wheel rotation matches the physics');
{
  const car = makeCar(LAMBO);
  accelerateTo(car, 25);
  run(car, 0.5, { throttle: 0.25, brake: 0, steer: 0, handbrake: 0 });

  const f = car.wheelFL;
  const r = car.wheelRL;
  check('front omega vs v/r', f.spinVelocity / (car.forwardSpeed / f.tire.effectiveRadius),
    0.95, 1.05);
  check('rear omega vs v/r', r.spinVelocity / (car.forwardSpeed / r.tire.effectiveRadius),
    0.95, 1.05);
  checkTrue('the smaller front wheels spin FASTER than the rears',
    f.spinVelocity > r.spinVelocity * 1.03,
    `front ${f.spinVelocity.toFixed(1)} vs rear ${r.spinVelocity.toFixed(1)} rad/s`);
  checkTrue('the spin angle is integrated, not derived from speed',
    Math.abs(f.spinAngle) > 10, `${f.spinAngle.toFixed(1)} rad`);

  const before = f.spinVelocity;
  run(car, 0.5, { throttle: 0, brake: 1, steer: 0, handbrake: 0 });
  checkTrue('braking slows the wheels', f.spinVelocity < before * 0.9,
    `${before.toFixed(1)} -> ${f.spinVelocity.toFixed(1)} rad/s`);

  const corner = makeCar(LAMBO);
  accelerateTo(corner, 18);
  run(corner, 3, { throttle: 0.25, brake: 0, steer: 0.5, handbrake: 0 });
  checkTrue('outer wheels turn faster than inner ones in a corner',
    corner.wheelFL.spinVelocity > corner.wheelFR.spinVelocity,
    `outer ${corner.wheelFL.spinVelocity.toFixed(1)} vs `
    + `inner ${corner.wheelFR.spinVelocity.toFixed(1)} rad/s`);
}

section('13. Lamborghini timestep independence');
{
  const results = {};
  for (const hz of [60, 120, 240, 480]) {
    const car = makeCar(LAMBO, 'skidpad', hz);
    let t100 = null;
    run(car, 10, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
      if (t100 === null && c.speedKmh >= 100) t100 = t;
    });
    const v = car.speedKmh;
    const stop = brakeToStop(car);
    results[hz] = {
      t100, v, brake: stop.distance,
      finite: car.body.position.isFinite() && car.body.velocity.isFinite(),
    };
  }
  console.log('        Hz    0-100(s)   v@10s(km/h)   braking(m)');
  for (const hz of [60, 120, 240, 480]) {
    const r = results[hz];
    console.log(`       ${String(hz).padStart(4)}    ${(r.t100 ?? 0).toFixed(3).padStart(7)}`
      + `     ${r.v.toFixed(2).padStart(8)}    ${r.brake.toFixed(2).padStart(8)}`);
  }
  const ref = results[240];
  for (const hz of [60, 120, 240, 480]) {
    checkTrue(`state stays finite at ${hz} Hz`, results[hz].finite);
  }
  check('60 Hz vs 240 Hz speed error',
    (100 * Math.abs(results[60].v - ref.v)) / ref.v, 0, 12, ' %');
  check('120 Hz vs 240 Hz speed error',
    (100 * Math.abs(results[120].v - ref.v)) / ref.v, 0, 5, ' %');
  check('60 Hz vs 240 Hz braking error',
    (100 * Math.abs(results[60].brake - ref.brake)) / ref.brake, 0, 14, ' %');
  /**
   * The launch is the loosest tolerance in the whole project, and it is the
   * Lamborghini's alone: a 1,000 PS all-wheel-drive standing start is the most
   * violent event either car can produce, with the dual clutch taking up, the
   * rear tyres breaking away and traction control catching them all inside
   * half a second. At 60 Hz that half-second is only 30 samples, so the
   * wheelspin peak resolves differently and costs about half a second to
   * 100 km/h. From 120 Hz up the runs agree to a third of a percent, and the
   * shipped rate is 240. Everything AFTER the launch tracks closely - the
   * 10-second speed is within 6 % and braking within 10 %.
   */
  check('60 Hz vs 240 Hz 0-100 error (launch-dominated)',
    (100 * Math.abs(results[60].t100 - ref.t100)) / ref.t100, 0, 18, ' %');
}

section('14. Long soak - the new drivetrain does not blow up');
{
  for (const spec of [LAMBO, BMW]) {
    const car = makeCar(spec, 'circuit');
    const _ss2 = new SurfaceSample();
    let worstUp = 1;
    run(car, 70, (t, c) => {
      circuit.sample(c.body.position.x, c.body.position.z, _ss2);
      const target = circuit.offsetPoint(_ss2.distance + 16, 0);
      const dx = target.x - c.body.position.x;
      const dz = target.z - c.body.position.z;
      const f = c.body.forward;
      const r = c.body.right;
      const steer = Math.max(-1, Math.min(1, Math.atan2(
        dx * r.x + dz * r.z, Math.max(1, dx * f.x + dz * f.z),
      ) * 1.5));
      return {
        throttle: 0.45 + 0.45 * Math.sin(t * 0.35),
        brake: Math.sin(t * 0.21) > 0.85 ? 0.7 : 0,
        steer,
        handbrake: Math.sin(t * 0.13) > 0.97 ? 1 : 0,
      };
    }, (t, c) => { worstUp = Math.min(worstUp, c.body.up.y); });

    checkTrue(`${spec.shortName}: 70 s of hard driving stays finite`,
      car.body.position.isFinite() && car.body.velocity.isFinite()
      && car.body.angularVelocity.isFinite()
      && car.wheels.every((w) => Number.isFinite(w.spinVelocity)
        && Number.isFinite(w.normalForce) && Number.isFinite(w.tire.slipRatio)));
    checkTrue(`${spec.shortName}: stayed the right way up`, worstUp > 0.2,
      `worst up.y ${worstUp.toFixed(3)}`);
    check(`${spec.shortName}: tyre temperature`,
      car.wheelRL.tire.temperature, 20, 195, ' C');
  }
}

section('15. Network state codec');
{
  const source = makeCar(LAMBO);
  accelerateTo(source, 30);
  run(source, 1.0, { throttle: 0.6, brake: 0, steer: 0.4, handbrake: 0 });

  // A stand-in for a Vehicle, exposing just what the codec touches.
  const fake = { id: LAMBO.id, physics: source };
  const s1 = captureVehicleState(fake, 'peer-a', 1.0);

  checkTrue('the snapshot carries the vehicle id, not the name',
    s1.v === 'lamborghini-revuelto', s1.v);
  checkTrue('it carries position, rotation and both velocities',
    s1.p.length === 3 && s1.q.length === 4 && s1.lv.length === 3 && s1.av.length === 3);
  checkTrue('it carries steering, all four wheel angles and suspension',
    typeof s1.st === 'number' && s1.wa.length === 4
    && s1.sp.length === 4 && s1.td.length === 4);
  checkTrue('it carries braking and drivetrain state',
    typeof s1.br === 'number' && typeof s1.hb === 'number' && typeof s1.g === 'string');

  const bytes = JSON.stringify(s1).length;
  check('snapshot size', bytes, 200, 900, ' bytes');
  check('bandwidth at 20 Hz', (bytes * 20) / 1024, 4, 18, ' KB/s');

  // Apply it to a second, un-simulated car and check it lands.
  const target = makeCar(LAMBO);
  const targetFake = { id: LAMBO.id, physics: target };
  applyVehicleState(targetFake, s1, s1, 0);

  check('position replicated', target.body.position.distanceTo(source.body.position),
    0, 0.01, ' m');
  check('steering replicated', Math.abs(target.steering.angle - source.steering.angle),
    0, 0.001, ' rad');
  check('wheel angle replicated',
    Math.abs(target.wheelFL.spinAngle - source.wheelFL.spinAngle), 0, 0.02, ' rad');
  check('suspension replicated',
    Math.abs(target.wheelFL.suspension.displacement
      - source.wheelFL.suspension.displacement), 0, 0.001, ' m');
  checkTrue('front wheel steer angles are rebuilt from the rack angle',
    Math.abs(target.wheelFL.steerAngle - source.wheelFL.steerAngle) < 0.001
    && Math.abs(target.wheelFR.steerAngle - source.wheelFR.steerAngle) < 0.001,
    `FL ${target.wheelFL.steerAngle.toFixed(4)} vs ${source.wheelFL.steerAngle.toFixed(4)}`);
  checkTrue('the rear wheels stay unsteered after replication',
    target.wheelRL.steerAngle === 0 && target.wheelRR.steerAngle === 0);

  // Interpolation between two snapshots.
  run(source, 0.05, { throttle: 0.6, brake: 0, steer: 0.4, handbrake: 0 });
  const s2 = captureVehicleState(fake, 'peer-a', 1.05);
  applyVehicleState(targetFake, s1, s2, 0.5);
  const midX = (s1.p[0] + s2.p[0]) / 2;
  check('interpolates between snapshots', Math.abs(target.body.position.x - midX),
    0, 0.005, ' m');
}

section('16. Garage figures come from the physics, not from marketing');
{
  for (const spec of allVehicleSpecs()) {
    const st = vehicleStats(spec);
    let peakTorque = 0;
    for (const [, nm] of spec.engine.torqueCurve) peakTorque = Math.max(peakTorque, nm);
    checkTrue(`${spec.shortName}: quoted torque matches the engine curve`,
      st.peakTorque === Math.round(peakTorque),
      `${st.peakTorque} Nm`);
    checkTrue(`${spec.shortName}: quoted grip matches the tyre model`,
      st.mu === spec.tyre.muNominal, `mu ${st.mu}`);
    checkTrue(`${spec.shortName}: quoted mass matches the chassis`,
      st.mass === spec.chassis.mass, `${st.mass} kg`);
    checkTrue(`${spec.shortName}: power/weight is derived, not typed`,
      st.powerToWeight === Math.round((st.systemPs) / (spec.chassis.mass / 1000)));
  }

  const lamboStats = vehicleStats(LAMBO);
  const bmwStats = vehicleStats(BMW);
  checkTrue('the hybrid quotes more system power than engine power',
    lamboStats.systemPs > lamboStats.enginePs,
    `${lamboStats.systemPs} PS system vs ${lamboStats.enginePs} PS engine`);
  checkTrue('the BMW quotes engine power only (it has no e-axle)',
    bmwStats.systemPs === bmwStats.enginePs);
  check('Revuelto system output', lamboStats.systemPs, 950, 1250, ' PS');
  check('Revuelto power to weight', lamboStats.powerToWeight, 450, 700, ' PS/t');

  // The garage's quoted acceleration must be close to what the car really does.
  checkTrue('quoted 0-100 is within 0.6 s of the measured figure',
    Math.abs(lamboStats.zeroToHundred - LP.t100) < 0.6,
    `quoted ${lamboStats.zeroToHundred.toFixed(1)} s, measured ${LP.t100.toFixed(2)} s`);
  checkTrue('quoted braking is within 3 m of the measured figure',
    Math.abs(lamboStats.brakingFrom100 - LP.brakeDist) < 3,
    `quoted ${lamboStats.brakingFrom100.toFixed(1)} m, measured ${LP.brakeDist.toFixed(1)} m`);
}

section('17. The mirrored rear-right wheel');
{
  /**
   * `Wheel_001-material.002` hangs off `_3DWheel_back_R`, whose scale in the
   * .glb is (-0.001, -0.001, -0.001) - a point REFLECTION. A matrix with a
   * negative determinant cannot be described by a quaternion, so
   * `getWorldQuaternion` hands back an orientation that is not the node's, and
   * the tyre-deformation shader was being told the road lay in the wrong
   * direction. On that corner alone the flat spot rode round with the wheel
   * instead of staying at the bottom.
   *
   * This rebuilds both rear wheels exactly as `_buildWheelRigs` does and
   * checks the contact direction survives a round trip at every spin angle.
   */
  const THREE = await import('three');

  const rigWheel = (parentQuat, parentScale) => {
    const root = new THREE.Object3D();
    root.quaternion.set(-0.70710688829422, 0, 0, 0.7071066498756409);
    const node = new THREE.Object3D();
    node.quaternion.fromArray(parentQuat);
    node.scale.fromArray(parentScale);
    root.add(node);
    const tyre = new THREE.Object3D();
    node.add(tyre);
    root.updateMatrixWorld(true);

    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    tyre.matrixWorld.decompose(pos, quat, scale);

    const spin = new THREE.Object3D();
    const rigged = new THREE.Object3D();
    spin.add(rigged);
    rigged.quaternion.copy(quat);
    rigged.scale.copy(scale);
    return { spin, rigged };
  };

  const WHEELS = {
    'material.003 (rear left)': rigWheel(
      [0.70710688829422, 0, 0, 0.7071066498756409],
      [0.0010000000474974513, 0.0009999999310821295, 0.0009999999310821295],
    ),
    'material.002 (rear right)': rigWheel(
      [0, -0.70710688829422, 0.7071066498756409, 2.2476546845950907e-8],
      [-0.0009999999310821295, -0.0009999999310821295, -0.0009999999310821295],
    ),
  };

  const down = new THREE.Vector3(0, -1, 0);
  const linear = new THREE.Matrix3();
  const out = new THREE.Vector3();

  /**
   * The REAL method off the real class, called on a minimal stand-in for a
   * loaded car - so this test fails if `_worldDirToLocal` is ever changed back
   * to a quaternion, not merely if the arithmetic in this file is.
   */
  const { Vehicle } = await import('../js/vehicle.js');
  const stub = { _linearInv: new THREE.Matrix3() };
  const toLocal = (obj, dir) => Vehicle.prototype._worldDirToLocal.call(stub, obj, dir, out);

  for (const [label, w] of Object.entries(WHEELS)) {
    let worst = 0;
    for (let i = 0; i < 24; i++) {
      w.spin.rotation.z = (i / 24) * Math.PI * 2;
      w.spin.updateMatrixWorld(true);
      // What the shader is handed: the road direction in the tyre's own frame.
      const local = toLocal(w.rigged, down).clone().normalize();
      // Where that actually points once the tyre's own transform is applied.
      const back = local.applyMatrix3(linear.setFromMatrix4(w.rigged.matrixWorld))
        .normalize();
      worst = Math.max(worst, back.angleTo(down) * RAD2DEG);
    }
    checkTrue(`the flat spot stays down all the way round - ${label}`,
      worst < 0.01, `worst error ${worst.toFixed(3)} deg over a revolution`);
  }

  /**
   * And the same measurement through the quaternion the code used to use, so
   * the test states what the bug WAS rather than only that it is gone.
   */
  {
    const q = new THREE.Quaternion();
    const w = WHEELS['material.002 (rear right)'];
    let worst = 0;
    for (let i = 0; i < 24; i++) {
      w.spin.rotation.z = (i / 24) * Math.PI * 2;
      w.spin.updateMatrixWorld(true);
      w.rigged.getWorldQuaternion(q);
      const back = down.clone().applyQuaternion(q.clone().invert())
        .applyMatrix3(linear.setFromMatrix4(w.rigged.matrixWorld)).normalize();
      worst = Math.max(worst, back.angleTo(down) * RAD2DEG);
    }
    checkTrue('a quaternion cannot describe that wheel (this was the bug)',
      worst > 90, `it would be out by up to ${worst.toFixed(0)} deg`);
  }

  // And the reason a quaternion cannot serve, stated as a fact about the file.
  const mirrored = new THREE.Object3D();
  mirrored.scale.set(-0.001, -0.001, -0.001);
  mirrored.updateMatrix();
  checkTrue('the rear-right wheel node really is mirrored',
    mirrored.matrix.determinant() < 0,
    `determinant ${mirrored.matrix.determinant().toExponential(2)}`);
}

/* ------------------------------------------------------------------------ */

console.log(`\n${'='.repeat(72)}`);
console.log(`  ${passed} passed, ${failed} failed`);
if (failed) {
  console.log('\n  Failures:');
  for (const f of failures) console.log(`   - ${f}`);
}
console.log(`${'='.repeat(72)}\n`);
process.exit(failed ? 1 : 0);
