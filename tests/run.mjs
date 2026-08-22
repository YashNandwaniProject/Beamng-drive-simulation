/**
 * tests/run.mjs
 * ---------------------------------------------------------------------------
 * Head-less regression harness for the physics core.
 *
 * The physics modules carry no three.js dependency precisely so they can be
 * driven from Node like this. Every scenario in the brief's closing checklist
 * is exercised and asserted against a physically sensible range - it is not
 * enough that the code runs, the numbers have to be believable for a 1720 kg
 * rear-drive sedan with 500 N.m.
 *
 * Two fixtures are used:
 *
 *   SKIDPAD  a perfectly flat infinite asphalt plane. Vehicle dynamics
 *            (acceleration, braking, roll, weight transfer, tyre pressure)
 *            are measured here so the numbers are reproducible and are not
 *            contaminated by track elevation, camber or run-off.
 *
 *   CIRCUIT  the real Terrain from terrain.js. Used for everything that is
 *            about the world: banking, kerbs, surface changes, collision.
 *
 *   node tests/run.mjs
 * ---------------------------------------------------------------------------
 */

import { Terrain, SurfaceSample } from '../js/terrain.js';
import { VehiclePhysics } from '../js/vehiclePhysics.js';
import { CollisionSystem } from '../js/collision.js';
import { Vec3 } from '../js/mathUtils.js';
import { SIM, SPAWN, SURFACES } from '../js/config.js';
import { getVehicleSpec } from '../js/vehicles/registry.js';

/** The car this suite pins. Its numbers must not move. */
const BMW = getVehicleSpec('bmw-m3-gtr');
const TYRE = BMW.tyre;

const RAD2DEG = 180 / Math.PI;

let passed = 0;
let failed = 0;
const failures = [];

function check(name, value, min, max, unit = '') {
  const ok = Number.isFinite(value) && value >= min && value <= max;
  const v = typeof value === 'number' ? value.toFixed(3) : String(value);
  console.log(`   ${ok ? 'PASS' : 'FAIL'}  ${name.padEnd(44)} ${v.padStart(10)}${unit}`
    + `${ok ? '' : `   expected ${min}..${max}`}`);
  if (ok) passed++; else { failed++; failures.push(name); }
  return ok;
}

function checkTrue(name, cond, detail = '') {
  console.log(`   ${cond ? 'PASS' : 'FAIL'}  ${name.padEnd(44)} ${detail}`);
  if (cond) passed++; else { failed++; failures.push(name); }
  return cond;
}

function section(title) {
  console.log(`\n=== ${title} ${'='.repeat(Math.max(0, 64 - title.length))}`);
}

/* ------------------------------------------------------------------------ */
/* Fixtures                                                                  */
/* ------------------------------------------------------------------------ */

const circuit = new Terrain();

/**
 * A flat, infinite asphalt plane implementing the same surface interface the
 * wheels query. Isolates vehicle dynamics from track geometry.
 */
class Skidpad {
  constructor(surface = SURFACES.ASPHALT) { this.surface = surface; this.length = 1e6; }
  sample(x, z, out) {
    out.height = 0;
    out.normal.set(0, 1, 0);
    out.surface = this.surface;
    out.lateral = 0;
    out.distance = 0;
    out.onRoad = true;
    out.index = 0;
    return out;
  }
  quickHeight() { return 0; }
}

const skidpad = new Skidpad();

const NEUTRAL = { throttle: 0, brake: 0, steer: 0, handbrake: 0 };

/** Build a vehicle. `where` is 'skidpad' or 'circuit'. */
function makeCar(where = 'skidpad', hz = SIM.physicsHz, opts = {}) {
  const terrain = where === 'circuit' ? circuit : skidpad;
  const v = new VehiclePhysics(terrain, opts.spec || BMW);
  if (opts.collision) v.collisionSystem = new CollisionSystem(terrain);
  if (where === 'circuit') {
    const s = opts.distance ?? SPAWN.trackDistance;
    const sp = circuit.spawnTransform(s, opts.lateral ?? SPAWN.lateralOffset);
    v.reset(sp.position, sp.heading);
  } else {
    v.reset(new Vec3(0, 0, 0), 0);       // facing +X on a flat plane
  }
  v.__dt = 1 / hz;
  return v;
}

/** Run `seconds` of simulation with a (possibly time-varying) input. */
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

/**
 * Pure-pursuit lane keeping for the circuit runs, so a long test does not end
 * up in the grass with the wheel held straight. It doubles as a check that the
 * steering system can hold a line at all.
 */
const _ss = new SurfaceSample();
function laneKeep(car, lookahead = 16, gain = 1.5) {
  circuit.sample(car.body.position.x, car.body.position.z, _ss);
  const target = circuit.offsetPoint(_ss.distance + lookahead, 0);
  const dx = target.x - car.body.position.x;
  const dz = target.z - car.body.position.z;
  const f = car.body.forward;
  const r = car.body.right;
  const ahead = dx * f.x + dz * f.z;
  const side = dx * r.x + dz * r.z;
  return Math.max(-1, Math.min(1, Math.atan2(side, Math.max(1, ahead)) * gain));
}

/**
 * Brake to a standstill and report the distance.
 *
 * It stops the moment the car does: holding the brake at a standstill is how
 * the driver SELECTS REVERSE (requirement #6), so a fixed-duration braking run
 * would drive back over its own stopping distance and report nonsense.
 */
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

/** Hold the car still without touching the brake pedal (see brakeToStop). */
const HANDBRAKE_ONLY = { throttle: 0, brake: 0, steer: 0, handbrake: 1 };

/** Bring a skidpad car up to `speed` under its own power, then settle. */
function accelerateTo(car, speed, maxSeconds = 25) {
  run(car, maxSeconds, (t, c) => ({
    throttle: c.forwardSpeed < speed ? 1 : 0,
    brake: 0,
    steer: 0,
    handbrake: 0,
  }));
  return car;
}

/* ========================================================================= */

section('1. Starting from rest - the car settles on its suspension');
{
  const car = makeCar('skidpad');
  run(car, 3.0, HANDBRAKE_ONLY);

  const contacts = car.wheels.filter((w) => w.hasContact).length;
  const loadSum = car.wheels.reduce((a, w) => a + w.normalForce, 0);
  const expectedWeight = 1720 * 9.80665;

  check('wheels in contact', contacts, 4, 4);
  check('total normal force / weight', loadSum / expectedWeight, 0.97, 1.03);
  check('speed with the handbrake on', car.speed, 0, 0.05, ' m/s');
  check('FL strut length', car.wheelFL.suspension.length, 0.02, 0.16, ' m');
  check('front static load', car.wheelFL.normalForce, 3800, 5000, ' N');
  check('rear static load', car.wheelRL.normalForce, 3400, 4700, ' N');
  check('front/rear load split',
    (car.wheelFL.normalForce + car.wheelFR.normalForce) / loadSum, 0.49, 0.55);
  check('body roll at rest', Math.abs(car.rollAngle * RAD2DEG), 0, 0.4, ' deg');
  check('body pitch at rest', Math.abs(car.pitchAngle * RAD2DEG), 0, 0.5, ' deg');
  check('vertical G at rest', car.gForce.y, 0.97, 1.03, ' G');
  check('idle rpm', car.rpm, 700, 900, ' rpm');

  // The tyre must sit ON the road: hub height above the contact point equals
  // the tyre radius less its carcass squash. No floating, no sinking.
  const hubAboveContact = car.wheelFL.hubWorld.y - car.wheelFL.contactPoint.y;
  check('FL hub height above contact', hubAboveContact,
    TYRE.radius - 0.05, TYRE.radius + 0.001, ' m');
  check('FL tyre carcass deflection', car.wheelFL.tire.deflection * 1000, 8, 40, ' mm');

  // With the brake released an automatic must creep, like the real car.
  const creeper = makeCar('skidpad');
  run(creeper, 6, NEUTRAL);
  check('idle creep speed (automatic)', creeper.speed * 3.6, 2, 12, ' km/h');
}

section('2. Full-throttle acceleration (traction control ON)');
{
  const car = makeCar('skidpad');
  let t100 = null, t60 = null, dist100 = 0;
  const start = car.body.position.clone();
  let maxG = 0, sawWheelspin = false, shifts = 0, lastGear = car.gear;

  run(car, 20, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    maxG = Math.max(maxG, c.gForce.x);
    if (c.anyWheelSpin) sawWheelspin = true;
    if (c.gear !== lastGear) { shifts++; lastGear = c.gear; }
    if (t60 === null && c.speedKmh >= 60) t60 = t;
    if (t100 === null && c.speedKmh >= 100) {
      t100 = t;
      dist100 = c.body.position.distanceTo(start);
    }
  });

  check('0-60 km/h', t60 ?? 999, 2.0, 4.0, ' s');
  check('0-100 km/h', t100 ?? 999, 4.0, 7.0, ' s');
  check('distance to 100 km/h', dist100, 45, 120, ' m');
  check('peak longitudinal G', maxG, 0.55, 1.15, ' G');
  check('speed after 20 s', car.speedKmh, 180, 300, ' km/h');
  check('gear after 20 s', Number(car.gear), 4, 8);
  check('number of upshifts', shifts, 3, 8);
  checkTrue('acceleration is progressive, not constant',
    maxG > 0.55 && car.gForce.x < maxG * 0.6,
    `peak ${maxG.toFixed(2)} G, at 20 s ${car.gForce.x.toFixed(2)} G`);

  const rearLoad = car.wheelRL.normalForce + car.wheelRR.normalForce;
  const frontLoad = car.wheelFL.normalForce + car.wheelFR.normalForce;
  checkTrue('rear axle carries load under power', rearLoad > frontLoad * 0.9,
    `front ${frontLoad.toFixed(0)} N, rear ${rearLoad.toFixed(0)} N`);

  // Squat: the driven axle must compress relative to standing still.
  const launch = makeCar('skidpad');
  run(launch, 0.05, NEUTRAL);
  const restRear = launch.wheelRL.suspension.displacement;
  let squat = -1;
  run(launch, 1.2, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    squat = Math.max(squat, c.wheelRL.suspension.displacement - restRear);
  });
  check('rear squat under launch', squat * 1000, 1, 40, ' mm');
  checkTrue('front rises as the rear squats',
    launch.wheelFL.suspension.displacement < launch.wheelRL.suspension.displacement,
    `front ${(launch.wheelFL.suspension.displacement * 1000).toFixed(1)} mm, `
    + `rear ${(launch.wheelRL.suspension.displacement * 1000).toFixed(1)} mm`);
}

section('3. Wheelspin off the line (traction control OFF)');
{
  const car = makeCar('skidpad');
  car.brakes.tcEnabled = false;
  let maxSlip = 0, maxSpinDiff = 0, maxTemp = 0;
  run(car, 2.5, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    maxSlip = Math.max(maxSlip, c.wheelRL.tire.slipRatio);
    maxSpinDiff = Math.max(maxSpinDiff, c.wheelRL.spinVelocity - c.wheelFL.spinVelocity);
    maxTemp = Math.max(maxTemp, c.wheelRL.tire.temperature);
  });
  check('peak rear slip ratio', maxSlip, 0.20, 8.0);
  check('rear-vs-front spin difference', maxSpinDiff, 3, 300, ' rad/s');
  checkTrue('undriven front wheels do not spin up',
    car.wheelFL.tire.slipRatio < 0.12, `front slip ${car.wheelFL.tire.slipRatio.toFixed(3)}`);
  checkTrue('wheelspin heats the rear tyres',
    maxTemp > TYRE.startTemp + 1, `${maxTemp.toFixed(1)} C`);

  // With TC on, the same launch must keep slip near the tyre's peak.
  const tc = makeCar('skidpad');
  let tcMaxSlip = 0;
  run(tc, 2.5, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    if (t > 0.6) tcMaxSlip = Math.max(tcMaxSlip, c.wheelRL.tire.slipRatio);
  });
  checkTrue('traction control reins the slip in',
    tcMaxSlip < maxSlip * 0.8,
    `TC off ${maxSlip.toFixed(2)} vs TC on ${tcMaxSlip.toFixed(2)}`);
}

section('4. Hard braking from 100 km/h (ABS ON)');
{
  const car = makeCar('skidpad');
  accelerateTo(car, 27.8);
  run(car, 0.4, NEUTRAL);

  const v0 = car.speed;
  let minG = 0, maxFrontLoad = 0, minRearLoad = 1e9, maxDive = 0, absSeen = false;
  let maxFrontComp = 0, minRearComp = 1e9;

  const stop = brakeToStop(car, (t, c) => {
    minG = Math.min(minG, c.gForce.x);
    maxFrontLoad = Math.max(maxFrontLoad, c.wheelFL.normalForce + c.wheelFR.normalForce);
    minRearLoad = Math.min(minRearLoad, c.wheelRL.normalForce + c.wheelRR.normalForce);
    maxDive = Math.min(maxDive, c.pitchAngle);
    maxFrontComp = Math.max(maxFrontComp, c.wheelFL.suspension.displacement);
    minRearComp = Math.min(minRearComp, c.wheelRL.suspension.displacement);
    if (c.brakes.anyAbsActive) absSeen = true;
  });

  const dist = stop.distance;
  const avgDecel = (v0 * v0) / (2 * Math.max(dist, 0.01));

  check('entry speed', v0 * 3.6, 95, 115, ' km/h');
  check('stopping distance from 100 km/h', dist, 30, 55, ' m');
  check('peak braking G', -minG, 0.85, 1.35, ' G');
  check('mean deceleration', avgDecel, 8.0, 13.0, ' m/s^2');
  checkTrue('ABS modulated at least one wheel', absSeen);
  checkTrue('front axle gains load under braking', maxFrontLoad > 11000,
    `front peak ${maxFrontLoad.toFixed(0)} N`);
  checkTrue('rear axle sheds load under braking', minRearLoad < 6000,
    `rear min ${minRearLoad.toFixed(0)} N`);
  check('nose-down pitch under braking', -maxDive * RAD2DEG, 0.5, 4.5, ' deg');
  checkTrue('front compresses AND rear extends',
    maxFrontComp > 0.004 && minRearComp < -0.004,
    `front +${(maxFrontComp * 1000).toFixed(1)} mm, rear ${(minRearComp * 1000).toFixed(1)} mm`);
  checkTrue('the car actually stops', car.speed <= 0.3 && stop.time < 11,
    `t = ${stop.time.toFixed(2)} s`);
}

section('5. Wheel lock with ABS OFF');
{
  const car = makeCar('skidpad');
  car.brakes.absEnabled = false;
  accelerateTo(car, 27.8);
  run(car, 0.4, NEUTRAL);
  let lockedSeen = false, minSlip = 0, lockedWhileRolling = false;
  const lockedStop = brakeToStop(car, (t, c) => {
    if (c.anyWheelLocked) lockedSeen = true;
    minSlip = Math.min(minSlip, c.wheelFL.tire.slipRatio);
    if (Math.abs(c.wheelFL.spinVelocity) < 2 && c.speed > 3) lockedWhileRolling = true;
  });
  checkTrue('a wheel locks with ABS disabled', lockedSeen);
  check('locked-wheel slip ratio', minSlip, -3.0, -0.35);
  checkTrue('the locked wheel stops turning while the car keeps moving',
    lockedWhileRolling);

  // Sliding tyres are past the peak of the curve, so ABS must stop shorter.
  const absCar = makeCar('skidpad');
  accelerateTo(absCar, 27.8);
  run(absCar, 0.4, NEUTRAL);
  const absStop = brakeToStop(absCar);
  checkTrue('ABS stops shorter than locked wheels',
    absStop.distance < lockedStop.distance,
    `ABS ${absStop.distance.toFixed(1)} m vs locked ${lockedStop.distance.toFixed(1)} m`);
}

section('6. Steering at low speed vs high speed');
{
  const slow = makeCar('skidpad');
  accelerateTo(slow, 8);
  let yawSlow = 0, latSlow = 0;
  run(slow, 4, { throttle: 0.12, brake: 0, steer: 1, handbrake: 0 }, (t, c) => {
    if (t > 1.5) {
      yawSlow = Math.max(yawSlow, Math.abs(c.yawRate));
      latSlow = Math.max(latSlow, Math.abs(c.gForce.z));
    }
  });

  const fast = makeCar('skidpad');
  accelerateTo(fast, 38);
  let yawFast = 0, latFast = 0;
  run(fast, 3.5, { throttle: 0.30, brake: 0, steer: 1, handbrake: 0 }, (t, c) => {
    if (t > 1.0) {
      yawFast = Math.max(yawFast, Math.abs(c.yawRate));
      latFast = Math.max(latFast, Math.abs(c.gForce.z));
    }
  });

  check('low-speed test speed', slow.speed * 3.6, 5, 45, ' km/h');
  check('low-speed yaw rate at full lock', yawSlow * RAD2DEG, 15, 90, ' deg/s');
  check('low-speed lateral G', latSlow, 0.1, 1.2, ' G');
  check('high-speed lateral G', latFast, 0.75, 1.35, ' G');
  checkTrue('steering lock is reduced at speed',
    fast.steering.availableLock < slow.steering.availableLock * 0.8,
    `${(slow.steering.availableLock * RAD2DEG).toFixed(1)} deg -> `
    + `${(fast.steering.availableLock * RAD2DEG).toFixed(1)} deg`);
  checkTrue('the car does not spin at speed',
    Math.abs(fast.sideslip * RAD2DEG) < 50,
    `sideslip ${(fast.sideslip * RAD2DEG).toFixed(1)} deg`);

  // Steering must be smoothed, never a step input.
  const smooth = makeCar('skidpad');
  accelerateTo(smooth, 20);
  let maxRate = 0, prev = smooth.steering.angle;
  run(smooth, 1.0, { throttle: 0.2, brake: 0, steer: 1, handbrake: 0 }, (t, c) => {
    maxRate = Math.max(maxRate, Math.abs(c.steering.angle - prev) / c.__dt);
    prev = c.steering.angle;
  });
  check('peak steering rate', maxRate * RAD2DEG, 5, 200, ' deg/s');
}

section('7. Ackermann geometry');
{
  const car = makeCar('skidpad');
  for (let i = 0; i < 400; i++) car.steering.update(1 / 60, 1, 0, car.wheelFL, car.wheelFR);
  const inner = Math.abs(car.wheelFR.steerAngle);
  const outer = Math.abs(car.wheelFL.steerAngle);
  check('inner (right) wheel angle', inner * RAD2DEG, 28, 42, ' deg');
  check('outer (left) wheel angle', outer * RAD2DEG, 22, 36, ' deg');
  checkTrue('the inner wheel steers more than the outer', inner > outer + 0.01,
    `${(inner * RAD2DEG).toFixed(1)} deg vs ${(outer * RAD2DEG).toFixed(1)} deg`);

  for (let i = 0; i < 400; i++) car.steering.update(1 / 60, -1, 0, car.wheelFL, car.wheelFR);
  const innerL = Math.abs(car.wheelFL.steerAngle);
  const outerL = Math.abs(car.wheelFR.steerAngle);
  checkTrue('a left turn mirrors a right turn',
    Math.abs(innerL - inner) < 0.01 && Math.abs(outerL - outer) < 0.01,
    `inner ${(innerL * RAD2DEG).toFixed(1)} outer ${(outerL * RAD2DEG).toFixed(1)}`);
  checkTrue('the rear wheels never steer',
    car.wheelRL.steerAngle === 0 && car.wheelRR.steerAngle === 0);
}

section('8. Cornering: body roll and lateral weight transfer');
{
  const car = makeCar('skidpad');
  accelerateTo(car, 26);
  let maxRoll = 0, maxLat = 0, outerLoad = 0, innerLoad = 1e9;
  let rollSign = 0, steerSign = 0;
  run(car, 4.0, { throttle: 0.30, brake: 0, steer: 0.45, handbrake: 0 }, (t, c) => {
    if (t > 1.2) {
      maxRoll = Math.max(maxRoll, Math.abs(c.rollAngle));
      maxLat = Math.max(maxLat, Math.abs(c.gForce.z));
      outerLoad = Math.max(outerLoad, c.wheelFL.normalForce);
      innerLoad = Math.min(innerLoad, c.wheelFR.normalForce);
      rollSign = Math.sign(c.rollAngle);
      steerSign = Math.sign(c.steering.angle);
    }
  });
  check('lateral G in the corner', maxLat, 0.4, 1.30, ' G');
  check('body roll angle', maxRoll * RAD2DEG, 0.8, 7.0, ' deg');
  check('roll gradient', (maxRoll * RAD2DEG) / Math.max(maxLat, 0.05), 1.0, 8.0, ' deg/G');
  checkTrue('outside wheels take load, inside wheels unload',
    outerLoad > innerLoad * 1.25,
    `outer ${outerLoad.toFixed(0)} N vs inner ${innerLoad.toFixed(0)} N`);
  checkTrue('the car leans OUT of the turn', rollSign * steerSign < 0,
    `roll ${(car.rollAngle * RAD2DEG).toFixed(2)} deg, `
    + `steer ${(car.steering.angle * RAD2DEG).toFixed(1)} deg`);
  checkTrue('total load is still the car weight',
    Math.abs(car.wheels.reduce((a, w) => a + w.normalForce, 0) / (1720 * 9.80665) - 1) < 0.25,
    `${(car.wheels.reduce((a, w) => a + w.normalForce, 0) / 1000).toFixed(1)} kN`);
}

section('9. Handbrake turn');
{
  const car = makeCar('skidpad');
  accelerateTo(car, 20);
  const yaw0 = car.heading;
  let maxSideslip = 0, rearSlip = 0, rearStoppedWhileMoving = false;
  let rearLat = 1e9, frontLat = 0;
  run(car, 2.5, { throttle: 0.15, brake: 0, steer: 0.8, handbrake: 1 }, (t, c) => {
    maxSideslip = Math.max(maxSideslip, Math.abs(c.sideslip));
    rearSlip = Math.max(rearSlip, Math.abs(c.wheelRL.tire.slipRatio));
    if (Math.abs(c.wheelRL.spinVelocity) < 6 && c.speed > 4) rearStoppedWhileMoving = true;
    if (t > 0.4 && c.speed > 4) {
      rearLat = Math.min(rearLat, Math.abs(c.wheelRL.latForce) + Math.abs(c.wheelRR.latForce));
      frontLat = Math.max(frontLat, Math.abs(c.wheelFL.latForce) + Math.abs(c.wheelFR.latForce));
    }
  });
  const yawDelta = Math.abs(((car.heading - yaw0 + Math.PI * 3) % (Math.PI * 2)) - Math.PI);
  check('rear slip ratio under handbrake', rearSlip, 0.5, 3.0);
  check('sideslip angle reached', maxSideslip * RAD2DEG, 8, 120, ' deg');
  checkTrue('the rear wheels stop while the car keeps moving', rearStoppedWhileMoving);
  checkTrue('locking the rears destroys their lateral grip', rearLat < frontLat,
    `rear ${rearLat.toFixed(0)} N vs front ${frontLat.toFixed(0)} N`);
  checkTrue('the car rotates rather than simply stopping',
    yawDelta * RAD2DEG > 15, `yaw change ${(yawDelta * RAD2DEG).toFixed(1)} deg`);
}

section('10. Suspension over the washboard section (circuit)');
{
  const car = makeCar('circuit', SIM.physicsHz, { distance: circuit.length * 0.78, lateral: 0 });
  run(car, 1.0, NEUTRAL);

  let minLen = 1e9, maxLen = -1e9, maxSusVel = 0, maxVertG = 0, bumpStops = 0;
  let lostContact = 0;
  run(car, 8, (t, c) => ({
    throttle: c.speedKmh < 85 ? 0.55 : 0.1, brake: 0, steer: laneKeep(c), handbrake: 0,
  }), (t, c) => {
    const s = c.wheelFL.suspension;
    minLen = Math.min(minLen, s.length);
    maxLen = Math.max(maxLen, s.length);
    maxSusVel = Math.max(maxSusVel, Math.abs(s.velocity));
    maxVertG = Math.max(maxVertG, Math.abs(c.gForce.y - 1));
    if (s.onBumpStop) bumpStops++;
    if (c.contactCount < 4) lostContact++;
  });

  check('suspension travel range used', maxLen - minLen, 0.010, 0.19, ' m');
  check('peak suspension velocity', maxSusVel, 0.05, 14, ' m/s');
  check('peak vertical G deviation', maxVertG, 0.03, 4.0, ' G');
  checkTrue('the suspension works without bottoming out constantly',
    bumpStops < 4000, `${bumpStops} bump-stop samples`);
  checkTrue('the wheels stay in contact almost all the time',
    lostContact < 8 * SIM.physicsHz * 0.4, `${lostContact} samples off the ground`);

  // A smooth surface must produce far less suspension movement than the
  // broken one - proof the movement comes from the ROAD, not from a shake.
  const smooth = makeCar('skidpad');
  accelerateTo(smooth, 24);
  let smoothRange = 0, lo = 1e9, hi = -1e9;
  run(smooth, 4, { throttle: 0.2, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    lo = Math.min(lo, c.wheelFL.suspension.length);
    hi = Math.max(hi, c.wheelFL.suspension.length);
  });
  smoothRange = hi - lo;
  checkTrue('a flat surface produces almost no suspension movement',
    smoothRange < (maxLen - minLen) * 0.5,
    `flat ${(smoothRange * 1000).toFixed(1)} mm vs broken ${((maxLen - minLen) * 1000).toFixed(1)} mm`);
}

section('11. Banked corner: the bank is physical, not painted on (circuit)');
{
  const car = makeCar('circuit', SIM.physicsHz, { distance: circuit.length * 0.40, lateral: 0 });
  run(car, 1.0, HANDBRAKE_ONLY);
  const rollOnBank = car.rollAngle * RAD2DEG;
  const normalTilt = Math.acos(Math.min(1, car.wheelFL.contactNormal.y)) * RAD2DEG;
  check('contact normal tilt on the bank', normalTilt, 8, 22, ' deg');
  check('the body follows the banking', Math.abs(rollOnBank), 6, 22, ' deg');
  checkTrue('all four wheels remain in contact on the bank',
    car.contactCount === 4, `${car.contactCount}`);
  checkTrue('the banking pushes the car into the corner',
    Math.abs(car.wheelFL.contactNormal.x) + Math.abs(car.wheelFL.contactNormal.z) > 0.1,
    `normal (${car.wheelFL.contactNormal.x.toFixed(2)}, `
    + `${car.wheelFL.contactNormal.y.toFixed(2)}, ${car.wheelFL.contactNormal.z.toFixed(2)})`);
}

section('12. Tyre pressure is not cosmetic');
{
  const results = {};
  for (const psi of [18, 32, 44]) {
    const car = makeCar('skidpad');
    car.setTirePressures(psi);
    run(car, 2.0, HANDBRAKE_ONLY);
    const staticDeflection = car.wheelFL.tire.deflection;   // same load every time

    accelerateTo(car, 26);
    let maxLat = 0;
    run(car, 3.5, { throttle: 0.3, brake: 0, steer: 0.5, handbrake: 0 }, (t, c) => {
      if (t > 1.2) maxLat = Math.max(maxLat, Math.abs(c.gForce.z));
    });
    results[psi] = {
      lat: maxLat,
      peakSlipAngle: car.wheelFL.tire.peakSlipAngle * RAD2DEG,
      deflection: staticDeflection,
      rollRes: car.wheelFL.tire.rollingResistance,
      radial: car.wheelFL.tire.radialStiffness,
    };
  }
  console.log('       psi   latG   peakSlipAngle  staticDeflection  rollingRes   radialStiffness');
  for (const psi of [18, 32, 44]) {
    const r = results[psi];
    console.log(`       ${String(psi).padStart(3)}  ${r.lat.toFixed(3)}      ${r.peakSlipAngle.toFixed(2)} deg`
      + `         ${(r.deflection * 1000).toFixed(1)} mm        ${r.rollRes.toFixed(4)}       ${(r.radial / 1000).toFixed(0)} kN/m`);
  }
  checkTrue('soft tyres need more slip angle (vaguer steering)',
    results[18].peakSlipAngle > results[32].peakSlipAngle * 1.1);
  checkTrue('hard tyres respond more sharply than nominal',
    results[44].peakSlipAngle < results[32].peakSlipAngle);
  checkTrue('soft tyres deform more under the same static load',
    results[18].deflection > results[32].deflection * 1.15,
    `${(results[18].deflection * 1000).toFixed(1)} mm vs `
    + `${(results[32].deflection * 1000).toFixed(1)} mm vs `
    + `${(results[44].deflection * 1000).toFixed(1)} mm`);
  checkTrue('soft tyres have more rolling resistance',
    results[18].rollRes > results[32].rollRes * 1.2);
  checkTrue('carcass stiffness rises with pressure',
    results[44].radial > results[32].radial && results[32].radial > results[18].radial);
  checkTrue('nominal pressure gives the most grip',
    results[32].lat >= results[18].lat * 0.98 && results[32].lat >= results[44].lat * 0.98,
    `18 psi ${results[18].lat.toFixed(3)} | 32 psi ${results[32].lat.toFixed(3)} `
    + `| 44 psi ${results[44].lat.toFixed(3)}`);
}

section('13. Wheel rotation matches the physics');
{
  const car = makeCar('skidpad');
  accelerateTo(car, 25);
  run(car, 0.5, { throttle: 0.25, brake: 0, steer: 0, handbrake: 0 });
  const w = car.wheelFL;
  const expectedOmega = car.forwardSpeed / w.tire.effectiveRadius;
  check('front wheel omega vs v/r', w.spinVelocity / expectedOmega, 0.95, 1.05);
  checkTrue('the spin angle is integrated from omega, not from speed',
    Math.abs(w.spinAngle) > 10, `angle ${w.spinAngle.toFixed(1)} rad`);

  const before = car.wheelFL.spinVelocity;
  run(car, 0.5, { throttle: 0, brake: 1, steer: 0, handbrake: 0 });
  checkTrue('braking reduces wheel rotation', car.wheelFL.spinVelocity < before * 0.9,
    `${before.toFixed(1)} -> ${car.wheelFL.spinVelocity.toFixed(1)} rad/s`);

  const car2 = makeCar('skidpad');
  accelerateTo(car2, 15);
  const signs = car2.wheels.map((x) => Math.sign(x.spinVelocity));
  checkTrue('all four wheels rotate the same way in a straight line',
    signs.every((s) => s === signs[0]) && signs[0] > 0, signs.join(','));

  // Cornering: the outer wheels must turn faster than the inner ones.
  const car3 = makeCar('skidpad');
  accelerateTo(car3, 18);
  run(car3, 3, { throttle: 0.25, brake: 0, steer: 0.5, handbrake: 0 });
  checkTrue('the outer wheels turn faster than the inner ones in a corner',
    car3.wheelFL.spinVelocity > car3.wheelFR.spinVelocity,
    `outer(FL) ${car3.wheelFL.spinVelocity.toFixed(1)} vs `
    + `inner(FR) ${car3.wheelFR.spinVelocity.toFixed(1)} rad/s`);
}

section('14. Reverse');
{
  const car = makeCar('skidpad');
  run(car, 1.5, { throttle: 0, brake: 1, steer: 0, handbrake: 0 });
  checkTrue('reverse engages from a standstill', car.gear === 'R', `gear ${car.gear}`);
  run(car, 3.0, { throttle: 0, brake: 1, steer: 0, handbrake: 0 });
  check('reverse speed after 3 s', -car.forwardSpeed * 3.6, 5, 70, ' km/h');
  checkTrue('the wheels turn backwards in reverse', car.wheelRL.spinVelocity < -1,
    `${car.wheelRL.spinVelocity.toFixed(2)} rad/s`);
  checkTrue('all four wheels turn backwards',
    car.wheels.every((x) => x.spinVelocity < 0));
  run(car, 4.0, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });
  checkTrue('throttle brings the car back to Drive', car.gear !== 'R', `gear ${car.gear}`);
  checkTrue('and it is moving forwards again', car.forwardSpeed > 0.5,
    `${(car.forwardSpeed * 3.6).toFixed(1)} km/h`);
}

section('15. Reset restores every piece of state');
{
  const car = makeCar('circuit');
  run(car, 8, (t, c) => ({ throttle: 1, brake: 0, steer: laneKeep(c), handbrake: 0 }));
  const movedSpeed = car.speed;
  const away = car.body.position.distanceTo(car.spawnGround);
  car.respawn();

  checkTrue('the car was actually moving before the reset',
    movedSpeed > 15 && away > 40, `${(movedSpeed * 3.6).toFixed(0)} km/h, ${away.toFixed(0)} m away`);
  check('speed immediately after reset', car.speed, 0, 0.001, ' m/s');
  check('yaw rate after reset', Math.abs(car.yawRate), 0, 0.001, ' rad/s');
  check('wheel spin after reset', Math.abs(car.wheelRL.spinVelocity), 0, 0.001, ' rad/s');
  check('steering after reset', Math.abs(car.steering.angle), 0, 0.001, ' rad');
  check('rpm after reset', car.rpm, 600, 950, ' rpm');
  check('suspension back at design height',
    Math.abs(car.wheelFL.suspension.displacement), 0, 0.001, ' m');
  checkTrue('the tyres are reset too',
    car.wheelRL.tire.slipRatio === 0 && car.wheelRL.tire.temperature === TYRE.startTemp);
  checkTrue('back on the start line',
    car.body.position.distanceTo(car.spawnGround) < 1.0,
    `${car.body.position.distanceTo(car.spawnGround).toFixed(3)} m`);

  // It must settle cleanly rather than dropping in.
  let maxVertG = 0;
  run(car, 1.5, HANDBRAKE_ONLY, (t, c) => {
    if (t > 2 * c.__dt) maxVertG = Math.max(maxVertG, Math.abs(c.gForce.y - 1));
  });
  check('vertical G spike on respawn', maxVertG, 0, 0.35, ' G');
}

section('16. Physics is stable and consistent across timesteps');
{
  // Deterministic, non-chaotic manoeuvre: accelerate, then brake to a stop.
  const results = {};
  for (const hz of [60, 120, 240, 480]) {
    const car = makeCar('skidpad', hz);
    let t100 = null;
    run(car, 10, { throttle: 1, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
      if (t100 === null && c.speedKmh >= 100) t100 = t;
    });
    const vAccel = car.speedKmh;
    const stop = brakeToStop(car);
    results[hz] = {
      t100,
      speed: vAccel,
      brakeDist: stop.distance,
      x: car.body.position.x,
      finite: car.body.position.isFinite() && car.body.velocity.isFinite(),
    };
  }
  console.log('        Hz    0-100(s)   v@10s(km/h)   braking(m)     x(m)');
  for (const hz of [60, 120, 240, 480]) {
    const r = results[hz];
    console.log(`       ${String(hz).padStart(4)}    ${(r.t100 ?? 0).toFixed(3).padStart(7)}     `
      + `${r.speed.toFixed(2).padStart(8)}    ${r.brakeDist.toFixed(2).padStart(8)}  ${r.x.toFixed(1).padStart(8)}`);
  }
  const ref = results[240];
  for (const hz of [60, 120, 240, 480]) {
    checkTrue(`state stays finite at ${hz} Hz`, results[hz].finite);
  }
  check('60 Hz vs 240 Hz speed error',
    (100 * Math.abs(results[60].speed - ref.speed)) / ref.speed, 0, 8, ' %');
  check('120 Hz vs 240 Hz speed error',
    (100 * Math.abs(results[120].speed - ref.speed)) / ref.speed, 0, 4, ' %');
  check('480 Hz vs 240 Hz speed error',
    (100 * Math.abs(results[480].speed - ref.speed)) / ref.speed, 0, 3, ' %');
  check('60 Hz vs 240 Hz braking distance error',
    (100 * Math.abs(results[60].brakeDist - ref.brakeDist)) / ref.brakeDist, 0, 12, ' %');
  check('60 Hz vs 240 Hz 0-100 error',
    (100 * Math.abs(results[60].t100 - ref.t100)) / ref.t100, 0, 10, ' %');
}

section('17. Off-road surfaces behave differently (circuit)');
{
  const onRoad = makeCar('circuit');
  run(onRoad, 6, (t, c) => ({ throttle: 1, brake: 0, steer: laneKeep(c), handbrake: 0 }));

  const offRoad = makeCar('circuit', SIM.physicsHz, { lateral: 13.5 });
  run(offRoad, 1.0, NEUTRAL);
  const surfaceName = offRoad.wheelFL.surface.name;
  run(offRoad, 6, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });

  checkTrue('the car is genuinely off the asphalt', surfaceName !== 'Asphalt', surfaceName);
  checkTrue('acceleration is much worse off-road',
    offRoad.speedKmh < onRoad.speedKmh * 0.85,
    `road ${onRoad.speedKmh.toFixed(1)} km/h vs ${surfaceName} ${offRoad.speedKmh.toFixed(1)} km/h`);

  const wet = makeCar('circuit', SIM.physicsHz, { distance: circuit.length * 0.50, lateral: 0 });
  run(wet, 1, NEUTRAL);
  checkTrue('the wet section is detected', wet.wheelFL.surface.name === 'Wet asphalt',
    wet.wheelFL.surface.name);

  // Same manoeuvre, dry vs wet skidpad: wet must give up first.
  const dryPad = new VehiclePhysics(new Skidpad(SURFACES.ASPHALT), BMW);
  const wetPad = new VehiclePhysics(new Skidpad(SURFACES.WET), BMW);
  const lat = [];
  for (const c of [dryPad, wetPad]) {
    c.reset(new Vec3(0, 0, 0), 0);
    c.__dt = 1 / SIM.physicsHz;
    accelerateTo(c, 26);
    let m = 0;
    run(c, 3.5, { throttle: 0.3, brake: 0, steer: 0.5, handbrake: 0 }, (t, x) => {
      if (t > 1.2) m = Math.max(m, Math.abs(x.gForce.z));
    });
    lat.push(m);
  }
  checkTrue('wet asphalt has clearly less grip than dry',
    lat[1] < lat[0] * 0.85, `dry ${lat[0].toFixed(2)} G vs wet ${lat[1].toFixed(2)} G`);
}

section('18. Collision with the static world (circuit)');
{
  const car = makeCar('circuit', SIM.physicsHz, { collision: true });
  const cs = car.collisionSystem;
  const s = SPAWN.trackDistance + 55;
  const ahead = circuit.spawnTransform(s, 0);
  const h = circuit.headingAt(s);
  const nx = -Math.sin(h), nz = Math.cos(h);
  cs.addWall(
    ahead.position.x - nx * 9, ahead.position.z - nz * 9,
    ahead.position.x + nx * 9, ahead.position.z + nz * 9,
    2.0, 0.6, ahead.position.y + 2.0,
  );

  let hit = false, maxImpact = 0;
  const speedBefore = [];
  run(car, 7, { throttle: 0.85, brake: 0, steer: 0, handbrake: 0 }, (t, c) => {
    if (!hit) speedBefore.push(c.speed);
    if (c.lastImpact > 0.5) { hit = true; maxImpact = Math.max(maxImpact, c.lastImpact); }
  });
  checkTrue('the car hit the barrier', hit, `impact ${maxImpact.toFixed(2)} m/s`);
  checkTrue('the car did not pass through it', car.body.position.isFinite());
  checkTrue('the impact slowed it down',
    car.speed < Math.max(...speedBefore) * 0.8,
    `${Math.max(...speedBefore).toFixed(1)} -> ${car.speed.toFixed(1)} m/s`);
  checkTrue('the car is still upright and stable', car.body.up.y > 0.3,
    `up.y ${car.body.up.y.toFixed(3)}`);
}

section('19. Long soak - no drift, no blow-up (circuit)');
{
  const car = makeCar('circuit');
  let worstUp = 1, maxSpeed = 0;
  run(car, 90, (t, c) => ({
    throttle: 0.5 + 0.5 * Math.sin(t * 0.35),
    brake: (Math.sin(t * 0.21) > 0.85) ? 0.7 : 0,
    steer: laneKeep(c) + 0.15 * Math.sin(t * 1.7),
    handbrake: (Math.sin(t * 0.13) > 0.97) ? 1 : 0,
  }), (t, c) => {
    worstUp = Math.min(worstUp, c.body.up.y);
    maxSpeed = Math.max(maxSpeed, c.speedKmh);
  });
  checkTrue('90 s of aggressive driving stays finite',
    car.body.position.isFinite() && car.body.velocity.isFinite()
    && car.body.angularVelocity.isFinite());
  check('peak speed over the soak', maxSpeed, 40, 320, ' km/h');
  check('tyre temperature after the soak', car.wheelRL.tire.temperature, 20, 190, ' C');
  checkTrue('no NaN anywhere in the wheels',
    car.wheels.every((w) => Number.isFinite(w.spinVelocity)
      && Number.isFinite(w.normalForce) && Number.isFinite(w.tire.slipRatio)
      && Number.isFinite(w.suspension.length)));
  checkTrue('the car stayed the right way up', worstUp > 0.2,
    `worst up.y ${worstUp.toFixed(3)}`);
}

section('20. Performance budget');
{
  const car = makeCar('circuit');
  const dt = 1 / SIM.physicsHz;
  const input = { throttle: 0.7, brake: 0, steer: 0.2, handbrake: 0 };
  for (let i = 0; i < 2000; i++) car.step(dt, input);       // warm up

  const t0 = process.hrtime.bigint();
  const N = 60000;
  for (let i = 0; i < N; i++) car.step(dt, input);
  const t1 = process.hrtime.bigint();
  const perStep = Number(t1 - t0) / 1e6 / N;
  const realtimeBudget = perStep * SIM.physicsHz;

  console.log(`       ${perStep.toFixed(4)} ms per step, `
    + `${realtimeBudget.toFixed(2)} ms of CPU per simulated second `
    + `(${SIM.physicsHz} Hz)`);
  check('CPU per simulated second', realtimeBudget, 0, 60, ' ms');
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
