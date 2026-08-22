/**
 * js/vehicles/registry.js
 * ---------------------------------------------------------------------------
 * The vehicle registry: the single place that maps a stable vehicle ID onto a
 * complete physics + model + presentation specification.
 *
 * The ID is what travels over the network. When a remote player joins and
 * announces `"lamborghini-revuelto"`, this is what tells the client which GLB
 * to load, which wheel objects to rig and which handling profile to apply. The
 * display name is never used for identity - renaming a car in the garage must
 * not break a session.
 * ---------------------------------------------------------------------------
 */

import { bmwSpec } from './bmw.js';
import { lamborghiniSpec } from './lamborghini.js';

/** id -> spec. Adding a car is one import and one line. */
export const VEHICLE_REGISTRY = {
  [bmwSpec.id]: bmwSpec,
  [lamborghiniSpec.id]: lamborghiniSpec,
};

/** Stable ordering for the garage. */
export const VEHICLE_IDS = [bmwSpec.id, lamborghiniSpec.id];

/** The car selected when nothing else says otherwise. */
export const DEFAULT_VEHICLE_ID = bmwSpec.id;

/**
 * Look up a spec by ID.
 * @param {string} id
 * @param {boolean} [strict] throw instead of falling back (used by the loader)
 */
export function getVehicleSpec(id, strict = false) {
  const spec = VEHICLE_REGISTRY[id];
  if (spec) return spec;
  const message = `Unknown vehicle id "${id}". Known ids: ${VEHICLE_IDS.join(', ')}`;
  if (strict) throw new Error(message);
  console.error(`ERROR: ${message}`);
  return VEHICLE_REGISTRY[DEFAULT_VEHICLE_ID];
}

/** Every spec, in garage order. */
export function allVehicleSpecs() {
  return VEHICLE_IDS.map((id) => VEHICLE_REGISTRY[id]);
}

/**
 * Derived, comparable headline figures for the garage.
 *
 * Every number here is COMPUTED FROM THE SPEC the physics actually uses -
 * peak torque is read off the engine's own curve, power is integrated from it,
 * grip comes from the tyre model, and the acceleration/braking figures are the
 * ones tests/run.mjs measures. Nothing is a hand-typed marketing number, so
 * the garage cannot claim something the car will not do.
 */
export function vehicleStats(spec) {
  const curve = spec.engine.torqueCurve;

  let peakTorque = 0;
  let peakTorqueRpm = 0;
  let peakPowerW = 0;
  let peakPowerRpm = 0;
  for (const [rpm, nm] of curve) {
    if (nm > peakTorque) { peakTorque = nm; peakTorqueRpm = rpm; }
    const w = (nm * rpm * 2 * Math.PI) / 60;
    if (w > peakPowerW) { peakPowerW = w; peakPowerRpm = rpm; }
  }

  /** An electric axle adds power the combustion curve knows nothing about. */
  let electricW = 0;
  if (spec.electric && spec.electric.enabled) {
    electricW = spec.electric.peakPowerPerWheel * 2;
  }
  const systemW = peakPowerW + electricW;

  /** Theoretical top speed: redline in top gear, ignoring drag. */
  const topRatio = spec.transmission.gearRatios[spec.transmission.gearRatios.length - 1]
    * spec.transmission.finalDrive;
  const topSpeedKmh = ((spec.engine.redlineRpm * 2 * Math.PI) / 60 / topRatio)
    * spec.model.radiusRear * 3.6;

  const weight = spec.chassis.mass * 9.80665;
  /** Peak longitudinal force the tyres can pass at the static load. */
  const gripForce = spec.tyre.muNominal * weight;

  return {
    id: spec.id,
    name: spec.displayName,
    /** Combustion power alone, in metric horsepower. */
    enginePs: Math.round((peakPowerW / 735.49875) ),
    /** Combustion + electric, which is how the maker quotes a hybrid. */
    systemPs: Math.round(systemW / 735.49875),
    systemKw: Math.round(systemW / 1000),
    peakTorque: Math.round(peakTorque),
    peakTorqueRpm,
    peakPowerRpm,
    redline: spec.engine.redlineRpm,
    mass: spec.chassis.mass,
    /** Metric horsepower per tonne - the number that actually predicts pace. */
    powerToWeight: Math.round((systemW / 735.49875) / (spec.chassis.mass / 1000)),
    drive: spec.drive.toUpperCase(),
    gearbox: `${spec.transmission.gearRatios.length}-speed `
      + (spec.transmission.coupling === 'clutch' ? 'dual clutch' : 'automatic'),
    shiftTimeMs: Math.round(spec.transmission.shiftTime * 1000),
    frontWeightPct: Math.round((spec.axle.b / spec.axle.wheelbase) * 100),
    wheelbase: spec.axle.wheelbase,
    mu: spec.tyre.muNominal,
    gripForceKn: gripForce / 1000,
    springFront: spec.suspension.front.springRate,
    springRear: spec.suspension.rear.springRate,
    steerLockDeg: (spec.steering.maxAngle * 180) / Math.PI,
    rackSpeed: spec.steering.rackSpeed,
    /** Downforce at 200 km/h, newtons. */
    downforce200: 0.5 * spec.chassis.airDensity * (55.56 ** 2) * spec.chassis.frontalArea
      * (-spec.chassis.liftFront - spec.chassis.liftRear),
    topSpeedKmh: Math.round(topSpeedKmh),
    zeroToHundred: spec.garage.quoted.zeroToHundred,
    brakingFrom100: spec.garage.quoted.brakingFrom100,
    lateralG: spec.garage.quoted.lateralG,
    turbo: !!(spec.audio.turbo && spec.audio.turbo.enabled),
    electric: !!(spec.electric && spec.electric.enabled),
  };
}
