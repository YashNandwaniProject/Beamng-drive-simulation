/**
 * js/vehicles/lamborghini.js
 * ---------------------------------------------------------------------------
 * Lamborghini Revuelto - complete vehicle specification.
 *
 * This car deliberately shares NONE of the BMW's tuning. It is a mid-engined,
 * all-wheel-drive, naturally aspirated V12 hybrid, and every subsystem below is
 * set up to make it feel like one:
 *
 *   - a 9,500 rpm NA V12 with no turbo lag at all, against the BMW's laggy
 *     turbo six;
 *   - an electric FRONT axle that delivers full torque from a standstill,
 *     which is what turns a 6.6 s car into a 2.x s one;
 *   - a dual-clutch gearbox instead of a torque converter, so there is no
 *     torque multiplication off the line and shifts take 90 ms not 160 ms;
 *   - roughly twice the spring rate, half the suspension travel;
 *   - a much lower polar moment (mid-engine), so it changes direction eagerly;
 *   - real downforce, five times the BMW's;
 *   - staggered tyres - 20" front, 21" rear - on genuinely different radii.
 *
 * MODEL GEOMETRY
 * --------------
 * Everything geometric was measured directly out of
 * assets/lamborghini-revuelto.glb by walking the glTF node tree and
 * transforming each object's bounding box into model space. The figures agree
 * with the real car to within a few millimetres, which is a good sign the
 * model is to scale:
 *
 *     measured        real Revuelto
 *     4.948 m long    4.947 m
 *     2.781 wheelbase 2.779 m
 *     1.712 front trk 1.700 m
 *     1.680 rear trk  1.660 m
 *     0.337 rear tyre 0.345 m section width
 *     0.075 clearance very low, as it should be
 *
 * IMPORTANT: this model is authored NOSE-ALONG-MINUS-X (its own front wheels
 * are at x = 0 and its rear wheels at x = +2.781), which is the opposite of the
 * BMW. `modelYaw: PI` is what carries it into the shared body frame. Without
 * it the car would drive backwards and steer the wrong way.
 * ---------------------------------------------------------------------------
 */

import { DEG2RAD } from '../mathUtils.js';
import { buildVehicleSpec } from './spec.js';

/* ========================================================================= */
/* Measured geometry from assets/lamborghini-revuelto.glb (model space, m)    */
/* ========================================================================= */

const MODEL = {
  path: 'assets/lamborghini-revuelto.glb',

  /** Model forward is -X, so rotate 180 deg about +Y into the body frame. */
  modelYaw: Math.PI,

  /**
   * The four wheel objects the brief pins down. All four exist in the GLB and
   * are unique; the loader fails loudly and by name if any is missing.
   */
  wheelObjects: {
    FL: 'Wheel_001-material.001',   // front left  / driver / steering side
    FR: 'Wheel_001-material',       // front right / passenger side
    RL: 'Wheel_001-material.003',   // rear  left
    RR: 'Wheel_001-material.002',   // rear  right
  },

  /**
   * Interior steering wheel. Unlike the BMW - whose interior must never move -
   * this one is animated, driven by the physical steering angle.
   *
   * The object's own node origin sits at the MODEL origin, not at the wheel,
   * so it cannot simply be rotated in place: doing that would swing it around
   * the car. `pivot` is the centre of the rim and `axis` is the column axis,
   * both recovered by principal-component analysis over the assembly's 12,707
   * vertices. The axis comes out at 22.5 deg from horizontal with essentially
   * zero lateral component, which is exactly what a real steering column is -
   * a good sign the fit found the right plane.
   */
  steeringWheel: {
    object: 'MAT_CarbonFiberMatte-material.006',
    pivot: { x: 0.8839, y: 0.5900, z: 0.3796 },
    axis: { x: 0.9241, y: 0.3822, z: 0.0 },
    /**
     * Steering-wheel degrees per road-wheel degree. With a 30 deg lock that
     * gives ~390 deg of rotation each way, i.e. about 2.2 turns lock to lock.
     */
    ratio: 13.0,
  },

  /** Wheel hub centres, from each wheel's bounding box (model space). */
  hubs: {
    FL: { x: 0.0000, y: 0.2222, z: 0.8570 },
    FR: { x: 0.0000, y: 0.2222, z: -0.8548 },
    RL: { x: 2.7812, y: 0.2348, z: 0.8422 },
    RR: { x: 2.7812, y: 0.2348, z: -0.8377 },
  },

  /**
   * Staggered tyres, measured from the wheel bounding boxes.
   *   front  0.639 m diameter, 0.257 m wide   (265/35 R20)
   *   rear   0.688 m diameter, 0.337 m wide   (345/30 R21)
   * The model's four wheels do not quite share one contact plane (the rears
   * sit 12 mm lower), which is normal for a downloaded asset. Nothing needs
   * correcting: each corner has its own suspension and simply finds its own
   * ground, absorbing the difference in a few millimetres of travel.
   */
  tyreRadiusFront: 0.31935,
  tyreRadiusRear: 0.34415,
  tyreWidthFront: 0.2573,
  tyreWidthRear: 0.3375,

  /**
   * The wheels are already square to the car and the rear right is MIRRORED in
   * Z, so the authored orientation is preserved rather than rebuilt - a rebuild
   * would flip that wheel inside out.
   */
  wheelOrientation: 'preserve',

  /**
   * No caliper rigging: this model's brake discs are already children of the
   * tyres, which is exactly the hierarchy the brief describes, so they spin
   * with the wheel as they should.
   */

  /**
   * The model uses one "Light" material for every lamp, so only the ones
   * behind the centre of mass are wired up as brake lights.
   */
  brakeLights: { material: 'Light', rearOnly: true, colour: 0xff2a08 },

  /**
   * This model is 829 separate meshes. Only parts at least 10 cm across cast
   * shadows - everything smaller still receives them, but does not get drawn a
   * second time into the shadow map for a result nobody can see.
   */
  shadowMinRadius: 0.10,

  /**
   * Batch the static bodywork by material after rigging. 829 meshes is 2,600
   * draw calls a frame; almost all of them share one of 31 materials, so this
   * collapses the body to a few dozen. The rigged wheels and the steering
   * wheel are excluded - they need their own transforms and their hierarchy
   * has to stay intact.
   */
  mergeBody: true,

  /** Fraction of the tyre radius taken up by the rim (for the tread shader). */
  rimRadiusRatio: 0.63,

  /**
   * Paint depth. A Revuelto's finish is a metallic base under a deep lacquer,
   * and the lacquer is most of what you actually see: a sharp, mirror-like
   * reflection lying over the softer colour underneath. Turning this on
   * re-issues the paint as a clear-coated material - see
   * `Vehicle._withClearcoat`.
   */
  paintClearcoat: 0.85,

  /**
   * How strongly this car's glossy surfaces take the real-time reflection
   * probe (js/reflections.js) relative to the sky IBL they use otherwise.
   * Above 1 because the probe includes the ground and is therefore darker
   * overall than a sky-only environment - at parity the panels lose their
   * shine the moment reflections are switched on.
   */
  reflectionIntensity: 1.25,

  /** Mean contact plane in model space. */
  groundY: -0.10325,

  /**
   * Centre of mass. Lamborghini quote 44:56 front:rear for the Revuelto, and
   * the CoG sits 0.44 m above the road - low even by supercar standards, which
   * is a large part of why it rolls so little.
   *     comX = rearAxle - 0.44 * wheelbase = 2.7812 - 1.2237
   */
  comX: 1.5575,
  comY: 0.33675,
  comZ: 0.0,

  bodyLength: 4.948,
  bodyWidth: 2.033,          // excluding the mirrors (bbox with them is 2.266)
  bodyHeight: 1.154,
  underbodyClearance: 0.0749,
};

/* ========================================================================= */
/* Chassis                                                                    */
/* ========================================================================= */

const CHASSIS = {
  /** Dry weight is 1772 kg; this is a realistic kerb figure with a driver. */
  mass: 1885,

  /**
   * Principal moments of inertia (kg.m^2).
   *  Ixx roll, Iyy yaw, Izz pitch.
   *
   * The yaw figure is the interesting one: 2200 against the BMW's 2650,
   * despite this car being 165 kg heavier and 30 cm longer. That is what
   * "mid-engined" means numerically - the masses are gathered near the centre
   * instead of hung off the nose - and it is why the Revuelto changes
   * direction so much more eagerly than the BMW does.
   */
  inertia: { x: 620, y: 2200, z: 2320 },

  /** Aerodynamics. Supercars are draggy; this one is also very slippery-shaped
   *  but runs a lot of downforce, so Cd is high for its class. */
  dragCoefficient: 0.36,
  frontalArea: 2.05,
  airDensity: 1.225,

  /**
   * Downforce, five times the BMW's. At 200 km/h this is ~2.8 kN, about 290 kg
   * pressing the car down, and it is why the Revuelto keeps gaining cornering
   * grip with speed while the BMW does not.
   */
  liftFront: -0.28,
  liftRear: -0.45,
  aeroCentreHeight: 0.03,

  /** Collision hull, from the measured bodywork. */
  hullHalf: { x: 2.474, y: 0.523, z: 1.017 },
  hullOffsetX: 0.2705,
  hullOffsetY: 0.183,
  restitution: 0.20,
  hullFriction: 0.35,
};

/* ========================================================================= */
/* Suspension - roughly twice the BMW's rate, half its travel                 */
/* ========================================================================= */

const SUSPENSION = {
  front: {
    /** 68 kN/m over a 415 kg corner is a ~2.0 Hz ride frequency. */
    springRate: 68000,
    bumpDamping: 3190,       // ~0.30 of critical
    reboundDamping: 5100,    // ~0.48 of critical
    maxCompression: 0.062,
    maxDroop: 0.070,
    bumpStopZone: 0.020,
    bumpStopRate: 420000,
    antiRollRate: 34000,
    camber: -1.6 * DEG2RAD,
    toe: 0.03 * DEG2RAD,
    caster: 7.2 * DEG2RAD,
    scrubRadius: 0.008,
  },
  rear: {
    /** 96 kN/m over a 528 kg corner is a ~2.15 Hz ride frequency. */
    springRate: 96000,
    bumpDamping: 4270,
    reboundDamping: 6830,
    maxCompression: 0.058,
    maxDroop: 0.066,
    bumpStopZone: 0.019,
    bumpStopRate: 470000,
    antiRollRate: 26000,
    camber: -2.2 * DEG2RAD,
    toe: 0.15 * DEG2RAD,
    caster: 0,
    scrubRadius: 0,
  },
  /** Bigger wheels, but carbon-ceramic discs claw some of it back. */
  unsprungMass: 46,
};

/* ========================================================================= */
/* Tyres - Corsa-compound, low profile, staggered                             */
/* ========================================================================= */

const TYRE = {
  /** Rotational inertia per wheel, kg.m^2 (~0.55 m r^2, bigger at the rear). */
  inertiaFront: 1.32,
  inertiaRear: 1.86,

  nominalPressure: 34,
  minPressure: 14,
  maxPressure: 52,

  /** Magic-Formula shape. Same structure as the BMW, different peaks. */
  mfB: 1.74,
  mfC: 1.65,
  mfE: 0.50,

  /**
   * A track-biased tyre reaches its peak at LESS slip than a road tyre and
   * holds more of it. That is the whole character difference: the Revuelto
   * responds faster and grips harder, but gives less warning at the limit.
   */
  peakSlipRatio: 0.105,
  peakSlipAngle: 8.0 * DEG2RAD,

  muNominal: 1.42,
  /** Reference load is higher - these are much bigger tyres. */
  refLoad: 5200,
  /** A stiffer carcass loses less grip as load piles on. */
  loadSensitivity: 0.100,

  /** Shorter relaxation lengths: the sidewall winds up far less. */
  relaxLongitudinal: 0.26,
  relaxLateral: 0.42,
  slipSpeedFloor: 1.6,

  /** Very stiff, low-profile carcass. */
  radialStiffness: 265000,
  radialStiffnessPerPsi: 6200,
  radialDamping: 640,
  effectiveRadiusFactor: 0.68,

  /** Wide, sticky rubber costs rolling resistance. */
  rollingResistance: 0.0135,

  pneumaticTrail: 0.026,

  ambientTemp: 22,
  /** A track compound wants more heat in it than a road tyre. */
  optimumTemp: 88,
  startTemp: 75,
  tempGripFalloff: 0.000090,
  /**
   * A bigger, heavier car making 1.4 G puts far more friction work into its
   * tyres than the BMW does, so the heating rate is lower and the cooling
   * higher than that car's. Left at the BMW's numbers these tyres pinned the
   * 190 C ceiling within a lap and quietly threw away half their grip.
   */
  tempHeatRate: 0.00012,
  tempCoolRate: 0.0110,
  tempCoolSpeedFactor: 0.030,

  /** A 30-profile sidewall simply cannot deflect as far as the BMW's. */
  maxVisualDeflection: 0.032,
  sidewallBulge: 0.40,
};

/* ========================================================================= */
/* Engine - 6.5 L naturally aspirated V12, 825 CV at 9250 rpm                 */
/* ========================================================================= */

const ENGINE = {
  /**
   * Full-load torque curve [rpm, Nm]. Peak 725 Nm at 6750 rpm, and the power
   * peak works out at 627 N.m x 9250 rpm = 607 kW = 825 CV, which is the
   * factory figure.
   */
  torqueCurve: [
    [900, 380], [1500, 480], [2500, 570], [3500, 630], [4500, 675],
    [5500, 700], [6750, 725], [7500, 715], [8500, 680], [9250, 627],
    [9500, 600], [10000, 480],
  ],
  idleRpm: 900,
  stallRpm: 600,
  redlineRpm: 9500,
  limiterRpm: 9750,
  limiterCutTime: 0.045,
  maxRpm: 10000,

  /** A V12 with a light flywheel spins up very fast. */
  inertia: 0.190,

  frictionBase: 18,
  frictionPerRpm: 0.0085,
  /** Twelve cylinders on a closed throttle make strong engine braking. */
  overrunFrictionScale: 2.10,

  /**
   * NATURALLY ASPIRATED. These time constants are near zero on purpose - there
   * is no turbo to spool, so the throttle is effectively instantaneous. Next to
   * the BMW's 0.28 s of boost build, this is the single most obvious
   * difference in how the two cars respond.
   */
  boostBuildTime: 0.045,
  boostDecayTime: 0.035,
  throttleResponseTime: 0.05,

  idleGain: 0.00075,
  idleIntegralGain: 0.0016,
};

/* ========================================================================= */
/* Transmission - 8-speed dual clutch, no torque converter                    */
/* ========================================================================= */

/**
 * A DCT, not an automatic. `coupling: 'clutch'` selects the dry-clutch path in
 * drivetrain.js: no torque multiplication off the line, and a launch that
 * holds the engine at a target rpm while the clutch slips, then bites.
 */
const CLUTCH = {
  /** Must comfortably hold 725 N.m with margin. */
  maxTorque: 1400,
  /** Engine speed the clutch slips against on a full-throttle launch. */
  launchRpm: 3400,
  /** Above this road speed the clutch is always fully closed. */
  lockSpeed: 3.0,
  /** Creep torque with the brake released, as a DCT in Drive does. */
  creepEngagement: 0.018,
  /** Gain of the launch clutch's engagement controller (per rpm-second). */
  engageGain: 0.00055,
  /** Driven-wheel slip above which launch control stops squeezing the clutch. */
  launchSlipLimit: 0.14,
  /** How fast the clutch closes once rolling, and how much slip slows that. */
  reengageRate: 9.0,
  reengageSlipFactor: 0.05,
  /** Floor on the closing rate, so a big mismatch still takes up in ~0.4 s. */
  reengageMinRate: 2.5,
  shiftOpenTime: 0.02,
};

const TRANSMISSION = {
  coupling: 'clutch',
  /**
   * Eight speeds. Top gear works out at 9000 rpm = 349 km/h, which is the
   * quoted >350 km/h.
   */
  gearRatios: [3.91, 2.44, 1.77, 1.38, 1.12, 0.92, 0.79, 0.72],
  reverseRatio: 3.40,
  finalDrive: 4.64,
  /** A DCT is mechanically more efficient than a converter automatic. */
  efficiency: 0.940,
  inertia: 0.028,

  autoShift: true,
  upshiftRpm: 9300,
  upshiftRpmLight: 2800,
  downshiftRpm: 1600,
  kickdownRpm: 7200,
  /** 90 ms. The BMW's converter box takes 160. */
  shiftTime: 0.09,
  shiftCooldown: 0.30,
  reverseSpeed: 0.7,
};

const DIFFERENTIAL = {
  type: 'lsd',
  preload: 110,
  torqueBias: 0.38,
  maxLockTorque: 1400,
};

/* ========================================================================= */
/* Electric front axle - what makes this car an AWD hybrid                    */
/* ========================================================================= */

/**
 * The Revuelto's front axle has no mechanical connection to the engine at all:
 * two 110 kW motors, one per wheel, each through its own reduction. That gives
 * three things a belt-driven front axle cannot:
 *
 *   1. FULL TORQUE AT ZERO RPM. There is no clutch to slip and no gear to be in
 *      - the front wheels pull from the instant the throttle opens, which is
 *      most of why the car reaches 100 km/h in under three seconds.
 *   2. TRUE PER-WHEEL TORQUE VECTORING, since the two motors are independent.
 *   3. A NATURAL HIGH-SPEED FADE. Each motor is power limited, so above about
 *      130 km/h its torque falls as 1/omega and the car quietly becomes
 *      rear-drive - exactly as the real one does.
 */
const ELECTRIC = {
  enabled: true,
  axle: 'front',
  /** Peak torque AT THE WHEEL, per wheel, N.m. */
  peakTorquePerWheel: 980,
  /** Continuous power per motor, W. Sets where the torque starts to fade. */
  peakPowerPerWheel: 110000,
  /** Rotor + reduction inertia reflected to the wheel, kg.m^2. */
  reflectedInertia: 0.90,
  /** Regenerative braking torque per wheel on a closed throttle, N.m. */
  regenTorque: 260,
  /**
   * Torque vectoring: fraction of the front axle's torque that can be shuffled
   * side to side in proportion to steering demand, which tightens the line.
   */
  torqueVectoring: 0.35,
  /** Electric torque is cut once the tyre is spinning this much. */
  slipCutoff: 0.16,
};

/* ========================================================================= */
/* Brakes - carbon ceramic, 410 mm front / 390 mm rear                        */
/* ========================================================================= */

const BRAKES = {
  maxTorqueFront: 4200,
  maxTorqueRear: 2600,
  handbrakeTorque: 2600,
  /** A firmer, faster pedal than the BMW's. */
  pedalResponseTime: 0.07,

  abs: {
    enabled: true,
    /** Sharper tyre, so the regulation band sits tighter to its peak. */
    targetSlip: -0.090,
    updateHz: 120,
    releaseRate: 105.0,
    applyRate: 30.0,
    minSpeed: 2.2,
    minPressure: 0.22,
  },

  tractionControl: {
    enabled: true,
    targetSlip: 0.090,
    /**
     * A modern supercar's traction control is FAST - it has to be, because
     * first gear is short enough to overwhelm the rear tyres in a few tens of
     * milliseconds. Cutting at 20/s and restoring at 3/s stops the slip
     * running away without the system then hunting on the way out.
     */
    cutRate: 20.0,
    restoreRate: 5.0,
    minThrottleCut: 0.18,
    slipBand: 0.060,
    /** Settle just under the target instead of hunting in and out of grip. */
    proportionalRestore: true,
  },

  stabilityControl: {
    enabled: false,
    yawGain: 0.55,
    maxBrakeTorque: 900,
  },
};

/* ========================================================================= */
/* Steering - quicker rack, less lock, racier geometry                        */
/* ========================================================================= */

const STEERING = {
  /** Supercars run less lock than saloons. */
  maxAngle: 30 * DEG2RAD,
  /** A noticeably faster rack than the BMW's 2.05. */
  rackSpeed: 2.60,
  returnSpeed: 3.90,
  inputSmoothTime: 0.060,

  highSpeed: 62,
  highSpeedScale: 0.26,

  aligningGain: 0.048,
  aligningMax: 0.40,

  /** Racier, more parallel geometry than the BMW's 0.78. */
  ackermann: 0.62,
  counterSteerAssist: 0.0,
};

/* ========================================================================= */
/* Presentation                                                               */
/* ========================================================================= */

const GARAGE = {
  manufacturer: 'Lamborghini',
  model: 'Revuelto',
  year: 2024,
  tagline: 'V12 hybrid, all-wheel drive, and an electric axle that never waits.',
  bodyStyle: 'Mid-engined coupe',
  colour: '#f2a71b',
  accent: '#ffb300',
  /**
   * MEASURED by tests/vehicles.mjs on the flat skidpad, not quoted from the
   * brochure. The real car claims 2.5 s to 100 km/h with launch control and
   * warmed track rubber; this model reaches 3.5 s, and the garage says 3.5 so
   * it cannot promise something the car will not do.
   */
  quoted: {
    zeroToHundred: 3.5,
    brakingFrom100: 30.4,
    lateralG: 1.40,
  },
};

/** Pulled back and lowered slightly: it is a longer, much lower car. */
const CAMERA_PROFILE = {
  chase: { x: -7.6, y: 2.35, z: 0.0 },
  chaseLookAt: { x: 1.6, y: 0.45, z: 0.0 },
  side: { x: -5.4, y: 1.40, z: -5.9 },
  sideLookAt: { x: 0.5, y: 0.40, z: 0.0 },
  hood: { x: 0.60, y: 0.62, z: 0.0 },
  hoodLookAt: { x: 12.0, y: 0.45, z: 0.0 },
  /**
   * The driver's eye, anchored to the measured steering-wheel rim rather than
   * guessed. MODEL.steeringWheel.pivot lands at body (0.674, 0.253, -0.380),
   * i.e. 0.69 m above the road and 0.38 m left of centre - this car is
   * left-hand drive. The eye sits 0.55 m behind that rim and 0.33 m above it,
   * which puts it 1.02 m above the road: right for a car that is only 1.16 m
   * tall, and it frames the binnacle through the top of the wheel exactly as
   * the real cabin does.
   */
  interior: { x: 0.124, y: 0.568, z: -0.380 },
  /** Down the road, and a touch toward the centreline as a driver looks. */
  interiorLookAt: { x: 12.0, y: 0.30, z: -0.28 },
  orbit: { x: -9.0, y: 3.0, z: 0.0 },
  orbitLookAt: { x: 0.0, y: 0.40, z: 0.0 },
};

/**
 * Audio. A V12 fires six times per revolution against the BMW six's three, so
 * at the same engine speed its fundamental is an octave higher - and it revs
 * to 9,500 rather than 6,750. The result is a hard, bright, screaming voice
 * with strong high harmonics and no turbo content whatsoever.
 */
const AUDIO = {
  harmonics: [
    [0.5, 0.10, 'sine'],
    [1.0, 0.46, 'sawtooth'],
    [2.0, 0.40, 'sawtooth'],
    [3.0, 0.30, 'sawtooth'],
    [4.0, 0.22, 'square'],
    [6.0, 0.14, 'square'],
  ],
  cylinders: 12,
  baseGain: 0.30,
  /** Opens much brighter than the BMW: an NA V12 has no muffled bottom end. */
  filterClosed: 1100,
  filterOpen: 6200,
  /** No turbo. */
  turbo: { enabled: false },
  noiseGain: 0.040,
  detune: 3,
  /** A short, hard cut on every DCT shift. */
  shiftBlip: 0.55,
  /** Faint electric-motor whine from the front axle - unique to this car. */
  electricWhine: { enabled: true, gain: 0.030, freqPerSpeed: 26, baseFreq: 90 },
};

/* ========================================================================= */

export const lamborghiniSpec = buildVehicleSpec({
  id: 'lamborghini-revuelto',
  displayName: 'Lamborghini Revuelto',
  shortName: 'Revuelto',
  drivetrainLayout: 'awd',
  model: MODEL,
  chassis: CHASSIS,
  suspension: SUSPENSION,
  tyre: TYRE,
  engine: ENGINE,
  clutch: CLUTCH,
  transmission: TRANSMISSION,
  differential: DIFFERENTIAL,
  electric: ELECTRIC,
  brakes: BRAKES,
  steering: STEERING,
  garage: GARAGE,
  cameraProfile: CAMERA_PROFILE,
  audio: AUDIO,
});

export default lamborghiniSpec;
