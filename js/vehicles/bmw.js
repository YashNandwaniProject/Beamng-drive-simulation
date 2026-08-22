/**
 * js/vehicles/bmw.js
 * ---------------------------------------------------------------------------
 * BMW - vehicle tuning and measured model geometry.
 *
 * Every value below was MOVED VERBATIM out of config.js when the simulation
 * gained a second vehicle. Not one number was retuned in the process: the
 * head-less suite pins this car's behaviour to three decimal places (0-100 in
 * 6.646 s, 35.032 m from 100 km/h, 5.026 deg of roll), and those figures are
 * unchanged. The only thing that actually changed is that the physics modules
 * now receive this object instead of importing the constants directly.
 *
 * See js/vehicles/lamborghini.js for the second car, and js/vehicles/spec.js
 * for the shared derivation of axle geometry and static corner loads.
 * ---------------------------------------------------------------------------
 */

import { DEG2RAD } from '../mathUtils.js';
import { buildVehicleSpec } from './spec.js';


/* ========================================================================= */
/* Measured geometry from assets/BMW.glb (model-space metres)                 */
/* ========================================================================= */

const MODEL = {
  path: 'assets/BMW.glb',

  /**
   * The four wheel objects the brief pins down. Names are matched
   * case-insensitively because the GLB authors them with a capital O
   * (`Object_221`) while the brief writes `object_221`.
   */
  wheelObjects: {
    FL: 'Object_221',   // front  left  / driver / steering side
    FR: 'Object_95',    // front  right / passenger side
    RL: 'Object_236',   // rear   left
    RR: 'Object_206',   // rear   right
  },

  /**
   * This model's front wheels carry a baked ~6 degree rotation, so the rig
   * rebuilds a clean spin axis rather than preserving what was authored.
   */
  wheelOrientation: 'realign',
  /** Body-mounted brake calipers, matched by material name. */
  caliperMaterial: 'CALIPER',
  /** Brake lights, matched by material name. */
  brakeLights: { material: 'BRAKELIGHT', colour: 0xff1408 },

  /** Measured tyre geometry (identical for all four). */
  tyreRadius: 0.3290,          // local bbox |x|,|z| extent
  tyreWidth: 0.2920,           // local bbox y extent
  /**
   * The tyre mesh pivot sits on its OUTER face, not at the wheel centre, and
   * its local +Y is the spin axis pointing inboard. Half the width therefore
   * separates the authored pivot from the true hub centre.
   */
  tyrePivotToCentre: 0.1460,
  rimRadius: 0.2500,

  /** Wheel hub centres in model space (pivot + half width, inboard). */
  hubs: {
    FL: { x: 1.6173, y: 0.2333, z: -0.8077 },
    FR: { x: 1.6173, y: 0.2323, z: 0.8101 },
    RL: { x: -1.1142, y: 0.2381, z: -0.8075 },
    RR: { x: -1.1135, y: 0.2371, z: 0.8099 },
  },

  /** Ground plane in model space = mean hub height - tyre radius. */
  groundY: -0.0925,

  /**
   * One design hub height for all four corners. The BMW runs the same tyre on
   * both axles, and the original config used a single AXLE.hubY everywhere, so
   * declaring it explicitly keeps this car bit-identical now that the spec
   * builder can otherwise derive a per-corner height (which the Lamborghini,
   * with its staggered 20/21-inch wheels, genuinely needs).
   *     0.2357 - comY(0.4075) = -0.1718, exactly the original AXLE.hubY.
   */
  uniformHubHeight: 0.2357,

  /**
   * Centre of mass in model space.
   *  x: 52 % front static weight distribution over the measured wheelbase
   *  y: 0.500 m above the contact plane (typical for a lowered 3-series)
   */
  comX: 0.3063,
  comY: 0.4075,
  comZ: 0.0,

  /** Bodywork extents (for the collision hull and camera framing). */
  bodyLength: 4.648,
  bodyWidth: 1.966,
  bodyHeight: 1.627,
  underbodyClearance: 0.0575,
};

/** Derived axle geometry - single source of truth for Ackermann etc. */
const AXLE = {
  /** CoM -> front axle (a) and CoM -> rear axle (b), both positive. */
  a: MODEL.hubs.FL.x - MODEL.comX,          // 1.3110
  b: MODEL.comX - MODEL.hubs.RL.x,          // 1.4205
  wheelbase: MODEL.hubs.FL.x - MODEL.hubs.RL.x,   // 2.7315
  trackFront: MODEL.hubs.FR.z - MODEL.hubs.FL.z,  // 1.6178
  trackRear: MODEL.hubs.RR.z - MODEL.hubs.RL.z,   // 1.6174
  /** Hub height relative to the CoM at design ride height. */
  hubY: 0.2357 - MODEL.comY,                // -0.1718
};

/* ========================================================================= */
/* Chassis                                                                    */
/* ========================================================================= */

const CHASSIS = {
  mass: 1720,                    // kg, kerb + driver
  /**
   * Principal moments of inertia in the body frame (kg.m^2).
   *  Ixx -> roll  (about the longitudinal axis, +X)
   *  Iyy -> yaw   (about the vertical axis,     +Y)
   *  Izz -> pitch (about the lateral axis,      +Z)
   * Values are in the normal range for a 4.6 m rear-drive sedan.
   */
  inertia: { x: 560, y: 2650, z: 2480 },

  /** Aerodynamics. */
  dragCoefficient: 0.29,
  frontalArea: 2.19,             // m^2
  airDensity: 1.225,             // kg/m^3
  /** Lift coefficients (negative = downforce) split front/rear. */
  liftFront: -0.06,
  liftRear: -0.09,
  /** Height of the aero centre of pressure above the CoM. */
  aeroCentreHeight: 0.05,

  /**
   * Collision hull half-extents in body space (x fwd, y up, z right), derived
   * from the measured bodywork bounds. The hull is NOT centred on the CoM:
   * the model spans -2.208..+2.440 in x and -0.0925..+1.421 in y, while the
   * CoM sits at (0.3063, 0.4075), so the box needs an offset on both axes.
   * The bottom face lands exactly on the measured 57.5 mm of ground clearance
   * so the underbody grounds out on kerbs instead of hovering or sinking.
   */
  hullHalf: { x: 2.324, y: 0.728, z: 0.950 },
  hullOffsetX: -0.190,
  hullOffsetY: 0.2855,
  restitution: 0.22,
  hullFriction: 0.35,
};

/* ========================================================================= */
/* Suspension - independent at all four corners                               */
/* ========================================================================= */

const staticFrontN = (CHASSIS.mass * 9.80665 * (AXLE.b / AXLE.wheelbase)) / 2;
const staticRearN = (CHASSIS.mass * 9.80665 * (AXLE.a / AXLE.wheelbase)) / 2;

const SUSPENSION = {
  front: {
    /** Wheel rate (N/m). 34 kN/m over a 447 kg corner ~ 1.39 Hz ride freq. */
    springRate: 34000,
    /** Damping coefficients (N per m/s). ~0.28 / 0.50 of critical. */
    bumpDamping: 2250,
    reboundDamping: 3950,
    /** Travel about the design ride height (m). */
    maxCompression: 0.090,
    maxDroop: 0.100,
    /** Progressive bump stop: engages over the last `bumpStopZone` of travel. */
    bumpStopZone: 0.028,
    bumpStopRate: 260000,
    /** Anti-roll bar rate, N per m of left/right travel difference. */
    antiRollRate: 17000,
    staticLoad: staticFrontN,
    /** Static camber / toe / caster of the steering axis. */
    camber: -0.8 * DEG2RAD,
    toe: 0.05 * DEG2RAD,
    caster: 6.5 * DEG2RAD,
    /** Lateral offset of the steering axis at the hub (scrub radius, m). */
    scrubRadius: 0.012,
  },
  rear: {
    springRate: 39000,
    bumpDamping: 2450,
    reboundDamping: 4200,
    maxCompression: 0.085,
    maxDroop: 0.095,
    bumpStopZone: 0.026,
    bumpStopRate: 290000,
    antiRollRate: 12000,
    staticLoad: staticRearN,
    camber: -1.6 * DEG2RAD,
    toe: 0.12 * DEG2RAD,
    caster: 0,
    scrubRadius: 0,
  },
  /** Unsprung mass per corner (wheel + hub + brake), kg. */
  unsprungMass: 42,
};

/* ========================================================================= */
/* Tyres                                                                      */
/* ========================================================================= */

const TYRE = {
  radius: MODEL.tyreRadius,
  width: MODEL.tyreWidth,
  /** Rotational inertia of wheel + tyre + disc, kg.m^2 (~0.55 m r^2). */
  inertia: 1.28,

  /** Nominal cold pressure, PSI. */
  nominalPressure: 32,
  minPressure: 12,
  maxPressure: 50,

  /**
   * Magic-Formula shape constants, expressed on NORMALISED slip so the same
   * curve serves longitudinal and lateral and combines cleanly on the friction
   * ellipse. B is solved so the peak lands exactly at normalised slip = 1.
   */
  /**
   * B is solved so the peak lands exactly at normalised slip = 1, given C and
   * E:  (1-E)*B + E*atan(B) = tan(pi/(2C)).
   * E controls the TAIL. At E = 0.9 the curve barely falls away, so a fully
   * locked tyre would still make 82 % of peak force and locking the wheels
   * would cost almost nothing - which is wrong, and would make ABS pointless.
   * E = 0.50 (with B re-solved to 1.74) gives 68 % at full lock, which is where
   * a real tyre sits.
   */
  mfB: 1.74,
  mfC: 1.65,
  mfE: 0.50,

  /** Peak slip ratio / slip angle at nominal pressure and nominal load. */
  peakSlipRatio: 0.115,
  peakSlipAngle: 9.2 * DEG2RAD,

  /** Base friction coefficient on dry asphalt at the reference load. */
  muNominal: 1.25,
  /** Reference vertical load for load sensitivity (N). */
  refLoad: 4200,
  /** mu falls this fraction per unit of (Fz/refLoad - 1). */
  loadSensitivity: 0.115,

  /** Transient (relaxation-length) slip filtering - metres. */
  relaxLongitudinal: 0.32,
  relaxLateral: 0.55,
  /** Longitudinal speed floor used in the slip denominators (m/s). */
  slipSpeedFloor: 1.6,

  /** Radial (vertical) stiffness at nominal pressure, N/m. */
  radialStiffness: 195000,
  /** Extra radial stiffness per PSI above nominal. */
  radialStiffnessPerPsi: 4600,
  /** Radial damping, N per m/s. */
  radialDamping: 520,
  /** Effective rolling radius loses this fraction of the radial deflection. */
  effectiveRadiusFactor: 0.68,

  /** Rolling resistance coefficient at nominal pressure. */
  rollingResistance: 0.0115,

  /** Pneumatic trail (m) - drives self-aligning torque and steering feel. */
  pneumaticTrail: 0.030,

  /** Simple thermal model: grip peaks at optimum temperature. */
  ambientTemp: 22,
  optimumTemp: 82,
  /** Temperature the tyres spawn at - warm, as if just off a warm-up lap. */
  startTemp: 72,
  tempGripFalloff: 0.000080,   // mu lost per (deg C)^2 away from optimum
  tempHeatRate: 0.00024,       // deg C per Joule of friction work
  /**
   * Convective cooling, deg C per second per deg C above ambient. Tuned with
   * the carcass-hysteresis heating so a steady cruise settles around 45-50 C
   * and a hard stint climbs into the 80-100 C working range.
   */
  tempCoolRate: 0.0075,
  tempCoolSpeedFactor: 0.030,

  /** Visual deformation limits (metres) fed to the tyre shader. */
  maxVisualDeflection: 0.042,
  sidewallBulge: 0.55,
};


/* ========================================================================= */
/* Drivetrain - rear-wheel drive, turbo inline-6, 8-speed automatic           */
/* ========================================================================= */

const ENGINE = {
  /** Full-load torque curve [rpm, Nm]. B58-like: 500 Nm plateau 1800-5000. */
  torqueCurve: [
    [700, 150], [1000, 260], [1400, 400], [1800, 500], [2500, 505],
    [3500, 505], [4500, 500], [5000, 492], [5500, 462], [6000, 420],
    [6500, 366], [7000, 300],
  ],
  idleRpm: 780,
  stallRpm: 480,
  redlineRpm: 6750,
  limiterRpm: 6900,
  limiterCutTime: 0.06,
  maxRpm: 7200,
  /** Crank + flywheel + clutch inertia, kg.m^2. */
  inertia: 0.235,
  /** Internal friction / pumping torque: a + b * rpm  (Nm). */
  frictionBase: 14,
  frictionPerRpm: 0.0092,
  /** Engine braking is stronger with a closed throttle. */
  overrunFrictionScale: 1.85,
  /** Turbo lag: torque approaches the steady-state curve with this time const. */
  boostBuildTime: 0.28,
  boostDecayTime: 0.16,
  /** Throttle plate response (s to travel 0 -> 1). */
  throttleResponseTime: 0.09,
  /** Idle governor proportional gain (throttle per rpm of error). */
  idleGain: 0.00085,
  /** Integral gain of the idle governor (throttle per rpm-second of error). */
  idleIntegralGain: 0.0016,
};

/**
 * Torque converter (the car is an 8-speed automatic, not a manual).
 *
 * `capacityFactor` K is in the usual engineering units of rpm / sqrt(N.m):
 * the pump absorbs (rpm / K)^2. K is small at stall so the converter loads the
 * engine hard and multiplies the torque by `torqueRatio`; K climbs steeply as
 * the turbine catches up, so transmitted torque collapses to the engine's own
 * output. With these numbers the stall speed works out at about 2650 rpm.
 */
const CONVERTER = {
  torqueRatio: [
    [0.00, 2.00], [0.20, 1.75], [0.40, 1.52], [0.60, 1.30],
    [0.80, 1.10], [0.90, 1.03], [0.95, 1.00], [1.00, 1.00],
  ],
  capacityFactor: [
    [0.00, 118], [0.20, 122], [0.40, 132], [0.60, 152],
    [0.80, 205], [0.90, 300], [0.95, 470], [1.00, 2000],
  ],
  /** Sanity clamp on the pump torque, N.m. */
  maxPumpTorque: 900,
  /** Above this speed ratio the turbine is driving the pump (overrun). */
  coastRatio: 1.0,
  /** Overrun coupling stiffness, N.m per rad/s. Deliberately weak. */
  coastCoupling: 1.1,
  /** Pumping drag in neutral, N.m at 1000 rpm (scales with rpm^2). */
  neutralDrag: 2.5,

  /**
   * Lock-up clutch schedule, with hysteresis so it cannot chatter.
   * Under full load the converter settles around a 0.88 speed ratio (that is
   * where pump absorption balances engine output), so the engage threshold has
   * to sit below that or the clutch would never close and the car would drag a
   * permanent 10 % slip - and the accompanying torque loss - all the way down
   * the straight.
   */
  lockupEngageRatio: 0.80,
  lockupReleaseRatio: 0.66,
  lockupMinSpeed: 4.0,
};

const TRANSMISSION = {
  /** ZF 8HP-style ratios. */
  gearRatios: [4.71, 3.14, 2.11, 1.67, 1.29, 1.00, 0.84, 0.67],
  reverseRatio: 3.30,
  finalDrive: 3.15,
  efficiency: 0.925,
  /** Gearbox + driveshaft inertia referred to the input shaft. */
  inertia: 0.035,

  /** Automatic shift logic. */
  autoShift: true,
  upshiftRpm: 6350,          // at full throttle
  upshiftRpmLight: 2450,     // at closed throttle (cruising)
  downshiftRpm: 1450,
  kickdownRpm: 5400,
  shiftTime: 0.16,           // torque interrupt duration
  shiftCooldown: 0.50,
  /** Below this speed with brake held, S selects reverse. */
  reverseSpeed: 0.7,
};

const DIFFERENTIAL = {
  type: 'lsd',
  /**
   * Clutch-pack LSD. Its locking CAPACITY is a fixed preload plus a share of
   * the torque passing through it (the ramp angle) - it is not a damper, so it
   * is expressed as a torque limit rather than a N.m-per-rad/s coefficient.
   */
  preload: 70,
  torqueBias: 0.30,
  maxLockTorque: 900,
};

/* ========================================================================= */
/* Brakes                                                                     */
/* ========================================================================= */

const BRAKES = {
  /** Maximum clamping torque per wheel at full pedal, Nm. */
  maxTorqueFront: 2700,
  maxTorqueRear: 1520,
  /** Handbrake acts on the rear axle only. */
  handbrakeTorque: 2350,
  /** Pedal travel response (s from 0 to full). */
  pedalResponseTime: 0.10,

  abs: {
    enabled: true,
    /**
     * The regulation band straddles the tyre's peak braking slip
     * (-TYRE.peakSlipRatio = -0.115). Releasing only once the wheel is well
     * past the peak, as a naive setting does, wastes most of the available
     * grip: the wheel spends its time on the falling side of the curve.
     */
    targetSlip: -0.100,
    /**
     * The pump's own actuation rate, Hz. The controller runs on this clock
     * rather than on the physics step, and simply HOLDS pressure in between -
     * which is what the real hardware does, and what keeps the braking model
     * free of any timestep dependence.
     */
    updateHz: 100,
    /** Proportional gains, in pressure-per-second per unit of slip error.
     *  Release is deliberately far quicker than re-apply. */
    releaseRate: 95.0,
    applyRate: 26.0,
    /** ABS is inhibited below this speed so the car can actually come to rest. */
    minSpeed: 2.2,
    /** Pressure is never released completely, or the car would not slow at all. */
    minPressure: 0.22,
  },

  tractionControl: {
    enabled: true,
    /**
     * Target slip sits just BELOW the tyre's peak (TYRE.peakSlipRatio = 0.115).
     * Aiming past the peak would hold the tyre on the falling side of its
     * curve, where more slip means less force - the opposite of what traction
     * control is for.
     */
    targetSlip: 0.105,
    cutRate: 6.0,
    restoreRate: 5.0,
    /** Throttle multiplier at full intervention (never a complete cut). */
    minThrottleCut: 0.25,
    /** Slip error over which the cut ramps to full authority. */
    slipBand: 0.09,
  },

  stabilityControl: {
    enabled: false,
    yawGain: 0.55,
    maxBrakeTorque: 900,
  },
};

/* ========================================================================= */
/* Steering                                                                   */
/* ========================================================================= */

const STEERING = {
  /** Maximum road-wheel angle at the inner wheel, radians. */
  maxAngle: 33 * DEG2RAD,
  /** Rack speed: radians of road-wheel angle per second. */
  rackSpeed: 2.05,
  /** Faster return-to-centre when the input is released. */
  returnSpeed: 3.30,
  /** Input smoothing so a digital key press is not a step input. */
  inputSmoothTime: 0.085,
  /**
   * Speed-sensitive steering: the available lock is scaled from 1.0 at rest
   * down to `highSpeedScale` at `highSpeed`.
   */
  highSpeed: 58,               // m/s (~209 km/h)
  highSpeedScale: 0.30,
  /** Extra self-centring torque from the tyres' aligning moment. */
  aligningGain: 0.055,
  aligningMax: 0.42,
  /** Ackermann blend: 1 = full geometric Ackermann, 0 = parallel steer. */
  ackermann: 0.78,
  /** Counter-steer assist during large rear slip (0 disables it). */
  counterSteerAssist: 0.0,
};

/* ========================================================================= */
/* Presentation: garage card, camera framing and audio character              */
/* ========================================================================= */

const GARAGE = {
  manufacturer: 'BMW',
  model: 'M3 GTR',
  year: 2004,
  tagline: 'Rear-drive and naturally balanced, with nothing hiding the limit.',
  bodyStyle: 'Coupe',
  colour: '#dbe3ec',
  accent: '#4fb3ff',
  /**
   * Quoted figures, all MEASURED by tests/run.mjs on the flat skidpad fixture
   * rather than typed in by hand - so the garage can never quote a number the
   * car cannot actually do.
   */
  quoted: {
    zeroToHundred: 6.6,
    brakingFrom100: 35.0,
    lateralG: 1.22,
  },
};

const CAMERA_PROFILE = {
  chase: { x: -7.1, y: 2.55, z: 0.0 },
  chaseLookAt: { x: 1.4, y: 0.55, z: 0.0 },
  side: { x: -5.0, y: 1.55, z: -5.6 },
  sideLookAt: { x: 0.4, y: 0.45, z: 0.0 },
  hood: { x: 0.28, y: 0.78, z: 0.0 },
  hoodLookAt: { x: 12.0, y: 0.60, z: 0.0 },
  /**
   * The driver's eye. This car sits 0.50 m of ride height under its CoM and a
   * racing seat puts the eye around 1.13 m above the road, so +0.63 from the
   * body origin. Left-hand drive, and further back than the Lamborghini's
   * because a saloon-based shell has its screen much further forward.
   */
  interior: { x: -0.26, y: 0.66, z: -0.36 },
  interiorLookAt: { x: 12.0, y: 0.46, z: -0.26 },
  orbit: { x: -8.5, y: 3.2, z: 0.0 },
  orbitLookAt: { x: 0.0, y: 0.45, z: 0.0 },
};

/**
 * Audio character. A turbocharged inline-six: even-firing, a strong second
 * order, comparatively smooth, and with obvious turbo content.
 */
const AUDIO = {
  /** [harmonic order relative to firing frequency, gain, waveform]. */
  harmonics: [
    [0.5, 0.20, 'sine'],
    [1.0, 0.55, 'sawtooth'],
    [2.0, 0.42, 'sawtooth'],
    [3.0, 0.18, 'square'],
    [4.0, 0.09, 'sine'],
  ],
  cylinders: 6,
  baseGain: 0.26,
  filterClosed: 700,
  filterOpen: 3400,
  turbo: { enabled: true, gain: 0.055, baseFreq: 1500, freqPerBoost: 5200 },
  noiseGain: 0.055,
  detune: 6,
  shiftBlip: 0.30,
};

/* ========================================================================= */

export const bmwSpec = buildVehicleSpec({
  id: 'bmw-m3-gtr',
  displayName: 'BMW M3 GTR',
  shortName: 'BMW',
  model: MODEL,
  axleOverride: AXLE,
  chassis: CHASSIS,
  suspension: SUSPENSION,
  tyre: TYRE,
  engine: ENGINE,
  converter: CONVERTER,
  transmission: TRANSMISSION,
  differential: DIFFERENTIAL,
  brakes: BRAKES,
  steering: STEERING,
  garage: GARAGE,
  cameraProfile: CAMERA_PROFILE,
  audio: AUDIO,
});

export default bmwSpec;
