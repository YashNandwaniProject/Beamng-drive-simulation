/**
 * config.js
 * ---------------------------------------------------------------------------
 * WORLD-level settings: simulation rate, surfaces, camera rig, spawn point and
 * renderer options. These are shared by every vehicle.
 *
 * Per-vehicle tuning does NOT live here. Each car owns a complete specification
 * under js/vehicles/ (bmw.js, lamborghini.js), assembled by js/vehicles/spec.js
 * and looked up by id through js/vehicles/registry.js. The physics engine is
 * common; the parameters are per-vehicle.
 *
 * PHYSICS BODY FRAME (the same for every car, whatever its model's own axes):
 *
 *        +Y  up
 *         |
 *         |
 *         o------ +Z  right (passenger side)
 *        /
 *      +X  forward (nose)
 *
 * ---------------------------------------------------------------------------
 */

/* ========================================================================= */
/* Simulation                                                                 */
/* ========================================================================= */

export const SIM = {
  /**
   * Fixed physics rate. Requirement #43 asks for a fixed timestep decoupled
   * from render FPS; 240 Hz (not 60) is used because the wheel rotational
   * dynamics are stiff: 1st gear puts ~6800 Nm through a 1.25 kg.m^2 wheel,
   * which is 5400 rad/s^2. At 60 Hz that is a 90 rad/s jump per step and the
   * slip solver rings. At 240 Hz the same event is a well-behaved 22 rad/s.
   * Rendering still runs free at 60/120/144 Hz.
   */
  physicsHz: 240,
  /** Never simulate more than this much wall time in one rAF callback. */
  maxFrameTime: 0.25,
  /** Hard cap on substeps per frame, prevents the spiral of death. */
  maxSubSteps: 24,
};

/**
 * Surface properties. `grip` multiplies mu, `roll` multiplies rolling
 * resistance, `roughness` scales the procedural micro-bumps under the wheel.
 */
export const SURFACES = {
  ASPHALT: { id: 0, name: 'Asphalt', grip: 1.00, roll: 1.00, roughness: 0.008, drag: 0.0 },
  WET: { id: 1, name: 'Wet asphalt', grip: 0.68, roll: 1.10, roughness: 0.008, drag: 0.0 },
  CONCRETE: { id: 2, name: 'Concrete', grip: 0.94, roll: 1.05, roughness: 0.012, drag: 0.0 },
  RUMBLE: { id: 3, name: 'Kerb', grip: 0.88, roll: 1.30, roughness: 0.050, drag: 0.0 },
  GRAVEL: { id: 4, name: 'Gravel', grip: 0.58, roll: 2.60, roughness: 0.030, drag: 260 },
  DIRT: { id: 5, name: 'Dirt', grip: 0.62, roll: 2.20, roughness: 0.035, drag: 210 },
  GRASS: { id: 6, name: 'Grass', grip: 0.48, roll: 3.20, roughness: 0.040, drag: 380 },
};
export const SURFACE_BY_ID = Object.values(SURFACES).sort((a, b) => a.id - b.id);

/* ========================================================================= */
/* Camera                                                                     */
/* ========================================================================= */

export const CAMERA = {
  near: 0.15,
  far: 4000,
  baseFov: 62,
  /** FOV widens with speed for a sense of pace. */
  fovPerSpeed: 0.30,
  maxFovGain: 15,

  modes: {
    chase: {
      name: 'Chase',
      offset: { x: -7.1, y: 2.55, z: 0.0 },     // body-space, +X is forward
      lookAt: { x: 1.4, y: 0.55, z: 0.0 },
      positionSmooth: 0.115,
      rotationSmooth: 0.135,
      /** Blend between body yaw and velocity heading (1 = pure velocity). */
      velocityBias: 0.30,
      gForceGain: { x: 0.30, y: 0.16, z: 0.24 },
      rollGain: 0.10,
    },
    side: {
      name: 'Side chase',
      offset: { x: -5.0, y: 1.55, z: -5.6 },
      lookAt: { x: 0.4, y: 0.45, z: 0.0 },
      positionSmooth: 0.20,
      rotationSmooth: 0.16,
      velocityBias: 0.55,
      gForceGain: { x: 0.22, y: 0.14, z: 0.38 },
      rollGain: 0.16,
    },
    hood: {
      name: 'Bonnet',
      offset: { x: 0.28, y: 0.78, z: 0.0 },
      lookAt: { x: 12.0, y: 0.60, z: 0.0 },
      positionSmooth: 0.02,
      rotationSmooth: 0.05,
      velocityBias: 0.0,
      gForceGain: { x: 0.055, y: 0.05, z: 0.05 },
      rollGain: 0.35,
      /** Bolted to the body: inherits its full attitude, pitch and roll. */
      rigid: true,
    },
    interior: {
      name: 'Interior',
      /**
       * The driver's eye. Both offsets are overridden per vehicle - a
       * Revuelto's seat is 30 cm lower than an M3's - and are measured from
       * the model rather than guessed; see each car's CAMERA_PROFILE.
       */
      offset: { x: 0.05, y: 0.60, z: -0.36 },
      lookAt: { x: 12.0, y: 0.35, z: -0.30 },
      /**
       * Tighter than the bonnet camera. A head does move a little inside a
       * car, but only a little, and any lag here reads as the whole cabin
       * sloshing about.
       */
      positionSmooth: 0.012,
      rotationSmooth: 0.035,
      velocityBias: 0.0,
      gForceGain: { x: 0.035, y: 0.030, z: 0.045 },
      rollGain: 0.45,
      rigid: true,
      /**
       * A cockpit view is close work: the wheel rim is around 40 cm from the
       * eye and the dash top around 70 cm, so the shared 0.15 m near plane
       * would clip neither - but the FOV matters. A narrower one keeps the
       * instrument binnacle the size it looks through a real windscreen.
       */
      fov: 55,
      /** No speed-FOV: it makes a fixed cockpit feel like it is breathing. */
      fovGain: 0.25,
    },
    orbit: {
      name: 'Orbit',
      offset: { x: -8.5, y: 3.2, z: 0.0 },
      lookAt: { x: 0.0, y: 0.45, z: 0.0 },
      positionSmooth: 0.25,
      rotationSmooth: 0.25,
      velocityBias: 0.0,
      gForceGain: { x: 0.0, y: 0.0, z: 0.0 },
      rollGain: 0.0,
      orbitSpeed: 0.16,
    },
  },
  order: ['chase', 'side', 'hood', 'interior', 'orbit'],

  /** Camera shake from vertical G (impacts, kerbs). */
  shakeGainVertical: 0.020,
  shakeDecay: 0.12,
  maxShake: 0.30,
  /** Keep the camera above the terrain by at least this much. */
  groundClearance: 0.45,
};

/* ========================================================================= */
/* Spawn                                                                      */
/* ========================================================================= */

export const SPAWN = {
  /** Distance along the track centreline (metres) of the start line. */
  trackDistance: 60,
  /** Lateral offset from the centreline, +ve = right of travel. */
  lateralOffset: -2.1,
  /** Extra height above the settled ride height when respawning. */
  dropHeight: 0.10,
};

export const RENDER = {
  shadowMapSize: 2048,
  /** Cascade-free single directional shadow: box half-size around the car. */
  shadowRadius: 46,
  maxPixelRatio: 1.75,
  toneMappingExposure: 1.02,
  /** Environment map resolution for the procedural sky IBL. */
  envMapSize: 256,

  /**
   * Real-time reflection probe (js/reflections.js). This is what gives the
   * bodywork and the road reflections of the WORLD rather than of a static
   * sky - see that file for what it does and does not do.
   *
   * `level` indexes REFLECTION_LEVELS: 0 off, 1 low, 2 high. It costs six
   * extra scene renders per capture, so it is throttled hard and can be cycled
   * at runtime with V.
   */
  reflections: {
    level: 2,
    /** Capture height above the car's centre of mass, in metres. */
    height: 0.85,
    far: 1600,
  },
  fogNear: 260,
  fogFar: 1500,
};
