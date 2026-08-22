/**
 * js/vehicles/spec.js
 * ---------------------------------------------------------------------------
 * Shared vehicle-specification builder.
 *
 * A vehicle spec is the ONLY thing that differs between cars. The physics
 * engine is common:
 *
 *      COMMON PHYSICS ENGINE      (vehiclePhysics, wheelPhysics, tirePhysics,
 *              |                   suspension, steering, drivetrain, brakes)
 *              v
 *      VEHICLE-SPECIFIC SPEC      (this file's output)
 *              v
 *      BMW M3 GTR / Lamborghini Revuelto
 *
 * Everything geometric is authored in MODEL SPACE - i.e. the numbers you get
 * by opening the .glb and measuring it - because those are the values a human
 * can re-verify against the file. This builder converts them into the physics
 * BODY FRAME, which is the same for every car no matter how its model was
 * authored:
 *
 *        +Y  up
 *         |
 *         |
 *         o------ +Z  right (passenger side)
 *        /
 *      +X  forward (nose)
 *
 * `modelYaw` is the rotation about +Y that carries the model's own forward
 * axis onto +X. The BMW was authored nose-along-+X so its yaw is 0; the
 * Lamborghini was authored nose-along-(-X) so its yaw is pi. Getting this
 * wrong is what makes a car drive backwards, so it is derived once, here, and
 * every downstream consumer works in the body frame only.
 * ---------------------------------------------------------------------------
 */

import { GRAVITY } from '../mathUtils.js';

/** Rotate a model-space point/vector by `yaw` about +Y into the body frame. */
export function modelToBody(v, yaw) {
  if (!yaw) return { x: v.x, y: v.y, z: v.z };
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  // Standard yaw about +Y:  x' = x*c + z*s,  z' = -x*s + z*c
  return { x: v.x * c + v.z * s, y: v.y, z: -v.x * s + v.z * c };
}

/**
 * Assemble a complete, frozen vehicle spec.
 *
 * @param {object} raw
 *   id, displayName, shortName
 *   model        measured model-space geometry (see the two vehicle files)
 *   axleOverride optional pre-computed AXLE block; the BMW supplies its
 *                original one verbatim so its numbers cannot drift by even a
 *                millimetre when this builder was introduced
 *   chassis, suspension, tyre, engine, converter, transmission,
 *   differential, brakes, steering, garage, cameraProfile, audio
 */
export function buildVehicleSpec(raw) {
  const m = raw.model;
  const yaw = m.modelYaw || 0;

  /* --- hub positions, model space -> body space, relative to the CoM ----- */
  const com = modelToBody({ x: m.comX, y: m.comY, z: m.comZ }, yaw);
  const hubs = {};
  for (const id of ['FL', 'FR', 'RL', 'RR']) {
    const h = modelToBody(m.hubs[id], yaw);
    hubs[id] = {
      x: h.x - com.x,
      /**
       * A car with staggered tyres (the Lamborghini runs 20" front / 21" rear)
       * has its axles at genuinely different heights. `uniformHubHeight` lets
       * a vehicle declare one design height for all four corners instead -
       * which is what the BMW did before this builder existed, so it keeps
       * that car bit-identical.
       */
      y: (m.uniformHubHeight !== undefined ? m.uniformHubHeight : h.y) - com.y,
      z: h.z - com.z,
    };
  }

  const a = hubs.FL.x;                      // CoM -> front axle
  const b = -hubs.RL.x;                     // CoM -> rear axle
  const wheelbase = a + b;
  const trackFront = hubs.FR.z - hubs.FL.z;
  const trackRear = hubs.RR.z - hubs.RL.z;

  const axle = raw.axleOverride || {
    a,
    b,
    wheelbase,
    trackFront,
    trackRear,
    /** Mean design hub height relative to the CoM (spawn placement uses it). */
    hubY: (hubs.FL.y + hubs.FR.y + hubs.RL.y + hubs.RR.y) / 4,
  };

  /* --- per-axle tyre radii ---------------------------------------------- */
  const radiusFront = m.tyreRadiusFront !== undefined ? m.tyreRadiusFront : m.tyreRadius;
  const radiusRear = m.tyreRadiusRear !== undefined ? m.tyreRadiusRear : m.tyreRadius;
  const widthFront = m.tyreWidthFront !== undefined ? m.tyreWidthFront : m.tyreWidth;
  const widthRear = m.tyreWidthRear !== undefined ? m.tyreWidthRear : m.tyreWidth;

  /**
   * Height of the CoM above the mean contact plane. Used to place the car at
   * spawn so all four tyres touch without a drop-in.
   */
  const rideHeight = ((-hubs.FL.y + radiusFront) + (-hubs.RL.y + radiusRear)) / 2;

  /* --- static corner loads ---------------------------------------------- */
  const weight = raw.chassis.mass * GRAVITY;
  const staticFront = (weight * (axle.b / axle.wheelbase)) / 2;
  const staticRear = (weight * (axle.a / axle.wheelbase)) / 2;

  /**
   * The BMW's suspension block carries its own pre-computed staticLoad values
   * (they were constants in config.js). Only fill them in when a vehicle has
   * not supplied them, so nothing is ever recomputed behind that car's back.
   */
  const suspension = {
    ...raw.suspension,
    front: {
      ...raw.suspension.front,
      staticLoad: raw.suspension.front.staticLoad ?? staticFront,
    },
    rear: {
      ...raw.suspension.rear,
      staticLoad: raw.suspension.rear.staticLoad ?? staticRear,
    },
  };

  const spec = {
    id: raw.id,
    displayName: raw.displayName,
    shortName: raw.shortName || raw.displayName,

    model: {
      ...m,
      modelYaw: yaw,
      /** CoM in body space - what the visual model is offset by. */
      comBody: com,
      radiusFront,
      radiusRear,
      widthFront,
      widthRear,
    },

    /** Body-space hub positions relative to the CoM, ready for the physics. */
    hubs,
    axle,
    rideHeight,

    chassis: raw.chassis,
    suspension,
    tyre: raw.tyre,
    engine: raw.engine,
    /** Exactly one of these is present, selected by transmission.coupling. */
    converter: raw.converter || null,
    clutch: raw.clutch || null,
    transmission: raw.transmission,
    differential: raw.differential,
    /** Electric drive axle, if the car has one (the Revuelto's front axle). */
    electric: raw.electric || null,
    brakes: raw.brakes,
    steering: raw.steering,

    garage: raw.garage,
    cameraProfile: raw.cameraProfile,
    audio: raw.audio,

    /** Which axle receives drive torque. */
    drive: raw.drivetrainLayout || 'rwd',
  };

  return spec;
}

/**
 * Per-wheel geometry the physics needs, resolved for one corner.
 * Keeps the "which radius does this wheel use" decision in one place.
 */
export function wheelGeometry(spec, id) {
  const isFront = id === 'FL' || id === 'FR';
  const t = spec.tyre;
  /** A staggered car has different rotating inertia on each axle. */
  const inertia = isFront
    ? (t.inertiaFront !== undefined ? t.inertiaFront : t.inertia)
    : (t.inertiaRear !== undefined ? t.inertiaRear : t.inertia);
  return {
    hubLocal: spec.hubs[id],
    isFront,
    isLeft: id === 'FL' || id === 'RL',
    radius: isFront ? spec.model.radiusFront : spec.model.radiusRear,
    width: isFront ? spec.model.widthFront : spec.model.widthRear,
    inertia,
    suspension: isFront ? spec.suspension.front : spec.suspension.rear,
    /** True when this wheel receives engine torque through the gearbox. */
    engineDriven: spec.drive === 'awd'
      ? !isFront                       // the Revuelto's engine drives the rear
      : (spec.drive === 'fwd' ? isFront : !isFront),
    /** True when this wheel receives torque from an electric motor. */
    electricDriven: !!(spec.electric && spec.electric.enabled
      && (spec.electric.axle === 'front' ? isFront : !isFront)),
  };
}
