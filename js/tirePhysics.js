/**
 * tirePhysics.js
 * ---------------------------------------------------------------------------
 * Tyre model: pressure, temperature, load sensitivity, transient (relaxation
 * length) slip, and a normalised Magic-Formula that produces combined
 * longitudinal + lateral force on a proper friction ellipse.
 *
 * Requirements covered: #9 (slip), #17 (contact patch), #18 (pressure is not
 * cosmetic), #19 (load/pressure driven deformation), #20 (friction), #21
 * (lateral behaviour).
 *
 * Sign conventions, all in the contact-patch frame:
 *   +x   the direction the wheel points (its rolling direction)
 *   +y   to the right of +x, in the contact plane
 *   kappa > 0  wheel surface faster than the road  -> drive / wheelspin
 *   kappa < 0  wheel surface slower than the road  -> braking slip / lock
 *   alpha      SAE slip angle, negative when the patch slides toward +y
 * ---------------------------------------------------------------------------
 */

import { clamp, clamp01, sq } from './mathUtils.js';

/**
 * Normalised Magic Formula. Input is slip already divided by its peak value,
 * so mf(1) == 1 and the curve falls away beyond the peak exactly as a real
 * tyre does. B is tuned in config so the maximum genuinely lands at s = 1.
 */
export function magicFormula(s, tyre) {
  const B = tyre.mfB, C = tyre.mfC, E = tyre.mfE;
  const bs = B * s;
  return Math.sin(C * Math.atan(bs - E * (bs - Math.atan(bs))));
}

/**
 * One tyre. Owns its own pressure and temperature; four independent
 * instances exist per vehicle so a single soft tyre changes that corner only.
 */
export class Tire {
  /**
   * @param {string} label  'FL' | 'FR' | 'RL' | 'RR'
   * @param {object} tyre   the vehicle's tyre block (see js/vehicles/*.js)
   * @param {number} radius this corner's free radius - a staggered car runs a
   *                        different one on each axle
   */
  constructor(label, tyre, radius) {
    this.label = label;
    /** The vehicle-specific tyre tuning. Never a module-level constant. */
    this.spec = tyre;
    /** Free (unloaded) radius of THIS tyre, metres. */
    this.radius = radius;

    /** Cold inflation pressure in PSI. Fully live - see updateDerived(). */
    this.pressure = tyre.nominalPressure;
    /** Bulk carcass temperature, deg C. Starts warm, as if out of the pits. */
    this.temperature = tyre.startTemp;

    /* --- transient slip state (persist across steps) --------------------- */
    /** Relaxed longitudinal slip ratio. */
    this.slipRatio = 0;
    /** Relaxed lateral contact-patch velocity, m/s. */
    this.lateralVelLag = 0;
    /** Slip angle derived from lateralVelLag, radians. */
    this.slipAngle = 0;

    /* --- last-step outputs (read by HUD / debug / visuals) --------------- */
    this.longForce = 0;
    this.latForce = 0;
    this.alignTorque = 0;
    /** Fraction of the available friction circle in use, 0..1+. */
    this.frictionUse = 0;
    /** Normalised combined slip; > 1 means past the grip peak (sliding). */
    this.combinedSlip = 0;
    /** Radial squash of the carcass, metres. Drives the deformation shader. */
    this.deflection = 0;
    this.effectiveRadius = radius;
    this.mu = tyre.muNominal;
    /** True while the contact patch is genuinely sliding (for smoke/skid). */
    this.sliding = false;
    this.slideSpeed = 0;

    this.updateDerived();
  }

  /**
   * Recompute everything that depends on pressure. Called whenever the
   * pressure changes (inflation UI, punctures, or once per step - it is cheap).
   */
  updateDerived() {
    const TYRE = this.spec;
    const p = clamp(this.pressure, TYRE.minPressure, TYRE.maxPressure);
    const ratio = TYRE.nominalPressure / p;

    /**
     * A softer tyre needs MORE slip before it reaches peak force: the carcass
     * winds up before the patch does. That is exactly why an under-inflated
     * car steers vaguely (requirement #18).
     */
    const slipScale = Math.pow(ratio, 0.35);
    this.peakSlipRatio = TYRE.peakSlipRatio * slipScale;
    this.peakSlipAngle = TYRE.peakSlipAngle * slipScale;

    /** Grip peaks slightly above the nominal pressure and falls away either side. */
    this.pressureGrip = clamp(1 - 0.00045 * sq(p - 34), 0.74, 1.0);

    /** Vertical carcass stiffness rises with pressure. */
    this.radialStiffness = Math.max(
      60000,
      TYRE.radialStiffness + TYRE.radialStiffnessPerPsi * (p - TYRE.nominalPressure),
    );

    /** A soft tyre has a bigger footprint and scrubs more. */
    this.rollingResistance = TYRE.rollingResistance
      * clamp(1 + 0.030 * (TYRE.nominalPressure - p), 0.55, 3.0);

    /**
     * Lateral carcass compliance, used for the visual sidewall lean. A soft
     * sidewall lets the rim roll over on the tread.
     */
    this.sidewallCompliance = clamp(ratio, 0.55, 2.2);
  }

  /** Grip multiplier from carcass temperature (cold and overheated both lose). */
  temperatureGrip() {
    const TYRE = this.spec;
    return clamp(1 - TYRE.tempGripFalloff * sq(this.temperature - TYRE.optimumTemp), 0.55, 1.0);
  }

  /**
   * Vertical (radial) behaviour. Returns the carcass squash for a given load,
   * and updates the effective rolling radius.
   *
   * Requirement #19: the deformation is a function of load and pressure, not a
   * scripted animation.
   */
  updateVertical(normalForce, normalRate = 0) {
    const TYRE = this.spec;
    const raw = (normalForce + TYRE.radialDamping * normalRate) / this.radialStiffness;
    this.deflection = clamp(raw, 0, TYRE.maxVisualDeflection * 1.4);
    // A loaded tyre rolls on a radius between its free radius and the hub
    // height; ~2/3 of the squash is recovered by the belt tension.
    this.effectiveRadius = Math.max(
      0.15,
      this.radius - this.deflection * TYRE.effectiveRadiusFactor,
    );
    return this.deflection;
  }

  /**
   * Current friction coefficient for a given vertical load and surface.
   * Load sensitivity: a tyre carrying twice the load does NOT make twice the
   * grip, which is where lateral weight transfer costs total axle grip.
   */
  frictionCoefficient(normalForce, surface) {
    const TYRE = this.spec;
    const loadRatio = normalForce / TYRE.refLoad;
    const loadTerm = clamp(1 - TYRE.loadSensitivity * (loadRatio - 1), 0.45, 1.35);
    this.mu = TYRE.muNominal * loadTerm * surface.grip
      * this.pressureGrip * this.temperatureGrip();
    return this.mu;
  }

  /**
   * Advance the transient slip states.
   *
   *   sigma * d(kappa)/dt + |vx| * kappa = omega*Re - vx
   *
   * The relaxation length keeps the model finite at standstill (where the
   * steady-state definition divides by zero) and gives the tyre the small
   * lag that makes weight transfer and countersteer feel right.
   */
  updateSlip(vLong, vLat, spinVelocity, dt) {
    const TYRE = this.spec;
    const vFloor = Math.max(Math.abs(vLong), TYRE.slipSpeedFloor);
    const surfaceSpeed = spinVelocity * this.effectiveRadius;

    /**
     * Both relaxation equations are integrated IMPLICITLY.
     *
     * The explicit form, kappa += dt*(u - kappa*|vx|)/sigma, has a decay rate
     * of |vx|/sigma. At 40 m/s with a 0.32 m relaxation length that is 125 per
     * second, so a 60 Hz step gives dt*rate = 2.08 - past the explicit
     * stability limit of 2, and the slip state rings and then diverges. It is
     * the one genuinely stiff term in the whole model, and it is why a naive
     * implementation only behaves at high physics rates.
     *
     * Rearranging for kappa at the END of the step costs one divide and is
     * unconditionally stable at any timestep:
     *
     *     kappa_new = (kappa + dt*u/sigma) / (1 + dt*|vx|/sigma)
     */
    const kRate = vFloor / TYRE.relaxLongitudinal;
    const kTarget = surfaceSpeed - vLong;
    this.slipRatio = clamp(
      (this.slipRatio + (dt * kTarget) / TYRE.relaxLongitudinal) / (1 + dt * kRate),
      -8, 8,
    );

    // Lateral: lag the patch velocity, then convert to a slip angle.
    const lRate = vFloor / TYRE.relaxLateral;
    this.lateralVelLag = (this.lateralVelLag + dt * lRate * vLat) / (1 + dt * lRate);
    this.slipAngle = Math.atan2(-this.lateralVelLag, vFloor);

    // True sliding velocity of the rubber over the road (for skids and heat).
    this.slideSpeed = Math.hypot(surfaceSpeed - vLong, vLat);
  }

  /**
   * Combined-slip tyre force.
   *
   * Both slips are normalised by their own peak, combined on a circle, run
   * through one Magic Formula, then split back along the slip direction. That
   * single step gives: peak force at the peak slip, progressive break-away
   * past it, and a proper friction ellipse where longitudinal force steals
   * from lateral force (and vice versa).
   *
   * Writes longForce / latForce / alignTorque and returns nothing.
   */
  computeForces(normalForce, surface) {
    const TYRE = this.spec;
    if (normalForce <= 1) {
      this.longForce = 0;
      this.latForce = 0;
      this.alignTorque = 0;
      this.frictionUse = 0;
      this.combinedSlip = 0;
      this.sliding = false;
      return;
    }

    const mu = this.frictionCoefficient(normalForce, surface);
    const fMax = mu * normalForce;

    const sx = this.slipRatio / this.peakSlipRatio;
    const sy = Math.tan(this.slipAngle) / Math.tan(this.peakSlipAngle);
    const s = Math.hypot(sx, sy);
    this.combinedSlip = s;

    if (s < 1e-6) {
      this.longForce = 0;
      this.latForce = 0;
      this.alignTorque = 0;
      this.frictionUse = 0;
      this.sliding = false;
      return;
    }

    const fMag = fMax * magicFormula(s, TYRE);
    this.longForce = (sx / s) * fMag;
    this.latForce = (sy / s) * fMag;
    this.frictionUse = clamp01(fMag / fMax);

    /**
     * Self-aligning torque: the lateral force acts behind the wheel centre by
     * the pneumatic trail, and the trail collapses as the tyre saturates -
     * which is the physical origin of steering going light at the limit.
     */
    const trail = TYRE.pneumaticTrail * clamp(1 - 0.85 * clamp01(s), 0.05, 1);
    this.alignTorque = -this.latForce * trail;

    // Past the peak of the curve the patch is sliding rather than gripping.
    this.sliding = s > 1.05 && this.slideSpeed > 1.2;
  }

  /**
   * Thermal model. Friction work heats the carcass, convection cools it.
   * Called once per step with the sliding velocities in the patch frame.
   */
  updateTemperature(vLong, vLat, spinVelocity, dt, speed, normalForce, surface) {
    const TYRE = this.spec;
    // Friction work in the contact patch: the dominant term when sliding.
    const slipVelX = spinVelocity * this.effectiveRadius - vLong;
    let power = Math.abs(this.longForce * slipVelX) + Math.abs(this.latForce * vLat);

    /**
     * Hysteresis in the carcass. A rolling tyre is continuously flexed, and
     * that loss is what keeps tyres warm while merely cruising. Without it the
     * only heat source is sliding, so a straight-line cruise cools the tyres to
     * ambient and quietly costs 20 % of the grip - which is not what happens.
     */
    if (normalForce > 1) {
      power += this.rollingResistance * surface.roll * normalForce * Math.abs(vLong);
    }

    const cooling = (this.temperature - TYRE.ambientTemp)
      * TYRE.tempCoolRate * (1 + speed * TYRE.tempCoolSpeedFactor);
    this.temperature += (power * TYRE.tempHeatRate - cooling) * dt;
    this.temperature = clamp(this.temperature, TYRE.ambientTemp - 5, 190);
  }

  /** Rolling-resistance torque opposing rotation, N.m. */
  rollingResistanceTorque(normalForce, surface, spinVelocity) {
    if (normalForce <= 1) return 0;
    const c = this.rollingResistance * surface.roll;
    // Blend the sign smoothly through zero so a parked car does not buzz.
    const dir = Math.tanh(spinVelocity * 3.0);
    return c * normalForce * this.effectiveRadius * dir;
  }

  /** Reset transient state (used by the R key). */
  reset() {
    const TYRE = this.spec;
    this.slipRatio = 0;
    this.lateralVelLag = 0;
    this.slipAngle = 0;
    this.longForce = 0;
    this.latForce = 0;
    this.alignTorque = 0;
    this.frictionUse = 0;
    this.combinedSlip = 0;
    this.deflection = 0;
    this.slideSpeed = 0;
    this.sliding = false;
    this.temperature = TYRE.startTemp;
    this.effectiveRadius = this.radius;
    this.updateDerived();
  }
}
