/**
 * wheelPhysics.js
 * ---------------------------------------------------------------------------
 * One physical wheel: hub kinematics, ground contact, suspension coupling,
 * tyre force generation and rotational dynamics.
 *
 * Requirements covered: #4 (full wheel state), #9 (slip detection), #16/#17
 * (real ground contact, no floating and no sinking), #25 (steering angle kept
 * separate from rolling rotation), #26 (rotation driven by real angular
 * velocity), #28 (wheel follows the road normal).
 *
 * The wheel is a KINEMATIC point on the chassis plus one rotational degree of
 * freedom. It owns no linear state of its own: its position is dictated by the
 * chassis transform and the strut length, which is exactly what a raycast
 * suspension model is. Everything the tyre does is fed back to the chassis as
 * a force at the contact point.
 * ---------------------------------------------------------------------------
 */

import { Vec3, clamp, sign } from './mathUtils.js';
import { SurfaceSample } from './terrain.js';
import { Suspension } from './suspension.js';
import { Tire } from './tirePhysics.js';
import { SURFACES } from './config.js';

/** Minimum |dot(strutAxis, groundNormal)| before the solve is abandoned. */
const MIN_AXIS_DOT = 0.20;

export class Wheel {
  /**
   * @param {string} id       'FL' | 'FR' | 'RL' | 'RR'
   * @param {object} opts
   *   hubLocal        {x,y,z} hub position in body space, relative to the CoM
   *   isFront         steered axle?
   *   isLeft          left-hand side of the car?
   *   engineDriven    receives torque through the gearbox?
   *   electricDriven  receives torque from an electric motor?
   *   radius          free tyre radius for THIS corner (staggered cars differ)
   *   width           tread width
   *   inertia         rotational inertia of wheel + tyre + disc, kg.m^2
   *   suspension      the axle's suspension block
   *   tyre            the vehicle's tyre block
   */
  constructor(id, opts) {
    this.id = id;
    this.isFront = opts.isFront;
    this.isLeft = opts.isLeft;
    this.engineDriven = opts.engineDriven;
    this.electricDriven = !!opts.electricDriven;
    /** Kept for readability: does this wheel get driven at all? */
    this.driven = this.engineDriven || this.electricDriven;

    /** Per-corner geometry. A staggered car has a different tyre each end. */
    this.radius = opts.radius;
    this.width = opts.width;
    this.inertia = opts.inertia;

    /** Design-height hub position in body space, relative to the CoM. */
    this.hubLocal = new Vec3(opts.hubLocal.x, opts.hubLocal.y, opts.hubLocal.z);
    /** Strut top, fixed to the body: restLength above the design hub. */
    this.strutTopLocal = new Vec3();

    this.suspension = new Suspension(opts.suspension);
    this.tire = new Tire(id, opts.tyre, opts.radius);
    this.tyreSpec = opts.tyre;

    this.staticCamber = opts.suspension.camber;
    this.staticToe = opts.suspension.toe * (this.isLeft ? 1 : -1); // toe-in
    this.caster = opts.suspension.caster;

    this.strutTopLocal.set(
      this.hubLocal.x,
      this.hubLocal.y + this.suspension.restLength,
      this.hubLocal.z,
    );

    /* ---------------------------------------------------- rotational DOF */
    /** Wheel angular velocity about its spin axis, rad/s (+ = rolling forward). */
    this.spinVelocity = 0;
    /** Accumulated spin angle, radians. THE value the visual wheel follows. */
    this.spinAngle = 0;
    this.spinAcceleration = 0;
    /** Rotational inertia reflected from the drivetrain (set each step). */
    this.reflectedInertia = 0;

    /* ------------------------------------------------------------ inputs */
    /** Road-wheel steering angle, radians. POSITIVE = steering right (+Z). */
    this.steerAngle = 0;
    /** Torque from the gearbox (via the differential), N.m. */
    this.driveTorque = 0;
    /** Torque from an electric motor at this corner, N.m. */
    this.electricTorque = 0;
    this.brakeTorque = 0;

    /* ------------------------------------------------------------ outputs */
    this.hasContact = false;
    this.normalForce = 0;
    this.longForce = 0;
    this.latForce = 0;
    this.surface = SURFACES.ASPHALT;
    /** Load in G relative to this corner's static load - for the HUD. */
    this.loadRatio = 1;

    /* -------------------------------------------------- world-space cache */
    this.strutTopWorld = new Vec3();
    this.hubWorld = new Vec3();
    this.contactPoint = new Vec3();
    this.contactNormal = new Vec3(0, 1, 0);
    this.forwardDir = new Vec3(1, 0, 0);
    this.lateralDir = new Vec3(0, 0, 1);
    this.strutAxis = new Vec3(0, 1, 0);      // body up, in world space
    this.contactVelocity = new Vec3();
    this.forceWorld = new Vec3();

    /* --------------------------------------------------- scratch (no GC) */
    this._surf = new SurfaceSample();
    this._t0 = new Vec3();
    this._t1 = new Vec3();
    this._t2 = new Vec3();
    this._rayDir = new Vec3();

    /** Contact-patch velocities in the wheel frame, m/s. */
    this.vLong = 0;
    this.vLat = 0;
  }

  /* ===================================================================== */
  /* 1. Ground contact                                                      */
  /* ===================================================================== */

  /**
   * Solve for the strut length at which a tyre of radius R touches the ground.
   *
   * For a locally planar surface with point Q and unit normal N, the wheel
   * centre C = T + L*D touches when the perpendicular distance equals R:
   *
   *     dot(T + L*D - Q, N) = R      =>    L = (R - dot(T - Q, N)) / dot(D, N)
   *
   * Two iterations are enough: the first uses the previous hub position to
   * pick the surface patch, the second re-samples at the corrected position.
   * This is what keeps the tyre ON the road on slopes and kerbs instead of
   * floating above it or sinking through (requirement #16).
   */
  updateContact(body, terrain, dt) {
    // Strut top and axis in world space.
    body.localToWorld(this.strutTopLocal, this.strutTopWorld);
    this.strutAxis.copy(body.up);
    this._rayDir.copy(body.up).scale(-1);

    const R = this.radius;
    let L = clamp(this.suspension.length, 0, this.suspension.maxLength);
    let hit = false;

    for (let iter = 0; iter < 2; iter++) {
      const px = this.strutTopWorld.x + this._rayDir.x * L;
      const pz = this.strutTopWorld.z + this._rayDir.z * L;
      terrain.sample(px, pz, this._surf);

      const N = this._surf.normal;
      const dn = this._rayDir.dot(N);
      if (dn > -MIN_AXIS_DOT) break;   // strut nearly parallel to the surface

      // dot(T - Q, N) with Q = (px, height, pz)
      const c0 = (this.strutTopWorld.x - px) * N.x
        + (this.strutTopWorld.y - this._surf.height) * N.y
        + (this.strutTopWorld.z - pz) * N.z;

      L = (R - c0) / dn;
      if (!Number.isFinite(L)) { L = this.suspension.maxLength; break; }
      hit = L <= this.suspension.maxLength;
      L = clamp(L, 0, this.suspension.maxLength);
    }

    this.hasContact = hit;
    this.contactNormal.copy(this._surf.normal);
    this.surface = this._surf.surface;

    this.suspension.setLength(hit ? L : this.suspension.maxLength, dt, hit);

    // Hub and contact point in world space.
    this.hubWorld.copy(this.strutTopWorld).addScaled(this._rayDir, this.suspension.length);
    this.contactPoint.copy(this.hubWorld)
      .addScaled(this.contactNormal, -(R - this.tire.deflection));

    return this.hasContact;
  }

  /* ===================================================================== */
  /* 2. Suspension -> vertical load                                         */
  /* ===================================================================== */

  /**
   * Convert the strut force (which acts along the body's up axis) into the
   * normal force the ground can actually deliver (which acts along the surface
   * normal). On a banked or cambered surface these differ.
   */
  computeNormalForce() {
    const strutForce = this.suspension.computeForce();
    if (!this.hasContact) {
      this.normalForce = 0;
      this.tire.updateVertical(0);
      return 0;
    }
    const axisDot = Math.max(MIN_AXIS_DOT, this.strutAxis.dot(this.contactNormal));
    this.normalForce = strutForce / axisDot;
    this.tire.updateVertical(this.normalForce, this.suspension.velocity);
    this.loadRatio = this.normalForce / this.suspension.staticLoad;
    return this.normalForce;
  }

  /* ===================================================================== */
  /* 3. Contact-patch kinematics                                            */
  /* ===================================================================== */

  /**
   * Build the wheel's rolling frame on the contact plane and measure how fast
   * the patch is travelling along and across it.
   *
   * The steering angle rotates the FRAME here. It never touches the rolling
   * rotation, which lives in `spinAngle` (requirement #25).
   */
  updatePatchFrame(body) {
    const delta = this.steerAngle + this.staticToe;

    // Wheel heading in body space. Body forward is +X, right is +Z, so a
    // positive (rightward) steer angle rotates +X toward +Z.
    this._t0.set(Math.cos(delta), 0, Math.sin(delta));
    body.quat.rotate(this._t0, this.forwardDir);

    // Project onto the contact plane so the frame follows the road (req #28).
    this.forwardDir.projectOnPlane(this.contactNormal);
    if (this.forwardDir.lengthSq() < 1e-8) {
      this.forwardDir.copy(body.forward).projectOnPlane(this.contactNormal);
    }
    this.forwardDir.normalize();

    // lateral = forward x normal  ->  points to the right of travel
    this.lateralDir.crossVectors(this.forwardDir, this.contactNormal).normalize();

    // Velocity of the material point of the chassis at the contact patch.
    body.pointVelocity(this.contactPoint, this.contactVelocity);
    this._t1.copy(this.contactVelocity).projectOnPlane(this.contactNormal);
    this.vLong = this._t1.dot(this.forwardDir);
    this.vLat = this._t1.dot(this.lateralDir);
  }

  /* ===================================================================== */
  /* 4. Tyre forces                                                         */
  /* ===================================================================== */

  computeTireForces(dt, speed) {
    if (!this.hasContact) {
      this.tire.updateSlip(this.vLong, 0, this.spinVelocity, dt);
      this.tire.longForce = 0;
      this.tire.latForce = 0;
      this.tire.alignTorque = 0;
      this.longForce = 0;
      this.latForce = 0;
      return;
    }

    this.tire.updateSlip(this.vLong, this.vLat, this.spinVelocity, dt);
    this.tire.computeForces(this.normalForce, this.surface);
    this.tire.updateTemperature(this.vLong, this.vLat, this.spinVelocity, dt, speed,
      this.normalForce, this.surface);

    this.longForce = this.tire.longForce;
    this.latForce = this.tire.latForce;

    /**
     * Low-speed stiction. The relaxation-length model is continuous through
     * zero but leaves a small residual creep, which would make a parked car
     * drift down a slope. Below walking pace, and only when the driver is not
     * asking for torque, blend in a direct velocity-cancelling force capped by
     * the friction circle. This is a numerical aid, not a fudge of the model:
     * above 0.8 m/s it is completely inactive.
     */
    const patchSpeed = Math.hypot(this.vLong, this.vLat);
    if (patchSpeed < 0.8 && Math.abs(this.driveTorque + this.electricTorque) < 60) {
      const blend = 1 - patchSpeed / 0.8;
      const cap = this.tire.mu * this.normalForce;
      const kStick = 5200;
      const fx = clamp(-this.vLong * kStick, -cap, cap);
      const fy = clamp(-this.vLat * kStick, -cap, cap);
      this.longForce = this.longForce * (1 - blend) + fx * blend;
      this.latForce = this.latForce * (1 - blend) + fy * blend;
      // Keep the combined force inside the friction circle.
      const mag = Math.hypot(this.longForce, this.latForce);
      if (mag > cap && mag > 1e-6) {
        const k = cap / mag;
        this.longForce *= k;
        this.latForce *= k;
      }
    }
  }

  /**
   * Push this wheel's suspension and tyre forces into the chassis.
   * Both act at the contact point, which is what generates weight transfer,
   * body roll and pitch without any of them being scripted (#29-#32).
   */
  applyForces(body) {
    if (!this.hasContact) return;

    // Suspension: along the surface normal.
    this.forceWorld.copy(this.contactNormal).scale(this.normalForce);

    // Tyre: in the contact plane.
    this.forceWorld.addScaled(this.forwardDir, this.longForce);
    this.forceWorld.addScaled(this.lateralDir, this.latForce);

    // Soft-surface ploughing drag (gravel, grass) - resists motion directly.
    if (this.surface.drag > 0) {
      const v = Math.hypot(this.vLong, this.vLat);
      if (v > 0.05) {
        const d = -this.surface.drag * Math.min(1, v / 2.5);
        this.forceWorld.addScaled(this.forwardDir, (this.vLong / v) * d);
        this.forceWorld.addScaled(this.lateralDir, (this.vLat / v) * d);
      }
    }

    body.applyForceAtPoint(this.forceWorld, this.contactPoint);

    // Self-aligning moment about the contact normal.
    this._t2.copy(this.contactNormal).scale(this.tire.alignTorque);
    body.applyTorque(this._t2);
  }

  /* ===================================================================== */
  /* 5. Wheel rotational dynamics                                           */
  /* ===================================================================== */

  /**
   * Integrate the spin degree of freedom.
   *
   *   I * dw/dt = T_drive - Fx * Re - T_rollingResistance
   *
   * then the brake is applied as a bounded decelerating impulse so it can
   * bring the wheel to a stop but never drive it backwards inside one step.
   * That bound is what makes wheel LOCK a stable state rather than a buzz.
   */
  integrateSpin(dt) {
    const I = this.inertia + this.reflectedInertia;

    let torque = this.driveTorque + this.electricTorque;
    if (this.hasContact) {
      // Reaction of the ground force on the wheel about its own axis.
      torque -= this.longForce * this.tire.effectiveRadius;
      torque -= this.tire.rollingResistanceTorque(
        this.normalForce, this.surface, this.spinVelocity,
      );
    } else {
      // Airborne: only bearing drag and windage.
      torque -= this.spinVelocity * 0.6;
    }

    /**
     * The wheel and its tyre form the stiffest loop in the simulation:
     * omega changes the slip ratio, the slip ratio changes Fx, and Fx feeds
     * straight back as a torque on the same wheel. Its natural rate is
     *
     *     (dFx/dkappa) * (dkappa/domega) * Re / I
     *
     * which at 30 m/s in a tall gear is a few hundred per second - far beyond
     * what explicit Euler can carry at 60 Hz, so a naive integration is fine
     * at 240 Hz and diverges the moment the physics rate drops.
     *
     * Linearising the tyre about its current operating point and solving for
     * the END-of-step omega costs one extra divide:
     *
     *     (I/dt + dFx/domega * Re) * dOmega = T_net
     *
     * Below the stability limit this matches explicit Euler; above it, it
     * simply damps instead of exploding. The steady state (dOmega -> 0 when
     * T_net -> 0) is untouched, so nothing about the equilibrium changes.
     */
    let stiffness = 0;
    if (this.hasContact) {
      const t = this.tire;
      // Slope of the Magic Formula at the origin, per unit of slip ratio.
      const TY = this.tyreSpec;
      const cx = (t.mu * this.normalForce * TY.mfC * TY.mfB) / t.peakSlipRatio;
      const vFloor = Math.max(Math.abs(this.vLong), TY.slipSpeedFloor);
      const kRate = vFloor / TY.relaxLongitudinal;
      // How much the relaxed slip ratio moves for a unit change in omega.
      const dKappaDomega = ((dt * t.effectiveRadius) / TY.relaxLongitudinal)
        / (1 + dt * kRate);
      stiffness = cx * dKappaDomega * t.effectiveRadius;
    }

    const denom = I / dt + stiffness;
    const dOmega = torque / denom;
    this.spinAcceleration = dOmega / dt;
    this.spinVelocity += dOmega;

    // --- braking ---------------------------------------------------------
    if (this.brakeTorque > 0) {
      /**
       * The brake goes through the SAME implicit denominator as the drive
       * torque, for the same reason: as the wheel slows, slip grows and the
       * tyre pushes back, and that reaction happens within the step.
       *
       * Dividing by the bare I/dt instead would make the brake step scale with
       * the timestep - at 60 Hz a single step would take 35 rad/s off a wheel
       * turning at 87, i.e. 40 % of its speed at once. The slip ratio then
       * swings wildly between steps, ABS has nothing stable to regulate, and
       * the stopping distance grows by a fifth purely as an artefact of the
       * integration rate.
       *
       * It is still applied as a BOUNDED change, so the brake can bring the
       * wheel to a stop but can never drive it backwards - which is what makes
       * a locked wheel a stable state rather than a buzz.
       */
      const dOmega = this.brakeTorque / denom;
      if (Math.abs(this.spinVelocity) <= dOmega) {
        this.spinVelocity = 0;                       // wheel locked
      } else {
        this.spinVelocity -= sign(this.spinVelocity) * dOmega;
      }
    }

    if (!Number.isFinite(this.spinVelocity)) this.spinVelocity = 0;
    this.spinVelocity = clamp(this.spinVelocity, -450, 450);

    // The visual wheel reads this and nothing else (requirement #26).
    this.spinAngle += this.spinVelocity * dt;
    if (this.spinAngle > 1e6 || this.spinAngle < -1e6) this.spinAngle %= Math.PI * 2;
  }

  /* ===================================================================== */
  /* Reporting                                                              */
  /* ===================================================================== */

  /** Signed slip ratio, the transient value the tyre actually used. */
  get slipRatio() { return this.tire.slipRatio; }
  /** Slip angle in radians. */
  get slipAngle() { return this.tire.slipAngle; }
  /** True when the wheel is turning much faster than the road (#9). */
  get isWheelSpinning() {
    return this.tire.slipRatio > 0.22 && (this.driveTorque + this.electricTorque) > 50;
  }
  /** True when the wheel has stopped while the car still moves (#9). */
  get isLocked() {
    return this.brakeTorque > 10
      && Math.abs(this.spinVelocity) < 0.6
      && Math.abs(this.vLong) > 1.2;
  }
  /** Surface speed of the tread minus road speed - the sliding velocity. */
  get slipVelocity() {
    return this.spinVelocity * this.tire.effectiveRadius - this.vLong;
  }
  /** Suspension compression, metres (+ = compressed). */
  get suspensionTravel() { return this.suspension.displacement; }

  reset() {
    this.spinVelocity = 0;
    this.spinAngle = 0;
    this.spinAcceleration = 0;
    this.steerAngle = 0;
    /** Torque from the gearbox (via the differential), N.m. */
    this.driveTorque = 0;
    /** Torque from an electric motor at this corner, N.m. */
    this.electricTorque = 0;
    this.brakeTorque = 0;
    this.normalForce = 0;
    this.longForce = 0;
    this.latForce = 0;
    this.vLong = 0;
    this.vLat = 0;
    this.hasContact = false;
    this.loadRatio = 1;
    this.suspension.reset();
    this.tire.reset();
  }
}
