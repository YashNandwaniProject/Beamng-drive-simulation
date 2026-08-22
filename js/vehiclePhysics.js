/**
 * vehiclePhysics.js
 * ---------------------------------------------------------------------------
 * The rigid body and the fixed-step simulation pipeline.
 *
 * RigidBody is a plain 6-DOF Newton-Euler integrator. Nothing about it knows
 * it is a car: it accumulates forces and torques and integrates. Every bit of
 * vehicle behaviour - dive, squat, roll, yaw, weight transfer - emerges from
 * WHERE those forces are applied (the four contact patches, below the centre
 * of mass), never from scripted rotations. That is requirement #29-#32 and
 * #51 in one sentence.
 *
 * VehiclePhysics runs the update order laid out in requirement #44.
 * ---------------------------------------------------------------------------
 */

import {
  Vec3, Quat, Mat3, clamp, clamp01, GRAVITY,
} from './mathUtils.js';
import { Wheel } from './wheelPhysics.js';
import { AntiRollBar } from './suspension.js';
import { Drivetrain, ElectricAxle, GEAR_REVERSE } from './drivetrain.js';
import { BrakeSystem } from './brakes.js';
import { SteeringSystem } from './steering.js';
import { wheelGeometry } from './vehicles/spec.js';

/* ========================================================================= */
/* RigidBody                                                                  */
/* ========================================================================= */

export class RigidBody {
  constructor(mass, inertiaDiag) {
    this.position = new Vec3();
    this.velocity = new Vec3();
    this.quat = new Quat();
    this.angularVelocity = new Vec3();

    this.mass = mass;
    this.invMass = 1 / mass;
    this.inertia = new Vec3(inertiaDiag.x, inertiaDiag.y, inertiaDiag.z);
    this.invInertia = new Vec3(1 / inertiaDiag.x, 1 / inertiaDiag.y, 1 / inertiaDiag.z);
    this.invInertiaWorld = new Mat3();

    this.force = new Vec3();
    this.torque = new Vec3();

    /** Cached body axes in world space, refreshed by updateDerived(). */
    this.forward = new Vec3(1, 0, 0);
    this.up = new Vec3(0, 1, 0);
    this.right = new Vec3(0, 0, 1);

    /** Linear acceleration of the last step, world space (for G-forces). */
    this.acceleration = new Vec3();

    this._t = new Vec3();
    this._t2 = new Vec3();
    this._t3 = new Vec3();
    this._axisX = new Vec3(1, 0, 0);
    this._axisY = new Vec3(0, 1, 0);
    this._axisZ = new Vec3(0, 0, 1);
  }

  updateDerived() {
    this.quat.rotate(this._axisX, this.forward);
    this.quat.rotate(this._axisY, this.up);
    this.quat.rotate(this._axisZ, this.right);
    this.invInertiaWorld.setWorldFromDiagonal(this.quat, this.invInertia);
  }

  clearAccumulators() {
    this.force.setZero();
    this.torque.setZero();
  }

  applyForce(f) { this.force.add(f); }

  applyTorque(t) { this.torque.add(t); }

  /** Apply a world-space force at a world-space point. */
  applyForceAtPoint(f, point) {
    this.force.add(f);
    this._t.subVectors(point, this.position);
    this._t2.crossVectors(this._t, f);
    this.torque.add(this._t2);
  }

  localToWorld(local, out) {
    this.quat.rotate(local, out);
    return out.add(this.position);
  }

  worldToLocal(world, out) {
    this._t.subVectors(world, this.position);
    return this.quat.rotateInverse(this._t, out);
  }

  /** Velocity of the material point of the body at a world position. */
  pointVelocity(worldPoint, out) {
    this._t.subVectors(worldPoint, this.position);
    out.crossVectors(this.angularVelocity, this._t);
    return out.add(this.velocity);
  }

  /** Semi-implicit Euler with the gyroscopic term. */
  integrate(dt) {
    // Linear
    this.acceleration.copy(this.force).scale(this.invMass);
    this.velocity.addScaled(this.acceleration, dt);
    this.position.addScaled(this.velocity, dt);

    // Angular:  I * dw/dt = T - w x (I * w)
    //
    // The inertia diagonal is expressed in the BODY frame, so the gyroscopic
    // term has to be evaluated there and rotated back to world - mixing the
    // two frames would give a term of the wrong magnitude. For a car it is a
    // small correction, but it is cheap and it keeps fast tumbles stable.
    const wBody = this._t;
    this.quat.rotateInverse(this.angularVelocity, wBody);
    this._t2.set(
      this.inertia.x * wBody.x,
      this.inertia.y * wBody.y,
      this.inertia.z * wBody.z,
    );
    this._t3.crossVectors(wBody, this._t2);      // w x (I w), body frame
    this.quat.rotate(this._t3, this._t2);        // -> world
    this._t2.scale(-1).add(this.torque);         // T - w x (I w)

    const alpha = this._t;
    this.invInertiaWorld.apply(this._t2, alpha);
    this.angularVelocity.addScaled(alpha, dt);

    this.quat.integrate(this.angularVelocity, dt);
    this.updateDerived();
  }

  /** Recover from a numerical blow-up rather than propagating NaNs. */
  sanitize() {
    if (!this.position.isFinite() || !this.velocity.isFinite()
      || !this.angularVelocity.isFinite()
      || !Number.isFinite(this.quat.x + this.quat.y + this.quat.z + this.quat.w)) {
      return false;
    }
    const vMax = 200;
    if (this.velocity.lengthSq() > vMax * vMax) {
      this.velocity.normalize().scale(vMax);
    }
    const wMax = 22;
    if (this.angularVelocity.lengthSq() > wMax * wMax) {
      this.angularVelocity.normalize().scale(wMax);
    }
    return true;
  }
}

/* ========================================================================= */
/* VehiclePhysics                                                             */
/* ========================================================================= */

export class VehiclePhysics {
  /**
   * @param {Terrain} terrain
   * @param {object}  spec  the vehicle specification (js/vehicles/*.js).
   *   The physics engine is common to every car; this object is the ONLY thing
   *   that differs between them.
   */
  constructor(terrain, spec) {
    this.terrain = terrain;
    this.spec = spec;
    this.id = spec.id;
    this.displayName = spec.displayName;

    const CHASSIS = spec.chassis;
    this.body = new RigidBody(CHASSIS.mass, CHASSIS.inertia);

    /* ------------------------------------------------------ the four wheels */
    const mk = (id) => {
      const g = wheelGeometry(spec, id);
      return new Wheel(id, { ...g, tyre: spec.tyre });
    };

    this.wheelFL = mk('FL');
    this.wheelFR = mk('FR');
    this.wheelRL = mk('RL');
    this.wheelRR = mk('RR');
    /** Canonical order: FL, FR, RL, RR. Every indexed array follows it. */
    this.wheels = [this.wheelFL, this.wheelFR, this.wheelRL, this.wheelRR];
    this.frontWheels = [this.wheelFL, this.wheelFR];
    this.rearWheels = [this.wheelRL, this.wheelRR];

    /**
     * Which wheels each power source reaches. The BMW is rear-drive, so the
     * engine set is the rear pair and the electric set is empty. The Revuelto
     * drives its rear axle mechanically and its front axle electrically, which
     * is what makes it all-wheel drive without a propshaft.
     */
    this.drivenWheels = this.wheels.filter((w) => w.engineDriven);
    this.electricWheels = this.wheels.filter((w) => w.electricDriven);
    /** Everything that receives torque, for traction control. */
    this.poweredWheels = this.wheels.filter((w) => w.driven);

    this.arbFront = new AntiRollBar(spec.suspension.front.antiRollRate);
    this.arbRear = new AntiRollBar(spec.suspension.rear.antiRollRate);

    this.drivetrain = new Drivetrain(spec);
    this.electricAxle = new ElectricAxle(spec.electric);
    this.brakes = new BrakeSystem(spec);
    this.steering = new SteeringSystem(spec);

    /**
     * Collision hull sample points in body space: eight corners plus four side
     * midpoints, so a glancing blow along a barrier is caught at the right
     * place along the car. Built from this vehicle's own measured bodywork.
     */
    const h = CHASSIS.hullHalf;
    const ox = CHASSIS.hullOffsetX;
    const oy = CHASSIS.hullOffsetY;
    this.hullPoints = [];
    for (const sx of [-1, 0, 1]) {
      for (const sy of [-1, 1]) {
        for (const sz of [-1, 1]) {
          this.hullPoints.push(new Vec3(ox + h.x * sx, oy + h.y * sy, h.z * sz));
        }
      }
    }

    /* --------------------------------------------------------- vehicle state */
    /** Signed forward road speed, m/s. */
    this.forwardSpeed = 0;
    /** Speed magnitude, m/s. */
    this.speed = 0;
    /** Sideslip angle of the whole car, radians. */
    this.sideslip = 0;
    /** Yaw rate, rad/s (about the body up axis). */
    this.yawRate = 0;
    /** G-forces in the body frame: x longitudinal, y vertical, z lateral. */
    this.gForce = new Vec3(0, 1, 0);
    this.gForceSmooth = new Vec3(0, 1, 0);
    /** Body-frame acceleration in m/s^2, before the /g. */
    this.bodyAcceleration = new Vec3();
    this.airborne = false;
    this.contactCount = 0;

    /** Forward/reverse shifter state. */
    this._reverseHold = 0;
    this._forwardHold = 0;

    /** Stored spawn point: a position ON the road plus its heading. */
    this.spawnGround = new Vec3();
    this.spawnHeading = 0;

    /* ------------------------------------------------------------- scratch */
    this._prevVelocity = new Vec3();
    this._tmpA = new Vec3();
    this._tmpB = new Vec3();
    this._tmpC = new Vec3();
    this._gravity = new Vec3(0, -GRAVITY, 0);
    this._localAxle = new Vec3();

    this.collisionSystem = null;   // injected by the Vehicle
    this.lastImpact = 0;
  }

  /* ===================================================================== */
  /* Placement                                                              */
  /* ===================================================================== */

  /**
   * Put the car on the road. `groundPosition` is a point ON the road surface;
   * the CoM is raised to the design ride height above it so all four tyres sit
   * in contact with their suspension already carrying the static load - no
   * drop-in, no floating, no wheels buried in the asphalt (requirement #49).
   */
  placeAt(groundPosition, headingRad) {
    const b = this.body;
    b.position.copy(groundPosition);
    b.quat.setFromEulerYXZ(-headingRad, 0, 0);
    b.velocity.setZero();
    b.angularVelocity.setZero();
    b.acceleration.setZero();
    b.updateDerived();

    // Static ride height: hub is |AXLE.hubY| below the CoM and the tyre adds
    // its radius under that.
    /**
     * Ride height: the CoM's height above the mean contact plane, derived in
     * the spec from each axle's own hub height and tyre radius. A staggered
     * car (20-inch front, 21-inch rear) genuinely differs end to end.
     */
    b.position.y += this.spec.rideHeight;

    this.spawnGround.copy(groundPosition);
    this.spawnHeading = headingRad;

    for (const w of this.wheels) w.suspension.reset();
    this._prevVelocity.setZero();

    // The derived read-outs are cached, so clear them too - otherwise the HUD
    // and any caller polling straight after a reset would see stale values
    // from before the respawn.
    this.forwardSpeed = 0;
    this.speed = 0;
    this.sideslip = 0;
    this.yawRate = 0;
    this.airborne = false;
    this.contactCount = 4;
    this.bodyAcceleration.setZero();
    this.gForce.set(0, 1, 0);
    this.gForceSmooth.set(0, 1, 0);
  }

  /** Return to the stored spawn transform (the R key). */
  respawn() {
    this.reset(this.spawnGround, this.spawnHeading);
  }

  /**
   * Heading is measured as a yaw about +Y such that the body +X axis points
   * along atan2(dz, dx). Rotating +X by an angle A about +Y gives
   * (cos A, 0, -sin A), so the quaternion yaw is the NEGATIVE of the compass
   * angle. Wrapped here so nothing else has to remember it.
   */
  setHeading(headingRad) {
    this.body.quat.setFromEulerYXZ(-headingRad, 0, 0);
    this.body.updateDerived();
  }

  get heading() {
    return Math.atan2(this.body.forward.z, this.body.forward.x);
  }

  /** Full reset - requirement #48. Does not reload anything. */
  reset(position, headingRad) {
    for (const w of this.wheels) w.reset();
    this.arbFront.reset();
    this.arbRear.reset();
    this.drivetrain.reset();
    this.electricAxle.reset();
    this.brakes.reset();
    this.steering.reset();
    this._reverseHold = 0;
    this._forwardHold = 0;
    this.gForce.set(0, 1, 0);
    this.gForceSmooth.set(0, 1, 0);
    this.bodyAcceleration.setZero();
    this.lastImpact = 0;
    this.placeAt(position, headingRad);
  }

  /* ===================================================================== */
  /* The fixed step - requirement #44's ordering                            */
  /* ===================================================================== */

  /**
   * @param {number} dt fixed timestep, seconds
   * @param {object} input { throttle, brake, steer, handbrake }
   */
  step(dt, input) {
    const body = this.body;
    body.updateDerived();
    this._prevVelocity.copy(body.velocity);

    /* -- 1/2. driver input, pedals, forward/reverse shifter --------------- */
    this.forwardSpeed = body.velocity.dot(body.forward);
    this.speed = body.velocity.length();
    const pedals = this._resolveShifter(dt, input);

    /* -- 3. steering rack (uses last step's aligning moment) -------------- */
    this.steering.update(dt, input.steer, this.speed, this.wheelFL, this.wheelFR);

    /* -- 4. ground contact for all four wheels ---------------------------- */
    this.contactCount = 0;
    for (let i = 0; i < 4; i++) {
      if (this.wheels[i].updateContact(body, this.terrain, dt)) this.contactCount++;
    }
    this.airborne = this.contactCount === 0;

    /* -- 5. anti-roll bars (need both sides' travel) ---------------------- */
    this.arbFront.apply(this.wheelFL, this.wheelFR);
    this.arbRear.apply(this.wheelRL, this.wheelRR);

    /* -- 6. suspension -> vertical load, tyre carcass deflection ---------- */
    for (let i = 0; i < 4; i++) this.wheels[i].computeNormalForce();

    /* -- 7. contact patch kinematics, slip, tyre forces ------------------- */
    for (let i = 0; i < 4; i++) {
      const w = this.wheels[i];
      w.updatePatchFrame(body);
      w.computeTireForces(dt, this.speed);
    }

    /* -- 8. traction control, then the whole drivetrain ------------------- */
    const throttleTC = this.brakes.applyTractionControl(
      dt, this.poweredWheels, pedals.throttle,
    );

    // Wheels the gearbox does not reach start each step with nothing.
    for (let i = 0; i < 4; i++) {
      const w = this.wheels[i];
      if (!w.engineDriven) { w.driveTorque = 0; w.reflectedInertia = 0; }
      if (!w.electricDriven) w.electricTorque = 0;
    }

    this.drivetrain.update(dt, throttleTC, this.drivenWheels, this.forwardSpeed,
      pedals.throttle);

    /**
     * The electric axle runs in parallel with the gearbox, not behind it. It
     * needs no gear and no clutch, so it delivers from a standstill - which is
     * exactly why an AWD hybrid launches so much harder than a rear-drive car.
     */
    this.electricAxle.update(
      dt, this.electricWheels, throttleTC, this.brakes.pedal, this.steering.input,
    );

    /* -- 9. brake torques (ABS regulates per corner) ---------------------- */
    this.brakes.update(dt, pedals.brake, input.handbrake, this.wheels, this.speed);

    /* -- 10. accumulate all forces on the chassis ------------------------- */
    body.clearAccumulators();

    // Gravity at the centre of mass - no torque.
    this._tmpA.copy(this._gravity).scale(body.mass);
    body.applyForce(this._tmpA);

    // Tyre + suspension forces at the four contact patches.
    for (let i = 0; i < 4; i++) this.wheels[i].applyForces(body);

    // Aerodynamics.
    this._applyAerodynamics(body);

    /* -- 11. integrate the rigid body ------------------------------------- */
    body.integrate(dt);
    if (!body.sanitize()) this.respawn();

    /* -- 12. collision with the static world ------------------------------ */
    if (this.collisionSystem) {
      this.lastImpact = this.collisionSystem.resolve(body, dt, this.hullPoints);
    }

    /* -- 13. wheel rotational dynamics ------------------------------------ */
    for (let i = 0; i < 4; i++) this.wheels[i].integrateSpin(dt);

    /* -- 14. derived state: yaw, sideslip, G-forces ----------------------- */
    this._updateDerivedState(dt);
  }

  /* ===================================================================== */

  /**
   * W and S map to different pedals depending on the selected direction, which
   * is what makes "S reverses when stopped" feel natural (requirement #6).
   */
  _resolveShifter(dt, input) {
    const dtx = this.drivetrain;
    const v = this.forwardSpeed;
    const rev = this.spec.transmission.reverseSpeed;

    if (dtx.gear === GEAR_REVERSE) {
      // In reverse: S drives, W brakes (and, held at a stop, selects Drive).
      if (input.throttle > 0.5 && v > -rev) {
        this._forwardHold += dt;
        if (this._forwardHold > 0.30) {
          dtx.setGearImmediate(1);
          this._forwardHold = 0;
        }
      } else {
        this._forwardHold = 0;
      }
      this._reverseHold = 0;
      return { throttle: input.brake, brake: input.throttle };
    }

    // Forward gears: hold the brake at a standstill to select reverse.
    if (input.brake > 0.5 && Math.abs(v) < rev) {
      this._reverseHold += dt;
      if (this._reverseHold > 0.30) {
        dtx.setGearImmediate(GEAR_REVERSE);
        this._reverseHold = 0;
      }
    } else {
      this._reverseHold = 0;
    }
    this._forwardHold = 0;
    return { throttle: input.throttle, brake: input.brake };
  }

  /**
   * Drag opposes the velocity vector; downforce is split front/rear so it also
   * shifts the aerodynamic balance with speed.
   */
  _applyAerodynamics(body) {
    const CHASSIS = this.spec.chassis;
    const AXLE = this.spec.axle;
    const v2 = body.velocity.lengthSq();
    if (v2 < 0.01) return;
    const v = Math.sqrt(v2);
    const q = 0.5 * CHASSIS.airDensity * v2;

    // Drag, applied slightly above the CoM so it also pitches the car.
    const dragMag = q * CHASSIS.dragCoefficient * CHASSIS.frontalArea;
    this._tmpA.copy(body.velocity).scale(-dragMag / v);
    this._tmpB.copy(body.position).addScaled(body.up, CHASSIS.aeroCentreHeight);
    body.applyForceAtPoint(this._tmpA, this._tmpB);

    // Downforce at each axle.
    const dfFront = q * CHASSIS.frontalArea * -CHASSIS.liftFront;
    const dfRear = q * CHASSIS.frontalArea * -CHASSIS.liftRear;

    this._tmpA.copy(body.up).scale(-dfFront);
    this._localAxle.set(AXLE.a, 0, 0);
    body.localToWorld(this._localAxle, this._tmpB);
    body.applyForceAtPoint(this._tmpA, this._tmpB);

    this._tmpA.copy(body.up).scale(-dfRear);
    this._localAxle.set(-AXLE.b, 0, 0);
    body.localToWorld(this._localAxle, this._tmpB);
    body.applyForceAtPoint(this._tmpA, this._tmpB);
  }

  /**
   * G-forces from the ACTUAL acceleration of the body (requirement #33), not
   * from the keyboard. What an accelerometer bolted to the car would read is
   * the specific force, a - g, so a stationary car reads +1 G vertically.
   */
  _updateDerivedState(dt) {
    const body = this.body;

    this.forwardSpeed = body.velocity.dot(body.forward);
    this.speed = body.velocity.length();
    this.yawRate = body.angularVelocity.dot(body.up);

    // Sideslip: the angle between where the car points and where it is going.
    const vLat = body.velocity.dot(body.right);
    this.sideslip = this.speed > 0.7 ? Math.atan2(vLat, Math.abs(this.forwardSpeed)) : 0;

    // Specific force in world space.
    this._tmpA.subVectors(body.velocity, this._prevVelocity).scale(1 / Math.max(dt, 1e-6));
    this._tmpA.sub(this._gravity);

    this.bodyAcceleration.set(
      this._tmpA.dot(body.forward),
      this._tmpA.dot(body.up),
      this._tmpA.dot(body.right),
    );

    this.gForce.set(
      this.bodyAcceleration.x / GRAVITY,
      this.bodyAcceleration.y / GRAVITY,
      this.bodyAcceleration.z / GRAVITY,
    );
    // A lightly filtered copy for the HUD and camera so single-step spikes on
    // kerbs do not make the readout or the view jitter.
    const k = clamp01(dt * 14);
    this.gForceSmooth.x += (this.gForce.x - this.gForceSmooth.x) * k;
    this.gForceSmooth.y += (this.gForce.y - this.gForceSmooth.y) * k;
    this.gForceSmooth.z += (this.gForce.z - this.gForceSmooth.z) * k;
  }

  /* ===================================================================== */
  /* Reporting helpers                                                      */
  /* ===================================================================== */

  get speedKmh() { return this.speed * 3.6; }
  get rpm() { return this.drivetrain.rpm; }
  get gear() { return this.drivetrain.gearLabel; }

  /** Body roll angle in radians (positive = leaning to the right). */
  get rollAngle() {
    return Math.asin(clamp(this.body.right.y, -1, 1)) * -1;
  }

  /** Body pitch angle in radians (positive = nose up). */
  get pitchAngle() {
    return Math.asin(clamp(this.body.forward.y, -1, 1));
  }

  /** Roughly how much of the available grip the car is using, 0..1. */
  get gripUsage() {
    let sum = 0, n = 0;
    for (const w of this.wheels) {
      if (w.hasContact) { sum += w.tire.frictionUse; n++; }
    }
    return n ? sum / n : 0;
  }

  get anyWheelSpin() {
    return this.wheelRL.isWheelSpinning || this.wheelRR.isWheelSpinning;
  }

  get anyWheelLocked() {
    for (const w of this.wheels) if (w.isLocked) return true;
    return false;
  }

  /** Set the pressure of every tyre (PSI). */
  setTirePressures(psi) {
    const t = this.spec.tyre;
    for (const w of this.wheels) {
      w.tire.pressure = clamp(psi, t.minPressure, t.maxPressure);
      w.tire.updateDerived();
    }
  }

  /** Nominal tyre pressure for this car, PSI. */
  get nominalPressure() { return this.spec.tyre.nominalPressure; }
}
