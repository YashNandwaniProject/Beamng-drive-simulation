/**
 * steering.js
 * ---------------------------------------------------------------------------
 * Steering rack: input smoothing, rate limiting, speed-sensitive lock,
 * Ackermann geometry and tyre-driven self-centring.
 * Requirements #22, #23, #24, #25.
 *
 * SIGN CONVENTION: a POSITIVE steering angle steers RIGHT, i.e. it rotates the
 * body's +X (forward) axis toward +Z (right). Everything downstream - the
 * physics patch frame and the three.js steer pivot - derives from this one
 * definition, which is what stops the front wheels ending up mirrored.
 *
 * This module produces the steering STATE and nothing else. What a vehicle
 * does with it visually is that vehicle's business:
 *
 *   BMW           the interior steering wheel is never touched - its rig
 *                 deliberately takes control of no interior object at all.
 *   Revuelto      its interior wheel IS animated, from `angle` below.
 *
 * In both cases the flow is one-way. The physics drives the animation; the
 * animation never drives the physics. `handWheelAngle` is a convenience for
 * the HUD indicator and for whichever 3D object a vehicle chooses to bind.
 * ---------------------------------------------------------------------------
 */

import { clamp, clamp01, damp, moveTowards, lerp, smoothstep } from './mathUtils.js';


export class SteeringSystem {
  /** @param {object} spec the vehicle spec (js/vehicles/*.js) */
  constructor(spec) {
    /** Vehicle-specific rack tuning and axle geometry. */
    this.cfg = spec.steering;
    this.axle = spec.axle;
    /** Smoothed driver input, -1 (full left) .. +1 (full right). */
    this.input = 0;
    /** Commanded average road-wheel angle, radians. */
    this.angle = 0;
    /** Available lock after the speed-sensitive reduction, radians. */
    this.availableLock = spec.steering.maxAngle;
    /** Left / right road-wheel angles after Ackermann, radians. */
    this.leftAngle = 0;
    this.rightAngle = 0;
    /** Aligning-moment feedback, rad/s of rack motion. HUD force-feedback. */
    this.selfCentring = 0;
    /** Cosmetic hand-wheel angle for the HUD ONLY. Nothing 3D reads this. */
    this.handWheelAngle = 0;
    /**
     * Steering ratio used to derive handWheelAngle. For a car with an animated
     * interior wheel (the Lamborghini) this is the model's own measured ratio,
     * so the HUD indicator and the 3D wheel always agree.
     */
    this.steeringRatio = (spec.model.steeringWheel && spec.model.steeringWheel.ratio)
      || 14.5;
  }

  /**
   * @param {number} dt
   * @param {number} rawInput  -1..1 from the keyboard/gamepad
   * @param {number} speed     absolute road speed, m/s
   * @param {Wheel} fl         front-left wheel (for the aligning moment)
   * @param {Wheel} fr         front-right wheel
   */
  update(dt, rawInput, speed, fl, fr) {
    const STEERING = this.cfg;
    /* --- 1. smooth the (digital) input --------------------------------- */
    this.input = damp(this.input, clamp(rawInput, -1, 1),
      STEERING.inputSmoothTime * 0.5, dt);

    /* --- 2. speed-sensitive lock ---------------------------------------
     * At a car park the driver gets the full 33 degrees; at 200 km/h the rack
     * only offers a third of it, so a flick of the key cannot spin the car.
     */
    const t = smoothstep(0, STEERING.highSpeed, speed);
    this.availableLock = STEERING.maxAngle * lerp(1, STEERING.highSpeedScale, t);

    const target = this.input * this.availableLock;

    /* --- 3. rate-limited rack -------------------------------------------
     * A real rack has a finite speed; returning to centre is faster than
     * winding on lock because the caster is helping.
     */
    const returning = Math.abs(target) < Math.abs(this.angle)
      && Math.sign(target) === Math.sign(this.angle || target);
    const rate = (returning || Math.abs(this.input) < 0.02)
      ? STEERING.returnSpeed : STEERING.rackSpeed;
    this.angle = moveTowards(this.angle, target, rate * dt);

    /* --- 4. self-centring from the tyres' aligning moment ----------------
     * Mz is negative when the tyres are pushing right, so it naturally winds
     * the wheel back toward straight. It fades out as the driver holds lock.
     */
    const mz = (fl.tire.alignTorque + fr.tire.alignTorque);
    this.selfCentring = clamp(mz * STEERING.aligningGain * 0.026,
      -STEERING.aligningMax, STEERING.aligningMax);
    const grip = 1 - clamp01(Math.abs(this.input));
    this.angle += this.selfCentring * grip * dt;
    this.angle = clamp(this.angle, -this.availableLock, this.availableLock);

    /* --- 5. Ackermann split and assignment ------------------------------ */
    this._applyAckermann(fl, fr);

    this.handWheelAngle = this.angle * this.steeringRatio;
  }

  /**
   * Ackermann geometry (requirement #23).
   *
   * For the whole car to turn about one instantaneous centre, the inner wheel
   * must be turned MORE than the outer one:
   *
   *     R          = wheelbase / tan(delta)          (radius at the CoM axis)
   *     delta_in   = atan(wheelbase / (R - track/2))
   *     delta_out  = atan(wheelbase / (R + track/2))
   *
   * Real cars run partial Ackermann, so the geometric result is blended with
   * parallel steer by STEERING.ackermann.
   */
  _applyAckermann(fl, fr) {
    const STEERING = this.cfg;
    const AXLE = this.axle;
    const d = this.angle;
    const mag = Math.abs(d);

    if (mag < 1e-4) {
      this.leftAngle = 0;
      this.rightAngle = 0;
      fl.steerAngle = 0;
      fr.steerAngle = 0;
      return;
    }

    const L = AXLE.wheelbase;
    const halfTrack = AXLE.trackFront * 0.5;
    const R = L / Math.tan(mag);

    // Guard the geometric singularity when the turn radius approaches the
    // half-track (a lock that tight is not physically reachable, but the
    // formula must not blow up if the parameters are ever retuned).
    const inner = Math.atan(L / Math.max(0.35, R - halfTrack));
    const outer = Math.atan(L / (R + halfTrack));

    const k = STEERING.ackermann;
    const innerAngle = lerp(mag, inner, k);
    const outerAngle = lerp(mag, outer, k);

    if (d > 0) {
      // Turning right: the right wheel is on the inside of the corner.
      this.rightAngle = innerAngle;
      this.leftAngle = outerAngle;
    } else {
      this.leftAngle = -innerAngle;
      this.rightAngle = -outerAngle;
    }

    fl.steerAngle = this.leftAngle;
    fr.steerAngle = this.rightAngle;
  }

  /** Steering angle in degrees, for the debug overlay. */
  get angleDegrees() { return (this.angle * 180) / Math.PI; }

  reset() {
    this.input = 0;
    this.angle = 0;
    this.leftAngle = 0;
    this.rightAngle = 0;
    this.selfCentring = 0;
    this.handWheelAngle = 0;
  }
}
