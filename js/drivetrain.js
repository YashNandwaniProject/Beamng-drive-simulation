/**
 * drivetrain.js
 * ---------------------------------------------------------------------------
 * Engine -> torque converter -> gearbox -> final drive -> differential -> wheels.
 *
 * Requirement #7 asks for the whole chain; #8 asks that a standing start feel
 * like a real one. Three details do most of that work:
 *
 *  1. TORQUE CONVERTER. The car is an 8-speed automatic, so the engine is not
 *     bolted to the gearbox - it drives a fluid coupling whose transmitted
 *     torque depends on the speed RATIO between pump and turbine. That is why
 *     the revs flare on a standing start, why the car creeps at idle, and why
 *     there is no shock when the driveline takes up. A friction clutch model
 *     cannot do any of this: at partial engagement it transmits its full
 *     clamp load, which in first gear is several thousand N.m at the wheel.
 *
 *  2. TURBO LAG. Torque follows a first-order lag on the throttle plate, so
 *     stamping the pedal does not instantly deliver 500 N.m.
 *
 *  3. REFLECTED INERTIA. Once the lock-up clutch closes, the crank is geared
 *     14.8:1 to the road, so its 0.235 kg.m^2 appears at the wheel as
 *     0.235 * 14.8^2 = 51 kg.m^2 - forty times the wheel's own inertia. That
 *     is a real ~20 % effective mass penalty in first gear and it is what
 *     stops the car accelerating instantly even with the tyres gripping.
 *
 * Both the lock-up clutch and the limited-slip differential are solved
 * SEMI-IMPLICITLY: each computes the torque that would exactly equalise the
 * two speeds over the step, then clamps it to what the hardware can transmit.
 * Below the limit it sticks, above it, it slips at exactly the Coulomb bound.
 * That is unconditionally stable, so neither can ring at any timestep - an
 * explicit stiffness would diverge the moment the physics rate dropped.
 * ---------------------------------------------------------------------------
 */

import { clamp, clamp01, damp, sampleCurve } from './mathUtils.js';

const RPM_PER_RADS = 60 / (2 * Math.PI);
const RADS_PER_RPM = (2 * Math.PI) / 60;

export const GEAR_REVERSE = -1;
export const GEAR_NEUTRAL = 0;

export class Drivetrain {
  /** @param {object} spec the vehicle spec (js/vehicles/*.js) */
  constructor(spec) {
    /* --- vehicle-specific tuning; never a module-level constant --------- */
    this.spec = spec;
    const ENGINE = spec.engine;
    const TRANSMISSION = spec.transmission;
    this.ENGINE = ENGINE;
    this.CONVERTER = spec.converter;
    this.CLUTCH = spec.clutch;
    this.TRANSMISSION = TRANSMISSION;
    this.DIFFERENTIAL = spec.differential;

    /**
     * How the engine is coupled to the gearbox.
     *   'converter'  fluid coupling + lock-up clutch (BMW, 8-speed automatic)
     *   'clutch'     dry dual clutch (Lamborghini, 8-speed DCT)
     * They behave very differently off the line, which is a large part of why
     * the two cars feel unrelated: a converter multiplies torque at stall, a
     * DCT does not, and a DCT takes up in a fraction of the time.
     */
    this.coupling = TRANSMISSION.coupling || 'converter';

    /* ------------------------------------------------------------ engine */
    this.engineOmega = ENGINE.idleRpm * RADS_PER_RPM;
    /** Smoothed throttle plate position, 0..1. */
    this.throttle = 0;
    /** Lagged boost / torque delivery, 0..1. */
    this.boost = 0;
    this.engineTorque = 0;
    this.frictionTorque = 0;
    this.limiterTimer = 0;
    this.onLimiter = false;
    /** Integrator of the idle governor. */
    this.idleIntegral = 0.35;

    /* -------------------------------------------------- torque converter */
    /** Turbine / pump speed ratio; 0 at stall, ~1 when coupled. */
    this.speedRatio = 0;
    /** Instantaneous torque multiplication, 1.0 once coupled (converter only). */
    this.torqueRatio = spec.converter ? spec.converter.torqueRatio[0][1] : 1;
    /** Dry-clutch engagement, 0..1, commanded by the launch controller. */
    this.clutchEngagement = 0;
    /** Torque absorbed from the crank, N.m. */
    this.pumpTorque = 0;
    /** Torque delivered to the gearbox input shaft, N.m. */
    this.turbineTorque = 0;
    /** Lock-up clutch, 0 = fluid drive, 1 = mechanically locked. */
    this.lockup = 0;
    this.locked = false;
    this.clutchSlip = 0;

    /* ----------------------------------------------------------- gearbox */
    /** -1 reverse, 0 neutral, 1..8 forward. */
    this.gear = 1;
    this.shifting = false;
    this.shiftTimer = 0;
    this.shiftCooldown = 0;
    this.pendingGear = 1;
    this.autoShift = TRANSMISSION.autoShift;

    /* ------------------------------------------------------------ output */
    this.wheelTorque = 0;         // total torque at the driven axle, N.m
    this.diffLockTorque = 0;
    this.reflectedInertia = 0;    // per driven wheel, kg.m^2
    this.inputShaftOmega = 0;
  }

  /** Total ratio from crank to wheel for the current gear (signed). */
  get totalRatio() {
    const T = this.TRANSMISSION;
    if (this.gear === GEAR_NEUTRAL) return 0;
    if (this.gear === GEAR_REVERSE) return -T.reverseRatio * T.finalDrive;
    return T.gearRatios[this.gear - 1] * T.finalDrive;
  }

  get rpm() { return this.engineOmega * RPM_PER_RADS; }

  get gearLabel() {
    if (this.gear === GEAR_REVERSE) return 'R';
    if (this.gear === GEAR_NEUTRAL) return 'N';
    return String(this.gear);
  }

  /** How hard the fluid coupling is slipping, 0..1 (HUD). */
  get converterSlip() { return clamp01(1 - this.speedRatio); }

  /* ===================================================================== */

  /**
   * One drivetrain step.
   *
   * @param {number} dt              fixed timestep
   * @param {number} throttleIn      0..1 after traction control
   * @param {Wheel[]} driven         the driven wheels (rear pair)
   * @param {number} speed           signed forward road speed, m/s
   * @param {number} driverThrottle  0..1 raw pedal, used for shift decisions
   */
  update(dt, throttleIn, driven, speed, driverThrottle = throttleIn) {
    const ENGINE = this.ENGINE;
    const TRANSMISSION = this.TRANSMISSION;
    this._updateShifting(dt, driverThrottle, speed);

    /* --- 1. throttle plate + idle governor ----------------------------- */
    this.throttle = damp(this.throttle, clamp01(throttleIn),
      ENGINE.throttleResponseTime * 0.5, dt);

    let throttleEff = this.throttle;
    const rpm = this.rpm;

    // Idle air control. A purely proportional governor cannot hold idle,
    // because the steady-state error IS the throttle needed to balance engine
    // friction plus converter drag, so it carries an integrator - which is
    // exactly what the real ECU does.
    const idleError = ENGINE.idleRpm - rpm;
    this.idleIntegral = clamp01(this.idleIntegral + idleError * ENGINE.idleIntegralGain * dt);
    if (rpm < ENGINE.idleRpm + 300) {
      throttleEff = Math.max(throttleEff,
        clamp01(idleError * ENGINE.idleGain + this.idleIntegral));
    }

    /* --- 2. torque interrupt during a shift ----------------------------
     * Without it the gearbox goes to neutral with the throttle still open, the
     * engine free-revs thousands of rpm during the shift, and the shift logic
     * then hunts between gears on the bogus reading.
     */
    if (this.shifting) throttleEff = 0;

    /* --- 3. rev limiter ------------------------------------------------ */
    if (this.limiterTimer > 0) {
      this.limiterTimer -= dt;
      throttleEff = 0;
      this.onLimiter = true;
    } else if (rpm > ENGINE.limiterRpm) {
      this.limiterTimer = ENGINE.limiterCutTime;
      throttleEff = 0;
      this.onLimiter = true;
    } else {
      this.onLimiter = false;
    }

    /* --- 4. torque delivery with turbo lag ----------------------------- */
    const rising = throttleEff > this.boost;
    this.boost = damp(this.boost, throttleEff,
      (rising ? ENGINE.boostBuildTime : ENGINE.boostDecayTime) * 0.5, dt);

    const fullLoad = sampleCurve(ENGINE.torqueCurve, clamp(rpm, 500, ENGINE.maxRpm));
    const grossTorque = fullLoad * this.boost;

    // Pumping and friction losses, much larger on a closed throttle - which is
    // where engine braking comes from.
    const overrun = 1 + (1 - throttleEff) * (ENGINE.overrunFrictionScale - 1);
    this.frictionTorque = (ENGINE.frictionBase + ENGINE.frictionPerRpm * rpm) * overrun;
    this.engineTorque = grossTorque - this.frictionTorque;

    /* --- 5. driveline speeds ------------------------------------------- */
    const ratio = this.totalRatio;
    const engaged = this.gear !== GEAR_NEUTRAL && !this.shifting;

    let drivenOmega = 0;
    for (let i = 0; i < driven.length; i++) drivenOmega += driven[i].spinVelocity;
    drivenOmega /= Math.max(1, driven.length);
    this.inputShaftOmega = drivenOmega * ratio;
    this.clutchSlip = this.engineOmega - this.inputShaftOmega;

    /* --- 6. coupling schedule ------------------------------------------ */
    const Ie = ENGINE.inertia + TRANSMISSION.inertia;
    if (this.coupling === 'clutch') {
      /** How hard the driven wheels are already spinning, for launch control. */
      let drivenSlip = 0;
      for (let i = 0; i < driven.length; i++) {
        drivenSlip = Math.max(drivenSlip, driven[i].tire.slipRatio);
      }
      this._updateClutchEngagement(dt, engaged, speed, driverThrottle, drivenSlip);
    } else {
      this._updateLockup(dt, engaged, speed);
    }

    /* --- 7. coupling: rigid (locked), slipping clutch, or fluid --------- */
    let axleTorque = 0;

    if (!engaged) {
      // Neutral or mid-shift: no torque path to the road.
      this.pumpTorque = this.coupling === 'clutch' ? 0 : this._converterDrag();
      this.turbineTorque = 0;
      this.speedRatio = 0;
      this.engineOmega += ((this.engineTorque - this.pumpTorque) / Ie) * dt;
      this.reflectedInertia = 0;
    } else if (this.locked) {
      /* Rigid: the crank is kinematically tied to the wheels. */
      this.engineOmega = Math.max(
        ENGINE.stallRpm * RADS_PER_RPM, Math.abs(this.inputShaftOmega),
      );
      this.speedRatio = 1;
      this.torqueRatio = 1;
      this.pumpTorque = this.engineTorque;
      this.turbineTorque = this.engineTorque;
      axleTorque = this.engineTorque * ratio * TRANSMISSION.efficiency;
      this.reflectedInertia = Ie * ratio * ratio * TRANSMISSION.efficiency
        / Math.max(1, driven.length);
    } else if (this.coupling === 'clutch') {
      /* Slipping dry clutch: a DCT off the line, or mid-downshift. */
      this._solveClutch(dt, Ie);
      axleTorque = this.turbineTorque * ratio * TRANSMISSION.efficiency;
      this.engineOmega += ((this.engineTorque - this.pumpTorque) / Ie) * dt;
      this.reflectedInertia = 0;
    } else {
      /* Fluid coupling. */
      this._solveConverter(ratio);
      axleTorque = this.turbineTorque * ratio * TRANSMISSION.efficiency;
      this.engineOmega += ((this.engineTorque - this.pumpTorque) / Ie) * dt;
      this.reflectedInertia = 0;
    }

    this.engineOmega = clamp(
      this.engineOmega,
      ENGINE.stallRpm * RADS_PER_RPM,
      ENGINE.maxRpm * RADS_PER_RPM,
    );
    this.wheelTorque = axleTorque;

    /* --- 8. differential ------------------------------------------------ */
    this._applyDifferential(driven, axleTorque, dt);
  }

  /* ===================================================================== */
  /* Torque converter                                                       */
  /* ===================================================================== */

  /**
   * Classic capacity-factor model.
   *
   *   sr = w_turbine / w_pump                    speed ratio
   *   K  = K(sr)                                 capacity factor, rpm/sqrt(N.m)
   *   Tp = (rpm_pump / K)^2                      absorbed from the crank
   *   Tt = Tp * TR(sr)                           delivered to the gearbox
   *
   * At stall (sr = 0) K is small, so the converter absorbs a lot of torque and
   * multiplies it by ~2 on the way out: that is the launch, and the reason a
   * standing start flares the revs. As the turbine catches up, K rises steeply
   * and the transmitted torque collapses toward the engine's own output - which
   * is why an idling automatic creeps to walking pace and then stops gaining.
   */
  _solveConverter(ratio) {
    const CONVERTER = this.CONVERTER;
    const ENGINE = this.ENGINE;
    const wPump = Math.max(this.engineOmega, ENGINE.stallRpm * RADS_PER_RPM);
    // inputShaftOmega already carries the gear sign, so it is positive whenever
    // the car is moving the way the selected gear intends.
    const wTurbine = this.inputShaftOmega;

    const sr = clamp(wTurbine / wPump, -1.5, 1.5);
    this.speedRatio = sr;

    if (sr >= CONVERTER.coastRatio) {
      /**
       * Overrun: the turbine is driving the pump. A real converter transmits
       * very little in this direction, which is exactly why an automatic
       * engine-brakes poorly with the lock-up clutch open. A light linear
       * coupling captures the effect without another table.
       */
      this.torqueRatio = 1;
      const t = -CONVERTER.coastCoupling * (wTurbine - wPump);
      this.pumpTorque = t;
      this.turbineTorque = t;
      return;
    }

    const srClamped = clamp(sr, 0, 1);
    const K = sampleCurve(CONVERTER.capacityFactor, srClamped);
    this.torqueRatio = sampleCurve(CONVERTER.torqueRatio, srClamped);

    const rpmPump = wPump * RPM_PER_RADS;
    const tp = Math.min((rpmPump / K) * (rpmPump / K), CONVERTER.maxPumpTorque);

    this.pumpTorque = tp;
    this.turbineTorque = tp * this.torqueRatio;
  }

  /** Windage / pumping drag when the converter has nothing to drive. */
  _converterDrag() {
    const k = Math.abs(this.rpm) / 1000;
    return this.CONVERTER.neutralDrag * k * k;
  }

  /**
   * Lock-up clutch schedule. It closes only once the turbine has nearly caught
   * the pump, so closing it can never shock the driveline, and it opens again
   * at low speed, during shifts, and whenever the slip grows (a downshift or a
   * hard stop). The engage/release ratios differ, giving hysteresis so it
   * cannot chatter around the threshold.
   */
  _updateLockup(dt, engaged, speed) {
    const CONVERTER = this.CONVERTER;
    const absSpeed = Math.abs(speed);

    if (!engaged || absSpeed < CONVERTER.lockupMinSpeed) {
      this.locked = false;
    } else if (!this.locked) {
      this.locked = this.speedRatio > CONVERTER.lockupEngageRatio;
    } else {
      this.locked = this.speedRatio > CONVERTER.lockupReleaseRatio;
    }

    this.lockup = damp(this.lockup, this.locked ? 1 : 0, 0.04, dt);
  }

  /* ===================================================================== */
  /* Dry dual clutch (the Lamborghini's DCT)                                */
  /* ===================================================================== */

  /**
   * Command the clutch's clamp load.
   *
   * A DCT does not have a fluid coupling to hide behind, so launching it is an
   * active control problem: the actuator has to slip the clutch against a
   * TARGET ENGINE SPEED while the car gets moving, then squeeze it shut. That
   * is modelled here as an integral controller on engine speed error - bite
   * harder when the engine is above target, ease off when it falls below -
   * which reproduces the characteristic DCT launch: revs flare to the launch
   * rpm, hold there, then converge on the road speed as the clutch closes.
   *
   * Above `lockSpeed` the clutch is simply held shut, as the real one is.
   */
  _updateClutchEngagement(dt, engaged, speed, driverThrottle, drivenSlip = 0) {
    const C = this.CLUTCH;
    const ENGINE = this.ENGINE;

    if (!engaged) {
      this.clutchEngagement = 0;
      this.locked = false;
      this.lockup = 0;
      return;
    }

    const absSpeed = Math.abs(speed);
    if (absSpeed > C.lockSpeed) {
      /**
       * Rolling - squeeze it shut, but SLOWER THE MORE IT IS SLIPPING.
       *
       * This matters most straight after an upshift. A V12 at 8,400 rpm has to
       * lose nearly 3,000 rpm to match the next gear, and it cannot do that on
       * engine friction alone inside a 90 ms shift. What actually absorbs the
       * mismatch in a real dual clutch is the incoming clutch slipping as it
       * takes up. Clamping it shut at a fixed rate instead dumps the crank's
       * inertia straight into the driveline and shoves the car in the back -
       * a 0.34 G lurch on every upshift, which is not what a DCT feels like.
       */
      const rate = Math.max(
        C.reengageMinRate,
        C.reengageRate / (1 + Math.abs(this.clutchSlip) * C.reengageSlipFactor),
      );
      this.clutchEngagement = Math.min(1, this.clutchEngagement + dt * rate);
    } else {
      /**
       * Launching. Target the launch rpm in proportion to how much throttle
       * the driver is actually asking for, so a gentle pull-away slips gently
       * at just above idle and a full-throttle start flares to launchRpm.
       */
      const target = ENGINE.idleRpm
        + (C.launchRpm - ENGINE.idleRpm) * clamp01(driverThrottle);
      const err = this.rpm - target;
      let rate = err * C.engageGain;

      /**
       * Launch control: stop squeezing once the driven tyres are already past
       * their peak slip. Without this the actuator reads "engine above target"
       * and keeps biting, which simply dumps more torque into wheels that have
       * already let go - the classic clutch-dump. A real launch control backs
       * off on exactly this signal.
       */
      if (rate > 0 && drivenSlip > C.launchSlipLimit) {
        rate *= Math.max(0, 1 - (drivenSlip - C.launchSlipLimit) / C.launchSlipLimit);
      }

      this.clutchEngagement = clamp01(this.clutchEngagement + rate * dt);
      // Always creep, exactly as a DCT in Drive does with the brake released.
      this.clutchEngagement = Math.max(this.clutchEngagement, C.creepEngagement);
    }

    this.lockup = this.clutchEngagement;
  }

  /**
   * Transmit torque across the slipping clutch.
   *
   * Semi-implicit, exactly like the lock-up clutch and the LSD: compute the
   * torque that would equalise crank and input-shaft speed over this step,
   * then clamp it to the clamp load the actuator has commanded. Below the
   * limit it sticks (and we switch to the rigid path); above it, it slips at
   * precisely the Coulomb bound. Unconditionally stable at any timestep.
   *
   * Note there is NO torque multiplication here. That single fact is why the
   * Revuelto's launch feels sharp and mechanical where the BMW's converter
   * feels soft and elastic.
   */
  _solveClutch(dt, Ie) {
    const capacity = this.CLUTCH.maxTorque * this.clutchEngagement;
    const equalising = this.engineTorque + (Ie * this.clutchSlip) / dt;
    const transmitted = clamp(equalising, -capacity, capacity);

    this.pumpTorque = transmitted;
    this.turbineTorque = transmitted;
    this.torqueRatio = 1;
    this.speedRatio = this.engineOmega > 1
      ? clamp(this.inputShaftOmega / this.engineOmega, -1.5, 1.5) : 0;

    /**
     * Stick when the clamp load can hold the whole equalising torque, and the
     * two speeds have actually converged. Both conditions matter: capacity
     * alone would lock the driveline while the crank was still 3000 rpm adrift
     * and snap the car forward.
     */
    if (this.clutchEngagement > 0.985
      && Math.abs(equalising) <= capacity
      && Math.abs(this.clutchSlip) < 12) {
      this.locked = true;
    }
  }

  /* ===================================================================== */
  /* Differential                                                           */
  /* ===================================================================== */

  /**
   * Split the axle torque and add the limited-slip locking torque.
   *
   * The locking torque is solved semi-implicitly: find the torque that would
   * exactly cancel the speed difference across the axle this step, then clamp
   * it to the diff's real capacity (a preload plus a torque-sensitive bias, as
   * a clutch-pack LSD behaves). Without the clamp it would be a spool; without
   * the implicit solve it rings, and below ~150 Hz it diverges outright.
   */
  _applyDifferential(driven, axleTorque, dt) {
    const DIFFERENTIAL = this.DIFFERENTIAL;
    const n = driven.length;
    if (n === 0) return;
    const share = axleTorque / n;

    if (n !== 2 || DIFFERENTIAL.type === 'open') {
      for (let i = 0; i < n; i++) {
        driven[i].driveTorque = share;
        driven[i].reflectedInertia = this.reflectedInertia;
      }
      this.diffLockTorque = 0;
      return;
    }

    const a = driven[0], b = driven[1];
    const dOmega = a.spinVelocity - b.spinVelocity;

    // Inertia each wheel presents to the diff on this step. A staggered car
    // carries different rotating mass on each axle, so ask the wheel itself.
    const I = a.inertia + this.reflectedInertia;
    const equalising = (I * dOmega) / (2 * dt);

    const capacity = Math.min(
      DIFFERENTIAL.maxLockTorque,
      DIFFERENTIAL.preload + DIFFERENTIAL.torqueBias * Math.abs(share),
    );
    const lock = clamp(equalising, -capacity, capacity);
    this.diffLockTorque = lock;

    a.driveTorque = share - lock;
    b.driveTorque = share + lock;
    a.reflectedInertia = this.reflectedInertia;
    b.reflectedInertia = this.reflectedInertia;
  }

  /* ===================================================================== */
  /* Gear selection                                                         */
  /* ===================================================================== */

  _updateShifting(dt, throttleIn, speed) {
    const TRANSMISSION = this.TRANSMISSION;
    const ENGINE = this.ENGINE;
    if (this.shiftCooldown > 0) this.shiftCooldown -= dt;

    if (this.shifting) {
      this.shiftTimer -= dt;
      if (this.shiftTimer <= 0) {
        this.gear = this.pendingGear;
        this.shifting = false;
        this.shiftCooldown = TRANSMISSION.shiftCooldown;
      }
      return;
    }

    if (!this.autoShift || this.gear <= 0 || this.shiftCooldown > 0) return;

    /**
     * Never shift again while the clutch is still taking up. With the clutch
     * open the crank is unloaded and sits on the limiter, so the shift logic
     * reads a redline it is not actually pulling and upshifts straight through
     * the gearbox - the car ends up in top at 60 km/h, driving on the electric
     * axle alone. Waiting for engagement is what a real ECU does anyway.
     */
    if (this.coupling === 'clutch' && this.clutchEngagement < 0.85) return;

    const rpm = this.rpm;
    const maxGear = TRANSMISSION.gearRatios.length;

    // The upshift point rises with load: cruising shifts early, full throttle
    // holds on to the redline.
    const upAt = TRANSMISSION.upshiftRpmLight
      + (TRANSMISSION.upshiftRpm - TRANSMISSION.upshiftRpmLight) * clamp01(throttleIn);

    if (this.gear < maxGear && rpm > upAt) {
      this.requestGear(this.gear + 1);
      return;
    }

    if (this.gear > 1) {
      const thisRatio = TRANSMISSION.gearRatios[this.gear - 1];
      const nextRatio = TRANSMISSION.gearRatios[this.gear - 2];
      const rpmAfter = rpm * (nextRatio / thisRatio);
      const kickdown = throttleIn > 0.82 && rpmAfter < TRANSMISSION.kickdownRpm;
      if (rpm < TRANSMISSION.downshiftRpm || kickdown) {
        // Hysteresis: never drop into a gear we would instantly shift back out
        // of, or the box oscillates between the two.
        if (rpmAfter < upAt - 350 && rpmAfter < ENGINE.limiterRpm) {
          this.requestGear(this.gear - 1);
        }
      }
    }
  }

  /** Begin a shift to `gear` (with the torque interrupt). */
  requestGear(gear) {
    if (gear === this.gear || this.shifting) return;
    this.pendingGear = gear;
    this.shifting = true;
    this.shiftTimer = this.TRANSMISSION.shiftTime;
    this.locked = false;
    /** A DCT re-engages from scratch after the swap. */
    if (this.coupling === 'clutch') this.clutchEngagement = 0;
  }

  /** Immediate selection, used by the forward/reverse shifter and by reset. */
  setGearImmediate(gear) {
    this.gear = gear;
    this.pendingGear = gear;
    this.shifting = false;
    this.shiftTimer = 0;
    this.shiftCooldown = this.TRANSMISSION.shiftCooldown;
    this.locked = false;
    this.lockup = 0;
    this.clutchEngagement = 0;
  }

  reset() {
    this.engineOmega = this.ENGINE.idleRpm * RADS_PER_RPM;
    this.throttle = 0;
    this.boost = 0;
    this.engineTorque = 0;
    this.limiterTimer = 0;
    this.onLimiter = false;
    this.idleIntegral = 0.35;
    this.speedRatio = 0;
    this.torqueRatio = this.CONVERTER ? this.CONVERTER.torqueRatio[0][1] : 1;
    this.clutchEngagement = 0;
    this.pumpTorque = 0;
    this.turbineTorque = 0;
    this.lockup = 0;
    this.locked = false;
    this.clutchSlip = 0;
    this.gear = 1;
    this.pendingGear = 1;
    this.shifting = false;
    this.shiftTimer = 0;
    this.shiftCooldown = 0;
    this.wheelTorque = 0;
    this.diffLockTorque = 0;
    this.reflectedInertia = 0;
    this.inputShaftOmega = 0;
  }
}

/* ========================================================================= */
/* Electric drive axle                                                       */
/* ========================================================================= */

/**
 * An independently-motored drive axle - the Lamborghini Revuelto's front end.
 *
 * This is NOT a second gearbox. Each wheel has its own motor through a fixed
 * reduction, which gives the axle three properties no mechanical drive has:
 *
 *   1. FULL TORQUE AT ZERO SPEED. There is no clutch to slip and no gear to be
 *      in, so the front wheels pull from the instant the throttle opens. This
 *      is most of the reason the Revuelto reaches 100 km/h in under three
 *      seconds while the rear-drive BMW needs more than six.
 *   2. CONSTANT-POWER FADE. Each motor is power limited, so above roughly
 *      130 km/h its torque falls away as 1/omega and the car quietly reverts
 *      to rear-wheel drive - which is exactly what the real one does.
 *   3. PER-WHEEL TORQUE VECTORING, because the two motors are independent.
 *      Torque is shuffled toward the outside wheel in proportion to steering
 *      demand, which tightens the line on corner exit.
 *
 * Torque is written to `wheel.electricTorque`, kept separate from the gearbox's
 * `wheel.driveTorque` so the two paths stay legible in the telemetry.
 */
export class ElectricAxle {
  /** @param {object} cfg spec.electric, or null for a car without one */
  constructor(cfg) {
    this.cfg = cfg;
    this.enabled = !!(cfg && cfg.enabled);
    /** Total torque delivered this step, N.m at the wheels. */
    this.totalTorque = 0;
    /** Instantaneous electrical power, W (negative while regenerating). */
    this.power = 0;
    this.regenerating = false;
  }

  /**
   * @param {number} dt
   * @param {Wheel[]} wheels   the motored wheels (the front pair)
   * @param {number} throttle  0..1 after traction control
   * @param {number} brake     0..1 brake pedal, for regeneration
   * @param {number} steer     -1..1 steering demand, for torque vectoring
   */
  update(dt, wheels, throttle, brake, steer) {
    this.totalTorque = 0;
    this.power = 0;
    this.regenerating = false;
    if (!this.enabled) {
      for (const w of wheels) { w.electricTorque = 0; w.reflectedInertia = 0; }
      return;
    }

    const c = this.cfg;

    for (let i = 0; i < wheels.length; i++) {
      const w = wheels[i];
      const omega = Math.abs(w.spinVelocity);

      /**
       * Torque envelope: flat at peak torque until the motor hits its power
       * limit, then constant power (T = P / omega) beyond it.
       */
      let available = c.peakTorquePerWheel;
      if (omega > 1) {
        available = Math.min(available, c.peakPowerPerWheel / omega);
      }

      /** Back off once the tyre starts spinning - the motor can react instantly. */
      const slip = w.tire.slipRatio;
      const slipScale = slip > c.slipCutoff
        ? Math.max(0, 1 - (slip - c.slipCutoff) / c.slipCutoff)
        : 1;

      /**
       * Torque vectoring: send more to the OUTSIDE wheel of the corner. A
       * right-hand turn (steer > 0) loads the left wheel, so that is the one
       * that gets the extra.
       */
      let vector = 1;
      if (c.torqueVectoring > 0) {
        const outside = w.isLeft ? steer : -steer;
        vector = 1 + clamp(outside, -1, 1) * c.torqueVectoring;
      }

      let torque = available * clamp01(throttle) * slipScale * vector;

      /** Regenerative braking on a closed throttle or under the brakes. */
      if (throttle < 0.02 && omega > 2) {
        const regen = c.regenTorque * Math.max(clamp01(brake), 0.35);
        torque -= regen * Math.sign(w.spinVelocity);
        this.regenerating = true;
      }

      w.electricTorque = torque;
      /** The rotor and its reduction spin with the wheel, so they add inertia. */
      w.reflectedInertia = c.reflectedInertia;

      this.totalTorque += torque;
      this.power += torque * w.spinVelocity;
    }
  }

  reset() {
    this.totalTorque = 0;
    this.power = 0;
    this.regenerating = false;
  }
}
