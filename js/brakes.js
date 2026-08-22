/**
 * brakes.js
 * ---------------------------------------------------------------------------
 * Brake system: pedal hydraulics, front/rear bias, handbrake, ABS and
 * traction control. Requirements #10, #11, #12.
 *
 * Nothing here subtracts velocity from the car. The system only produces a
 * BRAKE TORQUE per wheel; the wheel then decelerates, the tyre develops
 * negative slip, and the resulting longitudinal tyre force is what slows the
 * vehicle - which is also what generates the forward weight transfer and the
 * front-end dive.
 * ---------------------------------------------------------------------------
 */

import { clamp, clamp01, damp, lerp } from './mathUtils.js';


export class BrakeSystem {
  /** @param {object} spec the vehicle spec (js/vehicles/*.js) */
  constructor(spec) {
    /** Vehicle-specific brake tuning. */
    this.cfg = spec.brakes;
    const BRAKES = spec.brakes;
    /** Smoothed pedal travel, 0..1. */
    this.pedal = 0;
    /** Handbrake lever, 0..1. */
    this.handbrake = 0;

    /**
     * Per-wheel hydraulic pressure modulation from ABS, 0..1.
     * Keyed in the same order as the vehicle's wheel array.
     */
    this.pressure = [1, 1, 1, 1];
    this.absActive = [false, false, false, false];
    this.absEnabled = BRAKES.abs.enabled;

    /** Traction control: fraction of engine torque cut, 0..1. */
    this.tcCut = 0;
    this.tcEnabled = BRAKES.tractionControl.enabled;
    this.tcActive = false;

    this.appliedTorque = [0, 0, 0, 0];

    /**
     * The ABS controller runs on its own fixed clock, not on the physics step.
     * A real pump actuates its valves at a fixed rate (and simply HOLDS
     * pressure in between), so modelling it that way is both more faithful and
     * removes the last timestep dependence from the braking model: an
     * integrator driven by the physics dt would have four times the loop gain
     * per update at 60 Hz as at 240 Hz, over-modulate, and lengthen the
     * stopping distance by 15 %.
     */
    this._absAccum = 0;
  }

  /** True while ABS is modulating at least one corner (for the HUD lamp). */
  get anyAbsActive() {
    return this.absActive[0] || this.absActive[1] || this.absActive[2] || this.absActive[3];
  }

  /**
   * Compute and assign the brake torque for every wheel.
   *
   * @param {number} dt
   * @param {number} brakeInput   0..1 driver demand on the brake pedal
   * @param {number} handbrakeIn  0..1 handbrake lever
   * @param {Wheel[]} wheels      all four wheels
   * @param {number} speed        absolute road speed, m/s
   */
  update(dt, brakeInput, handbrakeIn, wheels, speed) {
    const BRAKES = this.cfg;
    this.pedal = damp(this.pedal, clamp01(brakeInput), BRAKES.pedalResponseTime * 0.5, dt);
    this.handbrake = damp(this.handbrake, clamp01(handbrakeIn), 0.035, dt);

    const abs = BRAKES.abs;
    const absAvailable = this.absEnabled && speed > abs.minSpeed;

    /**
     * Advance the pump clock and regulate only on its own tick, integrating
     * over the time that has actually elapsed since the last one. Using the
     * elapsed interval rather than a nominal 1/updateHz matters when the
     * physics step is longer than the pump period (60 Hz physics vs a 100 Hz
     * pump): otherwise the controller would tick every step but only ever
     * integrate 10 ms of gain per 16.7 ms of real time, respond 40 % too
     * slowly, and lengthen the stop.
     */
    const absStep = 1 / abs.updateHz;
    this._absAccum += dt;
    let absDt = 0;
    if (this._absAccum >= absStep) {
      absDt = this._absAccum;
      this._absAccum = 0;
    }

    for (let i = 0; i < wheels.length; i++) {
      const w = wheels[i];

      const maxTorque = w.isFront ? BRAKES.maxTorqueFront : BRAKES.maxTorqueRear;
      let torque = this.pedal * maxTorque;

      /* ---------------------------------------------------------- ABS ---
       * Each corner is regulated independently (requirement #11): a wheel on
       * ice releases while the other three keep braking. The controller is a
       * rate-limited bang-bang, which is what a real ABS pump does - hence the
       * characteristic pulsing rather than a smooth reduction.
       */
      if (absAvailable && torque > 1) {
        // Proportional regulation onto the target slip. A pure bang-bang
        // controller (dump, hold, re-apply) is what early ABS did and it costs
        // a lot of distance, because the wheel spends half its time either
        // side of the peak instead of on it. Modern systems close the loop on
        // slip; the asymmetric gains still give the characteristic pulsing.
        if (absDt > 0) {
          const err = w.tire.slipRatio - abs.targetSlip;
          const rate = err < 0 ? abs.releaseRate : abs.applyRate;
          this.pressure[i] = clamp(
            this.pressure[i] + err * rate * absDt, abs.minPressure, 1,
          );
        }
        this.absActive[i] = this.pressure[i] < 0.97;
      } else {
        this.pressure[i] = 1;
        this.absActive[i] = false;
      }

      torque *= this.pressure[i];

      /* ---------------------------------------------------- handbrake ---
       * Rear axle only, and deliberately NOT regulated by ABS: locking the
       * rears is the whole point (requirement #12).
       */
      if (!w.isFront) {
        torque += this.handbrake * BRAKES.handbrakeTorque;
      }

      w.brakeTorque = torque;
      this.appliedTorque[i] = torque;
    }
  }

  /**
   * Traction control. Returns the throttle the engine should actually get.
   * Cuts torque when a driven wheel spins up past the peak of its slip curve.
   */
  applyTractionControl(dt, drivenWheels, throttleIn) {
    const tc = this.cfg.tractionControl;
    if (!this.tcEnabled) {
      this.tcCut = 0;
      this.tcActive = false;
      return throttleIn;
    }

    let worst = 0;
    for (let i = 0; i < drivenWheels.length; i++) {
      worst = Math.max(worst, drivenWheels[i].tire.slipRatio);
    }

    const err = worst - tc.targetSlip;
    if (err > 0 && throttleIn > 0.02) {
      this.tcCut += tc.cutRate * dt * clamp01(err / tc.slipBand);
    } else if (tc.proportionalRestore) {
      /**
       * Ease the cut back in proportion to how far BELOW the target the slip
       * has fallen. A flat restore rate makes the controller hunt: it dumps
       * the cut entirely, the tyre lets go again, and the car surges in and
       * out of traction. Scaling the restore lets it settle just under the
       * target instead, which is how a modern system behaves.
       *
       * Opt-in per vehicle, so cars tuned against the flat restore keep it.
       */
      this.tcCut -= tc.restoreRate * dt * clamp01(-err / tc.slipBand);
    } else {
      this.tcCut -= tc.restoreRate * dt;
    }
    this.tcCut = clamp01(this.tcCut);
    this.tcActive = this.tcCut > 0.03;

    return throttleIn * lerp(1, tc.minThrottleCut, this.tcCut);
  }

  reset() {
    this.pedal = 0;
    this.handbrake = 0;
    this.tcCut = 0;
    this.tcActive = false;
    this._absAccum = 0;
    for (let i = 0; i < 4; i++) {
      this.pressure[i] = 1;
      this.absActive[i] = false;
      this.appliedTorque[i] = 0;
    }
  }
}
