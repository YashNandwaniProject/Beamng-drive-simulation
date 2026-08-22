/**
 * suspension.js
 * ---------------------------------------------------------------------------
 * Independent spring/damper strut for one corner (requirements #13-#15).
 *
 *   F_spring  = staticLoad + k * x            (x = compression from design height)
 *   F_damper  = c * v                         (c asymmetric: bump vs rebound)
 *   F_bumpstop= progressive, only in the last few mm of travel
 *   F_total   = max(0, F_spring + F_damper + F_bumpstop)
 *
 * Travel is measured as a strut LENGTH so it maps directly onto the raycast
 * that finds the ground:
 *
 *      strut top  (fixed to the body, maxCompression above the design hub)
 *          |
 *          | L  in [0 .. maxCompression + maxDroop]
 *          |
 *      wheel hub
 *
 *   L = maxCompression        -> design ride height, x = 0
 *   L = 0                     -> fully compressed,   x = +maxCompression
 *   L = maxCompression+maxDroop -> fully extended,   x = -maxDroop
 * ---------------------------------------------------------------------------
 */

import { clamp } from './mathUtils.js';

export class Suspension {
  /** @param {object} cfg one of SUSPENSION.front / SUSPENSION.rear */
  constructor(cfg) {
    this.cfg = cfg;

    this.springRate = cfg.springRate;
    this.bumpDamping = cfg.bumpDamping;
    this.reboundDamping = cfg.reboundDamping;
    this.maxCompression = cfg.maxCompression;
    this.maxDroop = cfg.maxDroop;
    this.bumpStopZone = cfg.bumpStopZone;
    this.bumpStopRate = cfg.bumpStopRate;
    this.staticLoad = cfg.staticLoad;

    /** Strut length at design ride height. */
    this.restLength = cfg.maxCompression;
    /** Total available stroke. */
    this.maxLength = cfg.maxCompression + cfg.maxDroop;

    /* ------------------------------ live state ------------------------- */
    /** Current strut length, metres. */
    this.length = this.restLength;
    this.prevLength = this.restLength;
    /** Compression from the design ride height (+ = compressed). */
    this.displacement = 0;
    /** Rate of compression, m/s (+ = compressing). */
    this.velocity = 0;

    this.springForce = 0;
    this.damperForce = 0;
    this.bumpStopForce = 0;
    /** Extra force from the anti-roll bar, injected by the axle. */
    this.antiRollForce = 0;
    this.force = 0;

    this.onBumpStop = false;
    this.onDroopStop = false;
  }

  /**
   * Set the new strut length from the ground raycast and derive the rate.
   * `hasContact` is false when the wheel is airborne, in which case the strut
   * falls to full droop under the wheel's own weight rather than snapping.
   */
  setLength(newLength, dt, hasContact) {
    this.prevLength = this.length;
    this.length = clamp(newLength, 0, this.maxLength);

    // Compression velocity straight from the geometry: this is what lets the
    // damper feel a bump in the ROAD, not just body motion (requirement #14).
    const raw = (this.prevLength - this.length) / Math.max(dt, 1e-6);
    // Clamp against single-step spikes (kerb strikes, first ground contact).
    this.velocity = clamp(raw, -14, 14);

    this.displacement = this.restLength - this.length;
    this.onBumpStop = this.displacement > this.maxCompression - this.bumpStopZone;
    this.onDroopStop = this.displacement < -(this.maxDroop - this.bumpStopZone);
    this.hasContact = hasContact;
  }

  /**
   * Total strut force pushing the body up along its own axis, newtons.
   * Never negative: a damper cannot pull the chassis down onto the road.
   */
  computeForce() {
    this.springForce = this.staticLoad + this.springRate * this.displacement;

    // Asymmetric damping: firm on rebound so the car settles instead of
    // pogoing, softer on bump so kerbs do not kick the body.
    const c = this.velocity > 0 ? this.bumpDamping : this.reboundDamping;
    this.damperForce = c * this.velocity;

    // Progressive bump / droop stops (requirement #15).
    this.bumpStopForce = 0;
    if (this.onBumpStop) {
      const into = this.displacement - (this.maxCompression - this.bumpStopZone);
      // Quadratic rise so the stop is felt before it is hit.
      this.bumpStopForce = this.bumpStopRate * into * (into / this.bumpStopZone);
    } else if (this.onDroopStop) {
      const into = -(this.displacement + (this.maxDroop - this.bumpStopZone));
      this.bumpStopForce = -this.bumpStopRate * 0.35 * into * (into / this.bumpStopZone);
    }

    const total = this.springForce + this.damperForce
      + this.bumpStopForce + this.antiRollForce;
    this.force = this.hasContact ? Math.max(0, total) : 0;
    return this.force;
  }

  /** Fraction of the compression stroke in use, -1 (full droop) .. +1 (full bump). */
  get travelFraction() {
    return this.displacement >= 0
      ? this.displacement / this.maxCompression
      : this.displacement / this.maxDroop;
  }

  reset() {
    this.length = this.restLength;
    this.prevLength = this.restLength;
    this.displacement = 0;
    this.velocity = 0;
    this.springForce = this.staticLoad;
    this.damperForce = 0;
    this.bumpStopForce = 0;
    this.antiRollForce = 0;
    this.force = this.staticLoad;
    this.onBumpStop = false;
    this.onDroopStop = false;
  }
}

/**
 * Anti-roll bar linking the two struts of one axle.
 * The bar resists a DIFFERENCE in travel, so it stiffens the car in roll
 * without stiffening it in pure heave - which is exactly how the real part
 * changes the lateral load transfer distribution (requirements #29, #30).
 */
export class AntiRollBar {
  constructor(rate) { this.rate = rate; this.force = 0; }

  apply(leftStrut, rightStrut) {
    const delta = leftStrut.displacement - rightStrut.displacement;
    this.force = this.rate * delta;
    // Resisting roll means pushing UP on the more compressed (outside) corner
    // and letting the inside corner go light - i.e. transferring load outward.
    leftStrut.antiRollForce = this.force;
    rightStrut.antiRollForce = -this.force;
  }

  reset() {
    this.force = 0;
  }
}
