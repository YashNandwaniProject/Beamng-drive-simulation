/**
 * audio.js
 * ---------------------------------------------------------------------------
 * Procedural engine audio, synthesised per vehicle from its physics state.
 *
 * Nothing here is a sample. The engine note is built from a harmonic stack
 * whose fundamental is the engine's actual FIRING FREQUENCY:
 *
 *     f = rpm / 60 * cylinders / 2
 *
 * which is why the two cars sound related to their mechanical layout rather
 * than merely different. A V12 fires six times per revolution against the
 * BMW six's three, so at the same engine speed its fundamental is an octave
 * higher - and it revs to 9,500 rather than 6,750. On top of that:
 *
 *   BMW        strong 2nd order, muffled top end, audible turbo whistle
 *              that rises with boost.
 *   Revuelto   bright high harmonics, a much wider open filter, NO turbo at
 *              all, plus a faint electric whine from the front axle that no
 *              other car in the game has.
 *
 * There are no audio assets to ship and nothing to license.
 *
 * Browsers will not start an AudioContext without a user gesture, so the graph
 * is built lazily and `resume()` is called on the first key press or click.
 * ---------------------------------------------------------------------------
 */

import { clamp, clamp01, lerp } from './mathUtils.js';

/** White noise, generated once and shared by every noise layer. */
function makeNoiseBuffer(ctx, seconds = 2) {
  const n = Math.floor(ctx.sampleRate * seconds);
  const buffer = ctx.createBuffer(1, n, ctx.sampleRate);
  const data = buffer.getChannelData(0);
  let last = 0;
  for (let i = 0; i < n; i++) {
    // Lightly low-passed white noise: less harsh than raw, still broadband.
    const white = Math.random() * 2 - 1;
    last = (last + 0.03 * white) / 1.03;
    data[i] = last * 3.2;
  }
  return buffer;
}

export class EngineAudio {
  constructor() {
    this.ctx = null;
    this.enabled = true;
    this.started = false;
    this.masterVolume = 0.55;
    this.spec = null;

    this._voices = [];
    this._shiftEnv = 0;
    this._lastGear = null;
    this._skid = 0;
  }

  /* ------------------------------------------------------------- lifecycle */

  /** Build the graph. Safe to call repeatedly; only the first call does work. */
  _ensureContext() {
    if (this.ctx) return true;
    const Ctx = window.AudioContext || window.webkitAudioContext;
    if (!Ctx) return false;
    this.ctx = new Ctx();
    this._noise = makeNoiseBuffer(this.ctx);

    this.master = this.ctx.createGain();
    this.master.gain.value = 0;                 // faded in once running
    this.master.connect(this.ctx.destination);

    return true;
  }

  /** Call from a user gesture (keydown / click). */
  resume() {
    if (!this.enabled) return;
    if (!this._ensureContext()) return;
    if (this.ctx.state === 'suspended') this.ctx.resume();
    this.started = true;
  }

  /**
   * Point the synth at a vehicle. Tears down the previous voices, so switching
   * cars in the garage genuinely changes the sound rather than layering it.
   */
  setVehicle(spec) {
    this.spec = spec;
    if (!this._ensureContext()) return;
    this._teardown();
    this._build(spec.audio);
  }

  _teardown() {
    for (const v of this._voices) {
      try { v.osc.stop(); } catch (e) { /* already stopped */ }
      try { v.osc.disconnect(); v.gain.disconnect(); } catch (e) { /* gone */ }
    }
    this._voices = [];
    for (const key of ['engineBus', 'noiseSrc', 'noiseGain', 'noiseFilter',
      'turboOsc', 'turboGain', 'whineOsc', 'whineGain',
      'skidSrc', 'skidGain', 'skidFilter', 'filter']) {
      const node = this[key];
      if (!node) continue;
      try { if (node.stop) node.stop(); } catch (e) { /* not a source */ }
      try { node.disconnect(); } catch (e) { /* already gone */ }
      this[key] = null;
    }
  }

  _build(cfg) {
    const ctx = this.ctx;

    /* --- engine bus: harmonic stack -> low-pass -> master ---------------- */
    this.filter = ctx.createBiquadFilter();
    this.filter.type = 'lowpass';
    this.filter.frequency.value = cfg.filterClosed;
    this.filter.Q.value = 0.7;
    this.filter.connect(this.master);

    this.engineBus = ctx.createGain();
    this.engineBus.gain.value = cfg.baseGain;
    this.engineBus.connect(this.filter);

    for (const [order, gain, type] of cfg.harmonics) {
      const osc = ctx.createOscillator();
      osc.type = type;
      osc.frequency.value = 60;
      if (cfg.detune) osc.detune.value = (Math.random() - 0.5) * cfg.detune * 2;
      const g = ctx.createGain();
      g.gain.value = gain;
      osc.connect(g);
      g.connect(this.engineBus);
      osc.start();
      this._voices.push({ osc, gain: g, order, baseGain: gain });
    }

    /* --- induction / exhaust noise bed ----------------------------------- */
    this.noiseSrc = ctx.createBufferSource();
    this.noiseSrc.buffer = this._noise;
    this.noiseSrc.loop = true;
    this.noiseFilter = ctx.createBiquadFilter();
    this.noiseFilter.type = 'bandpass';
    this.noiseFilter.frequency.value = 400;
    this.noiseFilter.Q.value = 0.8;
    this.noiseGain = ctx.createGain();
    this.noiseGain.gain.value = 0;
    this.noiseSrc.connect(this.noiseFilter);
    this.noiseFilter.connect(this.noiseGain);
    this.noiseGain.connect(this.filter);
    this.noiseSrc.start();

    /* --- turbo whistle (BMW only) ---------------------------------------- */
    if (cfg.turbo && cfg.turbo.enabled) {
      this.turboOsc = ctx.createOscillator();
      this.turboOsc.type = 'sine';
      this.turboOsc.frequency.value = cfg.turbo.baseFreq;
      this.turboGain = ctx.createGain();
      this.turboGain.gain.value = 0;
      this.turboOsc.connect(this.turboGain);
      this.turboGain.connect(this.master);
      this.turboOsc.start();
    }

    /* --- electric drive whine (Revuelto only) ---------------------------- */
    if (cfg.electricWhine && cfg.electricWhine.enabled) {
      this.whineOsc = ctx.createOscillator();
      this.whineOsc.type = 'triangle';
      this.whineOsc.frequency.value = cfg.electricWhine.baseFreq;
      this.whineGain = ctx.createGain();
      this.whineGain.gain.value = 0;
      this.whineOsc.connect(this.whineGain);
      this.whineGain.connect(this.master);
      this.whineOsc.start();
    }

    /* --- tyre scrub ------------------------------------------------------ */
    this.skidSrc = ctx.createBufferSource();
    this.skidSrc.buffer = this._noise;
    this.skidSrc.loop = true;
    this.skidFilter = ctx.createBiquadFilter();
    this.skidFilter.type = 'bandpass';
    this.skidFilter.frequency.value = 1500;
    this.skidFilter.Q.value = 1.6;
    this.skidGain = ctx.createGain();
    this.skidGain.gain.value = 0;
    this.skidSrc.connect(this.skidFilter);
    this.skidFilter.connect(this.skidGain);
    this.skidGain.connect(this.master);
    this.skidSrc.start();

    this._lastGear = null;
  }

  /* ---------------------------------------------------------------- update */

  /**
   * @param {number} dt  real frame time
   * @param {VehiclePhysics} p
   */
  update(dt, p) {
    if (!this.ctx || !this.started || !this.spec) return;
    const cfg = this.spec.audio;
    const now = this.ctx.time || this.ctx.currentTime;
    const smooth = clamp01(dt * 18);

    const dtx = p.drivetrain;
    const rpm = clamp(dtx.rpm, 300, this.spec.engine.maxRpm);
    const throttle = clamp01(dtx.throttle);

    /* --- shift cut: a brief dip, sharper on a DCT ----------------------- */
    if (this._lastGear !== null && dtx.gearLabel !== this._lastGear) {
      this._shiftEnv = cfg.shiftBlip;
    }
    this._lastGear = dtx.gearLabel;
    this._shiftEnv = Math.max(0, this._shiftEnv - dt * 6);

    /* --- engine harmonics ------------------------------------------------ */
    const firing = (rpm / 60) * (cfg.cylinders / 2);
    for (const v of this._voices) {
      const f = clamp(firing * v.order, 20, 12000);
      v.osc.frequency.setTargetAtTime(f, now, 0.012);
      /**
       * Higher orders come up with load, which is what makes a hard-driven
       * engine sound edgy rather than merely louder.
       */
      const loadBoost = v.order >= 2 ? lerp(0.55, 1.25, throttle) : 1.0;
      v.gain.gain.setTargetAtTime(v.baseGain * loadBoost, now, 0.03);
    }

    /* --- filter opens with load ------------------------------------------ */
    const cutoff = lerp(cfg.filterClosed, cfg.filterOpen, throttle);
    this.filter.frequency.setTargetAtTime(cutoff, now, 0.04);

    /* --- noise bed scales with revs and load ----------------------------- */
    const revFrac = clamp01(rpm / this.spec.engine.redlineRpm);
    this.noiseFilter.frequency.setTargetAtTime(
      lerp(220, 1600, revFrac), now, 0.05,
    );
    this.noiseGain.gain.setTargetAtTime(
      cfg.noiseGain * lerp(0.35, 1.0, throttle) * lerp(0.4, 1.0, revFrac),
      now, 0.05,
    );

    /* --- turbo ------------------------------------------------------------ */
    if (this.turboOsc) {
      const boost = clamp01(dtx.boost);
      this.turboOsc.frequency.setTargetAtTime(
        cfg.turbo.baseFreq + boost * cfg.turbo.freqPerBoost * revFrac, now, 0.06,
      );
      this.turboGain.gain.setTargetAtTime(
        cfg.turbo.gain * boost * revFrac, now, 0.08,
      );
    }

    /* --- electric whine, tied to road speed rather than engine speed ----- */
    if (this.whineOsc) {
      const w = cfg.electricWhine;
      this.whineOsc.frequency.setTargetAtTime(
        w.baseFreq + p.speed * w.freqPerSpeed, now, 0.05,
      );
      /** Loudest under electric load, and it fades as the motors drop out. */
      const load = p.electricAxle
        ? clamp01(Math.abs(p.electricAxle.totalTorque) / 1400) : 0;
      this.whineGain.gain.setTargetAtTime(
        w.gain * load * clamp01(p.speed / 6), now, 0.06,
      );
    }

    /* --- tyre scrub ------------------------------------------------------- */
    let slide = 0;
    for (const w of p.wheels) {
      if (w.hasContact && w.tire.sliding) {
        slide = Math.max(slide, clamp01(w.tire.slideSpeed / 14));
      }
    }
    this._skid += (slide - this._skid) * smooth;
    this.skidGain.gain.setTargetAtTime(this._skid * 0.16, now, 0.04);
    this.skidFilter.frequency.setTargetAtTime(
      lerp(900, 2400, this._skid), now, 0.05,
    );

    /* --- master ----------------------------------------------------------- */
    const target = this.enabled
      ? this.masterVolume * (1 - this._shiftEnv)
      : 0;
    this.master.gain.setTargetAtTime(target, now, 0.05);
  }

  /* ---------------------------------------------------------------- control */

  setEnabled(on) {
    this.enabled = on;
    if (on) this.resume();
    else if (this.master) {
      this.master.gain.setTargetAtTime(0, this.ctx.currentTime, 0.05);
    }
    return this.enabled;
  }

  toggle() { return this.setEnabled(!this.enabled); }

  dispose() {
    this._teardown();
    if (this.ctx) {
      try { this.ctx.close(); } catch (e) { /* already closed */ }
      this.ctx = null;
    }
  }
}
