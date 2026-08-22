/**
 * hud.js
 * ---------------------------------------------------------------------------
 * The instrument cluster: analogue tachometer, digital speed, gear, pedal and
 * steering traces, a live G-meter, per-corner tyre telemetry and warning lamps.
 *
 * Everything shown here is READ from the physics state. The HUD never writes
 * back into the simulation.
 *
 * GSAP drives the presentation only - the loading screen, the pause overlay,
 * lamp flashes and the camera-change toast (requirement #42). It is never used
 * to fake a physical quantity.
 * ---------------------------------------------------------------------------
 */

import { gsap } from 'gsap';
import { clamp, clamp01, damp, lerp } from './mathUtils.js';

import { CONTROL_HELP } from './input.js';

/** Tachometer sweep, degrees, measured from straight up. */
const GAUGE_START = -135;
const GAUGE_END = 135;
const GAUGE_RADIUS = 86;
const GAUGE_CIRC = 2 * Math.PI * GAUGE_RADIUS;
const GAUGE_SPAN = (GAUGE_END - GAUGE_START) / 360;

/** Two decimals, without the misleading "-0.00" for tiny negative values. */
const signed = (x) => (Math.abs(x) < 0.005 ? '0.00' : x.toFixed(2));

export class Hud {
  /**
   * @param {HTMLElement} root
   * @param {object} spec the vehicle spec, for the redline and the name plate
   */
  constructor(root, spec) {
    this.root = root;
    this.spec = spec;
    this.visible = true;

    /** Smoothed display values so the needles do not buzz. */
    this._rpm = 0;
    this._speed = 0;
    this._gLat = 0;
    this._gLong = 0;

    this._lastGear = '';
    this._toastTimer = null;

    this._build();
  }

  /* ------------------------------------------------------------------ DOM */

  _build() {
    const ENGINE = this.spec.engine;
    const redlineFrac = clamp01(ENGINE.redlineRpm / ENGINE.maxRpm);

    this.root.innerHTML = `
      <div class="hud-corner hud-tyres">
        <div class="hud-label">Tyres</div>
        <div class="tyre-grid">
          ${['FL', 'FR', 'RL', 'RR'].map((id) => `
            <div class="tyre" data-tyre="${id}">
              <div class="tyre-id">${id}</div>
              <div class="tyre-bar"><i data-load></i></div>
              <div class="tyre-num" data-psi>32.0</div>
              <div class="tyre-num tyre-temp" data-temp>72&deg;</div>
            </div>`).join('')}
        </div>
        <div class="hud-row"><span>Surface</span><b data-surface>Asphalt</b></div>
        <div class="hud-row"><span>Grip used</span><b data-grip>0 %</b></div>
      </div>

      <div class="hud-corner hud-gmeter">
        <div class="hud-label">G-force</div>
        <svg viewBox="0 0 120 120" class="gmeter">
          <circle cx="60" cy="60" r="54" class="g-ring"/>
          <circle cx="60" cy="60" r="36" class="g-ring g-ring-inner"/>
          <circle cx="60" cy="60" r="18" class="g-ring g-ring-inner"/>
          <line x1="6" y1="60" x2="114" y2="60" class="g-cross"/>
          <line x1="60" y1="6" x2="60" y2="114" class="g-cross"/>
          <circle cx="60" cy="60" r="6" class="g-dot" data-gdot/>
        </svg>
        <div class="g-readout">
          <span>LON <b data-glong>0.00</b></span>
          <span>LAT <b data-glat>0.00</b></span>
          <span>VRT <b data-gvert>1.00</b></span>
        </div>
      </div>

      <div class="hud-cluster">
        <svg viewBox="0 0 200 200" class="tacho">
          <circle cx="100" cy="100" r="${GAUGE_RADIUS}" class="tacho-track"
                  stroke-dasharray="${GAUGE_CIRC * GAUGE_SPAN} ${GAUGE_CIRC}"
                  transform="rotate(${GAUGE_START - 90} 100 100)"/>
          <circle cx="100" cy="100" r="${GAUGE_RADIUS}" class="tacho-redline"
                  stroke-dasharray="${GAUGE_CIRC * GAUGE_SPAN * (1 - redlineFrac)} ${GAUGE_CIRC}"
                  stroke-dashoffset="${-GAUGE_CIRC * GAUGE_SPAN * redlineFrac}"
                  transform="rotate(${GAUGE_START - 90} 100 100)"/>
          <circle cx="100" cy="100" r="${GAUGE_RADIUS}" class="tacho-fill" data-tacho
                  stroke-dasharray="0 ${GAUGE_CIRC}"
                  transform="rotate(${GAUGE_START - 90} 100 100)"/>
          <g data-needle transform="rotate(${GAUGE_START} 100 100)">
            <!-- Starts outside the digital readout so it never crosses it. -->
            <line x1="100" y1="66" x2="100" y2="26" class="tacho-needle"/>
          </g>
          <circle cx="100" cy="100" r="44" class="tacho-dial"/>
        </svg>
        <div class="cluster-readout">
          <div class="speed"><b data-speed>0</b><span>km/h</span></div>
          <div class="gear" data-gear>1</div>
          <div class="rpm"><b data-rpm>780</b><span>rpm</span></div>
        </div>
      </div>

      <div class="hud-inputs">
        <div class="pedal"><i data-throttle></i><span>W</span></div>
        <div class="pedal pedal-brake"><i data-brake></i><span>S</span></div>
        <div class="steer-bar"><i data-steer></i></div>
        <div class="steer-label" data-steerangle>0.0&deg;</div>
      </div>

      <div class="hud-lamps">
        <div class="lamp" data-lamp="abs">ABS</div>
        <div class="lamp" data-lamp="tc">TC</div>
        <div class="lamp" data-lamp="hand">HAND</div>
        <div class="lamp" data-lamp="spin">SPIN</div>
        <div class="lamp" data-lamp="lock">LOCK</div>
        <div class="lamp" data-lamp="air">AIR</div>
      </div>

      <div class="hud-vehicle">
        <b data-vehicle>${this.spec.displayName}</b>
        <span data-vehicledrive>${this.spec.drive.toUpperCase()}</span>
      </div>
      <div class="hud-mode"><span data-camera>Chase</span></div>
      <div class="hud-players" data-players></div>
      <div class="hud-toast" data-toast></div>
    `;

    const q = (sel) => this.root.querySelector(sel);
    this.el = {
      tacho: q('[data-tacho]'),
      needle: q('[data-needle]'),
      speed: q('[data-speed]'),
      rpm: q('[data-rpm]'),
      gear: q('[data-gear]'),
      throttle: q('[data-throttle]'),
      brake: q('[data-brake]'),
      steer: q('[data-steer]'),
      steerAngle: q('[data-steerangle]'),
      gdot: q('[data-gdot]'),
      glong: q('[data-glong]'),
      glat: q('[data-glat]'),
      gvert: q('[data-gvert]'),
      surface: q('[data-surface]'),
      grip: q('[data-grip]'),
      camera: q('[data-camera]'),
      toast: q('[data-toast]'),
      vehicle: q('[data-vehicle]'),
      vehicleDrive: q('[data-vehicledrive]'),
      players: q('[data-players]'),
    };

    this.lamps = {};
    for (const el of this.root.querySelectorAll('[data-lamp]')) {
      this.lamps[el.dataset.lamp] = el;
    }

    this.tyreEls = {};
    for (const el of this.root.querySelectorAll('[data-tyre]')) {
      this.tyreEls[el.dataset.tyre] = {
        root: el,
        load: el.querySelector('[data-load]'),
        psi: el.querySelector('[data-psi]'),
        temp: el.querySelector('[data-temp]'),
      };
    }

    // Entrance animation.
    gsap.from(this.root.querySelectorAll('.hud-corner, .hud-cluster, .hud-inputs, .hud-lamps'), {
      opacity: 0,
      y: 18,
      duration: 0.55,
      stagger: 0.06,
      ease: 'power2.out',
      delay: 0.15,
    });
  }

  /* --------------------------------------------------------------- update */

  /**
   * @param {VehiclePhysics} v
   * @param {object} input   the raw driver demand, for the pedal traces
   * @param {number} dt      real frame time
   */
  update(v, input, dt) {
    if (!this.visible) return;
    const d = this.el;

    const ENGINE = this.spec.engine;
    /* --- tachometer and speed ----------------------------------------- */
    this._rpm = damp(this._rpm, v.rpm, 0.045, dt);
    this._speed = damp(this._speed, Math.abs(v.speedKmh), 0.05, dt);

    const rpmFrac = clamp01(this._rpm / ENGINE.maxRpm);
    d.tacho.setAttribute('stroke-dasharray',
      `${GAUGE_CIRC * GAUGE_SPAN * rpmFrac} ${GAUGE_CIRC}`);
    d.needle.setAttribute('transform',
      `rotate(${lerp(GAUGE_START, GAUGE_END, rpmFrac)} 100 100)`);

    d.speed.textContent = Math.round(this._speed);
    d.rpm.textContent = Math.round(this._rpm);

    const gear = v.gear;
    if (gear !== this._lastGear) {
      d.gear.textContent = gear;
      // A quick pop on every shift - purely presentational.
      gsap.fromTo(d.gear, { scale: 1.35, opacity: 0.4 },
        { scale: 1, opacity: 1, duration: 0.22, ease: 'back.out(2.4)' });
      this._lastGear = gear;
    }
    d.gear.classList.toggle('gear-reverse', gear === 'R');
    this.root.classList.toggle('at-redline', v.rpm > ENGINE.redlineRpm);

    /* --- pedals and steering ------------------------------------------- */
    d.throttle.style.height = `${clamp01(v.drivetrain.throttle) * 100}%`;
    d.brake.style.height = `${clamp01(v.brakes.pedal) * 100}%`;
    const steerFrac = clamp(v.steering.angle / v.steering.availableLock, -1, 1);
    d.steer.style.transform = `translateX(${steerFrac * 50}%)`;
    d.steerAngle.textContent = `${v.steering.angleDegrees.toFixed(1)}°`;

    /* --- G-meter -------------------------------------------------------- */
    this._gLong = damp(this._gLong, v.gForceSmooth.x, 0.05, dt);
    this._gLat = damp(this._gLat, v.gForceSmooth.z, 0.05, dt);
    // 54 px of radius represents 1.5 G.
    const px = clamp(this._gLat / 1.5, -1, 1) * 54;
    const py = clamp(-this._gLong / 1.5, -1, 1) * 54;
    d.gdot.setAttribute('cx', (60 + px).toFixed(1));
    d.gdot.setAttribute('cy', (60 + py).toFixed(1));
    // signed() keeps a value hovering just below zero from reading "-0.00".
    d.glong.textContent = signed(this._gLong);
    d.glat.textContent = signed(this._gLat);
    d.gvert.textContent = signed(v.gForceSmooth.y);

    /* --- tyres ---------------------------------------------------------- */
    for (const id of ['FL', 'FR', 'RL', 'RR']) {
      const w = v.wheels.find((x) => x.id === id);
      const el = this.tyreEls[id];
      if (!w || !el) continue;

      // Load bar, normalised against twice the static corner load.
      const loadFrac = clamp01(w.normalForce / (w.suspension.staticLoad * 2));
      el.load.style.height = `${loadFrac * 100}%`;

      // Colour by how much of the friction circle the tyre is using.
      const use = clamp01(w.tire.frictionUse);
      const slipping = w.tire.combinedSlip > 1.05;
      el.load.style.background = slipping
        ? '#ff4d3d'
        : `hsl(${lerp(150, 45, use)} 78% ${lerp(46, 56, use)}%)`;
      el.root.classList.toggle('airborne', !w.hasContact);

      el.psi.textContent = w.tire.pressure.toFixed(1);
      el.psi.classList.toggle('warn',
        Math.abs(w.tire.pressure - this.spec.tyre.nominalPressure) > 7);
      el.temp.textContent = `${Math.round(w.tire.temperature)}°`;
      el.temp.classList.toggle('warn', w.tire.temperature > 125);
    }

    d.surface.textContent = v.wheelRL.surface.name;
    d.grip.textContent = `${Math.round(v.gripUsage * 100)} %`;

    /* --- lamps ---------------------------------------------------------- */
    this._lamp('abs', v.brakes.anyAbsActive, !v.brakes.absEnabled);
    this._lamp('tc', v.brakes.tcActive, !v.brakes.tcEnabled);
    this._lamp('hand', v.brakes.handbrake > 0.05);
    this._lamp('spin', v.anyWheelSpin);
    this._lamp('lock', v.anyWheelLocked);
    this._lamp('air', v.airborne);
  }

  /** Swap the cluster over to a different car without rebuilding the HUD. */
  setVehicle(spec) {
    this.spec = spec;
    this._lastGear = '';
    this._build();
    if (this.el.vehicle) {
      gsap.fromTo(this.el.vehicle.parentElement,
        { opacity: 0, y: -8 }, { opacity: 1, y: 0, duration: 0.35 });
    }
  }

  /** Show how many other drivers are in the session. */
  setPlayerCount(n) {
    if (!this.el.players) return;
    this.el.players.textContent = n > 0
      ? `${n} other driver${n === 1 ? '' : 's'}` : '';
    this.el.players.classList.toggle('on', n > 0);
  }

  _lamp(name, on, disabled = false) {
    const el = this.lamps[name];
    if (!el) return;
    el.classList.toggle('on', !!on && !disabled);
    el.classList.toggle('off', disabled);
  }

  /* --------------------------------------------------------------- extras */

  /** Short message in the middle of the screen (camera change, reset, ...). */
  toast(message) {
    const el = this.el.toast;
    el.textContent = message;
    gsap.killTweensOf(el);
    gsap.fromTo(el,
      { opacity: 0, y: 12 },
      {
        opacity: 1,
        y: 0,
        duration: 0.22,
        ease: 'power2.out',
        onComplete: () => {
          gsap.to(el, { opacity: 0, duration: 0.45, delay: 0.95, ease: 'power1.in' });
        },
      });
  }

  setCameraName(name) {
    this.el.camera.textContent = name;
    gsap.fromTo(this.el.camera.parentElement,
      { scale: 1.12 }, { scale: 1, duration: 0.3, ease: 'back.out(2)' });
  }

  setVisible(visible) {
    this.visible = visible;
    gsap.to(this.root, {
      opacity: visible ? 1 : 0,
      duration: 0.25,
      ease: 'power2.out',
      onStart: () => { if (visible) this.root.style.display = ''; },
      onComplete: () => { if (!visible) this.root.style.display = 'none'; },
    });
  }
}

/* ------------------------------------------------------------------------ */
/* Overlays                                                                  */
/* ------------------------------------------------------------------------ */

/** Loading screen with a real progress bar, faded out by GSAP when ready. */
export class LoadingScreen {
  constructor(root) {
    this.root = root;
    this.root.innerHTML = `
      <div class="loader-inner">
        <div class="loader-badge">BMW</div>
        <h1>Physics Driving Simulation</h1>
        <p class="loader-sub">BeamNG-inspired vehicle dynamics &middot; Three.js</p>
        <div class="loader-track"><i data-bar></i></div>
        <div class="loader-status" data-status>Loading&hellip;</div>
      </div>`;
    this.bar = this.root.querySelector('[data-bar]');
    this.status = this.root.querySelector('[data-status]');
    this._progress = 0;
  }

  setProgress(fraction, label) {
    this._progress = Math.max(this._progress, clamp01(fraction));
    gsap.to(this.bar, {
      scaleX: this._progress, duration: 0.4, ease: 'power2.out',
      transformOrigin: 'left center',
    });
    if (label) this.status.textContent = label;
  }

  fail(message) {
    this.status.textContent = message;
    this.status.classList.add('error');
    this.bar.style.background = '#e0453a';
  }

  hide() {
    return new Promise((resolve) => {
      gsap.to(this.root, {
        opacity: 0,
        duration: 0.7,
        ease: 'power2.inOut',
        onComplete: () => {
          this.root.style.display = 'none';
          resolve();
        },
      });
    });
  }
}

/** Pause overlay with the control list. */
export class PauseScreen {
  constructor(root) {
    this.root = root;
    this.root.innerHTML = `
      <div class="pause-inner">
        <h2>Paused</h2>
        <table class="controls">
          ${CONTROL_HELP.map(([k, v]) => `<tr><th>${k}</th><td>${v}</td></tr>`).join('')}
        </table>
        <p class="pause-hint">Press <kbd>Esc</kbd> to resume</p>
      </div>`;
    this.root.style.display = 'none';
    this.visible = false;
  }

  show() {
    this.visible = true;
    this.root.style.display = '';
    gsap.fromTo(this.root, { opacity: 0 }, { opacity: 1, duration: 0.25 });
    gsap.fromTo(this.root.querySelector('.pause-inner'),
      { scale: 0.94, y: 10 }, { scale: 1, y: 0, duration: 0.35, ease: 'back.out(1.6)' });
  }

  hide() {
    this.visible = false;
    gsap.to(this.root, {
      opacity: 0,
      duration: 0.2,
      onComplete: () => { this.root.style.display = 'none'; },
    });
  }

  toggle() {
    if (this.visible) this.hide(); else this.show();
    return this.visible;
  }
}
