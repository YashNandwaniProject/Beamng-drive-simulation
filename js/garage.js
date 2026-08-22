/**
 * garage.js
 * ---------------------------------------------------------------------------
 * Vehicle selection and side-by-side comparison.
 *
 * EVERY NUMBER SHOWN HERE IS READ OUT OF THE VEHICLE SPEC THE PHYSICS USES.
 * Power is integrated from the engine's own torque curve, grip is the tyre
 * model's friction coefficient, weight distribution comes from the measured
 * centre of mass, and the acceleration and braking figures are the ones
 * tests/run.mjs actually records on the skidpad. Nothing is a marketing number
 * typed in by hand, so the garage cannot promise something the car will not do
 * once you are driving it (requirement #20).
 *
 * The comparison bars are normalised across the whole registry, so adding a
 * third car re-scales them automatically.
 * ---------------------------------------------------------------------------
 */

import { gsap } from 'gsap';
import { allVehicleSpecs, vehicleStats, getVehicleSpec } from './vehicles/registry.js';

/**
 * The metrics shown as comparison bars.
 * `lowerIsBetter` flips the bar so longer always means better.
 */
const METRICS = [
  { key: 'systemPs', label: 'Power', unit: ' PS', digits: 0 },
  { key: 'peakTorque', label: 'Torque', unit: ' Nm', digits: 0 },
  { key: 'powerToWeight', label: 'Power / weight', unit: ' PS/t', digits: 0 },
  { key: 'mass', label: 'Weight', unit: ' kg', digits: 0, lowerIsBetter: true },
  {
    key: 'zeroToHundred', label: '0-100 km/h', unit: ' s', digits: 1,
    lowerIsBetter: true,
  },
  {
    key: 'brakingFrom100', label: 'Braking 100-0', unit: ' m', digits: 1,
    lowerIsBetter: true,
  },
  { key: 'lateralG', label: 'Cornering grip', unit: ' G', digits: 2 },
  { key: 'topSpeedKmh', label: 'Top speed', unit: ' km/h', digits: 0 },
  { key: 'rackSpeed', label: 'Steering speed', unit: ' rad/s', digits: 2 },
  { key: 'downforce200', label: 'Downforce @200', unit: ' N', digits: 0 },
];

export class Garage {
  /**
   * @param {HTMLElement} root
   * @param {(id:string)=>void} onSelect called when the player commits
   */
  constructor(root, onSelect) {
    this.root = root;
    this.onSelect = onSelect;
    this.visible = false;
    this.selectedId = null;
    this.specs = allVehicleSpecs();
    this.stats = this.specs.map((s) => vehicleStats(s));

    /** Per-metric min/max across the registry, for the bars. */
    this.ranges = {};
    for (const m of METRICS) {
      const values = this.stats.map((s) => s[m.key]).filter(Number.isFinite);
      this.ranges[m.key] = { min: Math.min(...values), max: Math.max(...values) };
    }

    this._build();
  }

  /* ------------------------------------------------------------------ DOM */

  _build() {
    const cards = this.specs.map((spec, i) => {
      const st = this.stats[i];
      const g = spec.garage;
      return `
        <button class="garage-card" data-vehicle="${spec.id}"
                style="--card-accent:${g.accent}">
          <div class="card-badge">${g.manufacturer}</div>
          <div class="card-name">${g.model}</div>
          <div class="card-sub">${g.year} &middot; ${g.bodyStyle}</div>
          <div class="card-tagline">${g.tagline}</div>
          <div class="card-key">
            <div><b>${st.systemPs}</b><span>PS</span></div>
            <div><b>${st.peakTorque}</b><span>Nm</span></div>
            <div><b>${st.mass}</b><span>kg</span></div>
            <div><b>${st.drive}</b><span>drive</span></div>
          </div>
        </button>`;
    }).join('');

    this.root.innerHTML = `
      <div class="garage-inner">
        <header class="garage-head">
          <div>
            <h1>Garage</h1>
            <p>Select a car. Each has its own physics profile &mdash; they do not
               drive alike.</p>
          </div>
          <div class="garage-hint"><kbd>1</kbd><kbd>2</kbd> select &middot;
            <kbd>Enter</kbd> drive &middot; <kbd>G</kbd> reopen</div>
        </header>

        <div class="garage-cards">${cards}</div>

        <section class="garage-compare">
          <div class="compare-head">
            <h2>Comparison</h2>
            <div class="compare-legend" data-legend></div>
          </div>
          <div class="compare-rows" data-compare></div>
        </section>

        <section class="garage-detail" data-detail></section>

        <footer class="garage-foot">
          <div class="garage-selected" data-selected></div>
          <button class="garage-go" data-go>Drive</button>
        </footer>
      </div>`;

    this.el = {
      compare: this.root.querySelector('[data-compare]'),
      legend: this.root.querySelector('[data-legend]'),
      detail: this.root.querySelector('[data-detail]'),
      selected: this.root.querySelector('[data-selected]'),
      go: this.root.querySelector('[data-go]'),
    };
    /**
     * Scoped to the cards specifically: the comparison bars also carry a
     * data-vehicle attribute, and a bare attribute selector would wire click
     * handlers onto all of them too.
     */
    this.cards = [...this.root.querySelectorAll('.garage-card[data-vehicle]')];

    for (const card of this.cards) {
      card.addEventListener('click', () => this.select(card.dataset.vehicle));
      card.addEventListener('dblclick', () => this.confirm());
    }
    this.el.go.addEventListener('click', () => this.confirm());

    this.el.legend.innerHTML = this.specs.map((s) => `
      <span class="legend-item">
        <i style="background:${s.garage.accent}"></i>${s.shortName}
      </span>`).join('');

    this._buildCompare();
    this.root.style.display = 'none';
  }

  _buildCompare() {
    this.el.compare.innerHTML = METRICS.map((m) => {
      const bars = this.specs.map((spec, i) => {
        const st = this.stats[i];
        const v = st[m.key];
        const { min, max } = this.ranges[m.key];
        const span = max - min;
        /**
         * Scale to 22..100 % so the weaker car still shows a readable bar,
         * and flip it where a smaller number is the better one.
         */
        let frac = span > 1e-9 ? (v - min) / span : 1;
        if (m.lowerIsBetter) frac = 1 - frac;
        const pct = 22 + frac * 78;
        return `
          <div class="cmp-bar" data-metric="${m.key}" data-vehicle="${spec.id}">
            <i style="width:${pct.toFixed(1)}%;background:${spec.garage.accent}"></i>
            <span>${v.toFixed(m.digits)}${m.unit}</span>
          </div>`;
      }).join('');
      return `<div class="cmp-row"><label>${m.label}</label>
                <div class="cmp-bars">${bars}</div></div>`;
    }).join('');
  }

  /* --------------------------------------------------------------- select */

  select(id) {
    const spec = getVehicleSpec(id);
    this.selectedId = spec.id;

    for (const card of this.cards) {
      card.classList.toggle('selected', card.dataset.vehicle === spec.id);
    }
    for (const bar of this.root.querySelectorAll('.cmp-bar')) {
      bar.classList.toggle('dim', bar.dataset.vehicle !== spec.id);
    }

    const st = vehicleStats(spec);
    this.el.selected.innerHTML =
      `<b>${spec.displayName}</b><span>${st.gearbox} &middot; ${st.drive}</span>`;
    this.root.style.setProperty('--card-accent', spec.garage.accent);
    this.el.detail.innerHTML = this._detailHtml(spec, st);

    gsap.fromTo(this.el.detail,
      { opacity: 0, y: 8 }, { opacity: 1, y: 0, duration: 0.28, ease: 'power2.out' });
    return spec;
  }

  /** The full handling profile for the selected car. */
  _detailHtml(spec, st) {
    const row = (label, value) => `<div><span>${label}</span><b>${value}</b></div>`;
    const e = spec.engine;
    const s = spec.suspension;

    return `
      <h3>${spec.displayName} &mdash; handling profile</h3>
      <div class="detail-grid">
        <div class="detail-col">
          <h4>Powertrain</h4>
          ${row('Engine', `${st.enginePs} PS @ ${st.peakPowerRpm} rpm`)}
          ${row('Torque', `${st.peakTorque} Nm @ ${st.peakTorqueRpm} rpm`)}
          ${row('Redline', `${e.redlineRpm} rpm`)}
          ${row('Induction', st.turbo ? 'Turbocharged' : 'Naturally aspirated')}
          ${row('Electric', st.electric ? 'Front e-axle' : 'None')}
          ${row('System output', `${st.systemPs} PS (${st.systemKw} kW)`)}
          ${row('Gearbox', st.gearbox)}
          ${row('Shift time', `${st.shiftTimeMs} ms`)}
          ${row('Drive', st.drive)}
        </div>
        <div class="detail-col">
          <h4>Chassis</h4>
          ${row('Mass', `${st.mass} kg`)}
          ${row('Weight distribution', `${st.frontWeightPct} / ${100 - st.frontWeightPct}`)}
          ${row('Wheelbase', `${st.wheelbase.toFixed(3)} m`)}
          ${row('Spring rate F/R', `${(s.front.springRate / 1000).toFixed(0)} / `
            + `${(s.rear.springRate / 1000).toFixed(0)} kN/m`)}
          ${row('Anti-roll F/R', `${(s.front.antiRollRate / 1000).toFixed(0)} / `
            + `${(s.rear.antiRollRate / 1000).toFixed(0)} kN/m`)}
          ${row('Suspension travel', `${((s.front.maxCompression
            + s.front.maxDroop) * 1000).toFixed(0)} mm front`)}
          ${row('Downforce @200 km/h', `${st.downforce200.toFixed(0)} N`)}
        </div>
        <div class="detail-col">
          <h4>Tyres &amp; steering</h4>
          ${row('Peak friction', st.mu.toFixed(2))}
          ${row('Tyre pressure', `${spec.tyre.nominalPressure} psi`)}
          ${row('Tyre radius F/R', `${spec.model.radiusFront.toFixed(3)} / `
            + `${spec.model.radiusRear.toFixed(3)} m`)}
          ${row('Peak slip angle', `${(spec.tyre.peakSlipAngle * 180 / Math.PI).toFixed(1)}&deg;`)}
          ${row('Steering lock', `${st.steerLockDeg.toFixed(0)}&deg;`)}
          ${row('Rack speed', `${st.rackSpeed.toFixed(2)} rad/s`)}
          ${row('Ackermann', `${(spec.steering.ackermann * 100).toFixed(0)} %`)}
          ${row('Interior wheel', spec.model.steeringWheel ? 'Animated' : 'Static')}
        </div>
      </div>
      <p class="detail-note">
        Measured on the flat skidpad by <code>npm test</code>:
        0&ndash;100 km/h in <b>${st.zeroToHundred.toFixed(1)} s</b>,
        100&ndash;0 in <b>${st.brakingFrom100.toFixed(1)} m</b>,
        peak <b>${st.lateralG.toFixed(2)} G</b> lateral.
      </p>`;
  }

  confirm() {
    if (!this.selectedId) return;
    const id = this.selectedId;
    this.hide().then(() => this.onSelect(id));
  }

  /* --------------------------------------------------------------- visible */

  show(currentId) {
    this.visible = true;
    this.root.style.display = '';
    this.select(currentId || this.selectedId || this.specs[0].id);
    gsap.fromTo(this.root, { opacity: 0 }, { opacity: 1, duration: 0.3 });
    gsap.fromTo(this.root.querySelectorAll('.garage-card'),
      { opacity: 0, y: 26 },
      { opacity: 1, y: 0, duration: 0.45, stagger: 0.08, ease: 'power3.out' });
    gsap.fromTo(this.root.querySelectorAll('.cmp-row'),
      { opacity: 0, x: -14 },
      { opacity: 1, x: 0, duration: 0.35, stagger: 0.03, ease: 'power2.out', delay: 0.12 });
  }

  hide() {
    this.visible = false;
    return new Promise((resolve) => {
      gsap.to(this.root, {
        opacity: 0,
        duration: 0.28,
        ease: 'power2.inOut',
        onComplete: () => { this.root.style.display = 'none'; resolve(); },
      });
    });
  }

  /** Keyboard handling while the garage is open. Returns true if consumed. */
  handleKey(code) {
    if (!this.visible) return false;
    const idx = this.specs.findIndex((s) => s.id === this.selectedId);
    if (code === 'Enter' || code === 'Space') { this.confirm(); return true; }
    if (code === 'ArrowRight' || code === 'ArrowDown' || code === 'KeyD') {
      this.select(this.specs[(idx + 1) % this.specs.length].id);
      return true;
    }
    if (code === 'ArrowLeft' || code === 'ArrowUp' || code === 'KeyA') {
      this.select(this.specs[(idx - 1 + this.specs.length) % this.specs.length].id);
      return true;
    }
    const n = { Digit1: 0, Digit2: 1, Digit3: 2, Digit4: 3 }[code];
    if (n !== undefined && this.specs[n]) { this.select(this.specs[n].id); return true; }
    return false;
  }
}
