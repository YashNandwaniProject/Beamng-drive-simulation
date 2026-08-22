/**
 * debug.js
 * ---------------------------------------------------------------------------
 * Developer overlay (requirement #47).
 *
 * Two halves:
 *   - a text panel with every value the brief asks to be inspectable, laid out
 *     per wheel so left/right and front/rear can be compared at a glance;
 *   - 3D gizmos in the scene: contact points, ground normals, the longitudinal
 *     and lateral tyre force vectors, suspension struts, steering axes, the
 *     centre of mass and the velocity vector.
 *
 * All of it is read-only. Toggle with H or F1.
 * ---------------------------------------------------------------------------
 */

import * as THREE from 'three';
import { RAD2DEG } from './mathUtils.js';
import { SIM } from './config.js';

/** Force vectors are drawn at this many metres per 10 kN. */
const FORCE_SCALE = 1 / 10000;
const WHEEL_IDS = ['FL', 'FR', 'RL', 'RR'];

/* Lines per wheel: normal, Fx, Fy, strut, steering axis. */
const LINES_PER_WHEEL = 5;
const EXTRA_LINES = 5;              // CoM cross (3) + velocity + heading
const TOTAL_VERTS = (LINES_PER_WHEEL * 4 + EXTRA_LINES) * 2;

export class DebugSystem {
  /**
   * @param {HTMLElement} panelRoot
   * @param {THREE.Scene} scene
   */
  constructor(panelRoot, scene) {
    this.panel = panelRoot;
    this.scene = scene;
    this.enabled = false;

    /* --------------------------------------------------------- text panel */
    this.panel.classList.add('debug-panel');
    this.panel.style.display = 'none';

    /* ------------------------------------------------------------ gizmos */
    this.gizmos = new THREE.Group();
    this.gizmos.name = 'DebugGizmos';
    this.gizmos.visible = false;
    this.gizmos.frustumCulled = false;
    scene.add(this.gizmos);

    const positions = new Float32Array(TOTAL_VERTS * 3);
    const colors = new Float32Array(TOTAL_VERTS * 3);
    this.lineGeometry = new THREE.BufferGeometry();
    this.lineGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.lineGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.lines = new THREE.LineSegments(
      this.lineGeometry,
      new THREE.LineBasicMaterial({
        vertexColors: true, depthTest: false, transparent: true, opacity: 0.95,
      }),
    );
    this.lines.renderOrder = 999;
    this.lines.frustumCulled = false;
    this.gizmos.add(this.lines);

    // Contact-patch markers.
    const dotGeo = new THREE.SphereGeometry(0.055, 10, 8);
    const dotMat = new THREE.MeshBasicMaterial({
      color: 0x36e08a, depthTest: false, transparent: true, opacity: 0.9,
    });
    this.contactDots = new THREE.InstancedMesh(dotGeo, dotMat, 4);
    this.contactDots.frustumCulled = false;
    this.contactDots.renderOrder = 999;
    this.gizmos.add(this.contactDots);

    // Centre of mass.
    this.comDot = new THREE.Mesh(
      new THREE.SphereGeometry(0.09, 12, 10),
      new THREE.MeshBasicMaterial({ color: 0xffcc33, depthTest: false }),
    );
    this.comDot.renderOrder = 999;
    this.comDot.frustumCulled = false;
    this.gizmos.add(this.comDot);

    /* -------------------------------------------------------- frame timing */
    this.fps = 0;
    this._fpsAccum = 0;
    this._fpsFrames = 0;
    this._textAccum = 0;

    this._m = new THREE.Matrix4();
    this._v = new THREE.Vector3();
    this._scale = new THREE.Vector3(1, 1, 1);
    this._quat = new THREE.Quaternion();
  }

  toggle() {
    this.enabled = !this.enabled;
    this.panel.style.display = this.enabled ? '' : 'none';
    this.gizmos.visible = this.enabled;
    return this.enabled;
  }

  /**
   * @param {VehiclePhysics} v
   * @param {number} dt          real frame time
   * @param {object} stats       { substeps, physicsMs, drawCalls, triangles }
   */
  update(v, dt, stats = {}) {
    // FPS is measured even while hidden, so it is correct the moment you open it.
    this._fpsAccum += dt;
    this._fpsFrames++;
    if (this._fpsAccum >= 0.4) {
      this.fps = this._fpsFrames / this._fpsAccum;
      this._fpsAccum = 0;
      this._fpsFrames = 0;
    }
    if (!this.enabled) return;

    this._updateGizmos(v);

    // The text panel is expensive to lay out; 12 Hz is plenty to read.
    this._textAccum += dt;
    if (this._textAccum < 1 / 12) return;
    this._textAccum = 0;
    this.panel.innerHTML = this._buildText(v, stats);
  }

  /* ---------------------------------------------------------------- text */

  _buildText(v, stats) {
    const num = (x, d = 2) => (Number.isFinite(x) ? x.toFixed(d) : '--');
    const cell = (x, d = 2) => `<td>${num(x, d)}</td>`;

    const wheelRow = (label, fn, d = 2) => `
      <tr><th>${label}</th>${WHEEL_IDS.map((id) => {
      const w = v.wheels.find((x) => x.id === id);
      return cell(fn(w), d);
    }).join('')}</tr>`;

    const dt = v.drivetrain;

    /**
     * Drivetrain read-out adapts to the car: a torque converter reports its
     * speed ratio and multiplication, a dual clutch reports clamp engagement,
     * and only a hybrid has an electric axle row at all.
     */
    const couplingRow = dt.coupling === 'clutch'
      ? `<tr><th>Clutch</th><td>${num(dt.clutchEngagement * 100, 0)} % clamped</td>
             <th>Slip</th><td>${num(dt.clutchSlip, 1)} rad/s</td></tr>`
      : `<tr><th>Converter</th><td>sr ${num(dt.speedRatio, 2)} &times;${num(dt.torqueRatio, 2)}</td>
             <th>Lock-up</th><td>${dt.locked ? 'locked' : 'slipping'}</td></tr>`;

    const electricRow = v.electricAxle && v.electricAxle.enabled
      ? `<tr><th>e-axle torque</th><td>${num(v.electricAxle.totalTorque, 0)} N&middot;m</td>
             <th>e-axle power</th><td>${num(v.electricAxle.power / 1000, 1)} kW`
        + `${v.electricAxle.regenerating ? ' (regen)' : ''}</td></tr>`
      : '';

    return `
      <div class="dbg-title">VEHICLE &mdash; ${v.displayName}</div>
      <table class="dbg-kv">
        <tr><th>Speed</th><td>${num(v.speedKmh, 1)} km/h</td>
            <th>Forward</th><td>${num(v.forwardSpeed, 2)} m/s</td></tr>
        <tr><th>Engine</th><td>${num(dt.rpm, 0)} rpm</td>
            <th>Gear</th><td>${dt.gearLabel}${dt.shifting ? ' (shift)' : ''}</td></tr>
        <tr><th>Throttle</th><td>${num(dt.throttle * 100, 0)} %</td>
            <th>Brake</th><td>${num(v.brakes.pedal * 100, 0)} %</td></tr>
        <tr><th>Engine torque</th><td>${num(dt.engineTorque, 0)} N&middot;m</td>
            <th>Axle torque</th><td>${num(dt.wheelTorque, 0)} N&middot;m</td></tr>
        ${couplingRow}
        ${electricRow}
        <tr><th>Diff lock</th><td>${num(dt.diffLockTorque, 0)} N&middot;m</td>
            <th>Drive</th><td>${v.spec.drive.toUpperCase()}</td></tr>
        <tr><th>Steering</th><td>${num(v.steering.angleDegrees, 1)}&deg;</td>
            <th>Available lock</th><td>${num(v.steering.availableLock * RAD2DEG, 1)}&deg;</td></tr>
        <tr><th>Yaw rate</th><td>${num(v.yawRate * RAD2DEG, 1)} &deg;/s</td>
            <th>Sideslip</th><td>${num(v.sideslip * RAD2DEG, 1)}&deg;</td></tr>
        <tr><th>Roll</th><td>${num(v.rollAngle * RAD2DEG, 2)}&deg;</td>
            <th>Pitch</th><td>${num(v.pitchAngle * RAD2DEG, 2)}&deg;</td></tr>
        <tr><th>G long</th><td>${num(v.gForce.x, 2)}</td>
            <th>G lat</th><td>${num(v.gForce.z, 2)}</td></tr>
        <tr><th>G vert</th><td>${num(v.gForce.y, 2)}</td>
            <th>Wheels down</th><td>${v.contactCount} / 4</td></tr>
      </table>

      <div class="dbg-title">WHEELS &mdash; ${WHEEL_IDS.join(' &middot; ')}</div>
      <table class="dbg-wheels">
        <tr><th></th>${WHEEL_IDS.map((id) => `<td class="hdr">${id}</td>`).join('')}</tr>
        ${wheelRow('&omega; rad/s', (w) => w.spinVelocity, 1)}
        ${wheelRow('&alpha; rad/s&sup2;', (w) => w.spinAcceleration, 0)}
        ${wheelRow('Steer &deg;', (w) => w.steerAngle * RAD2DEG, 1)}
        ${wheelRow('Slip ratio', (w) => w.tire.slipRatio, 3)}
        ${wheelRow('Slip angle &deg;', (w) => w.tire.slipAngle * RAD2DEG, 2)}
        ${wheelRow('Combined slip', (w) => w.tire.combinedSlip, 2)}
        ${wheelRow('Fz N', (w) => w.normalForce, 0)}
        ${wheelRow('Fx N', (w) => w.longForce, 0)}
        ${wheelRow('Fy N', (w) => w.latForce, 0)}
        ${wheelRow('Mz N&middot;m', (w) => w.tire.alignTorque, 1)}
        ${wheelRow('&mu;', (w) => w.tire.mu, 3)}
        ${wheelRow('Susp mm', (w) => w.suspension.displacement * 1000, 1)}
        ${wheelRow('Susp m/s', (w) => w.suspension.velocity, 2)}
        ${wheelRow('Spring N', (w) => w.suspension.springForce, 0)}
        ${wheelRow('Damper N', (w) => w.suspension.damperForce, 0)}
        ${wheelRow('ARB N', (w) => w.suspension.antiRollForce, 0)}
        ${wheelRow('Tyre PSI', (w) => w.tire.pressure, 1)}
        ${wheelRow('Tyre &deg;C', (w) => w.tire.temperature, 0)}
        ${wheelRow('Squash mm', (w) => w.tire.deflection * 1000, 1)}
        ${wheelRow('R eff m', (w) => w.tire.effectiveRadius, 3)}
        ${wheelRow('Brake N&middot;m', (w) => w.brakeTorque, 0)}
        ${wheelRow('Drive N&middot;m', (w) => w.driveTorque, 0)}
        ${wheelRow('e-motor N&middot;m', (w) => w.electricTorque, 0)}
        ${wheelRow('Radius m', (w) => w.radius, 3)}
        <tr><th>Contact</th>${WHEEL_IDS.map((id) => {
      const w = v.wheels.find((x) => x.id === id);
      return `<td class="${w.hasContact ? 'ok' : 'bad'}">${w.hasContact ? 'yes' : 'AIR'}</td>`;
    }).join('')}</tr>
        <tr><th>Surface</th>${WHEEL_IDS.map((id) => {
      const w = v.wheels.find((x) => x.id === id);
      return `<td class="small">${w.surface.name}</td>`;
    }).join('')}</tr>
      </table>

      <div class="dbg-title">STEERING &amp; NETWORK</div>
      <table class="dbg-kv">
        <tr><th>Interior wheel</th><td>${stats.steeringWheelDeg !== undefined
          ? `${num(stats.steeringWheelDeg, 1)}&deg;` : 'not animated'}</td>
            <th>Steering ratio</th><td>${num(v.steering.steeringRatio, 1)}:1</td></tr>
        <tr><th>Session</th><td>${stats.network || 'offline'}</td>
            <th>Remote cars</th><td>${stats.remotePlayers ?? 0}</td></tr>
      </table>

      <div class="dbg-title">FRAME</div>
      <table class="dbg-kv">
        <tr><th>Render</th><td>${num(this.fps, 1)} fps</td>
            <th>Physics</th><td>${SIM.physicsHz} Hz (${num(1000 / SIM.physicsHz, 2)} ms)</td></tr>
        <tr><th>Substeps/frame</th><td>${stats.substeps ?? '--'}</td>
            <th>Physics cost</th><td>${num(stats.physicsMs, 2)} ms</td></tr>
        <tr><th>Draw calls</th><td>${stats.drawCalls ?? '--'}</td>
            <th>Triangles</th><td>${(stats.triangles ?? 0).toLocaleString()}</td></tr>
        <tr><th>Reflections</th><td>${stats.reflections || 'off'}</td>
            <th>Probe cost</th><td>${num(stats.reflectionMs, 2)} ms/capture</td></tr>
      </table>
      <div class="dbg-legend">
        <span style="color:#4ad3ff">normal</span>
        <span style="color:#ff6b4a">Fx</span>
        <span style="color:#7dff5a">Fy</span>
        <span style="color:#ffd24a">strut</span>
        <span style="color:#c98bff">steer axis</span>
      </div>`;
  }

  /* -------------------------------------------------------------- gizmos */

  _updateGizmos(v) {
    const pos = this.lineGeometry.attributes.position.array;
    const col = this.lineGeometry.attributes.color.array;
    let i = 0;

    const line = (ax, ay, az, bx, by, bz, r, g, b) => {
      pos[i * 3] = ax; pos[i * 3 + 1] = ay; pos[i * 3 + 2] = az;
      col[i * 3] = r; col[i * 3 + 1] = g; col[i * 3 + 2] = b;
      i++;
      pos[i * 3] = bx; pos[i * 3 + 1] = by; pos[i * 3 + 2] = bz;
      col[i * 3] = r; col[i * 3 + 1] = g; col[i * 3 + 2] = b;
      i++;
    };

    for (let k = 0; k < 4; k++) {
      const w = v.wheels[k];
      const c = w.contactPoint;
      const n = w.contactNormal;

      // Ground normal.
      line(c.x, c.y, c.z,
        c.x + n.x * 0.65, c.y + n.y * 0.65, c.z + n.z * 0.65,
        0.29, 0.83, 1.0);

      // Longitudinal tyre force.
      const fx = w.longForce * FORCE_SCALE;
      line(c.x, c.y + 0.02, c.z,
        c.x + w.forwardDir.x * fx, c.y + 0.02 + w.forwardDir.y * fx, c.z + w.forwardDir.z * fx,
        1.0, 0.42, 0.29);

      // Lateral tyre force.
      const fy = w.latForce * FORCE_SCALE;
      line(c.x, c.y + 0.02, c.z,
        c.x + w.lateralDir.x * fy, c.y + 0.02 + w.lateralDir.y * fy, c.z + w.lateralDir.z * fy,
        0.49, 1.0, 0.35);

      // Suspension strut, from its top mount down to the hub.
      const t = w.strutTopWorld;
      const h = w.hubWorld;
      line(t.x, t.y, t.z, h.x, h.y, h.z, 1.0, 0.82, 0.29);

      // Steering axis (front wheels only carry one).
      if (w.isFront) {
        const u = v.body.up;
        line(h.x - u.x * 0.38, h.y - u.y * 0.38, h.z - u.z * 0.38,
          h.x + u.x * 0.38, h.y + u.y * 0.38, h.z + u.z * 0.38,
          0.79, 0.55, 1.0);
      } else {
        line(0, 0, 0, 0, 0, 0, 0, 0, 0);
      }

      // Contact-patch marker.
      this._m.compose(
        this._v.set(c.x, c.y, c.z), this._quat,
        this._scale.setScalar(w.hasContact ? 1 : 0.001),
      );
      this.contactDots.setMatrixAt(k, this._m);
    }
    this.contactDots.instanceMatrix.needsUpdate = true;

    /* --- centre of mass, velocity and heading ---------------------------- */
    const p = v.body.position;
    this.comDot.position.set(p.x, p.y, p.z);

    const AX = 0.45;
    line(p.x - AX, p.y, p.z, p.x + AX, p.y, p.z, 1, 0.8, 0.2);
    line(p.x, p.y - AX, p.z, p.x, p.y + AX, p.z, 1, 0.8, 0.2);
    line(p.x, p.y, p.z - AX, p.x, p.y, p.z + AX, 1, 0.8, 0.2);

    const vel = v.body.velocity;
    const vs = 0.12;
    line(p.x, p.y, p.z, p.x + vel.x * vs, p.y + vel.y * vs, p.z + vel.z * vs,
      1.0, 1.0, 1.0);

    const f = v.body.forward;
    line(p.x, p.y, p.z, p.x + f.x * 3, p.y + f.y * 3, p.z + f.z * 3,
      0.4, 0.6, 1.0);

    // Blank any unused slots.
    for (; i < TOTAL_VERTS; i++) {
      pos[i * 3] = 0; pos[i * 3 + 1] = 0; pos[i * 3 + 2] = 0;
      col[i * 3] = 0; col[i * 3 + 1] = 0; col[i * 3 + 2] = 0;
    }

    this.lineGeometry.attributes.position.needsUpdate = true;
    this.lineGeometry.attributes.color.needsUpdate = true;
  }
}
