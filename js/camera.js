/**
 * camera.js
 * ---------------------------------------------------------------------------
 * Third-person camera rig. Requirements #34, #35, #36.
 *
 * Five modes - rear chase, side/rear tracking, bonnet, an in-car cockpit view
 * from the driver's seat, and a slow cinematic orbit - all built on the same
 * rig:
 *
 *   1. Pick a FOLLOW YAW: a blend of where the car is pointing and where it is
 *      actually travelling. The blend is what makes a drift readable; a camera
 *      welded to the body yaw swings wildly, one welded to the velocity ignores
 *      the slide entirely.
 *   2. Place the desired eye and target in that yaw frame.
 *   3. Displace both by the car's REAL acceleration, so hard braking pushes the
 *      view forward and cornering leans it. This comes from the measured
 *      G-force, never from the key that is being held.
 *   4. Critically damp position and target toward those goals, then keep the
 *      eye above the ground.
 *
 * GSAP drives only the transition between modes - a purely presentational
 * cross-fade. It is never allowed near the vehicle state.
 * ---------------------------------------------------------------------------
 */

import * as THREE from 'three';
import { gsap } from 'gsap';
import { clamp, clamp01, damp, smoothstep } from './mathUtils.js';
import { CAMERA } from './config.js';

export class CameraSystem {
  constructor(aspect = 1, terrain = null) {
    this.terrain = terrain;

    this.camera = new THREE.PerspectiveCamera(
      CAMERA.baseFov, aspect, CAMERA.near, CAMERA.far,
    );
    this.camera.position.set(-10, 4, 0);

    this.modeIndex = 0;
    this.mode = CAMERA.order[0];
    /**
     * Per-mode config. Smoothing, G-force gains and shake stay global (they are
     * properties of the RIG), but the framing offsets are per vehicle, because
     * a 1.15 m tall supercar wants a different camera from a saloon.
     */
    this.modes = {};
    for (const name of CAMERA.order) this.modes[name] = { ...CAMERA.modes[name] };
    this.config = this.modes[this.mode];

    /** Smoothed rig state. */
    this.position = new THREE.Vector3(-10, 4, 0);
    this.target = new THREE.Vector3();
    this.followYaw = 0;
    this.roll = 0;
    this.fov = CAMERA.baseFov;
    this.orbitAngle = 0;

    /** Vertical impulse energy left over from the last impact. */
    this.shake = 0;

    /** 0..1 cross-fade when switching modes; tweened by GSAP. */
    this._blend = 1;
    this._fromPos = new THREE.Vector3();
    this._fromTarget = new THREE.Vector3();

    /* ------------------------------------------------------- scratch */
    this._desiredPos = new THREE.Vector3();
    this._desiredTarget = new THREE.Vector3();
    this._fwd = new THREE.Vector3();
    this._right = new THREE.Vector3();
    this._up = new THREE.Vector3(0, 1, 0);
    this._tmp = new THREE.Vector3();
    this._carPos = new THREE.Vector3();
    this._quat = new THREE.Quaternion();
  }

  resize(aspect) {
    this.camera.aspect = aspect;
    this.camera.updateProjectionMatrix();
  }

  /* ------------------------------------------------------------- modes */

  get modeName() { return this.config.name; }

  cycleMode() {
    this.setMode(CAMERA.order[(this.modeIndex + 1) % CAMERA.order.length]);
    return this.config.name;
  }

  /**
   * Apply a vehicle's camera framing. Called whenever the player changes car,
   * so the view is always tuned to the body it is following rather than to
   * whichever car happened to be first.
   *
   * @param {object} profile spec.cameraProfile
   */
  setVehicleProfile(profile) {
    if (!profile) return;
    const put = (mode, offsetKey, lookKey) => {
      const m = this.modes[mode];
      if (!m) return;
      if (profile[offsetKey]) m.offset = { ...profile[offsetKey] };
      if (lookKey && profile[lookKey]) m.lookAt = { ...profile[lookKey] };
    };
    put('chase', 'chase', 'chaseLookAt');
    put('side', 'side', 'sideLookAt');
    put('hood', 'hood', 'hoodLookAt');
    put('interior', 'interior', 'interiorLookAt');
    put('orbit', 'orbit', 'orbitLookAt');
    this.config = this.modes[this.mode];
  }

  setMode(name) {
    const idx = CAMERA.order.indexOf(name);
    if (idx < 0 || name === this.mode) return;

    // Freeze the current framing and cross-fade to the new one.
    this._fromPos.copy(this.position);
    this._fromTarget.copy(this.target);
    this._blend = 0;

    this.modeIndex = idx;
    this.mode = name;
    this.config = this.modes[name];
    this.orbitAngle = 0;

    gsap.killTweensOf(this);
    gsap.to(this, {
      _blend: 1,
      duration: 0.65,
      ease: 'power2.inOut',
      overwrite: true,
    });
  }

  /* ------------------------------------------------------------ update */

  /**
   * @param {number} dt          real frame time (this is presentation, so it
   *                             runs on the render clock, not the physics one)
   * @param {VehiclePhysics} v
   */
  update(dt, v) {
    const cfg = this.config;
    const body = v.body;
    this._carPos.set(body.position.x, body.position.y, body.position.z);

    /* --- 1. follow yaw ------------------------------------------------- */
    const bodyYaw = Math.atan2(body.forward.z, body.forward.x);
    let goalYaw = bodyYaw;

    if (cfg.velocityBias > 0 && v.speed > 3) {
      const velYaw = Math.atan2(body.velocity.z, body.velocity.x);
      // Only trust the velocity heading when the car is going forwards.
      const forwardness = clamp01(v.forwardSpeed / Math.max(v.speed, 1e-3));
      const bias = cfg.velocityBias * forwardness * smoothstep(3, 12, v.speed);
      goalYaw = bodyYaw + shortestAngle(bodyYaw, velYaw) * bias;
    }
    this.followYaw = this.followYaw
      + shortestAngle(this.followYaw, goalYaw) * clamp01(dt / Math.max(cfg.rotationSmooth, 1e-3));

    if (this.mode === 'orbit') {
      this.orbitAngle += dt * cfg.orbitSpeed;
      this.followYaw = bodyYaw + this.orbitAngle;
    }

    /* --- 2. rig basis (flat: no body pitch, only a hint of roll) -------- */
    this._fwd.set(Math.cos(this.followYaw), 0, Math.sin(this.followYaw));
    this._right.set(-Math.sin(this.followYaw), 0, Math.cos(this.followYaw));

    /* --- 3. desired eye and target ------------------------------------- */
    const g = v.gForceSmooth;
    const gGain = cfg.gForceGain;

    // Longitudinal G pushes the eye back under power and forward under braking.
    const dx = cfg.offset.x - g.x * gGain.x;
    const dy = cfg.offset.y + (g.y - 1) * gGain.y;
    const dz = cfg.offset.z - g.z * gGain.z;

    this._desiredPos.copy(this._carPos)
      .addScaledVector(this._fwd, dx)
      .addScaledVector(this._up, dy)
      .addScaledVector(this._right, dz);

    this._desiredTarget.copy(this._carPos)
      .addScaledVector(this._fwd, cfg.lookAt.x)
      .addScaledVector(this._up, cfg.lookAt.y)
      .addScaledVector(this._right, cfg.lookAt.z);

    /**
     * The bonnet and interior cameras are bolted to the shell, so they inherit
     * the full body attitude - that is exactly what makes them feel like being
     * in the car rather than watching it. Everything else rides the flat yaw
     * frame, which is what keeps a chase camera readable through a slide.
     */
    if (cfg.rigid) {
      this._tmp.set(cfg.offset.x, cfg.offset.y, cfg.offset.z);
      this._quat.set(body.quat.x, body.quat.y, body.quat.z, body.quat.w);
      this._tmp.applyQuaternion(this._quat);
      this._desiredPos.copy(this._carPos).add(this._tmp);
      this._tmp.set(cfg.lookAt.x, cfg.lookAt.y, cfg.lookAt.z);
      this._tmp.applyQuaternion(this._quat);
      this._desiredTarget.copy(this._carPos).add(this._tmp);
    }

    /* --- 4. critically damped follow ------------------------------------ */
    const ps = Math.max(cfg.positionSmooth, 1e-4);
    const rs = Math.max(cfg.rotationSmooth, 1e-4);
    this.position.x = damp(this.position.x, this._desiredPos.x, ps, dt);
    this.position.y = damp(this.position.y, this._desiredPos.y, ps, dt);
    this.position.z = damp(this.position.z, this._desiredPos.z, ps, dt);
    this.target.x = damp(this.target.x, this._desiredTarget.x, rs, dt);
    this.target.y = damp(this.target.y, this._desiredTarget.y, rs, dt);
    this.target.z = damp(this.target.z, this._desiredTarget.z, rs, dt);

    /* --- 5. mode cross-fade --------------------------------------------- */
    if (this._blend < 1) {
      this.position.lerpVectors(this._fromPos, this.position, this._blend);
      this.target.lerpVectors(this._fromTarget, this.target, this._blend);
    }

    /* --- 6. vertical shake from real impacts ----------------------------- */
    const vertG = Math.abs(g.y - 1);
    if (vertG > 0.8) {
      this.shake = Math.min(CAMERA.maxShake, this.shake + vertG * CAMERA.shakeGainVertical);
    }
    this.shake = damp(this.shake, 0, CAMERA.shakeDecay, dt);
    if (this.shake > 1e-4) {
      const t = performance.now() * 0.001;
      this.position.y += Math.sin(t * 47.3) * this.shake;
      this.position.x += Math.sin(t * 39.1) * this.shake * 0.4;
    }

    /* --- 7. never let the eye go under the ground ------------------------ */
    if (this.terrain) {
      const floor = this.terrain.quickHeight(this.position.x, this.position.z)
        + CAMERA.groundClearance;
      if (this.position.y < floor) this.position.y = floor;
    }

    /* --- 8. field of view and camera roll -------------------------------- */
    // A mode may set its own base FOV and damp down the speed widening: from
    // the driver's seat that effect reads as the cabin breathing.
    const baseFov = cfg.fov ?? CAMERA.baseFov;
    const fovGain = cfg.fovGain ?? 1;
    const fovGoal = baseFov
      + Math.min(CAMERA.maxFovGain, v.speed * CAMERA.fovPerSpeed) * fovGain;
    this.fov = damp(this.fov, fovGoal, 0.30, dt);
    if (Math.abs(this.fov - this.camera.fov) > 0.01) {
      this.camera.fov = this.fov;
      this.camera.updateProjectionMatrix();
    }

    // A touch of camera roll, from measured lateral G, sells the cornering.
    const rollGoal = cfg.rigid
      ? v.rollAngle * cfg.rollGain * 4
      : -g.z * cfg.rollGain * 0.09;
    this.roll = damp(this.roll, clamp(rollGoal, -0.10, 0.10), 0.18, dt);

    /* --- 9. commit --------------------------------------------------------- */
    this.camera.position.copy(this.position);
    this.camera.up.set(Math.sin(this.roll), Math.cos(this.roll), 0)
      .applyAxisAngle(this._up, this.followYaw);
    this.camera.lookAt(this.target);
  }

  /** Snap straight to the ideal framing (used after a respawn). */
  snapTo(v) {
    this._blend = 1;
    gsap.killTweensOf(this);
    this.followYaw = Math.atan2(v.body.forward.z, v.body.forward.x);
    this.shake = 0;
    // One update with a huge dt collapses every damper onto its target.
    this.update(10, v);
  }
}

/** Signed shortest angular difference from a to b, in (-pi, pi]. */
function shortestAngle(a, b) {
  let d = (b - a) % (Math.PI * 2);
  if (d > Math.PI) d -= Math.PI * 2;
  if (d < -Math.PI) d += Math.PI * 2;
  return d;
}
