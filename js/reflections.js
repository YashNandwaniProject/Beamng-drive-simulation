/**
 * reflections.js
 * ---------------------------------------------------------------------------
 * Real-time reflection probe.
 *
 * WHAT THIS IS, PLAINLY: WebGL has no hardware ray tracing. There is no API to
 * call, so nothing here traces a ray. What it does instead is the technique
 * that ray tracing is usually WANTED for on a car - reflections that contain
 * the actual world rather than a static sky:
 *
 *   - a cube camera rides with the car and renders the real scene around it -
 *     road, kerbs, barriers, trees, gantry, other players - into a cube map;
 *   - three.js pre-filters that cube into a roughness pyramid, so a mirror
 *     finish gets a sharp reflection and rough asphalt gets a broad, soft one
 *     from the same capture;
 *   - the bodywork and the road sample it as their environment map.
 *
 * The visible difference is that the car now reflects the road it is on and
 * the scenery it is passing, and the asphalt picks up the sky and the barriers
 * - which is what "ray traced" looks like in a driving game. It is an
 * approximation: reflections are captured from ONE point, so a reflection of
 * something very close to the car is not perfectly placed, and the car itself
 * is excluded from its own capture.
 *
 * COST, because this is not free: one probe update renders the scene six more
 * times. That is why the capture is small (128 px a face by default), why it
 * runs at a fraction of the frame rate rather than every frame, and why the
 * whole thing has an off switch on `V`. On a machine that cannot afford it,
 * turning it off returns exactly the previous renderer.
 * ---------------------------------------------------------------------------
 */

import * as THREE from 'three';

/** Quality steps cycled by the V key. `size` 0 means off. */
export const REFLECTION_LEVELS = [
  { name: 'Off', size: 0, hz: 0 },
  { name: 'Low', size: 96, hz: 4 },
  { name: 'High', size: 160, hz: 10 },
];

export class ReflectionProbe {
  /**
   * @param {THREE.WebGLRenderer} renderer
   * @param {THREE.Scene} scene
   * @param {object} cfg  RENDER.reflections
   */
  constructor(renderer, scene, cfg = {}) {
    this.renderer = renderer;
    this.scene = scene;
    this.cfg = cfg;

    this.levelIndex = clampIndex(cfg.level ?? 2);
    this.level = REFLECTION_LEVELS[this.levelIndex];

    this.renderTarget = null;
    this.cubeCamera = null;
    this._size = 0;

    /** Seconds since the last capture. */
    this._since = 1e9;
    /** Everything that has been handed this probe's texture. */
    this._consumers = new Set();
    this._position = new THREE.Vector3();
    this._hidden = [];

    /** Wall-clock cost of the last capture, for the telemetry overlay. */
    this.lastCaptureMs = 0;
    this.captures = 0;

    this._build();
  }

  get enabled() { return this.level.size > 0; }

  get texture() {
    return this.renderTarget ? this.renderTarget.texture : null;
  }

  /* --------------------------------------------------------------- build */

  _build() {
    const size = this.level.size;
    if (size === this._size) return;

    if (this.renderTarget) {
      this.renderTarget.dispose();
      this.renderTarget = null;
      this.cubeCamera = null;
    }
    this._size = size;
    if (size === 0) return;

    /**
     * Half float, because the sky shader outputs values well above 1 for the
     * sun disc and an 8-bit target would clip them to white - the reflection
     * would lose the very highlight that sells a polished panel.
     */
    this.renderTarget = new THREE.WebGLCubeRenderTarget(size, {
      generateMipmaps: true,
      minFilter: THREE.LinearMipmapLinearFilter,
      magFilter: THREE.LinearFilter,
      type: THREE.HalfFloatType,
    });

    /**
     * Near plane at 0.4 m: closer than the bodywork the probe sits inside, so
     * the car's own shell (hidden during capture anyway) can never intersect
     * it. Far plane matches the scene's fog distance - anything past that is
     * fog-coloured and not worth the fill rate.
     */
    this.cubeCamera = new THREE.CubeCamera(0.4, this.cfg.far ?? 1600, this.renderTarget);
    this._since = 1e9;                       // capture on the very next frame
  }

  /* -------------------------------------------------------------- levels */

  /** Cycle Off -> Low -> High. Returns the new level's name. */
  cycle() {
    return this.setLevel(this.levelIndex + 1);
  }

  setLevel(index) {
    this.levelIndex = clampIndex(index);
    this.level = REFLECTION_LEVELS[this.levelIndex];
    this._build();
    // Re-point every consumer, or they keep sampling a disposed target.
    for (const c of this._consumers) c(this.texture);
    return this.level.name;
  }

  /**
   * Register something that wants the probe texture. The callback is invoked
   * now and again whenever the texture is rebuilt, so a consumer never has to
   * poll or care about quality changes.
   *
   * @param {(texture: THREE.Texture|null) => void} apply
   */
  register(apply) {
    this._consumers.add(apply);
    apply(this.texture);
    return () => this._consumers.delete(apply);
  }

  /* -------------------------------------------------------------- update */

  /**
   * @param {number} dt        real frame time
   * @param {THREE.Vector3} at where to capture from (the car)
   * @param {THREE.Object3D[]} hide objects to leave out of the capture -
   *                           the local car, which would otherwise reflect the
   *                           inside of its own shell back onto its paint
   */
  update(dt, at, hide = []) {
    if (!this.enabled) return false;

    this._since += dt;
    const interval = 1 / Math.max(1e-3, this.level.hz);
    if (this._since < interval) return false;
    this._since = 0;

    const t0 = performance.now();

    this._hidden.length = 0;
    for (const o of hide) {
      if (o && o.visible) { this._hidden.push(o); o.visible = false; }
    }

    this.cubeCamera.position.copy(at);
    this.cubeCamera.updateMatrixWorld(true);
    this.cubeCamera.update(this.renderer, this.scene);

    for (const o of this._hidden) o.visible = true;
    this._hidden.length = 0;

    /**
     * Tell three.js the cube changed, so its roughness pre-filter is rebuilt.
     * Without this the materials keep sampling the pyramid from the first
     * capture and the reflections freeze at the spawn point.
     */
    this.renderTarget.texture.needsPMREMUpdate = true;

    this.lastCaptureMs = performance.now() - t0;
    this.captures++;
    return true;
  }

  dispose() {
    if (this.renderTarget) this.renderTarget.dispose();
    this.renderTarget = null;
    this.cubeCamera = null;
    this._consumers.clear();
  }
}

function clampIndex(i) {
  const n = REFLECTION_LEVELS.length;
  return ((i % n) + n) % n;
}
