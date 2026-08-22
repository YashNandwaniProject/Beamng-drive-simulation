/**
 * environment.js
 * ---------------------------------------------------------------------------
 * Everything the player sees that is not the car: sky, image-based lighting,
 * sun, road, kerbs, run-off, terrain, barriers and scenery.
 * Requirements #37, #38, #39, #40.
 *
 * THE IMPORTANT PART is that the road mesh is tessellated by calling exactly
 * the same Terrain.heightAt() the tyres raycast against. There is no separate
 * "collision road" that could drift out of alignment with the visual one - the
 * banking you can see IS the banking the suspension feels (#38), and the kerb
 * the wheel climbs is the kerb that is drawn.
 *
 * LIGHTING / ASSET LICENSING (#39). No HDRI is bundled, because a random
 * environment map found online is very unlikely to be licensed for
 * redistribution. Instead the sky - and the image-based lighting derived from
 * it - is generated procedurally at run time and pre-filtered with three.js's
 * own PMREMGenerator, so the project ships with zero third-party image assets
 * and no licensing question at all. If you would rather use a real capture,
 * drop a .hdr into assets/hdri/ and call Environment.loadHdri(); that path is
 * implemented below and documented in the README.
 * ---------------------------------------------------------------------------
 */

import * as THREE from 'three';
import { HDRLoader } from 'three/addons/loaders/HDRLoader.js';
import { Terrain } from './terrain.js';
import { CylinderCollider } from './collision.js';
import { RENDER } from './config.js';
import { clamp01, lerp, smoothstep, makeRng, fbm2 } from './mathUtils.js';

/* ========================================================================= */
/* Seamless procedural textures                                              */
/* ========================================================================= */

/**
 * Seamless fBm over the unit square.
 *
 * Value noise is not periodic, so a texture sampled straight from it shows a
 * hard seam wherever it wraps - which on a road that repeats every 16 m is a
 * line right across the carriageway. Blending four copies of the field,
 * offset by exactly one tile in x and y and weighted by the position within
 * the tile, makes opposite edges identical by construction.
 */
function seamlessFbm(u, v, scale, octaves = 4) {
  const x = u * scale;
  const y = v * scale;
  const a = fbm2(x, y, octaves);
  const b = fbm2(x - scale, y, octaves);
  const c = fbm2(x, y - scale, octaves);
  const d = fbm2(x - scale, y - scale, octaves);
  return lerp(lerp(a, b, u), lerp(c, d, u), v);
}

/** As above but periodic in v only (u is clamped on the road texture). */
function seamlessFbmV(u, v, scaleU, scaleV, octaves = 4) {
  const x = u * scaleU;
  const y = v * scaleV;
  const a = fbm2(x, y, octaves);
  const c = fbm2(x, y - scaleV, octaves);
  return lerp(a, c, v);
}

const insideBand = (u, centre, halfWidth) => Math.abs(u - centre) < halfWidth;

/** Half-extent of the sky box. Large enough that the camera is always inside. */
const SKY_SIZE = 20000;

/**
 * One texture carrying the whole road surface: asphalt aggregate, polished
 * wheel tracks, a tar seam, solid edge lines and a dashed centre line.
 *
 * U spans the road WIDTH exactly once (clamped), so the markings follow the
 * carriageway as it narrows through the hairpin and widens on the banking.
 * V repeats every `tileMetres` along the length.
 */
function makeRoadTexture(tileMetres, w = 512, h = 1024) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const ctx = c.getContext('2d');
  const img = ctx.createImageData(w, h);
  const d = img.data;

  for (let y = 0; y < h; y++) {
    const v = y / h;
    for (let x = 0; x < w; x++) {
      const u = x / w;

      // Two scales of aggregate.
      const coarse = seamlessFbmV(u, v, 26, tileMetres * 1.5, 4) * 0.5 + 0.5;
      const fine = seamlessFbmV(u, v, 150, tileMetres * 9, 2) * 0.5 + 0.5;
      const n = coarse * 0.6 + fine * 0.4;

      let base = 25 + n * 32;

      // Polished wheel tracks, and the tar seam down the middle.
      const track = Math.exp(-((u - 0.30) ** 2) / 0.006)
        + Math.exp(-((u - 0.70) ** 2) / 0.006);
      base -= track * 6;
      base -= Math.exp(-((u - 0.5) ** 2) / 0.0004) * 4;

      let r = base, g = base, b = base * 1.04;

      /* --- markings ------------------------------------------------- */
      const edge = insideBand(u, 0.028, 0.013) || insideBand(u, 0.972, 0.013);
      const along = v * tileMetres;
      // 2 m dash, 2 m gap.
      const dash = (along % 4.0) < 2.0 && insideBand(u, 0.5, 0.010);

      if (edge || dash) {
        const wear = 0.70 + n * 0.30;      // let the aggregate show through
        r = 228 * wear; g = 228 * wear; b = 220 * wear;
      }

      const i = (y * w + x) * 4;
      d[i] = r; d[i + 1] = g; d[i + 2] = b; d[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);

  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.ClampToEdgeWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** Roughness map for the road: paint and worn tracks are smoother than grain. */
function makeRoadRoughness(tileMetres, w = 256, h = 512) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const ctx = c.getContext('2d');
  const img = ctx.createImageData(w, h);
  const d = img.data;
  for (let y = 0; y < h; y++) {
    const v = y / h;
    for (let x = 0; x < w; x++) {
      const u = x / w;
      const n = seamlessFbmV(u, v, 22, tileMetres * 1.5, 3) * 0.5 + 0.5;
      let rough = 0.74 + n * 0.20;
      rough -= (Math.exp(-((u - 0.30) ** 2) / 0.006)
        + Math.exp(-((u - 0.70) ** 2) / 0.006)) * 0.16;
      if (insideBand(u, 0.028, 0.013) || insideBand(u, 0.972, 0.013)) rough -= 0.18;
      const i = (y * w + x) * 4;
      const val = clamp01(rough) * 255;
      d[i] = val; d[i + 1] = val; d[i + 2] = val; d[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.ClampToEdgeWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  return tex;
}

/** Generic seamless tiling noise texture: grass, gravel. */
function makeNoiseTexture(size, colorA, colorB, scale, octaves = 4, speckle = 0) {
  const c = document.createElement('canvas');
  c.width = size; c.height = size;
  const ctx = c.getContext('2d');
  const img = ctx.createImageData(size, size);
  const d = img.data;
  const a = new THREE.Color(colorA);
  const b = new THREE.Color(colorB);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const u = x / size;
      const v = y / size;
      let n = seamlessFbm(u, v, scale, octaves) * 0.5 + 0.5;
      if (speckle > 0) n += (seamlessFbm(u, v, scale * 9, 2) * 0.5) * speckle;
      n = clamp01(n);
      const i = (y * size + x) * 4;
      d[i] = (a.r + (b.r - a.r) * n) * 255;
      d[i + 1] = (a.g + (b.g - a.g) * n) * 255;
      d[i + 2] = (a.b + (b.b - a.b) * n) * 255;
      d[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** Red/white kerb blocks. U spans the kerb width, V repeats every block. */
function makeKerbTexture(size = 64) {
  const c = document.createElement('canvas');
  c.width = size; c.height = size;
  const ctx = c.getContext('2d');
  const img = ctx.createImageData(size, size);
  const d = img.data;
  for (let y = 0; y < size; y++) {
    const red = (y / size) < 0.5;
    for (let x = 0; x < size; x++) {
      const grime = 0.80 + (seamlessFbm(x / size, y / size, 6, 3) * 0.5 + 0.5) * 0.28;
      const i = (y * size + x) * 4;
      d[i] = (red ? 178 : 232) * grime;
      d[i + 1] = (red ? 50 : 230) * grime;
      d[i + 2] = (red ? 46 : 224) * grime;
      d[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.ClampToEdgeWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/* ========================================================================= */
/* Sky                                                                       */
/* ========================================================================= */

const SKY_VERT = /* glsl */`
varying vec3 vWorldDir;
void main() {
  vWorldDir = normalize((modelMatrix * vec4(position, 1.0)).xyz - cameraPosition);
  vec4 mvp = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  gl_Position = mvp.xyww;             // pin to the far plane
}`;

/**
 * A compact analytic sky: a zenith-to-horizon gradient, warm haze that
 * thickens toward the sun, a sun disc with a broad glow, and a dimmer ground
 * hemisphere. Pre-filtered into a PMREM cube it becomes the scene's IBL, so
 * the BMW's paint and glass reflect the same sky that is drawn overhead.
 */
const SKY_FRAG = /* glsl */`
varying vec3 vWorldDir;
uniform vec3 uSunDirection;
uniform vec3 uZenith;
uniform vec3 uHorizon;
uniform vec3 uGround;
uniform vec3 uSunColor;
uniform float uSunIntensity;
uniform float uTurbidity;

void main() {
  vec3 dir = normalize(vWorldDir);
  float h = dir.y;

  float t = pow(clamp(h, 0.0, 1.0), 0.42);
  vec3 sky = mix(uHorizon, uZenith, t);

  vec3 sunFlat = normalize(vec3(uSunDirection.x, 0.0, uSunDirection.z));
  vec3 dirFlat = normalize(vec3(dir.x, 0.0, dir.z) + vec3(1e-5));
  float sunAzimuth = clamp(dot(dirFlat, sunFlat), 0.0, 1.0);
  float haze = exp(-abs(h) * 7.0) * (0.35 + 0.65 * pow(sunAzimuth, 3.0));
  sky += uSunColor * haze * 0.30 * uTurbidity;

  float cosAngle = dot(dir, uSunDirection);
  float disc = smoothstep(0.99965, 0.99992, cosAngle);
  float glow = pow(clamp(cosAngle, 0.0, 1.0), 220.0) * 0.55
             + pow(clamp(cosAngle, 0.0, 1.0), 12.0) * 0.10;
  sky += uSunColor * (disc * uSunIntensity + glow * 2.2);

  vec3 ground = uGround * (0.55 + 0.45 * exp(h * 5.0));
  vec3 col = mix(ground, sky, smoothstep(-0.045, 0.035, h));

  gl_FragColor = vec4(col, 1.0);
}`;

/* ========================================================================= */
/* Environment                                                               */
/* ========================================================================= */

export class Environment {
  /**
   * @param {THREE.WebGLRenderer} renderer
   * @param {Terrain} terrain
   * @param {CollisionSystem} collision  may be null
   */
  constructor(renderer, terrain, collision = null) {
    this.renderer = renderer;
    this.terrain = terrain;
    this.collision = collision;

    this.scene = new THREE.Scene();
    this.sunDirection = new THREE.Vector3(-0.42, 0.58, 0.70).normalize();
    this.scene.fog = new THREE.Fog(0xa8bcce, RENDER.fogNear, RENDER.fogFar);

    /**
     * Road texture tile length, chosen so a whole number of tiles fits the lap.
     * Without this the texture would not meet itself where the circuit closes.
     */
    this.roadTileMetres = terrain.length / Math.max(1, Math.round(terrain.length / 16));

    this._sunFocus = new THREE.Vector3();

    this._buildSky();
    this._buildLights();
    this._buildRoad();
    this._buildTerrain();
    this._buildProps();
  }

  /* ------------------------------------------------------------------ sky */

  _buildSky() {
    this.skyUniforms = {
      uSunDirection: { value: this.sunDirection.clone() },
      uZenith: { value: new THREE.Color(0x2f66b8) },
      uHorizon: { value: new THREE.Color(0xc2d4e4) },
      uGround: { value: new THREE.Color(0x4c4842) },
      uSunColor: { value: new THREE.Color(0xfff3dc) },
      uSunIntensity: { value: 320.0 },
      uTurbidity: { value: 1.15 },
    };

    const material = new THREE.ShaderMaterial({
      vertexShader: SKY_VERT,
      fragmentShader: SKY_FRAG,
      uniforms: this.skyUniforms,
      side: THREE.BackSide,
      depthWrite: false,
      depthTest: false,
      fog: false,
      toneMapped: false,
    });

    /**
     * The box has to be big enough that the camera is always inside it, or the
     * unit cube projects to a handful of pixels and there is simply no sky on
     * screen. Depth is pinned to the far plane in the vertex shader, so the
     * size has no bearing on the depth buffer and it can safely sit outside the
     * camera's far distance.
     */
    this.sky = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), material);
    this.sky.scale.setScalar(SKY_SIZE);
    this.sky.frustumCulled = false;
    this.sky.renderOrder = -1000;
    this.scene.add(this.sky);

    this._generateEnvironmentMap();
  }

  /** Pre-filter the procedural sky into an IBL cube (requirements #39, #40). */
  _generateEnvironmentMap() {
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    pmrem.compileEquirectangularShader();

    // Render the sky on its own so no geometry leaks into the reflection probe.
    const skyScene = new THREE.Scene();
    const probe = new THREE.Mesh(this.sky.geometry, this.sky.material);
    probe.scale.setScalar(SKY_SIZE);
    probe.frustumCulled = false;
    skyScene.add(probe);

    if (this.envRenderTarget) this.envRenderTarget.dispose();
    this.envRenderTarget = pmrem.fromScene(skyScene, 0, 0.1, 1000);
    this.envMap = this.envRenderTarget.texture;
    this.scene.environment = this.envMap;
    this.scene.environmentIntensity = 1.0;

    pmrem.dispose();
  }

  /**
   * Optional: swap the procedural lighting for a real HDRI.
   * Nothing is bundled - see the licensing note at the top of this file.
   * @param {string} url e.g. 'assets/hdri/your_sky_2k.hdr'
   */
  loadHdri(url) {
    return new Promise((resolve, reject) => {
      new HDRLoader().load(url, (texture) => {
        const pmrem = new THREE.PMREMGenerator(this.renderer);
        const rt = pmrem.fromEquirectangular(texture);
        if (this.envRenderTarget) this.envRenderTarget.dispose();
        this.envRenderTarget = rt;
        this.envMap = rt.texture;
        this.scene.environment = this.envMap;
        this.scene.background = this.envMap;
        if (this.sky) this.sky.visible = false;
        texture.dispose();
        pmrem.dispose();
        resolve(this.envMap);
      }, undefined, reject);
    });
  }

  /**
   * Point the circuit's surfaces at a real-time reflection probe. Called with
   * `null` when reflections are switched off, which restores the scene's own
   * sky IBL - so the road always has SOMETHING to reflect.
   *
   * @param {THREE.Texture|null} texture
   */
  applyReflectionProbe(texture) {
    for (const entry of this._reflective || []) {
      const m = entry.material;
      m.envMap = texture;
      // The probe contains the ground as well as the sky, so it is darker
      // overall than the sky-only IBL; the road would go flat at the same
      // intensity it used before.
      m.envMapIntensity = texture ? entry.intensity : 0.5;
      m.needsUpdate = true;
    }
  }

  /* --------------------------------------------------------------- lights */

  _buildLights() {
    // Key light: colour and direction match the sun drawn in the sky shader.
    this.sun = new THREE.DirectionalLight(0xfff1d8, 3.0);
    this.sun.position.copy(this.sunDirection).multiplyScalar(170);
    this.sun.castShadow = true;

    const r = RENDER.shadowRadius;
    const cam = this.sun.shadow.camera;
    cam.left = -r; cam.right = r; cam.top = r; cam.bottom = -r;
    cam.near = 1; cam.far = 430;
    this.sun.shadow.mapSize.set(RENDER.shadowMapSize, RENDER.shadowMapSize);
    this.sun.shadow.bias = -0.0006;
    this.sun.shadow.normalBias = 0.05;

    this.scene.add(this.sun);
    this.scene.add(this.sun.target);

    // Sky/ground bounce on top of the IBL: keeps shadowed bodywork from going
    // flat black, for the cost of one more light.
    this.hemi = new THREE.HemisphereLight(0xbcd6f2, 0x4a4436, 0.5);
    this.scene.add(this.hemi);
  }

  /**
   * Keep the shadow frustum wrapped tightly around the car. One cascade
   * covering the whole 1.4 km circuit would waste the entire shadow map on
   * scenery the player is nowhere near.
   */
  updateSun(x, y, z) {
    this._sunFocus.set(x, y, z);
    this.sun.target.position.copy(this._sunFocus);
    this.sun.position.copy(this._sunFocus).addScaledVector(this.sunDirection, 175);
    this.sun.target.updateMatrixWorld();
  }

  /* ----------------------------------------------------------------- road */

  _buildRoad() {
    const t = this.terrain;
    const RUMBLE = Terrain.RUMBLE_WIDTH;
    const RUNOFF = Terrain.RUNOFF_WIDTH;

    /** Longitudinal tessellation. 1.25 m resolves the kerbs and the crest. */
    const stations = Math.round(t.length / 1.25);
    const ds = t.length / stations;
    const tileV = this.roadTileMetres;

    const newMesh = () => ({ pos: [], uv: [], idx: [], count: 0 });
    const asphalt = newMesh();
    const kerbs = newMesh();
    const runoff = newMesh();

    const p = new THREE.Vector3();
    const push = (mesh, x, y, z, u, v) => {
      mesh.pos.push(x, y, z);
      mesh.uv.push(u, v);
      return mesh.count++;
    };

    const asphaltRows = [];
    const kerbRows = [];
    const runoffRows = [];

    for (let i = 0; i <= stations; i++) {
      const s = (i % stations) * ds;
      const along = i * ds;
      const hw = t.halfWidthAt(s);

      // --- carriageway ---------------------------------------------------
      const rowA = [];
      for (const frac of [-1, -0.5, 0, 0.5, 1]) {
        const off = hw * frac;
        t.offsetPoint(s, off, p);
        rowA.push(push(asphalt, p.x, t.heightAt(p.x, p.z, null), p.z,
          frac * 0.5 + 0.5, along / tileV));
      }
      asphaltRows.push(rowA);

      // --- kerbs, one strip per side --------------------------------------
      const rowK = [];
      for (const pair of [[-hw - RUMBLE, -hw], [hw, hw + RUMBLE]]) {
        for (let k = 0; k < 2; k++) {
          const off = pair[k];
          t.offsetPoint(s, off, p);
          rowK.push(push(kerbs, p.x, t.heightAt(p.x, p.z, null), p.z,
            k, along / 1.4));
        }
      }
      kerbRows.push(rowK);

      // --- gravel run-off ---------------------------------------------------
      const rowR = [];
      for (const pair of [
        [-hw - RUMBLE - RUNOFF, -hw - RUMBLE],
        [hw + RUMBLE, hw + RUMBLE + RUNOFF],
      ]) {
        for (let k = 0; k < 2; k++) {
          const off = pair[k];
          t.offsetPoint(s, off, p);
          rowR.push(push(runoff, p.x, t.heightAt(p.x, p.z, null), p.z,
            k * 1.4, along / 5));
        }
      }
      runoffRows.push(rowR);
    }

    /** Stitch quads. `strips` independent ribbons of `vertsPerStrip` each. */
    const stitch = (mesh, rows, vertsPerStrip, strips) => {
      for (let i = 0; i < rows.length - 1; i++) {
        const a = rows[i];
        const b = rows[i + 1];
        for (let sIdx = 0; sIdx < strips; sIdx++) {
          const base = sIdx * vertsPerStrip;
          for (let k = 0; k < vertsPerStrip - 1; k++) {
            const i0 = a[base + k], i1 = a[base + k + 1];
            const i2 = b[base + k], i3 = b[base + k + 1];
            /**
             * Counter-clockwise winding, so computeVertexNormals() gives an
             * UPWARD normal. Get this backwards and the road still draws, but
             * every face is lit from underneath and the whole circuit renders
             * black - which is exactly what happened the first time.
             */
            mesh.idx.push(i0, i1, i2, i1, i3, i2);
          }
        }
      }
    };

    stitch(asphalt, asphaltRows, 5, 1);
    stitch(kerbs, kerbRows, 2, 2);
    stitch(runoff, runoffRows, 2, 2);

    /* ---------------------------------------------------------- materials */
    this.roadTexture = makeRoadTexture(tileV);
    this.roadRoughness = makeRoadRoughness(tileV);

    /**
     * Asphalt as a PHYSICAL material, so it can carry a clear coat.
     *
     * Real asphalt is not a mirror, but it is not matte either: the bitumen
     * between the aggregate is smooth, and that is why a road picks up the
     * sky, the barriers and the low sun and goes silver-grey at a glancing
     * angle. A single roughness value cannot express both at once - the stone
     * is rough and the binder is not - so the aggregate stays rough in the
     * base layer and the binder becomes a weak clear coat over the top.
     *
     * With the reflection probe running, that coat is what the world actually
     * reflects in.
     */
    const roadMat = new THREE.MeshPhysicalMaterial({
      map: this.roadTexture,
      roughnessMap: this.roadRoughness,
      roughness: 0.94,
      metalness: 0.0,
      envMapIntensity: 0.5,
      clearcoat: 0.35,
      clearcoatRoughness: 0.42,
      polygonOffset: true,
      polygonOffsetFactor: -3,
      polygonOffsetUnits: -3,
    });

    this.kerbTexture = makeKerbTexture();
    const kerbMat = new THREE.MeshStandardMaterial({
      map: this.kerbTexture,
      roughness: 0.78,
      metalness: 0.0,
      envMapIntensity: 0.5,
      polygonOffset: true,
      polygonOffsetFactor: -3,
      polygonOffsetUnits: -3,
    });

    this.gravelTexture = makeNoiseTexture(256, 0x5f584f, 0x93897c, 9, 4, 0.5);
    this.gravelTexture.repeat.set(2, 1);
    const runoffMat = new THREE.MeshStandardMaterial({
      map: this.gravelTexture,
      roughness: 0.99,
      metalness: 0.0,
      envMapIntensity: 0.42,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
    });

    this.roadGroup = new THREE.Group();
    this.roadGroup.name = 'Circuit';

    const finish = (mesh, material, name) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(mesh.pos, 3));
      g.setAttribute('uv', new THREE.Float32BufferAttribute(mesh.uv, 2));
      g.setIndex(mesh.idx);
      g.computeVertexNormals();
      g.computeBoundingSphere();
      const m = new THREE.Mesh(g, material);
      m.name = name;
      m.receiveShadow = true;
      this.roadGroup.add(m);
      return m;
    };

    this.roadMesh = finish(asphalt, roadMat, 'Asphalt');
    this.kerbMesh = finish(kerbs, kerbMat, 'Kerbs');
    this.runoffMesh = finish(runoff, runoffMat, 'Runoff');

    /**
     * Surfaces that take the real-time probe, and how strongly. Gravel is
     * deliberately absent: loose stone has no specular lobe worth reflecting
     * in, and giving it one makes the run-off look like water.
     */
    this._reflective = [
      { material: roadMat, intensity: 0.85 },
      { material: kerbMat, intensity: 0.55 },
    ];

    this.scene.add(this.roadGroup);

    this.stats = {
      roadTriangles: (asphalt.idx.length + kerbs.idx.length + runoff.idx.length) / 3,
    };
  }

  /* -------------------------------------------------------------- terrain */

  _buildTerrain() {
    const t = this.terrain;
    const b = t.bounds(260);
    const STEP = 7;
    const nx = Math.ceil((b.maxX - b.minX) / STEP);
    const nz = Math.ceil((b.maxZ - b.minZ) / STEP);

    const geo = new THREE.PlaneGeometry(b.maxX - b.minX, b.maxZ - b.minZ, nx, nz);
    geo.rotateX(-Math.PI / 2);
    geo.translate((b.minX + b.maxX) / 2, 0, (b.minZ + b.maxZ) / 2);

    const pos = geo.attributes.position;
    const colors = new Float32Array(pos.count * 3);
    const grass = new THREE.Color(0x4e6336);
    const dry = new THREE.Color(0x80794a);
    const rock = new THREE.Color(0x6b665d);
    const c = new THREE.Color();

    const corridor = Terrain.RUMBLE_WIDTH + Terrain.RUNOFF_WIDTH;

    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i);
      const z = pos.getZ(i);
      let y = t.quickHeight(x, z);

      /**
       * Sink the terrain where the road ribbons cover it. Both surfaces come
       * from the same height function, so left alone they would be coplanar
       * and z-fight; the drop fades out just past the run-off, so the join
       * stays invisible.
       */
      const info = t.lateralAt(x, z);
      if (info.onTrack) {
        const d = Math.abs(info.lateral) - (info.halfWidth + corridor);
        y -= 0.32 * (1 - smoothstep(0, 7, Math.max(0, d)));
      }
      pos.setY(i, y);

      // Vertex colours: drier in patches, rockier as the ground steepens.
      const slope = Math.abs(t.quickHeight(x + 3, z) - t.quickHeight(x - 3, z)) / 6;
      const patch = fbm2(x * 0.006, z * 0.006, 3) * 0.5 + 0.5;
      c.copy(grass).lerp(dry, clamp01(patch * 1.4 - 0.2));
      c.lerp(rock, clamp01(slope * 1.8));
      const brightness = 0.85 + (fbm2(x * 0.09, z * 0.09, 2) * 0.5 + 0.5) * 0.3;
      colors[i * 3] = c.r * brightness;
      colors[i * 3 + 1] = c.g * brightness;
      colors[i * 3 + 2] = c.b * brightness;
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.computeVertexNormals();

    /**
     * A DETAIL map, not a colour map. The vertex colours above already carry
     * the ground's albedo, and three.js multiplies map x vertexColor - so a
     * dark green texture on top of a dark green vertex colour would square the
     * darkness and crush the terrain to black. This one averages about 1.0 and
     * only adds variation.
     */
    this.grassTexture = makeNoiseTexture(256, 0x9a9a9a, 0xf0f0f0, 12, 4, 0.35);
    this.grassTexture.repeat.set(nx * 0.5, nz * 0.5);

    this.terrainMesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
      map: this.grassTexture,
      vertexColors: true,
      roughness: 0.99,
      metalness: 0.0,
      envMapIntensity: 0.4,
    }));
    this.terrainMesh.name = 'Terrain';
    this.terrainMesh.receiveShadow = true;
    this.scene.add(this.terrainMesh);
  }

  /* ---------------------------------------------------------------- props */

  _buildProps() {
    this.props = new THREE.Group();
    this.props.name = 'Scenery';
    this.scene.add(this.props);

    this._buildBarriers();
    this._buildTrees();
    this._buildLightPoles();
    this._buildStartGantry();
  }

  /**
   * Armco barriers outside the run-off, each with a matching physics collider
   * so they actually stop the car rather than being scenery it drives through.
   */
  _buildBarriers() {
    const t = this.terrain;
    const SEG = 8;
    const count = Math.floor(t.length / SEG);

    const railGeo = new THREE.BoxGeometry(SEG * 0.98, 0.42, 0.10);
    const postGeo = new THREE.BoxGeometry(0.09, 0.78, 0.09);
    const rails = new THREE.InstancedMesh(railGeo, new THREE.MeshStandardMaterial({
      color: 0xb9bec4, roughness: 0.40, metalness: 0.88, envMapIntensity: 1.0,
    }), count * 2);
    const posts = new THREE.InstancedMesh(postGeo, new THREE.MeshStandardMaterial({
      color: 0x7d838a, roughness: 0.65, metalness: 0.6,
    }), count * 4);
    rails.castShadow = rails.receiveShadow = true;
    posts.castShadow = posts.receiveShadow = true;

    const m = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const one = new THREE.Vector3(1, 1, 1);
    const up = new THREE.Vector3(0, 1, 0);
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    const mid = new THREE.Vector3();
    const tmp = new THREE.Vector3();
    let ri = 0, pi = 0;

    for (let i = 0; i < count; i++) {
      const s0 = i * SEG;
      const s1 = (s0 + SEG) % t.length;
      const off = t.halfWidthAt(s0) + Terrain.RUMBLE_WIDTH + Terrain.RUNOFF_WIDTH + 1.6;

      for (const side of [-1, 1]) {
        t.offsetPoint(s0, off * side, a);
        t.offsetPoint(s1, off * side, b);
        a.y = t.quickHeight(a.x, a.z);
        b.y = t.quickHeight(b.x, b.z);
        mid.addVectors(a, b).multiplyScalar(0.5);

        // The mesh's local +X runs along the rail, so the yaw about +Y that
        // maps +X onto the segment direction is the negative compass angle.
        q.setFromAxisAngle(up, -Math.atan2(b.z - a.z, b.x - a.x));

        tmp.set(mid.x, mid.y + 0.62, mid.z);
        m.compose(tmp, q, one);
        rails.setMatrixAt(ri++, m);

        for (const f of [0.18, 0.82]) {
          const px = a.x + (b.x - a.x) * f;
          const pz = a.z + (b.z - a.z) * f;
          tmp.set(px, t.quickHeight(px, pz) + 0.39, pz);
          m.compose(tmp, q, one);
          posts.setMatrixAt(pi++, m);
        }

        if (this.collision) {
          this.collision.addWall(a.x, a.z, b.x, b.z, 0.9, 0.24, mid.y + 0.9);
        }
      }
    }
    rails.count = ri;
    posts.count = pi;
    rails.instanceMatrix.needsUpdate = true;
    posts.instanceMatrix.needsUpdate = true;
    this.props.add(rails, posts);
  }

  /** Instanced low-poly trees scattered on the grass, well clear of the circuit. */
  _buildTrees() {
    const t = this.terrain;
    const rng = makeRng(20260820);
    const bounds = t.bounds(180);

    const trunkGeo = new THREE.CylinderGeometry(0.16, 0.26, 2.6, 5);
    trunkGeo.translate(0, 1.3, 0);
    const canopyGeo = new THREE.IcosahedronGeometry(2.1, 0);
    canopyGeo.translate(0, 4.0, 0);

    const MAX = 850;
    const trunks = new THREE.InstancedMesh(trunkGeo, new THREE.MeshStandardMaterial({
      color: 0x53412c, roughness: 0.95,
    }), MAX);
    const canopies = new THREE.InstancedMesh(canopyGeo, new THREE.MeshStandardMaterial({
      color: 0x39572a, roughness: 0.92, flatShading: true,
    }), MAX);
    trunks.castShadow = true;
    canopies.castShadow = true;

    const m = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const up = new THREE.Vector3(0, 1, 0);
    const p = new THREE.Vector3();
    const s = new THREE.Vector3();
    let n = 0;

    for (let guard = 0; guard < MAX * 14 && n < MAX; guard++) {
      const x = bounds.minX + rng() * (bounds.maxX - bounds.minX);
      const z = bounds.minZ + rng() * (bounds.maxZ - bounds.minZ);

      const info = t.lateralAt(x, z);
      if (info.onTrack && Math.abs(info.lateral) < info.halfWidth + 17) continue;

      // Nothing on steep ground - it would visibly float.
      const slope = Math.abs(t.quickHeight(x + 4, z) - t.quickHeight(x - 4, z)) / 8;
      if (slope > 0.35) continue;

      const scale = 0.7 + rng() * 0.85;
      p.set(x, t.quickHeight(x, z) - 0.15, z);
      q.setFromAxisAngle(up, rng() * Math.PI * 2);
      s.set(scale, scale * (0.85 + rng() * 0.45), scale);
      m.compose(p, q, s);
      trunks.setMatrixAt(n, m);
      canopies.setMatrixAt(n, m);
      n++;
    }
    trunks.count = n;
    canopies.count = n;
    trunks.instanceMatrix.needsUpdate = true;
    canopies.instanceMatrix.needsUpdate = true;
    this.props.add(trunks, canopies);
  }

  /** Light poles along the circuit, with colliders. */
  _buildLightPoles() {
    const t = this.terrain;
    const poleGeo = new THREE.CylinderGeometry(0.10, 0.15, 8.5, 6);
    poleGeo.translate(0, 4.25, 0);
    const armGeo = new THREE.BoxGeometry(1.7, 0.14, 0.16);
    armGeo.translate(0.85, 8.35, 0);
    const headGeo = new THREE.BoxGeometry(0.7, 0.16, 0.34);
    headGeo.translate(1.55, 8.28, 0);
    const mat = new THREE.MeshStandardMaterial({
      color: 0x8c9298, roughness: 0.5, metalness: 0.75,
    });
    const lensMat = new THREE.MeshStandardMaterial({
      color: 0xdfe6ec, roughness: 0.25, metalness: 0.1,
      emissive: 0x1a1c1e, envMapIntensity: 1.2,
    });

    const group = new THREE.Group();
    const p = new THREE.Vector3();
    for (let s = 24; s < t.length; s += 58) {
      const off = t.halfWidthAt(s) + Terrain.RUMBLE_WIDTH + Terrain.RUNOFF_WIDTH + 3.4;
      t.offsetPoint(s, off, p);
      const y = t.quickHeight(p.x, p.z);
      // Point the arm back over the track: local +X faces away from the road.
      const yaw = -t.headingAt(s) + Math.PI * 0.5;

      for (const [geo, material] of [[poleGeo, mat], [armGeo, mat], [headGeo, lensMat]]) {
        const mesh = new THREE.Mesh(geo, material);
        mesh.position.set(p.x, y, p.z);
        mesh.rotation.y = yaw;
        mesh.castShadow = true;
        group.add(mesh);
      }

      if (this.collision) {
        this.collision.add(new CylinderCollider(
          { x: p.x, y: y + 4.0, z: p.z }, 0.22, 8.0,
          { restitution: 0.08, friction: 0.6 },
        ));
      }
    }
    this.props.add(group);
  }

  /** Start / finish gantry and the painted line across the grid. */
  _buildStartGantry() {
    const t = this.terrain;
    // A little ahead of the spawn point, so the car lines up behind the line.
    const s = 78;
    const hw = t.halfWidthAt(s);
    const span = (hw + 2.4) * 2;

    const legGeo = new THREE.BoxGeometry(0.34, 6.6, 0.34);
    legGeo.translate(0, 3.3, 0);
    const beamGeo = new THREE.BoxGeometry(0.42, 0.55, span);
    beamGeo.translate(0, 6.85, 0);
    const bannerGeo = new THREE.BoxGeometry(0.12, 1.05, span * 0.86);
    bannerGeo.translate(0, 5.95, 0);

    const mat = new THREE.MeshStandardMaterial({
      color: 0xd9dde0, roughness: 0.45, metalness: 0.5,
    });
    const bannerMat = new THREE.MeshStandardMaterial({
      color: 0x151a22, roughness: 0.7, metalness: 0.1,
    });

    const g = new THREE.Group();
    const p = new THREE.Vector3();
    const yaw = -t.headingAt(s);

    t.offsetPoint(s, 0, p);
    const y = t.quickHeight(p.x, p.z);
    for (const [geo, material] of [[beamGeo, mat], [bannerGeo, bannerMat]]) {
      const mesh = new THREE.Mesh(geo, material);
      mesh.position.set(p.x, y, p.z);
      mesh.rotation.y = yaw;
      mesh.castShadow = true;
      g.add(mesh);
    }

    for (const side of [-1, 1]) {
      t.offsetPoint(s, (hw + 2.2) * side, p);
      const ly = t.quickHeight(p.x, p.z);
      const leg = new THREE.Mesh(legGeo, mat);
      leg.position.set(p.x, ly, p.z);
      leg.rotation.y = yaw;
      leg.castShadow = true;
      g.add(leg);
      if (this.collision) {
        this.collision.add(new CylinderCollider({ x: p.x, y: ly + 3.3, z: p.z }, 0.34, 6.6));
      }
    }

    // Painted start line, laid on the road surface itself.
    const verts = [];
    const idx = [];
    const q = new THREE.Vector3();
    let vi = 0;
    for (let i = 0; i <= 10; i++) {
      const lat = -hw + (2 * hw * i) / 10;
      for (const off of [-0.32, 0.32]) {
        t.offsetPoint(s + off, lat, q);
        verts.push(q.x, t.heightAt(q.x, q.z, null) + 0.012, q.z);
      }
      // Same counter-clockwise winding as the road ribbons above.
      if (i > 0) idx.push(vi - 2, vi - 1, vi, vi - 1, vi + 1, vi);
      vi += 2;
    }
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    lineGeo.setIndex(idx);
    lineGeo.computeVertexNormals();
    g.add(new THREE.Mesh(lineGeo, new THREE.MeshStandardMaterial({
      color: 0xf2f2ee, roughness: 0.7,
      polygonOffset: true, polygonOffsetFactor: -5, polygonOffsetUnits: -5,
    })));

    this.props.add(g);
  }

  /* -------------------------------------------------------------- disposal */

  dispose() {
    this.scene.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      const mats = o.material
        ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
      for (const m of mats) {
        for (const key of Object.keys(m)) {
          const val = m[key];
          if (val && val.isTexture) val.dispose();
        }
        m.dispose();
      }
    });
    if (this.envRenderTarget) this.envRenderTarget.dispose();
  }
}
