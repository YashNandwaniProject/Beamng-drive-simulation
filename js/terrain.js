/**
 * terrain.js
 * ---------------------------------------------------------------------------
 * The world surface: a closed asphalt circuit laid over rolling terrain.
 *
 * This module is the SINGLE source of truth for ground geometry (requirement
 * #37: "the rendered road and collision road must remain aligned"). It exposes
 * a pure analytic height field; environment.js tessellates meshes by calling
 * exactly the same functions the tyres raycast against, so the visual surface
 * and the physical surface cannot drift apart.
 *
 * It has no three.js dependency so it can be exercised head-less.
 * ---------------------------------------------------------------------------
 */

import {
  Vec3, clamp, lerp, smoothstep, fbm2, valueNoise2, DEG2RAD,
} from './mathUtils.js';
import { SURFACES } from './config.js';

/* ------------------------------------------------------------------------ */
/* Circuit definition                                                        */
/* ------------------------------------------------------------------------ */

/**
 * Control points of the centreline, in order of travel.
 *   x, z        world position (metres)
 *   y           centreline elevation
 *   bank        cross-slope in degrees; NEGATIVE banks the road for a RIGHT
 *               turn (the left / outside edge is raised)
 *   halfWidth   half the asphalt width
 *   surface     surface key from config.SURFACES
 *   bumpAmp     amplitude of the section's washboard ripples (metres)
 *   bumpWave    wavelength of those ripples (metres)
 *
 * The loop runs clockwise when seen from above with +X right and +Z down,
 * so almost every corner is a right-hander.
 */
const CONTROL_POINTS = [
  { x: -260, z: -80, y: 0.0, bank: 0, halfWidth: 5.5, surface: 'ASPHALT' },
  { x: -180, z: -80, y: 0.0, bank: 0, halfWidth: 5.5, surface: 'ASPHALT' },
  { x: -80, z: -80, y: 0.0, bank: 0, halfWidth: 5.5, surface: 'ASPHALT' },
  { x: 20, z: -80, y: 0.0, bank: 0, halfWidth: 5.5, surface: 'ASPHALT' },
  { x: 110, z: -80, y: 0.4, bank: 0, halfWidth: 5.5, surface: 'ASPHALT' },
  // Long banked right-hand sweeper, climbing.
  { x: 200, z: -73, y: 1.6, bank: -6, halfWidth: 6.0, surface: 'ASPHALT' },
  { x: 262, z: -36, y: 4.0, bank: -15, halfWidth: 6.5, surface: 'ASPHALT' },
  { x: 285, z: 14, y: 5.2, bank: -16, halfWidth: 6.5, surface: 'ASPHALT' },
  { x: 258, z: 62, y: 3.0, bank: -8, halfWidth: 6.0, surface: 'ASPHALT' },
  // Wet patch on the exit - deliberately low grip.
  { x: 196, z: 88, y: 0.6, bank: 0, halfWidth: 5.5, surface: 'WET' },
  { x: 120, z: 96, y: 3.0, bank: 0, halfWidth: 5.5, surface: 'WET' },
  // Crest: hard compression on the way up, rebound over the top.
  { x: 48, z: 104, y: 7.6, bank: 0, halfWidth: 5.5, surface: 'ASPHALT' },
  { x: -14, z: 96, y: 4.0, bank: 0, halfWidth: 5.5, surface: 'ASPHALT' },
  // Esses: left then right.
  { x: -70, z: 118, y: 1.4, bank: 5, halfWidth: 5.5, surface: 'ASPHALT' },
  // Broken, washboard section.
  {
    x: -124, z: 92, y: 0.6, bank: -5, halfWidth: 5.0, surface: 'ASPHALT',
    bumpAmp: 0.042, bumpWave: 3.4,
  },
  {
    x: -186, z: 104, y: 0.2, bank: 0, halfWidth: 5.0, surface: 'ASPHALT',
    bumpAmp: 0.038, bumpWave: 2.6,
  },
  // Tight right-hand hairpin.
  { x: -240, z: 82, y: 0.0, bank: -5, halfWidth: 4.8, surface: 'ASPHALT' },
  { x: -278, z: 34, y: 0.0, bank: -7, halfWidth: 4.6, surface: 'ASPHALT' },
  { x: -272, z: -18, y: 0.0, bank: -4, halfWidth: 5.0, surface: 'ASPHALT' },
  { x: -290, z: -56, y: 0.0, bank: -3, halfWidth: 5.2, surface: 'ASPHALT' },
];

/** Resolution of the resampled centreline (metres between samples). */
const SAMPLE_STEP = 1.0;
/** Kerb / rumble strip width outside the asphalt edge. */
const RUMBLE_WIDTH = 0.55;
/** Gravel run-off width outside the kerb. */
const RUNOFF_WIDTH = 3.2;
/** Distance over which the terrain blends from the road edge to natural. */
const APRON_BLEND = 26.0;
/** Broad-phase grid cell size. */
const CELL = 16;

/* ------------------------------------------------------------------------ */

/** Uniform Catmull-Rom on a closed loop of scalar values. */
function catmullRom(p0, p1, p2, p3, t) {
  const t2 = t * t;
  const t3 = t2 * t;
  return 0.5 * (
    2 * p1
    + (-p0 + p2) * t
    + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
    + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
  );
}

/** Reusable result object for Terrain.sample(). */
export class SurfaceSample {
  constructor() {
    this.height = 0;
    this.normal = new Vec3(0, 1, 0);
    this.surface = SURFACES.GRASS;
    /** Signed lateral offset from the centreline, +ve to the right of travel. */
    this.lateral = 0;
    /** Arc length along the centreline. */
    this.distance = 0;
    this.onRoad = false;
    /** Index of the nearest centreline sample (useful for AI / minimap). */
    this.index = 0;
  }
}

/** Internal projection result, reused to keep the hot path allocation-free. */
class Projection {
  constructor() {
    this.valid = false;
    this.index = 0;
    this.distance = 0;
    this.lateral = 0;
    this.elev = 0;
    this.bank = 0;
    this.halfWidth = 0;
    this.surfaceKey = 'ASPHALT';
    this.bumpAmp = 0;
    this.bumpWave = 3;
    this.tangentX = 1;
    this.tangentZ = 0;
    this.centreX = 0;
    this.centreZ = 0;
  }
}

export class Terrain {
  constructor() {
    this._buildCentreline();
    this._buildGrid();

    // Scratch objects - the physics loop must not allocate.
    this._proj = new Projection();
    this._projB = new Projection();
    this._tmp = new Vec3();
  }

  /* ---------------------------------------------------------------- build */

  _buildCentreline() {
    const cp = CONTROL_POINTS;
    const n = cp.length;

    // 1. Dense uniform-parameter sampling of the closed Catmull-Rom spline.
    const dense = [];
    const SUB = 24;
    for (let i = 0; i < n; i++) {
      const p0 = cp[(i - 1 + n) % n];
      const p1 = cp[i];
      const p2 = cp[(i + 1) % n];
      const p3 = cp[(i + 2) % n];
      for (let s = 0; s < SUB; s++) {
        const t = s / SUB;
        dense.push({
          x: catmullRom(p0.x, p1.x, p2.x, p3.x, t),
          z: catmullRom(p0.z, p1.z, p2.z, p3.z, t),
          y: catmullRom(p0.y, p1.y, p2.y, p3.y, t),
          bank: catmullRom(p0.bank, p1.bank, p2.bank, p3.bank, t) * DEG2RAD,
          halfWidth: catmullRom(p0.halfWidth, p1.halfWidth, p2.halfWidth, p3.halfWidth, t),
          bumpAmp: catmullRom(
            p0.bumpAmp || 0, p1.bumpAmp || 0, p2.bumpAmp || 0, p3.bumpAmp || 0, t,
          ),
          bumpWave: lerp(p1.bumpWave || 3, p2.bumpWave || 3, t),
          // Surface is a category, so it snaps rather than blends.
          surfaceKey: (t < 0.5 ? p1 : p2).surface || 'ASPHALT',
        });
      }
    }

    // 2. Arc-length parameterise and resample at a uniform spacing.
    const m = dense.length;
    const cum = new Float64Array(m + 1);
    for (let i = 0; i < m; i++) {
      const a = dense[i];
      const b = dense[(i + 1) % m];
      cum[i + 1] = cum[i] + Math.hypot(b.x - a.x, b.z - a.z);
    }
    const total = cum[m];
    const count = Math.max(8, Math.round(total / SAMPLE_STEP));
    const step = total / count;

    this.length = total;
    this.count = count;
    this.step = step;

    this.px = new Float64Array(count);
    this.pz = new Float64Array(count);
    this.py = new Float64Array(count);
    this.bank = new Float64Array(count);
    this.halfWidth = new Float64Array(count);
    this.bumpAmp = new Float64Array(count);
    this.bumpWave = new Float64Array(count);
    this.tx = new Float64Array(count);
    this.tz = new Float64Array(count);
    this.surfaceKey = new Array(count);

    let seg = 0;
    for (let i = 0; i < count; i++) {
      const target = i * step;
      while (seg < m - 1 && cum[seg + 1] < target) seg++;
      const segLen = cum[seg + 1] - cum[seg] || 1e-9;
      const t = clamp((target - cum[seg]) / segLen, 0, 1);
      const a = dense[seg];
      const b = dense[(seg + 1) % m];
      this.px[i] = lerp(a.x, b.x, t);
      this.pz[i] = lerp(a.z, b.z, t);
      this.py[i] = lerp(a.y, b.y, t);
      this.bank[i] = lerp(a.bank, b.bank, t);
      this.halfWidth[i] = lerp(a.halfWidth, b.halfWidth, t);
      this.bumpAmp[i] = lerp(a.bumpAmp, b.bumpAmp, t);
      this.bumpWave[i] = lerp(a.bumpWave, b.bumpWave, t);
      this.surfaceKey[i] = (t < 0.5 ? a : b).surfaceKey;
    }

    // 3. Central-difference tangents (normalised, XZ only).
    for (let i = 0; i < count; i++) {
      const p = (i - 1 + count) % count;
      const q = (i + 1) % count;
      let dx = this.px[q] - this.px[p];
      let dz = this.pz[q] - this.pz[p];
      const l = Math.hypot(dx, dz) || 1;
      this.tx[i] = dx / l;
      this.tz[i] = dz / l;
    }

    // 4. Light smoothing pass on elevation so the resampling does not leave
    //    kinks that the suspension would read as bumps.
    const smoothY = new Float64Array(count);
    for (let i = 0; i < count; i++) {
      const a = this.py[(i - 1 + count) % count];
      const b = this.py[i];
      const c = this.py[(i + 1) % count];
      smoothY[i] = (a + 2 * b + c) * 0.25;
    }
    this.py.set(smoothY);
  }

  _buildGrid() {
    this.grid = new Map();
    this.maxReach = 0;
    for (let i = 0; i < this.count; i++) {
      this.maxReach = Math.max(
        this.maxReach,
        this.halfWidth[i] + RUMBLE_WIDTH + RUNOFF_WIDTH + APRON_BLEND,
      );
    }
    const reach = this.maxReach + this.step;
    for (let i = 0; i < this.count; i++) {
      const x = this.px[i], z = this.pz[i];
      const x0 = Math.floor((x - reach) / CELL);
      const x1 = Math.floor((x + reach) / CELL);
      const z0 = Math.floor((z - reach) / CELL);
      const z1 = Math.floor((z + reach) / CELL);
      for (let cx = x0; cx <= x1; cx++) {
        for (let cz = z0; cz <= z1; cz++) {
          const key = cx * 73856093 ^ cz * 19349663;
          let bucket = this.grid.get(key);
          if (!bucket) { bucket = []; this.grid.set(key, bucket); }
          bucket.push(i);
        }
      }
    }
  }

  /* ------------------------------------------------------------ centreline */

  /** World position of the centreline at arc length s (wraps). */
  pointAt(s, out = new Vec3()) {
    const i = this._indexAt(s);
    const t = (s / this.step) - Math.floor(s / this.step);
    const j = (i + 1) % this.count;
    out.set(
      lerp(this.px[i], this.px[j], t),
      lerp(this.py[i], this.py[j], t),
      lerp(this.pz[i], this.pz[j], t),
    );
    return out;
  }

  /** Unit tangent (direction of travel) at arc length s. */
  tangentAt(s, out = new Vec3()) {
    const i = this._indexAt(s);
    out.set(this.tx[i], 0, this.tz[i]);
    return out.normalize();
  }

  /** Heading angle (radians, atan2 of the tangent) at arc length s. */
  headingAt(s) {
    const i = this._indexAt(s);
    return Math.atan2(this.tz[i], this.tx[i]);
  }

  bankAt(s) { return this.bank[this._indexAt(s)]; }
  halfWidthAt(s) { return this.halfWidth[this._indexAt(s)]; }

  _indexAt(s) {
    let i = Math.floor(s / this.step) % this.count;
    if (i < 0) i += this.count;
    return i;
  }

  /**
   * A point offset laterally from the centreline, on the road surface.
   * `lateral` is positive to the right of the direction of travel.
   */
  offsetPoint(s, lateral, out = new Vec3()) {
    this.pointAt(s, out);
    const i = this._indexAt(s);
    // right = forward x up = (tx,0,tz) x (0,1,0) = (-tz, 0, tx)
    out.x += -this.tz[i] * lateral;
    out.z += this.tx[i] * lateral;
    out.y += lateral * Math.tan(this.bank[i]);
    return out;
  }

  /* -------------------------------------------------------------- sampling */

  /** Nearest-point projection of a world XZ onto the centreline. */
  _project(x, z, proj) {
    const cx = Math.floor(x / CELL);
    const cz = Math.floor(z / CELL);
    const bucket = this.grid.get(cx * 73856093 ^ cz * 19349663);
    proj.valid = false;
    if (!bucket) return proj;

    let best = Infinity;
    let bi = -1;
    for (let k = 0; k < bucket.length; k++) {
      const i = bucket[k];
      const dx = x - this.px[i];
      const dz = z - this.pz[i];
      const d2 = dx * dx + dz * dz;
      if (d2 < best) { best = d2; bi = i; }
    }
    if (bi < 0) return proj;

    // Refine against the two adjacent segments for a smooth lateral value.
    const n = this.count;
    let bestT = 0, bestIdx = bi, bestD2 = Infinity;
    for (let o = -1; o <= 0; o++) {
      const i = (bi + o + n) % n;
      const j = (i + 1) % n;
      const ax = this.px[i], az = this.pz[i];
      const bx = this.px[j], bz = this.pz[j];
      const ex = bx - ax, ez = bz - az;
      const len2 = ex * ex + ez * ez || 1e-9;
      const t = clamp(((x - ax) * ex + (z - az) * ez) / len2, 0, 1);
      const qx = ax + ex * t, qz = az + ez * t;
      const d2 = (x - qx) * (x - qx) + (z - qz) * (z - qz);
      if (d2 < bestD2) { bestD2 = d2; bestT = t; bestIdx = i; }
    }

    const i = bestIdx;
    const j = (i + 1) % n;
    const t = bestT;

    proj.valid = true;
    proj.index = i;
    proj.distance = (i + t) * this.step;
    proj.centreX = lerp(this.px[i], this.px[j], t);
    proj.centreZ = lerp(this.pz[i], this.pz[j], t);
    proj.elev = lerp(this.py[i], this.py[j], t);
    proj.bank = lerp(this.bank[i], this.bank[j], t);
    proj.halfWidth = lerp(this.halfWidth[i], this.halfWidth[j], t);
    proj.bumpAmp = lerp(this.bumpAmp[i], this.bumpAmp[j], t);
    proj.bumpWave = lerp(this.bumpWave[i], this.bumpWave[j], t);
    proj.surfaceKey = (t < 0.5 ? this.surfaceKey[i] : this.surfaceKey[j]);

    let tx = lerp(this.tx[i], this.tx[j], t);
    let tz = lerp(this.tz[i], this.tz[j], t);
    const l = Math.hypot(tx, tz) || 1;
    tx /= l; tz /= l;
    proj.tangentX = tx;
    proj.tangentZ = tz;
    // right = (-tz, 0, tx)
    proj.lateral = (x - proj.centreX) * -tz + (z - proj.centreZ) * tx;
    return proj;
  }

  /** Natural (off-circuit) terrain height: gentle rolling hills. */
  naturalHeight(x, z) {
    const broad = fbm2(x * 0.0026, z * 0.0026, 3) * 11.0;
    const mid = fbm2(x * 0.011, z * 0.011, 3) * 2.2;
    const fine = fbm2(x * 0.075, z * 0.075, 2) * 0.28;
    return broad + mid + fine - 2.0;
  }

  /**
   * Height of the drivable surface at a world XZ.
   * `proj` may be supplied to reuse a projection (the finite-difference normal
   * calls this five times for the same neighbourhood).
   */
  heightAt(x, z, proj) {
    const p = proj && proj.valid ? proj : this._project(x, z, this._projB);
    if (!p.valid) return this.naturalHeight(x, z);

    const hw = p.halfWidth;
    const lat = p.lateral;
    const d = Math.abs(lat);
    const kerbEdge = hw + RUMBLE_WIDTH;

    // Cross-slope plane, clamped so the banking does not keep rising off-road.
    const latClamped = clamp(lat, -kerbEdge, kerbEdge);
    const roadPlane = p.elev + latClamped * Math.tan(p.bank);

    if (d <= kerbEdge) {
      let h = p.elev + lat * Math.tan(p.bank);
      // Micro texture: every asphalt surface has some unevenness, which is
      // what makes the suspension work at all on a "flat" road.
      h += fbm2(x * 0.31, z * 0.31, 2) * 0.011;
      h += valueNoise2(x * 1.7, z * 1.7) * 0.0035;
      // Section-specific washboard ripples.
      if (p.bumpAmp > 1e-4) {
        h += Math.sin((p.distance / p.bumpWave) * Math.PI * 2) * p.bumpAmp;
      }
      // Rumble strip ridges outside the white line.
      if (d > hw) {
        const ridge = 0.5 + 0.5 * Math.sin(p.distance * 4.4);
        const fade = smoothstep(hw, hw + 0.10, d) * smoothstep(kerbEdge, kerbEdge - 0.10, d);
        h += ridge * 0.026 * fade;
      }
      return h;
    }

    // Off the kerb: drop to the run-off, then blend into natural terrain.
    const dropT = smoothstep(kerbEdge, kerbEdge + 3.0, d);
    const shoulder = roadPlane - 0.14 * dropT;
    const natural = this.naturalHeight(x, z)
      + fbm2(x * 0.22, z * 0.22, 2) * 0.09;
    const blend = smoothstep(kerbEdge + 1.0, kerbEdge + APRON_BLEND, d);
    return lerp(shoulder, natural, blend);
  }

  /**
   * Full surface query used by the tyre contact solver.
   * Fills `out` (a SurfaceSample) and returns it. Allocation-free.
   */
  sample(x, z, out) {
    const p = this._project(x, z, this._proj);
    const h = this.heightAt(x, z, p);

    // Analytic normal by central differences of the SAME height function that
    // the mesh is built from, so the collider can never disagree with what is
    // drawn. e is small enough to resolve kerbs, large enough to stay stable.
    const e = 0.14;
    const hx1 = this.heightAt(x + e, z, null);
    const hx0 = this.heightAt(x - e, z, null);
    const hz1 = this.heightAt(x, z + e, null);
    const hz0 = this.heightAt(x, z - e, null);
    out.normal.set(-(hx1 - hx0) / (2 * e), 1, -(hz1 - hz0) / (2 * e)).normalize();

    out.height = h;
    out.index = p.valid ? p.index : 0;
    out.lateral = p.valid ? p.lateral : 1e9;
    out.distance = p.valid ? p.distance : 0;

    if (!p.valid) {
      out.surface = SURFACES.GRASS;
      out.onRoad = false;
      return out;
    }

    const d = Math.abs(p.lateral);
    const hw = p.halfWidth;
    if (d <= hw) {
      out.surface = SURFACES[p.surfaceKey] || SURFACES.ASPHALT;
      out.onRoad = true;
    } else if (d <= hw + RUMBLE_WIDTH) {
      out.surface = SURFACES.RUMBLE;
      out.onRoad = true;
    } else if (d <= hw + RUMBLE_WIDTH + RUNOFF_WIDTH) {
      out.surface = SURFACES.GRAVEL;
      out.onRoad = false;
    } else {
      out.surface = SURFACES.GRASS;
      out.onRoad = false;
    }
    return out;
  }

  /** Cheap height-only query (used by camera collision and prop scattering). */
  quickHeight(x, z) {
    return this.heightAt(x, z, null);
  }

  /**
   * Where a world point sits relative to the circuit, without the cost of a
   * full surface query. Used by the mesh builder and by prop scattering to
   * keep scenery off the racing surface.
   *
   * @returns {{onTrack: boolean, lateral: number, halfWidth: number,
   *            distance: number}} `lateral` is signed, +ve to the right of
   *            travel; `onTrack` is false when the point is nowhere near the
   *            circuit at all.
   */
  lateralAt(x, z) {
    const p = this._project(x, z, this._projB);
    if (!p.valid) {
      return { onTrack: false, lateral: Infinity, halfWidth: 0, distance: 0 };
    }
    return {
      onTrack: true,
      lateral: p.lateral,
      halfWidth: p.halfWidth,
      distance: p.distance,
    };
  }

  /* --------------------------------------------------------------- helpers */

  /**
   * A spawn transform on the racing surface.
   * Returns { position: Vec3, heading: number } where heading is the yaw about
   * +Y that aligns the body's +X axis with the direction of travel.
   */
  spawnTransform(distance, lateral) {
    const s = ((distance % this.length) + this.length) % this.length;
    const pos = this.offsetPoint(s, lateral, new Vec3());
    pos.y = this.quickHeight(pos.x, pos.z);
    return { position: pos, heading: this.headingAt(s), distance: s };
  }

  /** World-space bounds of the circuit, padded, for terrain mesh extents. */
  bounds(pad = 220) {
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (let i = 0; i < this.count; i++) {
      if (this.px[i] < minX) minX = this.px[i];
      if (this.px[i] > maxX) maxX = this.px[i];
      if (this.pz[i] < minZ) minZ = this.pz[i];
      if (this.pz[i] > maxZ) maxZ = this.pz[i];
    }
    return {
      minX: minX - pad, maxX: maxX + pad,
      minZ: minZ - pad, maxZ: maxZ + pad,
    };
  }

  /** Constants exposed for the mesh builder. */
  static get RUMBLE_WIDTH() { return RUMBLE_WIDTH; }
  static get RUNOFF_WIDTH() { return RUNOFF_WIDTH; }
  static get APRON_BLEND() { return APRON_BLEND; }
}
