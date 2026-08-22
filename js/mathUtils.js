/**
 * mathUtils.js
 * ---------------------------------------------------------------------------
 * Minimal, allocation-conscious math layer used by the physics core.
 *
 * The physics modules deliberately do NOT import three.js: keeping them on a
 * private, dependency-free vector type means the whole simulation can be run
 * head-less under Node for regression testing (see /tests), and it guarantees
 * that no renderer state can leak into the physics state (requirement #45).
 *
 * Every hot-path method has an in-place variant so the fixed-step loop can run
 * without allocating (requirement #46).
 * ---------------------------------------------------------------------------
 */

export const EPSILON = 1e-9;
export const GRAVITY = 9.80665;          // m/s^2 - also the 1 G reference
export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;
export const KMH = 3.6;                  // m/s -> km/h
export const PSI2PA = 6894.757;

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const clamp01 = (v) => (v < 0 ? 0 : v > 1 ? 1 : v);
export const lerp = (a, b, t) => a + (b - a) * t;
export const sign = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);
export const sq = (v) => v * v;

/** Smooth Hermite interpolation, 0 below a, 1 above b. */
export function smoothstep(a, b, x) {
  const t = clamp01((x - a) / (b - a || EPSILON));
  return t * t * (3 - 2 * t);
}

/** Move cur toward target by at most maxDelta. Used for steering rate limits. */
export function moveTowards(cur, target, maxDelta) {
  const d = target - cur;
  if (Math.abs(d) <= maxDelta) return target;
  return cur + Math.sign(d) * maxDelta;
}

/**
 * Frame-rate independent exponential smoothing.
 * halfLife is the time in which the remaining error halves.
 */
export function damp(cur, target, halfLife, dt) {
  if (halfLife <= 0) return target;
  const k = 1 - Math.pow(2, -dt / halfLife);
  return cur + (target - cur) * k;
}

/** Piecewise-linear lookup over an [[x,y], ...] table (x ascending). */
export function sampleCurve(table, x) {
  const n = table.length;
  if (x <= table[0][0]) return table[0][1];
  if (x >= table[n - 1][0]) return table[n - 1][1];
  for (let i = 1; i < n; i++) {
    if (x <= table[i][0]) {
      const x0 = table[i - 1][0], y0 = table[i - 1][1];
      const x1 = table[i][0], y1 = table[i][1];
      return y0 + ((y1 - y0) * (x - x0)) / (x1 - x0 || EPSILON);
    }
  }
  return table[n - 1][1];
}

/* ========================================================================= */
/* Vec3                                                                       */
/* ========================================================================= */

export class Vec3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }

  set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
  copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
  clone() { return new Vec3(this.x, this.y, this.z); }
  setZero() { this.x = 0; this.y = 0; this.z = 0; return this; }

  add(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
  sub(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
  scale(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
  addScaled(v, s) { this.x += v.x * s; this.y += v.y * s; this.z += v.z * s; return this; }

  addVectors(a, b) { this.x = a.x + b.x; this.y = a.y + b.y; this.z = a.z + b.z; return this; }
  subVectors(a, b) { this.x = a.x - b.x; this.y = a.y - b.y; this.z = a.z - b.z; return this; }

  crossVectors(a, b) {
    const ax = a.x, ay = a.y, az = a.z, bx = b.x, by = b.y, bz = b.z;
    this.x = ay * bz - az * by;
    this.y = az * bx - ax * bz;
    this.z = ax * by - ay * bx;
    return this;
  }

  dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
  lengthSq() { return this.x * this.x + this.y * this.y + this.z * this.z; }
  length() { return Math.sqrt(this.lengthSq()); }

  normalize() {
    const l = this.length();
    if (l > EPSILON) { this.x /= l; this.y /= l; this.z /= l; }
    return this;
  }

  /** Remove the component of this vector along the unit normal n. */
  projectOnPlane(n) {
    const d = this.dot(n);
    this.x -= n.x * d; this.y -= n.y * d; this.z -= n.z * d;
    return this;
  }

  lerpVectors(a, b, t) {
    this.x = a.x + (b.x - a.x) * t;
    this.y = a.y + (b.y - a.y) * t;
    this.z = a.z + (b.z - a.z) * t;
    return this;
  }

  distanceTo(v) {
    const dx = this.x - v.x, dy = this.y - v.y, dz = this.z - v.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  isFinite() {
    return Number.isFinite(this.x) && Number.isFinite(this.y) && Number.isFinite(this.z);
  }
}

/* ========================================================================= */
/* Quat - unit quaternion (x, y, z, w)                                        */
/* ========================================================================= */

export class Quat {
  constructor(x = 0, y = 0, z = 0, w = 1) { this.x = x; this.y = y; this.z = z; this.w = w; }

  set(x, y, z, w) { this.x = x; this.y = y; this.z = z; this.w = w; return this; }
  copy(q) { this.x = q.x; this.y = q.y; this.z = q.z; this.w = q.w; return this; }
  clone() { return new Quat(this.x, this.y, this.z, this.w); }
  identity() { return this.set(0, 0, 0, 1); }

  setFromAxisAngle(axis, angle) {
    const h = angle * 0.5, s = Math.sin(h);
    return this.set(axis.x * s, axis.y * s, axis.z * s, Math.cos(h));
  }

  /** this = a * b (apply b first, then a). */
  multiplyQuaternions(a, b) {
    const ax = a.x, ay = a.y, az = a.z, aw = a.w;
    const bx = b.x, by = b.y, bz = b.z, bw = b.w;
    this.x = ax * bw + aw * bx + ay * bz - az * by;
    this.y = ay * bw + aw * by + az * bx - ax * bz;
    this.z = az * bw + aw * bz + ax * by - ay * bx;
    this.w = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }

  normalize() {
    let l = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
    if (l < EPSILON) return this.identity();
    l = 1 / l;
    this.x *= l; this.y *= l; this.z *= l; this.w *= l;
    return this;
  }

  conjugate() { this.x = -this.x; this.y = -this.y; this.z = -this.z; return this; }

  /**
   * Integrate this orientation by a world-space angular velocity for dt.
   * dq/dt = 0.5 * omega_quat * q
   */
  integrate(omega, dt) {
    const hx = omega.x * dt * 0.5, hy = omega.y * dt * 0.5, hz = omega.z * dt * 0.5;
    const x = this.x, y = this.y, z = this.z, w = this.w;
    this.x += hx * w + hy * z - hz * y;
    this.y += hy * w + hz * x - hx * z;
    this.z += hz * w + hx * y - hy * x;
    this.w += -(hx * x + hy * y + hz * z);
    return this.normalize();
  }

  /** Rotate v by this quaternion, writing to out. */
  rotate(v, out) {
    const qx = this.x, qy = this.y, qz = this.z, qw = this.w;
    const ix = qw * v.x + qy * v.z - qz * v.y;
    const iy = qw * v.y + qz * v.x - qx * v.z;
    const iz = qw * v.z + qx * v.y - qy * v.x;
    const iw = -qx * v.x - qy * v.y - qz * v.z;
    out.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    out.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    out.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;
    return out;
  }

  /** Rotate v by the inverse of this quaternion (world -> body). */
  rotateInverse(v, out) {
    const qx = -this.x, qy = -this.y, qz = -this.z, qw = this.w;
    const ix = qw * v.x + qy * v.z - qz * v.y;
    const iy = qw * v.y + qz * v.x - qx * v.z;
    const iz = qw * v.z + qx * v.y - qy * v.x;
    const iw = -qx * v.x - qy * v.y - qz * v.z;
    out.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    out.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    out.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;
    return out;
  }

  /** Intrinsic yaw(Y) -> pitch(X) -> roll(Z). */
  setFromEulerYXZ(yaw, pitch, roll) {
    const c1 = Math.cos(yaw * 0.5), s1 = Math.sin(yaw * 0.5);
    const c2 = Math.cos(pitch * 0.5), s2 = Math.sin(pitch * 0.5);
    const c3 = Math.cos(roll * 0.5), s3 = Math.sin(roll * 0.5);
    this.x = s2 * c1 * c3 + c2 * s1 * s3;
    this.y = c2 * s1 * c3 - s2 * c1 * s3;
    this.z = c2 * c1 * s3 - s2 * s1 * c3;
    this.w = c2 * c1 * c3 + s2 * s1 * s3;
    return this.normalize();
  }

  /**
   * Shortest-arc rotation taking unit vector `from` to unit vector `to`.
   * Used when rigging the GLB wheels into the physics frame.
   */
  setFromUnitVectors(from, to) {
    let r = from.dot(to) + 1;
    if (r < 1e-6) {
      r = 0;
      if (Math.abs(from.x) > Math.abs(from.z)) this.set(-from.y, from.x, 0, r);
      else this.set(0, -from.z, from.y, r);
    } else {
      this.set(
        from.y * to.z - from.z * to.y,
        from.z * to.x - from.x * to.z,
        from.x * to.y - from.y * to.x,
        r,
      );
    }
    return this.normalize();
  }
}

/* ========================================================================= */
/* Mat3 - column-major 3x3, used for the inertia tensor                       */
/* ========================================================================= */

export class Mat3 {
  constructor() { this.e = new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1]); }

  /** out = this * v */
  apply(v, out) {
    const e = this.e;
    const x = v.x, y = v.y, z = v.z;
    out.x = e[0] * x + e[3] * y + e[6] * z;
    out.y = e[1] * x + e[4] * y + e[7] * z;
    out.z = e[2] * x + e[5] * y + e[8] * z;
    return out;
  }

  /**
   * Rebuild a world-space (inverse) inertia tensor:  M_world = R * diag * R^T
   * Pass the body-frame inverse-inertia diagonal to get the inverse tensor,
   * or the plain diagonal to get the tensor itself.
   */
  setWorldFromDiagonal(q, diag) {
    const x = q.x, y = q.y, z = q.z, w = q.w;
    const x2 = x + x, y2 = y + y, z2 = z + z;
    const xx = x * x2, xy = x * y2, xz = x * z2;
    const yy = y * y2, yz = y * z2, zz = z * z2;
    const wx = w * x2, wy = w * y2, wz = w * z2;
    const r0 = 1 - (yy + zz), r1 = xy + wz, r2 = xz - wy;
    const r3 = xy - wz, r4 = 1 - (xx + zz), r5 = yz + wx;
    const r6 = xz + wy, r7 = yz - wx, r8 = 1 - (xx + yy);
    const a = diag.x, b = diag.y, c = diag.z;
    const m0 = r0 * a, m3 = r3 * b, m6 = r6 * c;
    const m1 = r1 * a, m4 = r4 * b, m7 = r7 * c;
    const m2 = r2 * a, m5 = r5 * b, m8 = r8 * c;
    const e = this.e;
    e[0] = m0 * r0 + m3 * r3 + m6 * r6;
    e[1] = m1 * r0 + m4 * r3 + m7 * r6;
    e[2] = m2 * r0 + m5 * r3 + m8 * r6;
    e[3] = m0 * r1 + m3 * r4 + m6 * r7;
    e[4] = m1 * r1 + m4 * r4 + m7 * r7;
    e[5] = m2 * r1 + m5 * r4 + m8 * r7;
    e[6] = m0 * r2 + m3 * r5 + m6 * r8;
    e[7] = m1 * r2 + m4 * r5 + m7 * r8;
    e[8] = m2 * r2 + m5 * r5 + m8 * r8;
    return this;
  }
}

/* ========================================================================= */
/* Deterministic noise (value noise + fBm) - shared by physics and rendering  */
/* ========================================================================= */

/** Integer hash -> [0,1). Deterministic, so mesh and collider always agree. */
export function hash2(ix, iy) {
  let h = (Math.imul(ix, 374761393) + Math.imul(iy, 668265263)) | 0;
  h = (h ^ (h >>> 13)) | 0;
  h = Math.imul(h, 1274126177) | 0;
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

/** Smooth 2D value noise in [-1,1]. */
export function valueNoise2(x, y) {
  const ix = Math.floor(x), iy = Math.floor(y);
  const fx = x - ix, fy = y - iy;
  const ux = fx * fx * (3 - 2 * fx);
  const uy = fy * fy * (3 - 2 * fy);
  const a = hash2(ix, iy), b = hash2(ix + 1, iy);
  const c = hash2(ix, iy + 1), d = hash2(ix + 1, iy + 1);
  const top = a + (b - a) * ux;
  const bot = c + (d - c) * ux;
  return (top + (bot - top) * uy) * 2 - 1;
}

/** Fractal brownian motion: `octaves` layers of value noise. */
export function fbm2(x, y, octaves = 4, lacunarity = 2.0, gain = 0.5) {
  let amp = 1, freq = 1, sum = 0, norm = 0;
  for (let i = 0; i < octaves; i++) {
    sum += valueNoise2(x * freq, y * freq) * amp;
    norm += amp;
    amp *= gain;
    freq *= lacunarity;
  }
  return sum / (norm || 1);
}

/** Small deterministic PRNG (mulberry32) for scene scattering. */
export function makeRng(seed = 1) {
  let a = seed >>> 0;
  return function rng() {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
