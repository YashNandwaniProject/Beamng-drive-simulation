/**
 * collision.js
 * ---------------------------------------------------------------------------
 * Chassis collision against the static world: guardrails, walls, poles and the
 * terrain itself.
 *
 * The tyres already handle the road through the suspension raycasts, so this
 * system exists for everything the wheels cannot catch - hitting a barrier
 * side-on, landing on the roof, scraping the underbody over a kerb.
 *
 * The chassis is sampled as a set of points on its oriented bounding box.
 * Each penetrating point produces a normal impulse plus Coulomb friction at
 * that point, so an impact off-centre correctly spins the car rather than just
 * pushing it back.
 * ---------------------------------------------------------------------------
 */

import { Vec3, clamp } from './mathUtils.js';

/** Broad-phase cell size, metres. */
const CELL = 24;

/**
 * Default surface properties for world colliders (barriers, poles, walls).
 * These describe the WORLD, not any particular car, so they stay here rather
 * than in a vehicle spec. A collider may still override them individually.
 */
const DEFAULT_RESTITUTION = 0.22;
const DEFAULT_FRICTION = 0.35;

/* ------------------------------------------------------------------------ */
/* Colliders                                                                 */
/* ------------------------------------------------------------------------ */

/** Yaw-aligned oriented box: walls, barriers, kerb blocks, ramps. */
export class BoxCollider {
  constructor(center, halfExtents, yaw = 0, opts = {}) {
    this.center = new Vec3(center.x, center.y, center.z);
    this.half = new Vec3(halfExtents.x, halfExtents.y, halfExtents.z);
    this.yaw = yaw;
    this.cos = Math.cos(-yaw);
    this.sin = Math.sin(-yaw);
    this.restitution = opts.restitution ?? DEFAULT_RESTITUTION;
    this.friction = opts.friction ?? DEFAULT_FRICTION;
    this.radius = Math.hypot(this.half.x, this.half.y, this.half.z);
  }

  /**
   * Deepest-penetration test of a world point.
   * Returns the penetration depth (>0 when inside) and writes the outward
   * normal into `outNormal`.
   */
  test(p, outNormal) {
    // World -> box space (yaw only).
    const dx = p.x - this.center.x;
    const dy = p.y - this.center.y;
    const dz = p.z - this.center.z;
    const lx = dx * this.cos - dz * this.sin;
    const lz = dx * this.sin + dz * this.cos;

    const px = this.half.x - Math.abs(lx);
    const py = this.half.y - Math.abs(dy);
    const pz = this.half.z - Math.abs(lz);
    if (px <= 0 || py <= 0 || pz <= 0) return 0;

    // Push out along the axis of least penetration.
    let nx = 0, ny = 0, nz = 0, depth;
    if (px <= py && px <= pz) {
      depth = px; nx = Math.sign(lx) || 1;
    } else if (py <= pz) {
      depth = py; ny = Math.sign(dy) || 1;
    } else {
      depth = pz; nz = Math.sign(lz) || 1;
    }

    // Box space -> world (inverse yaw).
    outNormal.set(
      nx * this.cos + nz * this.sin,
      ny,
      -nx * this.sin + nz * this.cos,
    );
    return depth;
  }
}

/** Upright cylinder: light poles, trees, cones, tyre stacks. */
export class CylinderCollider {
  constructor(center, radius, height, opts = {}) {
    this.center = new Vec3(center.x, center.y, center.z);
    this.cylRadius = radius;
    this.halfHeight = height * 0.5;
    this.restitution = opts.restitution ?? 0.15;
    this.friction = opts.friction ?? 0.5;
    this.radius = Math.hypot(radius, this.halfHeight);
  }

  test(p, outNormal) {
    const dy = p.y - this.center.y;
    const py = this.halfHeight - Math.abs(dy);
    if (py <= 0) return 0;

    const dx = p.x - this.center.x;
    const dz = p.z - this.center.z;
    const r = Math.hypot(dx, dz);
    const pr = this.cylRadius - r;
    if (pr <= 0) return 0;

    if (pr <= py) {
      if (r < 1e-5) { outNormal.set(1, 0, 0); return pr; }
      outNormal.set(dx / r, 0, dz / r);
      return pr;
    }
    outNormal.set(0, Math.sign(dy) || 1, 0);
    return py;
  }
}

/* ------------------------------------------------------------------------ */
/* CollisionSystem                                                           */
/* ------------------------------------------------------------------------ */

export class CollisionSystem {
  /** @param {Terrain} terrain */
  constructor(terrain) {
    this.terrain = terrain;
    this.colliders = [];
    this.grid = new Map();

    this.contacts = 0;
    this.lastImpactSpeed = 0;

    /* --------------------------------------------------------- scratch */
    this._p = new Vec3();
    this._n = new Vec3();
    this._r = new Vec3();
    this._v = new Vec3();
    this._j = new Vec3();
    this._t = new Vec3();
    this._t2 = new Vec3();
    this._surfN = new Vec3();
  }

  add(collider) {
    this.colliders.push(collider);
    const c = collider.center;
    const reach = collider.radius + 3;
    const x0 = Math.floor((c.x - reach) / CELL);
    const x1 = Math.floor((c.x + reach) / CELL);
    const z0 = Math.floor((c.z - reach) / CELL);
    const z1 = Math.floor((c.z + reach) / CELL);
    for (let cx = x0; cx <= x1; cx++) {
      for (let cz = z0; cz <= z1; cz++) {
        const key = cx * 73856093 ^ cz * 19349663;
        let bucket = this.grid.get(key);
        if (!bucket) { bucket = []; this.grid.set(key, bucket); }
        bucket.push(collider);
      }
    }
    return collider;
  }

  /** Convenience: a wall segment between two ground points. */
  addWall(x0, z0, x1, z1, height, thickness, yTop) {
    const dx = x1 - x0, dz = z1 - z0;
    const len = Math.hypot(dx, dz);
    if (len < 1e-4) return null;
    const yaw = Math.atan2(dz, dx);
    const cx = (x0 + x1) * 0.5;
    const cz = (z0 + z1) * 0.5;
    const y = yTop - height * 0.5;
    return this.add(new BoxCollider(
      { x: cx, y, z: cz },
      { x: len * 0.5, y: height * 0.5, z: thickness * 0.5 },
      yaw,
    ));
  }

  _query(x, z) {
    const key = Math.floor(x / CELL) * 73856093 ^ Math.floor(z / CELL) * 19349663;
    return this.grid.get(key);
  }

  /* ===================================================================== */

  /**
   * Resolve every penetrating hull point against the world.
   *
   * The hull belongs to the VEHICLE, not to this system - a Revuelto is a
   * very different shape from an M3 - so the points are passed in rather than
   * built here. One CollisionSystem serves every car in the session.
   *
   * Returns the largest closing speed seen this step (used for impact FX).
   */
  resolve(body, dt, hullPoints) {
    this.contacts = 0;
    this.lastImpactSpeed = 0;
    if (!hullPoints) return 0;

    for (let i = 0; i < hullPoints.length; i++) {
      body.localToWorld(hullPoints[i], this._p);

      // --- static colliders --------------------------------------------
      const bucket = this._query(this._p.x, this._p.z);
      if (bucket) {
        for (let k = 0; k < bucket.length; k++) {
          const depth = bucket[k].test(this._p, this._n);
          if (depth > 0) {
            this._resolveContact(body, this._p, this._n, depth,
              bucket[k].restitution, bucket[k].friction);
          }
        }
      }

      // --- terrain ------------------------------------------------------
      // Only the lower half of the hull can hit the ground in normal driving;
      // testing every point keeps the car out of the terrain when it rolls.
      const gh = this.terrain.quickHeight(this._p.x, this._p.z);
      const depth = gh - this._p.y;
      if (depth > 0) {
        // Cheap surface normal from finite differences.
        const e = 0.5;
        this._surfN.set(
          -(this.terrain.quickHeight(this._p.x + e, this._p.z)
            - this.terrain.quickHeight(this._p.x - e, this._p.z)) / (2 * e),
          1,
          -(this.terrain.quickHeight(this._p.x, this._p.z + e)
            - this.terrain.quickHeight(this._p.x, this._p.z - e)) / (2 * e),
        ).normalize();
        this._resolveContact(body, this._p, this._surfN, Math.min(depth, 0.6), 0.05, 0.55);
      }
    }

    return this.lastImpactSpeed;
  }

  /**
   * Impulse response at one contact point.
   *
   *            -(1 + e) * v_n
   *   j = ----------------------------
   *        1/m + n . ((I^-1 (r x n)) x r)
   */
  _resolveContact(body, point, normal, depth, restitution, friction) {
    this._r.subVectors(point, body.position);
    body.pointVelocity(point, this._v);
    const vn = this._v.dot(normal);

    // Positional correction (Baumgarte-style, damped so it does not add energy).
    const correction = Math.min(depth, 0.25) * 0.35;
    body.position.addScaled(normal, correction);

    if (vn >= 0) return;   // already separating

    this.contacts++;
    this.lastImpactSpeed = Math.max(this.lastImpactSpeed, -vn);

    // Effective mass along the normal.
    this._t.crossVectors(this._r, normal);
    body.invInertiaWorld.apply(this._t, this._t2);
    this._t.crossVectors(this._t2, this._r);
    const denom = body.invMass + normal.dot(this._t);
    if (denom < 1e-9) return;

    // Soften the restitution for gentle contacts so the car settles.
    const e = -vn > 1.2 ? restitution : 0;
    const jn = (-(1 + e) * vn) / denom;

    this._j.copy(normal).scale(jn);
    this._applyImpulse(body, this._j, this._r);

    /* --- Coulomb friction in the contact plane ------------------------- */
    body.pointVelocity(point, this._v);
    this._t2.copy(this._v).addScaled(normal, -this._v.dot(normal));
    const vtLen = this._t2.length();
    if (vtLen < 1e-4) return;
    this._t2.scale(1 / vtLen);

    this._t.crossVectors(this._r, this._t2);
    body.invInertiaWorld.apply(this._t, this._j);
    this._t.crossVectors(this._j, this._r);
    const denomT = body.invMass + this._t2.dot(this._t);
    if (denomT < 1e-9) return;

    let jt = -vtLen / denomT;
    jt = clamp(jt, -friction * jn, friction * jn);
    this._j.copy(this._t2).scale(jt);
    this._applyImpulse(body, this._j, this._r);
  }

  _applyImpulse(body, impulse, r) {
    body.velocity.addScaled(impulse, body.invMass);
    this._t.crossVectors(r, impulse);
    body.invInertiaWorld.apply(this._t, this._t2);
    body.angularVelocity.add(this._t2);
  }
}
