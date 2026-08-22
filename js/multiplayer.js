/**
 * multiplayer.js
 * ---------------------------------------------------------------------------
 * Manages the OTHER players' cars.
 *
 * The important design decision here is that a remote car is a full `Vehicle`,
 * not a simplified stand-in. It loads the same GLB, gets the same wheel rig,
 * the same tyre-deformation shader and the same interior steering wheel - and
 * is drawn by the same `syncVisuals()`. The only difference is where its state
 * comes from: replicated snapshots instead of an integrator.
 *
 * That is deliberate. A separate, lesser renderer for remote cars is how you
 * end up with wheels that do not turn and steering that does not move on
 * everyone else's screen, which is exactly what requirement #23 rules out.
 * ---------------------------------------------------------------------------
 */

import * as THREE from 'three';
import { Vehicle } from './vehicle.js';
import { applyVehicleState } from './network.js';
import { getVehicleSpec } from './vehicles/registry.js';

/** Draw a name plate into a canvas texture. */
function makeNameplate(name, carName, accent) {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 128;
  const ctx = c.getContext('2d');

  ctx.fillStyle = 'rgba(8, 11, 16, 0.78)';
  ctx.beginPath();
  const r = 26;
  ctx.moveTo(r, 0);
  ctx.arcTo(c.width, 0, c.width, c.height, r);
  ctx.arcTo(c.width, c.height, 0, c.height, r);
  ctx.arcTo(0, c.height, 0, 0, r);
  ctx.arcTo(0, 0, c.width, 0, r);
  ctx.fill();

  ctx.fillStyle = accent || '#4fb3ff';
  ctx.fillRect(0, c.height - 6, c.width, 6);

  ctx.textAlign = 'center';
  ctx.fillStyle = '#e8eef6';
  ctx.font = '600 46px Inter, system-ui, sans-serif';
  ctx.fillText(name, c.width / 2, 52);
  ctx.fillStyle = 'rgba(232,238,246,0.6)';
  ctx.font = '400 28px Inter, system-ui, sans-serif';
  ctx.fillText(carName, c.width / 2, 92);

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

export class RemotePlayers {
  /**
   * @param {THREE.Scene} scene
   * @param {Terrain} terrain
   * @param {NetworkSession} session
   */
  constructor(scene, terrain, session) {
    this.scene = scene;
    this.terrain = terrain;
    this.session = session;

    /** peerId -> { vehicle, label, peer, ready } */
    this.entries = new Map();

    session.onPeerJoin = (peer) => this._spawn(peer);
    session.onPeerLeave = (peer) => this._despawn(peer.id);
    session.onPeerVehicleChange = (peer) => {
      // A player picked a different car: rebuild them from scratch.
      this._despawn(peer.id);
      this._spawn(peer);
    };
  }

  async _spawn(peer) {
    if (!peer.vehicleId || this.entries.has(peer.id)) return;

    // Reserve the slot immediately so a second snapshot cannot double-spawn.
    const entry = { vehicle: null, label: null, peer, ready: false };
    this.entries.set(peer.id, entry);

    try {
      const spec = getVehicleSpec(peer.vehicleId);
      /** No collision system: remote cars are never simulated. */
      const vehicle = new Vehicle(this.terrain, spec, null);
      await vehicle.load();

      // The peer may have left while the model was loading.
      if (!this.entries.has(peer.id)) {
        vehicle.dispose();
        return;
      }

      this.scene.add(vehicle.object3D);

      const label = new THREE.Sprite(new THREE.SpriteMaterial({
        map: makeNameplate(peer.name, spec.shortName, spec.garage.accent),
        depthTest: false,
        transparent: true,
      }));
      label.scale.set(2.6, 0.65, 1);
      label.renderOrder = 900;
      vehicle.object3D.add(label);
      label.position.set(0, 1.9, 0);

      entry.vehicle = vehicle;
      entry.label = label;
      entry.ready = true;
    } catch (err) {
      console.error(`Failed to spawn remote player ${peer.id}:`, err);
      this.entries.delete(peer.id);
    }
  }

  _despawn(peerId) {
    const entry = this.entries.get(peerId);
    if (!entry) return;
    this.entries.delete(peerId);
    if (entry.vehicle) {
      if (entry.label) {
        if (entry.label.material.map) entry.label.material.map.dispose();
        entry.label.material.dispose();
      }
      this.scene.remove(entry.vehicle.object3D);
      entry.vehicle.dispose();
    }
  }

  /**
   * Replay every remote car from its interpolated snapshot pair, then draw it
   * through the normal visual path.
   */
  update(dt, camera) {
    for (const entry of this.entries.values()) {
      if (!entry.ready) continue;
      const sample = this.session.samplePeer(entry.peer);
      if (!sample) continue;

      applyVehicleState(entry.vehicle, sample.a, sample.b, sample.alpha);

      /**
       * The remote car's transform came straight from the snapshot, so both
       * interpolation endpoints are that same pose - `syncVisuals` then has
       * nothing left to blend and simply draws it.
       */
      const b = entry.vehicle.physics.body;
      entry.vehicle._prevPos.set(b.position.x, b.position.y, b.position.z);
      entry.vehicle._curPos.copy(entry.vehicle._prevPos);
      entry.vehicle._prevQuat.set(b.quat.x, b.quat.y, b.quat.z, b.quat.w);
      entry.vehicle._curQuat.copy(entry.vehicle._prevQuat);

      entry.vehicle.syncVisuals(1, dt);

      // Keep the name plate upright and facing the camera.
      if (entry.label && camera) {
        entry.label.quaternion.copy(camera.quaternion);
        entry.label.position.set(0, 1.9, 0);
        entry.label.quaternion.premultiply(
          entry.vehicle.object3D.getWorldQuaternion(_q).invert(),
        );
      }
    }
  }

  /** Number of remote cars currently drawn. */
  get count() {
    let n = 0;
    for (const e of this.entries.values()) if (e.ready) n++;
    return n;
  }

  dispose() {
    for (const id of [...this.entries.keys()]) this._despawn(id);
  }
}

const _q = new THREE.Quaternion();
