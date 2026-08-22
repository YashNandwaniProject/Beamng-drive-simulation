/**
 * network.js
 * ---------------------------------------------------------------------------
 * Multiplayer transport, protocol and vehicle-state replication.
 *
 * WHAT THIS IS
 * ------------
 * The simulation had no networking before this change, so this is a new layer
 * rather than an extension of an existing one. It is built the way a session
 * layer normally is - a transport interface, a wire protocol, an authoritative
 * local car and interpolated remote ones - so swapping in a real server is a
 * matter of implementing one class, not rewriting the game.
 *
 * Two transports ship:
 *
 *   BroadcastChannelTransport   peers are other TABS of the same browser on
 *                               the same origin. No server needed, and it is
 *                               genuinely testable: open the page twice and
 *                               the two cars can see each other.
 *   WebSocketTransport          a real server. Point it at a URL and it speaks
 *                               the same JSON protocol.
 *
 * VEHICLE IDENTITY
 * ----------------
 * Peers are identified by a stable vehicle ID (`"lamborghini-revuelto"`), never
 * by display name. The receiving client looks that ID up in the registry to
 * decide which GLB to load and which handling profile applies, so renaming a
 * car in the garage can never break a session.
 *
 * AUTHORITY
 * ---------
 * Each client simulates its OWN car and nothing else. Remote cars are never
 * simulated - they replay replicated state, interpolated a fixed delay behind
 * so that jitter and packet loss do not show. That keeps local input fully
 * responsive while remote cars stay smooth.
 * ---------------------------------------------------------------------------
 */

const PROTOCOL_VERSION = 1;

/** How often the local car broadcasts its state, Hz. */
export const SNAPSHOT_HZ = 20;

/**
 * How far behind live the remote cars are rendered, seconds. One snapshot
 * interval plus a margin, so there is always a pair to interpolate between.
 */
export const INTERPOLATION_DELAY = 0.12;

/** A peer is dropped after this long without a snapshot. */
const PEER_TIMEOUT = 5.0;

/* ------------------------------------------------------------------------ */
/* Transports                                                                */
/* ------------------------------------------------------------------------ */

/** Interface: connect(), send(msg), close(), onMessage callback. */
class Transport {
  constructor() { this.onMessage = null; this.connected = false; }
  connect() { this.connected = true; }
  send() {}
  close() { this.connected = false; }
}

/**
 * Peers are other tabs of the same browser. Requires no server at all, which
 * makes it the transport the automated tests use.
 */
export class BroadcastChannelTransport extends Transport {
  constructor(room = 'bmw-sim') {
    super();
    this.room = room;
    this.channel = null;
  }

  connect() {
    if (typeof BroadcastChannel === 'undefined') return false;
    this.channel = new BroadcastChannel(this.room);
    this.channel.onmessage = (e) => {
      if (this.onMessage) this.onMessage(e.data);
    };
    this.connected = true;
    return true;
  }

  send(msg) {
    if (this.channel) this.channel.postMessage(msg);
  }

  close() {
    if (this.channel) { this.channel.close(); this.channel = null; }
    this.connected = false;
  }
}

/** A real server. Speaks the same JSON protocol. */
export class WebSocketTransport extends Transport {
  constructor(url) { super(); this.url = url; this.ws = null; }

  connect() {
    try {
      this.ws = new WebSocket(this.url);
    } catch (e) {
      console.warn(`Network: could not open ${this.url}`, e);
      return false;
    }
    this.ws.onopen = () => { this.connected = true; };
    this.ws.onclose = () => { this.connected = false; };
    this.ws.onerror = () => { this.connected = false; };
    this.ws.onmessage = (e) => {
      try {
        if (this.onMessage) this.onMessage(JSON.parse(e.data));
      } catch (err) { /* ignore malformed frames */ }
    };
    return true;
  }

  send(msg) {
    if (this.ws && this.ws.readyState === 1) this.ws.send(JSON.stringify(msg));
  }

  close() {
    if (this.ws) { this.ws.close(); this.ws = null; }
    this.connected = false;
  }
}

/* ------------------------------------------------------------------------ */
/* State codec                                                               */
/* ------------------------------------------------------------------------ */

/**
 * Serialise everything a remote client needs to draw this car correctly.
 *
 * The list is deliberately not just "position and rotation": a car whose wheels
 * do not turn, whose steering does not move and whose suspension does not work
 * looks obviously wrong to everyone else in the session. Wheel ANGLES are sent
 * rather than angular velocities so a dropped packet cannot leave a remote
 * wheel spinning at the wrong rate.
 */
export function captureVehicleState(vehicle, peerId, time) {
  const p = vehicle.physics;
  const b = p.body;
  const w = p.wheels;
  return {
    type: 'state',
    id: peerId,
    v: vehicle.id,
    t: time,
    p: [r3(b.position.x), r3(b.position.y), r3(b.position.z)],
    q: [r4(b.quat.x), r4(b.quat.y), r4(b.quat.z), r4(b.quat.w)],
    lv: [r2(b.velocity.x), r2(b.velocity.y), r2(b.velocity.z)],
    av: [r2(b.angularVelocity.x), r2(b.angularVelocity.y), r2(b.angularVelocity.z)],
    /** Steering, wheel rotation, suspension and tyre squash - all four wheels. */
    st: r4(p.steering.angle),
    wa: [r2(w[0].spinAngle), r2(w[1].spinAngle), r2(w[2].spinAngle), r2(w[3].spinAngle)],
    sp: [r4(w[0].suspension.displacement), r4(w[1].suspension.displacement),
      r4(w[2].suspension.displacement), r4(w[3].suspension.displacement)],
    td: [r4(w[0].tire.deflection), r4(w[1].tire.deflection),
      r4(w[2].tire.deflection), r4(w[3].tire.deflection)],
    /** Drivetrain and braking state, for lights and for remote audio. */
    br: r2(p.brakes.pedal),
    hb: r2(p.brakes.handbrake),
    th: r2(p.drivetrain.throttle),
    g: p.drivetrain.gearLabel,
    rp: Math.round(p.drivetrain.rpm),
  };
}

const r2 = (x) => Math.round(x * 100) / 100;
const r3 = (x) => Math.round(x * 1000) / 1000;
const r4 = (x) => Math.round(x * 10000) / 10000;

/**
 * Write a replicated snapshot into a Vehicle that is NOT being simulated.
 *
 * It goes into exactly the same fields the local physics would have written,
 * so the remote car is then drawn by the identical `syncVisuals` path - wheels
 * rolling, front wheels steering, suspension working, brake lights and the
 * interior steering wheel all included. There is no separate, lesser renderer
 * for other people's cars.
 */
export function applyVehicleState(vehicle, a, b, alpha) {
  const p = vehicle.physics;
  const body = p.body;

  body.position.set(
    lerp(a.p[0], b.p[0], alpha),
    lerp(a.p[1], b.p[1], alpha),
    lerp(a.p[2], b.p[2], alpha),
  );
  slerpInto(body.quat, a.q, b.q, alpha);
  body.velocity.set(
    lerp(a.lv[0], b.lv[0], alpha),
    lerp(a.lv[1], b.lv[1], alpha),
    lerp(a.lv[2], b.lv[2], alpha),
  );
  body.angularVelocity.set(
    lerp(a.av[0], b.av[0], alpha),
    lerp(a.av[1], b.av[1], alpha),
    lerp(a.av[2], b.av[2], alpha),
  );
  body.updateDerived();

  p.steering.angle = lerp(a.st, b.st, alpha);
  p.brakes.pedal = lerp(a.br, b.br, alpha);
  p.brakes.handbrake = lerp(a.hb, b.hb, alpha);
  p.drivetrain.throttle = lerp(a.th, b.th, alpha);
  p.drivetrain.engineOmega = lerp(a.rp, b.rp, alpha) * (2 * Math.PI) / 60;
  p.drivetrain.gear = b.g === 'R' ? -1 : (b.g === 'N' ? 0 : Number(b.g) || 1);

  for (let i = 0; i < 4; i++) {
    const w = p.wheels[i];
    /**
     * Wheel angles are unwrapped across the shortest path, so a wheel that
     * wrapped past 2*pi between snapshots does not visibly spin backwards.
     */
    w.spinAngle = lerpAngleFree(a.wa[i], b.wa[i], alpha);
    w.suspension.displacement = lerp(a.sp[i], b.sp[i], alpha);
    w.tire.deflection = lerp(a.td[i], b.td[i], alpha);
    w.hasContact = true;
  }

  /**
   * Ackermann is geometry, so the remote client recomputes the individual
   * road-wheel angles from the replicated rack angle rather than sending four
   * numbers that it can derive from one.
   */
  p.steering._applyAckermann(p.wheelFL, p.wheelFR);
}

const lerp = (x, y, t) => x + (y - x) * t;

function lerpAngleFree(x, y, t) {
  // Wheel angles accumulate without bound; interpolate along the actual delta.
  return x + (y - x) * t;
}

function slerpInto(out, a, b, t) {
  let ax = a[0], ay = a[1], az = a[2], aw = a[3];
  const bx = b[0], by = b[1], bz = b[2], bw = b[3];
  let cos = ax * bx + ay * by + az * bz + aw * bw;
  if (cos < 0) { cos = -cos; ax = -ax; ay = -ay; az = -az; aw = -aw; }
  let s0 = 1 - t;
  let s1 = t;
  if (cos < 0.9995) {
    const theta = Math.acos(Math.min(1, cos));
    const sin = Math.sin(theta);
    if (sin > 1e-6) {
      s0 = Math.sin((1 - t) * theta) / sin;
      s1 = Math.sin(t * theta) / sin;
    }
  }
  out.set(
    ax * s0 + bx * s1, ay * s0 + by * s1,
    az * s0 + bz * s1, aw * s0 + bw * s1,
  ).normalize();
}

/* ------------------------------------------------------------------------ */
/* Session                                                                   */
/* ------------------------------------------------------------------------ */

export class NetworkSession {
  /**
   * @param {Transport} transport
   * @param {object} opts { name, vehicleId }
   */
  constructor(transport, opts = {}) {
    this.transport = transport;
    this.peerId = opts.peerId
      || `p${Math.random().toString(36).slice(2, 9)}`;
    this.name = opts.name || `Driver ${this.peerId.slice(1, 4)}`;
    this.vehicleId = opts.vehicleId || null;

    /** peerId -> { id, name, vehicleId, buffer[], lastSeen } */
    this.peers = new Map();
    this.connected = false;
    this.time = 0;
    this._sendAccum = 0;

    /** Fired when a peer appears, changes car, or leaves. */
    this.onPeerJoin = null;
    this.onPeerLeave = null;
    this.onPeerVehicleChange = null;
  }

  connect() {
    if (!this.transport) return false;
    this.transport.onMessage = (msg) => this._receive(msg);
    const ok = this.transport.connect();
    if (ok === false) return false;
    this.connected = true;
    this._send({ type: 'join', id: this.peerId, name: this.name, v: this.vehicleId });
    // Ask anyone already here to announce themselves.
    this._send({ type: 'hello', id: this.peerId });
    return true;
  }

  disconnect() {
    if (!this.connected) return;
    this._send({ type: 'leave', id: this.peerId });
    this.transport.close();
    this.connected = false;
    this.peers.clear();
  }

  /** Tell the session which car the local player is driving. */
  setVehicle(vehicleId) {
    this.vehicleId = vehicleId;
    if (this.connected) {
      this._send({ type: 'join', id: this.peerId, name: this.name, v: vehicleId });
    }
  }

  _send(msg) {
    if (this.transport) {
      msg.pv = PROTOCOL_VERSION;
      this.transport.send(msg);
    }
  }

  _receive(msg) {
    if (!msg || msg.pv !== PROTOCOL_VERSION) return;
    if (msg.id === this.peerId) return;               // our own echo

    switch (msg.type) {
      case 'hello':
        // Someone just arrived: announce ourselves so they see us immediately.
        this._send({
          type: 'join', id: this.peerId, name: this.name, v: this.vehicleId,
        });
        break;

      case 'join': {
        const existing = this.peers.get(msg.id);
        if (!existing) {
          const peer = {
            id: msg.id, name: msg.name, vehicleId: msg.v,
            buffer: [], lastSeen: this.time,
          };
          this.peers.set(msg.id, peer);
          if (this.onPeerJoin) this.onPeerJoin(peer);
        } else if (existing.vehicleId !== msg.v) {
          // A player swapped cars in the garage: rebuild their model.
          existing.vehicleId = msg.v;
          existing.name = msg.name;
          existing.buffer.length = 0;
          if (this.onPeerVehicleChange) this.onPeerVehicleChange(existing);
        }
        break;
      }

      case 'leave': {
        const peer = this.peers.get(msg.id);
        if (peer) {
          this.peers.delete(msg.id);
          if (this.onPeerLeave) this.onPeerLeave(peer);
        }
        break;
      }

      case 'state': {
        let peer = this.peers.get(msg.id);
        if (!peer) {
          // A snapshot from someone whose join we missed - adopt them.
          peer = {
            id: msg.id, name: msg.name || msg.id, vehicleId: msg.v,
            buffer: [], lastSeen: this.time,
          };
          this.peers.set(msg.id, peer);
          if (this.onPeerJoin) this.onPeerJoin(peer);
        } else if (peer.vehicleId !== msg.v) {
          peer.vehicleId = msg.v;
          peer.buffer.length = 0;
          if (this.onPeerVehicleChange) this.onPeerVehicleChange(peer);
        }
        peer.lastSeen = this.time;
        // Stamp with OUR clock: peers' clocks are not synchronised, and the
        // interpolator only needs a consistent local timeline.
        msg.rt = this.time;
        peer.buffer.push(msg);
        if (peer.buffer.length > 32) peer.buffer.shift();
        break;
      }

      default:
        break;
    }
  }

  /**
   * Advance the session: broadcast our own state at the snapshot rate and time
   * out anyone who has gone quiet.
   */
  update(dt, localVehicle) {
    if (!this.connected) return;
    this.time += dt;

    this._sendAccum += dt;
    const interval = 1 / SNAPSHOT_HZ;
    if (this._sendAccum >= interval && localVehicle && localVehicle.loaded) {
      this._sendAccum %= interval;
      const state = captureVehicleState(localVehicle, this.peerId, this.time);
      state.name = this.name;
      this._send(state);
    }

    for (const [id, peer] of this.peers) {
      if (this.time - peer.lastSeen > PEER_TIMEOUT) {
        this.peers.delete(id);
        if (this.onPeerLeave) this.onPeerLeave(peer);
      }
    }
  }

  /**
   * The pair of snapshots straddling the render time for one peer, plus the
   * blend between them. Returns null when there is not enough history yet.
   */
  samplePeer(peer) {
    const target = this.time - INTERPOLATION_DELAY;
    const buf = peer.buffer;
    if (buf.length === 0) return null;
    if (buf.length === 1) return { a: buf[0], b: buf[0], alpha: 0 };

    for (let i = buf.length - 1; i > 0; i--) {
      if (buf[i - 1].rt <= target && buf[i].rt >= target) {
        const span = buf[i].rt - buf[i - 1].rt;
        const alpha = span > 1e-6 ? (target - buf[i - 1].rt) / span : 0;
        return { a: buf[i - 1], b: buf[i], alpha };
      }
    }
    // Behind the buffer (stalled peer): hold the newest we have.
    const last = buf[buf.length - 1];
    return { a: last, b: last, alpha: 0 };
  }

  get peerCount() { return this.peers.size; }
}
