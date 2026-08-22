/**
 * input.js
 * ---------------------------------------------------------------------------
 * Keyboard (and optional gamepad) input.
 *
 * This module produces nothing but four normalised numbers - throttle, brake,
 * steer, handbrake - plus edge-triggered actions. It NEVER touches the car's
 * transform. Everything downstream of here is force and torque
 * (requirement #51).
 *
 * Analogue smoothing deliberately does not live here: the throttle plate is
 * smoothed by the engine, the pedal by the brake hydraulics, and the wheel by
 * the steering rack, each with its own physical time constant. Filtering the
 * raw key here as well would just add a second, fictitious lag.
 * ---------------------------------------------------------------------------
 */

export const ACTIONS = {
  RESET: 'reset',
  PAUSE: 'pause',
  CAMERA: 'camera',
  DEBUG: 'debug',
  TOGGLE_ABS: 'toggleAbs',
  TOGGLE_TC: 'toggleTc',
  GEAR_UP: 'gearUp',
  GEAR_DOWN: 'gearDown',
  PRESSURE_UP: 'pressureUp',
  PRESSURE_DOWN: 'pressureDown',
  HELP: 'help',
  GARAGE: 'garage',
  MUTE: 'mute',
  REFLECTIONS: 'reflections',
};

/** code -> action, for the one-shot keys. */
const ACTION_KEYS = {
  KeyR: ACTIONS.RESET,
  Escape: ACTIONS.PAUSE,
  KeyC: ACTIONS.CAMERA,
  KeyH: ACTIONS.DEBUG,
  F1: ACTIONS.DEBUG,
  KeyB: ACTIONS.TOGGLE_ABS,
  KeyT: ACTIONS.TOGGLE_TC,
  KeyE: ACTIONS.GEAR_UP,
  KeyQ: ACTIONS.GEAR_DOWN,
  BracketRight: ACTIONS.PRESSURE_UP,
  BracketLeft: ACTIONS.PRESSURE_DOWN,
  Slash: ACTIONS.HELP,
  KeyG: ACTIONS.GARAGE,
  KeyM: ACTIONS.MUTE,
  KeyV: ACTIONS.REFLECTIONS,
};

export class InputSystem {
  constructor(target = window) {
    this.target = target;
    this.keys = new Set();

    /** Normalised driver demand, all 0..1 except steer which is -1..+1. */
    this.throttle = 0;
    this.brake = 0;
    /** POSITIVE = steering RIGHT, matching the physics convention. */
    this.steer = 0;
    this.handbrake = 0;

    /** True while any analogue source is providing the value. */
    this.usingGamepad = false;
    this.gamepadIndex = null;

    this._listeners = new Map();
    this._onKeyDown = this._onKeyDown.bind(this);
    this._onKeyUp = this._onKeyUp.bind(this);
    this._onBlur = this._onBlur.bind(this);
    this.enabled = true;

    target.addEventListener('keydown', this._onKeyDown, { passive: false });
    target.addEventListener('keyup', this._onKeyUp);
    target.addEventListener('blur', this._onBlur);
    target.addEventListener('gamepadconnected', (e) => {
      this.gamepadIndex = e.gamepad.index;
    });
    target.addEventListener('gamepaddisconnected', () => {
      this.gamepadIndex = null;
      this.usingGamepad = false;
    });
  }

  /** Subscribe to a one-shot action. Returns an unsubscribe function. */
  on(action, fn) {
    if (!this._listeners.has(action)) this._listeners.set(action, new Set());
    this._listeners.get(action).add(fn);
    return () => this._listeners.get(action).delete(fn);
  }

  _emit(action, payload) {
    const set = this._listeners.get(action);
    if (set) for (const fn of set) fn(payload);
  }

  _onKeyDown(e) {
    if (e.repeat) return;
    // Do not steal keys from a focused text field.
    const el = document.activeElement;
    if (el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA')) return;

    this.keys.add(e.code);

    const action = ACTION_KEYS[e.code];
    if (action) {
      // Escape must still work while paused, so it is exempt from `enabled`.
      if (this.enabled || action === ACTIONS.PAUSE) this._emit(action);
      e.preventDefault();
    } else if (DRIVING_KEYS.has(e.code)) {
      e.preventDefault();      // stop Space and the arrows scrolling the page
    }
  }

  _onKeyUp(e) { this.keys.delete(e.code); }

  /** Releasing everything on blur stops the car driving off while alt-tabbed. */
  _onBlur() {
    this.keys.clear();
    this.throttle = 0;
    this.brake = 0;
    this.steer = 0;
    this.handbrake = 0;
  }

  down(code) { return this.keys.has(code); }

  /** Sample the current state. Call once per rendered frame. */
  update() {
    if (!this.enabled) {
      this.throttle = 0; this.brake = 0; this.steer = 0; this.handbrake = 0;
      return this;
    }

    const up = this.down('KeyW') || this.down('ArrowUp');
    const dn = this.down('KeyS') || this.down('ArrowDown');
    const lf = this.down('KeyA') || this.down('ArrowLeft');
    const rt = this.down('KeyD') || this.down('ArrowRight');

    this.throttle = up ? 1 : 0;
    this.brake = dn ? 1 : 0;
    this.steer = (rt ? 1 : 0) - (lf ? 1 : 0);
    this.handbrake = this.down('Space') ? 1 : 0;

    this._pollGamepad();
    return this;
  }

  /**
   * An attached gamepad overrides the keyboard whenever it is actually being
   * used, which gives genuinely analogue throttle, brake and steering.
   */
  _pollGamepad() {
    if (this.gamepadIndex === null || !navigator.getGamepads) return;
    const pad = navigator.getGamepads()[this.gamepadIndex];
    if (!pad) return;

    const dead = (v, d = 0.08) => (Math.abs(v) < d ? 0 : (v - Math.sign(v) * d) / (1 - d));
    const steer = dead(pad.axes[0] ?? 0);
    const rt = pad.buttons[7]?.value ?? 0;
    const lt = pad.buttons[6]?.value ?? 0;
    const hb = pad.buttons[0]?.pressed ? 1 : 0;

    const active = Math.abs(steer) > 0.01 || rt > 0.01 || lt > 0.01 || hb > 0;
    this.usingGamepad = active;
    if (!active) return;

    this.steer = steer;
    this.throttle = rt;
    this.brake = lt;
    this.handbrake = hb;
  }

  dispose() {
    this.target.removeEventListener('keydown', this._onKeyDown);
    this.target.removeEventListener('keyup', this._onKeyUp);
    this.target.removeEventListener('blur', this._onBlur);
    this._listeners.clear();
  }
}

const DRIVING_KEYS = new Set([
  'KeyW', 'KeyA', 'KeyS', 'KeyD', 'Space',
  'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight',
]);

/** Human-readable control list, shared by the HUD and the pause screen. */
export const CONTROL_HELP = [
  ['W / Up', 'Throttle'],
  ['S / Down', 'Brake  (hold at a standstill for reverse)'],
  ['A / D', 'Steer left / right'],
  ['Space', 'Handbrake'],
  ['C', 'Change camera (chase, side, bonnet, interior, orbit)'],
  ['R', 'Reset the car'],
  ['Esc', 'Pause'],
  ['H / F1', 'Debug overlay'],
  ['B / T', 'Toggle ABS / traction control'],
  ['Q / E', 'Manual downshift / upshift'],
  ['[ / ]', 'Tyre pressure down / up'],
  ['G', 'Open the garage and change car'],
  ['M', 'Mute / unmute'],
  ['V', 'Reflection quality: off / low / high'],
];
