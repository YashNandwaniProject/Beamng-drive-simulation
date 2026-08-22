/**
 * main.js
 * ---------------------------------------------------------------------------
 * Bootstrap and the main loop.
 *
 * The loop runs a FIXED physics timestep behind an accumulator, decoupled from
 * the render rate. The renderer is free to run at 60, 120 or 144 Hz; the
 * simulation always advances in identical 1/240 s steps, and whatever time is
 * left over becomes the interpolation factor for drawing.
 *
 * Session flow:
 *
 *     Garage  ->  pick a car  ->  load its GLB  ->  spawn  ->  drive
 *       ^                                                       |
 *       +---------------------- G -----------------------------+
 *
 * Only ONE car is simulated locally, whichever the player is driving. Other
 * players' cars are replayed from replicated state by js/multiplayer.js.
 * ---------------------------------------------------------------------------
 */

import * as THREE from 'three';
import { gsap } from 'gsap';

import { SIM, SPAWN, RENDER } from './config.js';
import { Terrain } from './terrain.js';
import { CollisionSystem } from './collision.js';
import { Environment } from './environment.js';
import { Vehicle } from './vehicle.js';
import { CameraSystem } from './camera.js';
import { InputSystem, ACTIONS } from './input.js';
import { Hud, LoadingScreen, PauseScreen } from './hud.js';
import { DebugSystem } from './debug.js';
import { Garage } from './garage.js';
import { EngineAudio } from './audio.js';
import { ReflectionProbe } from './reflections.js';
import { NetworkSession, BroadcastChannelTransport } from './network.js';
import { RemotePlayers } from './multiplayer.js';
import { getVehicleSpec, DEFAULT_VEHICLE_ID } from './vehicles/registry.js';
import { clamp } from './mathUtils.js';

class Simulation {
  constructor() {
    this.canvas = document.getElementById('viewport');
    this.loading = new LoadingScreen(document.getElementById('loading'));
    this.paused = false;
    this.running = false;
    this.swapping = false;

    this.accumulator = 0;
    /** Where the reflection probe captures from, reused every frame. */
    this._probeAt = new THREE.Vector3();
    // Total SIMULATED seconds stepped. Not wall clock: on a machine that
    // cannot keep up, the substep cap makes these two diverge.
    this.simSeconds = 0;
    this.lastTime = 0;
    this.substeps = 0;
    this.physicsMs = 0;

    this.vehicle = null;
    this.vehicleId = null;

    this._input = { throttle: 0, brake: 0, steer: 0, handbrake: 0 };
    this._loop = this._loop.bind(this);
  }

  /* ===================================================================== */

  async init() {
    try {
      this._createRenderer();
      this.loading.setProgress(0.05, 'Starting renderer…');
      await frame();

      /* --- world --------------------------------------------------------- */
      this.loading.setProgress(0.15, 'Generating circuit…');
      await frame();
      this.terrain = new Terrain();
      this.collision = new CollisionSystem(this.terrain);

      this.loading.setProgress(0.30, 'Building environment…');
      await frame();
      this.environment = new Environment(this.renderer, this.terrain, this.collision);
      this.scene = this.environment.scene;

      /**
       * Real-time reflections. The probe hands its cube map to anything that
       * registers for it and re-issues it whenever the quality changes, so the
       * road and whichever car is loaded stay in step without either of them
       * knowing the probe exists.
       */
      /**
       * `?reflections=off|low|high` overrides the configured quality. Same
       * reasoning as `?audio=off`: a headless or low-powered machine should be
       * able to turn six extra scene renders off without editing a file.
       */
      const q = new URLSearchParams(window.location.search).get('reflections');
      const level = { off: 0, low: 1, high: 2 }[String(q).toLowerCase()];
      this.reflections = new ReflectionProbe(this.renderer, this.scene, {
        ...RENDER.reflections,
        ...(level === undefined ? {} : { level }),
      });
      this.reflections.register((tex) => this.environment.applyReflectionProbe(tex));
      this.reflections.register((tex) => {
        if (this.vehicle && this.vehicle.loaded) this.vehicle.applyReflectionProbe(tex);
      });

      /* --- presentation shell -------------------------------------------- */
      this.cameraSystem = new CameraSystem(
        window.innerWidth / window.innerHeight, this.terrain,
      );
      this.pauseScreen = new PauseScreen(document.getElementById('pause'));
      this.debug = new DebugSystem(document.getElementById('debug'), this.scene);
      this.audio = new EngineAudio();
      /**
       * Headless and CI environments have no audio device, and an
       * AudioContext there logs an error that has nothing to do with this
       * code. `?audio=off` turns the synth off entirely; it is also handy for
       * anyone who just wants the simulation silent from the start.
       */
      const params = new URLSearchParams(window.location.search);
      if (params.get('audio') === 'off' || window.__DISABLE_AUDIO) {
        this.audio.enabled = false;
      }

      /* --- multiplayer session -------------------------------------------
       * BroadcastChannel means two TABS of this page are two players, with no
       * server involved. Swap in WebSocketTransport to point at a real one.
       */
      this.network = new NetworkSession(new BroadcastChannelTransport('bmw-sim'), {
        name: `Driver ${Math.floor(Math.random() * 900 + 100)}`,
      });
      this.remotePlayers = new RemotePlayers(this.scene, this.terrain, this.network);

      /* --- input ---------------------------------------------------------- */
      this.input = new InputSystem(window);
      this._bindActions();
      this._bindGarageKeys();

      window.addEventListener('resize', () => this._resize());
      document.addEventListener('visibilitychange', () => {
        if (document.hidden) this.accumulator = 0;
        this.lastTime = 0;
      });
      this._resize();

      /* --- garage ---------------------------------------------------------- */
      this.garage = new Garage(
        document.getElementById('garage'), (id) => this._selectVehicle(id),
      );

      this.loading.setProgress(1, 'Ready');
      await frame();
      await this.loading.hide();

      this.running = true;
      requestAnimationFrame(this._loop);

      this.garage.show(DEFAULT_VEHICLE_ID);
      window.sim = this;
    } catch (err) {
      console.error(err);
      this.loading.fail(String(err && err.message ? err.message : err));
    }
  }

  /* --------------------------------------------------------------- setup */

  _createRenderer() {
    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
      powerPreference: 'high-performance',
      stencil: false,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, RENDER.maxPixelRatio));
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = RENDER.toneMappingExposure;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
  }

  /* ===================================================================== */
  /* Vehicle lifecycle                                                      */
  /* ===================================================================== */

  /** Called by the garage when the player commits to a car. */
  async _selectVehicle(id) {
    if (this.swapping) return;
    this.swapping = true;
    const spec = getVehicleSpec(id);

    // Keep the spawn point stable across a swap so the two cars can be
    // compared back to back from the same place on the circuit.
    const spawn = this.vehicle
      ? { position: this.vehicle.physics.spawnGround, heading: this.vehicle.physics.spawnHeading }
      : this.terrain.spawnTransform(SPAWN.trackDistance, SPAWN.lateralOffset);

    const banner = document.getElementById('swap');
    banner.textContent = `Loading ${spec.displayName}…`;
    banner.style.display = '';
    gsap.fromTo(banner, { opacity: 0 }, { opacity: 1, duration: 0.2 });
    await frame();

    try {
      if (this.vehicle) {
        this.scene.remove(this.vehicle.object3D);
        this.vehicle.dispose();
        this.vehicle = null;
      }

      const vehicle = new Vehicle(this.terrain, spec, this.collision);
      await vehicle.load((f) => {
        banner.textContent = `Loading ${spec.displayName}… ${Math.round(f * 100)} %`;
      });
      this.scene.add(vehicle.object3D);
      vehicle.spawn(spawn.position, spawn.heading);

      this.vehicle = vehicle;
      this.vehicleId = spec.id;

      /* --- rewire everything that is vehicle-specific -------------------- */
      // Hand the new car the probe. Cars opt in per material, so one that
      // does not ask for reflections simply keeps the sky IBL.
      vehicle.applyReflectionProbe(this.reflections.texture);
      this.cameraSystem.setVehicleProfile(spec.cameraProfile);
      this.cameraSystem.snapTo(vehicle.physics);

      if (!this.hud) {
        this.hud = new Hud(document.getElementById('hud'), spec);
      } else {
        this.hud.setVehicle(spec);
      }
      this.hud.setCameraName(this.cameraSystem.modeName);

      this.audio.setVehicle(spec);
      this.network.setVehicle(spec.id);
      if (!this.network.connected) this.network.connect();

      this._bindVehicleActions();

      this.renderer.compile(this.scene, this.cameraSystem.camera);
      this.hud.toast(`${spec.displayName} — ${spec.garage.tagline}`);
    } catch (err) {
      console.error(err);
      banner.textContent = String(err && err.message ? err.message : err);
      banner.classList.add('error');
      this.swapping = false;
      return;
    }

    gsap.to(banner, {
      opacity: 0,
      duration: 0.3,
      onComplete: () => { banner.style.display = 'none'; },
    });
    this.lastTime = 0;
    this.swapping = false;
  }

  /* ------------------------------------------------------------- controls */

  _bindActions() {
    // Audio needs a user gesture before it is allowed to make a sound.
    const wake = () => this.audio.resume();
    window.addEventListener('keydown', wake, { once: true });
    window.addEventListener('pointerdown', wake, { once: true });

    this.input.on(ACTIONS.CAMERA, () => {
      this.hud.setCameraName(this.cameraSystem.cycleMode());
    });
    this.input.on(ACTIONS.PAUSE, () => {
      if (this.garage.visible) return;
      this._togglePause();
    });
    this.input.on(ACTIONS.DEBUG, () => {
      const on = this.debug.toggle();
      this.hud.toast(on ? 'Telemetry on' : 'Telemetry off');
    });
    this.input.on(ACTIONS.GARAGE, () => this._openGarage());
    this.input.on(ACTIONS.MUTE, () => {
      const on = this.audio.toggle();
      this.hud.toast(on ? 'Sound on' : 'Sound off');
    });
    this.input.on(ACTIONS.REFLECTIONS, () => {
      this.hud.toast(`Reflections: ${this.reflections.cycle()}`);
    });
  }

  /** Actions that need the current vehicle; rebound whenever the car changes. */
  _bindVehicleActions() {
    if (this._unbind) for (const fn of this._unbind) fn();
    this._unbind = [];
    const p = this.vehicle.physics;
    const spec = this.vehicle.spec;

    this._unbind.push(this.input.on(ACTIONS.RESET, () => {
      this.vehicle.respawn();
      this.cameraSystem.snapTo(p);
      p.drivetrain.autoShift = spec.transmission.autoShift;
      this.hud.toast('Vehicle reset');
    }));

    this._unbind.push(this.input.on(ACTIONS.TOGGLE_ABS, () => {
      p.brakes.absEnabled = !p.brakes.absEnabled;
      this.hud.toast(`ABS ${p.brakes.absEnabled ? 'on' : 'off'}`);
    }));

    this._unbind.push(this.input.on(ACTIONS.TOGGLE_TC, () => {
      p.brakes.tcEnabled = !p.brakes.tcEnabled;
      this.hud.toast(`Traction control ${p.brakes.tcEnabled ? 'on' : 'off'}`);
    }));

    const manualShift = (delta) => {
      const dt = p.drivetrain;
      if (dt.autoShift) {
        dt.autoShift = false;
        this.hud.toast('Manual gearbox  (R resets to auto)');
      }
      const max = spec.transmission.gearRatios.length;
      if (dt.gear >= 1) dt.requestGear(clamp(dt.gear + delta, 1, max));
    };
    this._unbind.push(this.input.on(ACTIONS.GEAR_UP, () => manualShift(1)));
    this._unbind.push(this.input.on(ACTIONS.GEAR_DOWN, () => manualShift(-1)));

    const adjustPressure = (delta) => {
      const psi = clamp(
        p.wheelFL.tire.pressure + delta, spec.tyre.minPressure, spec.tyre.maxPressure,
      );
      p.setTirePressures(psi);
      this.hud.toast(`Tyre pressure ${psi.toFixed(1)} psi`);
    };
    this._unbind.push(this.input.on(ACTIONS.PRESSURE_UP, () => adjustPressure(2)));
    this._unbind.push(this.input.on(ACTIONS.PRESSURE_DOWN, () => adjustPressure(-2)));
    this._unbind.push(this.input.on(ACTIONS.HELP, () => this._togglePause()));
  }

  /** Raw key handling while the garage is open. */
  _bindGarageKeys() {
    window.addEventListener('keydown', (e) => {
      if (!this.garage || !this.garage.visible) return;
      if (this.garage.handleKey(e.code)) e.preventDefault();
      else if (e.code === 'Escape' && this.vehicle) {
        // Cancel only if there is already a car to go back to.
        this.garage.hide();
        this.input.enabled = true;
      }
    });
  }

  _openGarage() {
    if (this.garage.visible || this.swapping) return;
    if (this.paused) this._togglePause();
    this.input.enabled = false;
    this.garage.show(this.vehicleId);
  }

  _togglePause() {
    this.paused = this.pauseScreen.toggle();
    this.input.enabled = !this.paused;
    if (!this.paused) this.lastTime = 0;
  }

  _resize() {
    const w = window.innerWidth;
    const h = window.innerHeight;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, RENDER.maxPixelRatio));
    this.renderer.setSize(w, h);
    this.cameraSystem.resize(w / h);
  }

  /* ===================================================================== */
  /* Main loop                                                              */
  /* ===================================================================== */

  _loop(now) {
    requestAnimationFrame(this._loop);
    if (!this.running) return;

    const time = now * 0.001;
    if (!this.lastTime) this.lastTime = time;
    const frameTime = Math.min(time - this.lastTime, SIM.maxFrameTime);
    this.lastTime = time;

    /* --- no car yet (garage still open on first run) --------------------- */
    if (!this.vehicle) {
      this.renderer.render(this.scene, this.cameraSystem.camera);
      return;
    }

    /* --- 1. input -------------------------------------------------------- */
    const driving = !this.paused && !this.garage.visible && !this.swapping;
    this.input.enabled = driving;
    this.input.update();
    this._input.throttle = this.input.throttle;
    this._input.brake = this.input.brake;
    this._input.steer = this.input.steer;
    this._input.handbrake = this.input.handbrake;

    /* --- 2. fixed-step physics ------------------------------------------- */
    const step = 1 / SIM.physicsHz;
    this.substeps = 0;
    if (driving) {
      const t0 = performance.now();
      this.accumulator += frameTime;
      while (this.accumulator >= step && this.substeps < SIM.maxSubSteps) {
        this.vehicle.step(step, this._input);
        this.accumulator -= step;
        this.simSeconds += step;
        this.substeps++;
      }
      if (this.substeps >= SIM.maxSubSteps) this.accumulator = 0;
      this.physicsMs = performance.now() - t0;
    }

    /* --- 3. graphics follow the physics (never the reverse) --------------- */
    const alpha = driving ? this.accumulator / step : 1;
    this.vehicle.syncVisuals(alpha, frameTime);

    const p = this.vehicle.physics;
    this.cameraSystem.update(frameTime, p);
    this.environment.updateSun(
      this.vehicle.object3D.position.x,
      this.vehicle.object3D.position.y,
      this.vehicle.object3D.position.z,
    );

    /* --- 4. multiplayer --------------------------------------------------- */
    this.network.update(frameTime, this.vehicle);
    this.remotePlayers.update(frameTime, this.cameraSystem.camera);

    /* --- 5. audio and overlays -------------------------------------------- */
    this.audio.update(frameTime, p);
    this.hud.update(p, this._input, frameTime);
    this.hud.setPlayerCount(this.remotePlayers.count);
    this.debug.update(p, frameTime, {
      substeps: this.substeps,
      physicsMs: this.physicsMs,
      drawCalls: this.renderer.info.render.calls,
      triangles: this.renderer.info.render.triangles,
      steeringWheelDeg: this.vehicle.steeringWheelPivot
        ? this.vehicle.steeringWheelDegrees : undefined,
      network: this.network.connected ? 'connected' : 'offline',
      remotePlayers: this.remotePlayers.count,
      reflections: this.reflections.enabled
        ? `${this.reflections.level.name} ${this.reflections.level.size}px @ ${this.reflections.level.hz} Hz`
        : 'off',
      reflectionMs: this.reflections.lastCaptureMs,
    });

    /* --- 6. reflection probe ---------------------------------------------- */
    /**
     * Before the main draw, and from just above the car. The local car is
     * excluded: a probe sitting inside it would capture the inside of its own
     * shell and smear that over its paint.
     */
    this._probeAt.set(
      this.vehicle.object3D.position.x,
      this.vehicle.object3D.position.y + (RENDER.reflections.height || 0.85),
      this.vehicle.object3D.position.z,
    );
    this.reflections.update(frameTime, this._probeAt, [this.vehicle.object3D]);

    /* --- 7. draw ---------------------------------------------------------- */
    this.renderer.render(this.scene, this.cameraSystem.camera);
  }
}

/** Wait for one painted frame, so the loading bar actually animates. */
function frame() {
  return new Promise((resolve) => {
    requestAnimationFrame(() => setTimeout(resolve, 0));
  });
}

/* ------------------------------------------------------------------------ */

function supportsWebGL() {
  try {
    const canvas = document.createElement('canvas');
    return !!(window.WebGLRenderingContext
      && (canvas.getContext('webgl2') || canvas.getContext('webgl')));
  } catch (e) {
    return false;
  }
}

if (!supportsWebGL()) {
  document.getElementById('loading').innerHTML =
    '<div class="loader-inner"><h1>WebGL is not available</h1>'
    + '<p class="loader-sub">This simulation needs a WebGL2-capable browser.</p></div>';
} else {
  gsap.ticker.lagSmoothing(200, 33);
  new Simulation().init();
}
