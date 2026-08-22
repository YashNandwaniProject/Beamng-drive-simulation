/**
 * vehicle.js
 * ---------------------------------------------------------------------------
 * Loads a vehicle's GLB, validates and rigs its wheels, and drives the whole
 * visual model from the physics state.
 *
 * ONE RIG, TWO VERY DIFFERENT MODELS
 * ----------------------------------
 * The two cars are authored nothing alike, and the rig has to absorb that
 * without either of them needing special-case code at the call site:
 *
 *                          BMW.glb                lamborghini-revuelto.glb
 *   forward axis           +X                     -X   (modelYaw = PI)
 *   units                  metres                 millimetres (0.001 node scale)
 *   wheel node origin      on the tyre's outer    at the MODEL ORIGIN - metres
 *                          face, on the axle      away from the wheel itself
 *   wheel spin axis        local +Y               local +Z (RR mirrored in Z)
 *   authored orientation   a baked ~6 deg wobble  clean and axis-aligned
 *                          on Object_221
 *   interior steering      must NEVER move        must be animated
 *
 * The rig therefore never assumes where a wheel's origin is. It MEASURES each
 * wheel's geometric centre from its own bounding box and builds the pivot
 * there. That single decision is what makes both models work: rotating the
 * Lamborghini's wheel nodes in place would swing them around the car, because
 * their origins are at the model origin, not at the wheels.
 *
 *   chassis                       physics transform (centre of mass)
 *   +- bodyModel                  the loaded GLB, yawed into the body frame
 *   |                             and offset by -CoM
 *   +- hub          [x4]          suspension travel + tyre squash   (translate)
 *   |  +- steer                   steering angle             (rotate about up)
 *   |     +- camber               static camber + sidewall flex (about fwd)
 *   |        +- calipers          follow steering, never spin  (if the model
 *   |        |                    has body-mounted calipers)
 *   |        +- spin              rolling rotation        (about the axle)
 *   |           +- THE AUTHORED TYRE - untouched, with its own children
 *   |              +- rim / bolts / brake disc
 *   +- steeringWheelPivot         interior wheel, if this vehicle animates one
 *
 * Steering and rolling always get SEPARATE nodes, which is what stops the two
 * combining into one wrong axis.
 *
 * Requirements: #3/#5 (hierarchy preserved), #6/#26 (rotation from real angular
 * velocity), #8/#9 (interior steering wheel), #10 (physics drives animation,
 * never the reverse), #30/#31 (explicit validation), #32/#33 (transform safety).
 * ---------------------------------------------------------------------------
 */

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { VehiclePhysics } from './vehiclePhysics.js';
import { clamp, clamp01, damp, lerp } from './mathUtils.js';

/* ------------------------------------------------------------------------ */
/* Model cache                                                               */
/* ------------------------------------------------------------------------ */

/** url -> Promise<gltf>. One parse per model, however many cars use it. */
const MODEL_CACHE = new Map();

/** Load a GLB once and reuse it for every vehicle that needs it. */
export function loadModelCached(url, onProgress) {
  let entry = MODEL_CACHE.get(url);
  if (!entry) {
    const loader = new GLTFLoader();
    entry = new Promise((resolve, reject) => {
      loader.load(url, resolve, (e) => {
        if (onProgress && e.lengthComputable) onProgress(e.loaded / e.total);
      }, reject);
    });
    MODEL_CACHE.set(url, entry);
  } else if (onProgress) {
    onProgress(1);
  }
  return entry;
}

/** Pre-warm the cache so a later spawn is instant. */
export function preloadVehicleModel(spec, onProgress) {
  return loadModelCached(spec.model.path, onProgress);
}

/* ------------------------------------------------------------------------ */
/* Tyre deformation shader                                                   */
/* ------------------------------------------------------------------------ */

/**
 * Injected into a tyre's material. Flattens the tread against the road over
 * the contact arc and bulges the sidewall by the same amount.
 *
 * Driven by `uDeflection`, which the tyre model computes from normal load
 * divided by carcass stiffness (itself a function of pressure) - so a heavily
 * loaded outside front visibly squashes, and a soft tyre squashes further under
 * the same load.
 *
 * It has to be a shader rather than an object scale for two reasons: the flat
 * spot must stay at the BOTTOM of a spinning wheel, and scaling the tyre would
 * squash the rim and brake disc along with it.
 *
 * Everything is expressed in the tyre's OWN LOCAL SPACE and supplied per
 * vehicle, because the two models disagree about all of it - axis, centre and
 * even units (the Lamborghini's geometry is in millimetres).
 */
function installTyreDeformation(material, uniforms) {
  material.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, uniforms);
    shader.vertexShader = `
      uniform float uDeflection;
      uniform vec3  uContactDir;
      uniform vec3  uCentre;
      uniform vec3  uAxis;
      uniform float uArcCos;
      uniform float uRadius;
      uniform float uRimRadius;
      uniform float uBulge;
    ` + shader.vertexShader;

    shader.vertexShader = shader.vertexShader.replace(
      '#include <begin_vertex>',
      `#include <begin_vertex>
      {
        vec3 rel = transformed - uCentre;
        float axial = dot(rel, uAxis);
        vec3 radialV = rel - uAxis * axial;
        float r = length(radialV);
        if (r > 1e-4 && uDeflection > 1e-5) {
          vec3 rn = radialV / r;
          // 1 at the contact patch, fading over the contact arc.
          float arc = smoothstep(uArcCos, 1.0, dot(rn, uContactDir));
          // Only the tread deforms; the bead sitting on the rim cannot.
          float band = smoothstep(uRimRadius, uRadius, r);
          float t = arc * band;
          transformed -= rn * (uDeflection * t);
          // Rubber is near enough incompressible, so what leaves the tread has
          // to go somewhere: the sidewall bulges outward.
          transformed += uAxis * (axial * uBulge * t);
        }
      }`,
    );
  };
  material.customProgramCacheKey = () => 'tyreDeform';
  material.needsUpdate = true;
}

/* ------------------------------------------------------------------------ */
/* Vehicle                                                                   */
/* ------------------------------------------------------------------------ */

export class Vehicle {
  /**
   * @param {Terrain} terrain
   * @param {object}  spec       the vehicle spec (js/vehicles/*.js)
   * @param {CollisionSystem} collision
   */
  constructor(terrain, spec, collision = null) {
    this.terrain = terrain;
    this.spec = spec;
    this.id = spec.id;
    this.displayName = spec.displayName;

    this.physics = new VehiclePhysics(terrain, spec);
    this.physics.collisionSystem = collision;

    /** Root that carries the physics transform. Add THIS to the scene. */
    this.object3D = new THREE.Group();
    this.object3D.name = spec.id;

    this.bodyModel = null;
    /** id -> { hub, steer, camber, spin, tyre, uniforms, ... } */
    this.wheelRigs = {};
    /** Audit list of every object this rig takes control of. */
    this.riggedObjects = [];

    /** The interior steering wheel pivot, if this vehicle animates one. */
    this.steeringWheelPivot = null;
    this.steeringWheelObject = null;
    this._swAxis = new THREE.Vector3(1, 0, 0);
    this._swAngle = 0;

    this.brakeLightMaterials = [];
    /** [{ material, intensity }] - surfaces that take the reflection probe. */
    this.reflectiveMaterials = [];
    this.loaded = false;

    /* ------------------------------------ interpolation between steps */
    this._prevPos = new THREE.Vector3();
    this._prevQuat = new THREE.Quaternion();
    this._curPos = new THREE.Vector3();
    this._curQuat = new THREE.Quaternion();

    /* ------------------------------------------------------- scratch */
    this._contactWorld = new THREE.Vector3();
    this._contactLocal = new THREE.Vector3();
    this._axisTmp = new THREE.Vector3();
    /**
     * Inverse of a tyre's linear (3x3) transform. Used instead of a quaternion
     * because one of the Lamborghini's wheels is MIRRORED - see
     * `_worldDirToLocal`.
     */
    this._linearInv = new THREE.Matrix3();
    this._box = new THREE.Box3();
    this._v = new THREE.Vector3();
  }

  /* ===================================================================== */
  /* Loading                                                                */
  /* ===================================================================== */

  /**
   * @param {(fraction:number)=>void} [onProgress]
   */
  async load(onProgress) {
    const url = this.spec.model.path;
    const gltf = await loadModelCached(url, onProgress);

    /**
     * Clone rather than use the cached scene directly. A multiplayer session
     * can hold several copies of the same car at once, and the Lamborghini's
     * GLB is 20 MB - re-downloading and re-parsing it per player would be
     * absurd. Cloning shares geometries and textures and only duplicates the
     * node graph, which is exactly what is needed since the rig reparents
     * nodes and clones a handful of materials.
     */
    this.bodyModel = gltf.scene.clone(true);
    this.bodyModel.name = `${this.spec.id}_body`;

    /**
     * Carry the model into the shared body frame (nose along +X) and put the
     * physics origin at the centre of mass. Doing it here, once, is why no
     * downstream code has to know how a particular artist laid their car out.
     */
    const m = this.spec.model;
    this.bodyModel.rotation.y = m.modelYaw || 0;
    this.bodyModel.position.set(-m.comBody.x, -m.comBody.y, -m.comBody.z);
    this.object3D.add(this.bodyModel);
    this.object3D.updateMatrixWorld(true);

    this._index();
    this._validate();
    this._prepareMaterials();
    this._buildWheelRigs();
    this._rigCalipers();
    this._rigSteeringWheel();
    this._mergeBody();

    this.loaded = true;
    this._logRigSummary();
    return this;
  }

  /** Build a case-insensitive name index once, rather than traversing per lookup. */
  _index() {
    this._byName = new Map();
    this.bodyModel.traverse((o) => {
      if (o.name) this._byName.set(o.name.toLowerCase(), o);
    });
  }

  /**
   * Resolve an authored object name to the loaded object.
   *
   * THIS IS NOT AS SIMPLE AS IT LOOKS. three.js's GLTFLoader runs every node
   * name through `PropertyBinding.sanitizeNodeName`, which strips the
   * characters its animation-binding paths reserve - `[ ] . : /` - and turns
   * whitespace into underscores. So a wheel authored as
   *
   *     Wheel_001-material.001      (what the .glb actually contains)
   *
   * is called
   *
   *     Wheel_001-material001       (what three.js hands back)
   *
   * by the time it reaches us. Looking up the authored name alone finds three
   * of the Lamborghini's four wheels and silently misses the rest - the one
   * without a dot in its name works, which is exactly the kind of near-miss
   * that is easy to mistake for a bad model.
   *
   * Nothing is renamed here: the objects keep whatever name the loader gave
   * them. We simply look up both spellings, and still refuse to fall back to
   * an arbitrary mesh if neither matches.
   */
  _find(name) {
    const wanted = String(name);
    const direct = this._byName.get(wanted.toLowerCase());
    if (direct) return direct;

    const sanitized = THREE.PropertyBinding.sanitizeNodeName(wanted);
    if (sanitized !== wanted) {
      const viaSanitized = this._byName.get(sanitized.toLowerCase());
      if (viaSanitized) {
        if (!this._sanitizedNotice) {
          this._sanitizedNotice = true;
          console.info(
            `  (three.js sanitised the authored names on import, e.g. `
            + `"${wanted}" -> "${sanitized}"; resolved through both.)`,
          );
        }
        return viaSanitized;
      }
    }
    return null;
  }

  /**
   * The named objects must exist. If one does not we say so loudly and BY NAME
   * and refuse to load - we never silently substitute another mesh.
   * Requirements #4, #30, #31.
   */
  _validate() {
    const wanted = this.spec.model.wheelObjects;
    const found = {};
    const missing = [];

    for (const id of ['FL', 'FR', 'RL', 'RR']) {
      const obj = this._find(wanted[id]);
      if (obj) found[id] = obj;
      else missing.push(wanted[id]);
    }

    if (missing.length) {
      for (const name of missing) {
        console.error(
          `ERROR: ${this.spec.displayName} wheel object not found:\n${name}`,
        );
      }
      throw new Error(
        `${this.spec.displayName} wheel initialisation failed.\n`
        + missing.map((n) => `Missing object: ${n}`).join('\n'),
      );
    }
    this.wheelObjects = found;

    /* --- interior steering wheel, if this vehicle has one --------------- */
    const sw = this.spec.model.steeringWheel;
    if (sw) {
      const obj = this._find(sw.object);
      if (!obj) {
        console.error(
          `ERROR: ${this.spec.displayName} steering parent\n"${sw.object}"\n`
          + 'was not found.',
        );
        throw new Error(
          `${this.spec.displayName} steering wheel initialisation failed.\n`
          + `Missing object: ${sw.object}`,
        );
      }
      this.steeringWheelObject = obj;
    }

    console.info(`${this.spec.displayName} wheel system initialized successfully.`);
    console.info(
      `  ${wanted.FL} -> Front Left      ${wanted.FR} -> Front Right\n`
      + `  ${wanted.RL} -> Rear Left       ${wanted.RR} -> Rear Right`,
    );
    if (this.steeringWheelObject) {
      console.info(
        `  ${this.spec.model.steeringWheel.object} -> interior steering wheel`,
      );
    }
  }

  /**
   * Where a wheel's geometric centre actually is, in the car-root frame.
   *
   * This is measured rather than assumed, and it is the hinge the whole rig
   * turns on. The BMW's wheel pivot sits on the tyre's outer face; the
   * Lamborghini's sits at the model origin, metres away from the wheel. Only
   * the bounding box tells the truth for both.
   */
  _wheelCentre(obj, out) {
    this._box.setFromObject(obj, true);
    this._box.getCenter(out);
    return this.object3D.worldToLocal(out);
  }

  _buildWheelRigs() {
    const m = this.spec.model;
    /** 'realign' rebuilds a clean spin axis; 'preserve' keeps what was authored. */
    const mode = m.wheelOrientation || 'preserve';

    for (const wheel of this.physics.wheels) {
      const id = wheel.id;
      const tyre = this.wheelObjects[id];

      /* --- the transform chain ------------------------------------------ */
      const hub = new THREE.Group();
      hub.name = `Hub_${id}`;
      hub.position.set(wheel.hubLocal.x, wheel.hubLocal.y, wheel.hubLocal.z);

      const steer = new THREE.Group();
      steer.name = `Steer_${id}`;
      const camber = new THREE.Group();
      camber.name = `Camber_${id}`;
      const spin = new THREE.Group();
      spin.name = `Spin_${id}`;

      hub.add(steer);
      steer.add(camber);
      camber.add(spin);
      this.object3D.add(hub);

      /* --- measure the wheel before moving it --------------------------- */
      tyre.updateWorldMatrix(true, true);
      const centre = this._wheelCentre(tyre, new THREE.Vector3());

      const worldPos = new THREE.Vector3();
      const worldQuat = new THREE.Quaternion();
      const worldScale = new THREE.Vector3();
      tyre.matrixWorld.decompose(worldPos, worldQuat, worldScale);
      const localPos = this.object3D.worldToLocal(worldPos.clone());

      /* --- move it into the rig, preserving its placement exactly -------- */
      tyre.removeFromParent();
      spin.add(tyre);
      tyre.scale.copy(worldScale);

      if (mode === 'realign') {
        /**
         * The BMW's front wheels carry a baked rotation - Object_221's spin
         * axis is about six degrees off the lateral - which would show up as a
         * wobble the moment it starts turning. A tyre is a solid of revolution
         * about its own axis, so any in-plane phase is equivalent; all that
         * matters is putting that axis on the axle. Its local +Y must point
         * INBOARD: left wheel +Y -> +Z, right wheel +Y -> -Z.
         */
        tyre.quaternion.setFromAxisAngle(
          new THREE.Vector3(1, 0, 0), wheel.isLeft ? Math.PI / 2 : -Math.PI / 2,
        );
      } else {
        /**
         * The Lamborghini's wheels are already square to the car, and its rear
         * right is MIRRORED in Z. Preserving the authored orientation keeps
         * that mirror intact - rebuilding it would flip the wheel inside out.
         */
        tyre.quaternion.copy(worldQuat);
      }

      /**
       * Offset so the wheel's measured CENTRE lands on the pivot. For the BMW
       * this works out at half a tyre width; for the Lamborghini it is a
       * couple of metres. Either way the wheel ends up exactly where the
       * physics says its hub is, which is what makes the tyre touch the road
       * where the contact solver thinks it does.
       */
      tyre.position.set(
        localPos.x - centre.x,
        localPos.y - centre.y,
        localPos.z - centre.z,
      );

      this.riggedObjects.push(tyre.name);

      /* --- tyre deformation, in this tyre's own local space -------------- */
      const uniforms = this._buildTyreUniforms(tyre, wheel, worldScale);
      this._applyTyreMaterial(tyre, uniforms);

      /* --- brake discs, so they can glow under heavy braking ------------ */
      const discMaterials = [];
      tyre.traverse((child) => {
        if (!child.isMesh || !child.material) return;
        const name = (child.material.name || '').toUpperCase();
        if (name.includes('BRAKEDISC') || name.includes('ROTOR')) {
          child.material = child.material.clone();
          child.material.emissive = new THREE.Color(0x000000);
          discMaterials.push(child.material);
        }
      });

      this.wheelRigs[id] = {
        wheel, hub, steer, camber, spin, tyre, uniforms, discMaterials,
        calipers: [],
        camberSign: wheel.isLeft ? -1 : 1,
        staticCamber: wheel.staticCamber,
        lean: 0,
        discHeat: 0,
        restHubY: wheel.hubLocal.y,
        childCount: tyre.children.length,
      };
    }
  }

  /**
   * Express a WORLD direction in an object's own local frame.
   *
   * The obvious implementation - invert the object's world quaternion and
   * rotate by it - is wrong for a mirrored node, and this model has one.
   * `Wheel_001-material.002` (the rear right) sits under `_3DWheel_back_R`,
   * which the .glb scales by (-0.001, -0.001, -0.001). That is a reflection:
   * its matrix determinant is negative, so there is no quaternion that
   * describes it. `Matrix4.decompose` copes by flipping the sign of a single
   * scale axis, which reproduces the MATRIX correctly - the wheel draws in the
   * right place - but hands back an orientation that is not the node's.
   *
   * The visible symptom was precise: on that one wheel the flat spot at the
   * contact patch rode ROUND WITH THE TYRE instead of staying at the bottom,
   * because the direction being fed to the shader was mirrored and so failed
   * to cancel the spin.
   *
   * Inverting the linear part of the world matrix is exact for a reflection,
   * a non-uniform scale, or any combination, so every corner now uses it.
   */
  _worldDirToLocal(object, dirWorld, out) {
    object.updateWorldMatrix(true, false);
    this._linearInv.setFromMatrix4(object.matrixWorld).invert();
    return out.copy(dirWorld).applyMatrix3(this._linearInv);
  }

  /**
   * Work out, from the model itself, everything the deformation shader needs
   * in the tyre's own local space: which local direction is the axle, where
   * the wheel centre is, and what one metre is worth in local units.
   */
  _buildTyreUniforms(tyre, wheel, worldScale) {
    /**
     * The rig spins about the body's lateral axis (+Z), and the Spin node
     * rotates about that same axis - so the axis is invariant along the whole
     * chain and the tyre's LOCAL matrix is all that is needed to pull it back
     * into the tyre's own frame.
     *
     * This deliberately does NOT go through a quaternion. The Lamborghini's
     * rear-right wheel hangs off `_3DWheel_back_R`, whose scale in the .glb is
     * (-0.001, -0.001, -0.001): a point REFLECTION, not a rotation. A matrix
     * decomposition cannot represent that in a quaternion - three.js negates a
     * single scale axis and hands back an orientation that is simply not the
     * one the node has - so any quaternion-based frame change is wrong for
     * that corner alone. Inverting the linear part is exact for a reflection.
     */
    tyre.updateMatrix();
    const axis = this._axisTmp.set(0, 0, 1)
      .applyMatrix3(this._linearInv.setFromMatrix4(tyre.matrix).invert())
      .normalize();

    /** Local units per metre (1 for the BMW, 1000 for the Lamborghini). */
    const scale = (Math.abs(worldScale.x) + Math.abs(worldScale.y)
      + Math.abs(worldScale.z)) / 3;
    const perMetre = scale > 1e-9 ? 1 / scale : 1;

    /** The tyre's own geometry centre, in its own local space. */
    const centre = new THREE.Vector3();
    if (tyre.isMesh && tyre.geometry) {
      tyre.geometry.computeBoundingBox();
      tyre.geometry.boundingBox.getCenter(centre);
    }

    const radiusLocal = wheel.radius * perMetre;
    const ratio = this.spec.model.rimRadiusRatio
      ?? (this.spec.model.rimRadius ? this.spec.model.rimRadius / wheel.radius : 0.76);

    return {
      uDeflection: { value: 0 },
      uContactDir: { value: new THREE.Vector3(0, -1, 0) },
      uCentre: { value: centre },
      uAxis: { value: axis.clone() },
      uArcCos: { value: 0.90 },
      uRadius: { value: radiusLocal },
      uRimRadius: { value: radiusLocal * ratio },
      uBulge: { value: this.spec.tyre.sidewallBulge },
      /** Metres -> local units, so the physics deflection can be applied. */
      _perMetre: perMetre,
    };
  }

  /** Clone the tyre material for this corner and wire up the deformation. */
  _applyTyreMaterial(tyre, uniforms) {
    if (!tyre.isMesh || !tyre.material) return;
    tyre.material = tyre.material.clone();
    tyre.material.roughness = Math.max(0.82, tyre.material.roughness ?? 0.9);
    tyre.material.metalness = 0.0;
    tyre.material.envMapIntensity = 0.35;
    installTyreDeformation(tyre.material, uniforms);
  }

  /**
   * Body-mounted brake calipers, where a model has them as separate meshes.
   * They belong on the STEER node: a caliper turns with the steering and
   * travels with the suspension, but it must not spin with the wheel.
   *
   * Opt-in per vehicle. The Lamborghini's discs are already children of its
   * tyres (exactly the hierarchy the brief describes), so it needs none.
   */
  _rigCalipers() {
    const key = this.spec.model.caliperMaterial;
    if (!key) return;

    const candidates = [];
    this.bodyModel.traverse((o) => {
      if (!o.isMesh || !o.material) return;
      if ((o.material.name || '').toUpperCase().includes(key)) candidates.push(o);
    });

    this.object3D.updateMatrixWorld(true);
    const worldPos = new THREE.Vector3();

    for (const caliper of candidates) {
      caliper.updateWorldMatrix(true, false);
      worldPos.setFromMatrixPosition(caliper.matrixWorld);
      const local = this.object3D.worldToLocal(worldPos.clone());

      let best = null;
      let bestD = Infinity;
      for (const rig of Object.values(this.wheelRigs)) {
        const d = local.distanceToSquared(rig.hub.position);
        if (d < bestD) { bestD = d; best = rig; }
      }
      // A "caliper" more than a metre from any hub is not a caliper.
      if (!best || bestD > 1.0) continue;

      const quat = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      const pos = new THREE.Vector3();
      caliper.matrixWorld.decompose(pos, quat, scale);

      caliper.removeFromParent();
      best.steer.add(caliper);
      caliper.position.set(
        local.x - best.hub.position.x,
        local.y - best.hub.position.y,
        local.z - best.hub.position.z,
      );
      caliper.quaternion.copy(quat);
      caliper.scale.copy(scale);
      best.calipers.push(caliper);
      this.riggedObjects.push(caliper.name);
    }
  }

  /**
   * Interior steering wheel (requirements #8, #9, #31, #33).
   *
   * Same problem as the wheels, same solution: the object's node origin is at
   * the MODEL origin, not at the steering wheel, so rotating it in place would
   * swing it across the cabin. A pivot group is built at the measured centre of
   * the rim and the object is moved into it, keeping its own transform.
   *
   * The pivot's axis is the steering column, recovered by fitting a plane to
   * the assembly's vertices - 22.5 degrees from horizontal, which is what a
   * real column is.
   */
  _rigSteeringWheel() {
    const cfg = this.spec.model.steeringWheel;
    if (!cfg || !this.steeringWheelObject) return;

    const obj = this.steeringWheelObject;
    const yaw = this.spec.model.modelYaw || 0;
    const com = this.spec.model.comBody;

    /** Model-space pivot and axis, carried into the body frame. */
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const pivot = new THREE.Vector3(
      cfg.pivot.x * c + cfg.pivot.z * s - com.x,
      cfg.pivot.y - com.y,
      -cfg.pivot.x * s + cfg.pivot.z * c - com.z,
    );
    this._swAxis.set(
      cfg.axis.x * c + cfg.axis.z * s,
      cfg.axis.y,
      -cfg.axis.x * s + cfg.axis.z * c,
    ).normalize();

    const group = new THREE.Group();
    group.name = 'SteeringWheelPivot';
    group.position.copy(pivot);
    this.object3D.add(group);

    obj.updateWorldMatrix(true, false);
    const worldPos = new THREE.Vector3();
    const worldQuat = new THREE.Quaternion();
    const worldScale = new THREE.Vector3();
    obj.matrixWorld.decompose(worldPos, worldQuat, worldScale);
    const local = this.object3D.worldToLocal(worldPos.clone());

    obj.removeFromParent();
    group.add(obj);
    obj.position.set(local.x - pivot.x, local.y - pivot.y, local.z - pivot.z);
    obj.quaternion.copy(worldQuat);
    obj.scale.copy(worldScale);

    this.steeringWheelPivot = group;
    this.riggedObjects.push(obj.name);
  }

  /**
   * Shadows and material tuning. Nothing here changes any TRANSFORM, so a
   * vehicle whose interior must not move is only ever touched cosmetically.
   */
  _prepareMaterials() {
    const bl = this.spec.model.brakeLights;
    const comX = this.spec.model.comBody ? this.spec.model.comBody.x : 0;
    const paintClearcoat = this.spec.model.paintClearcoat || 0;
    const probeIntensity = this.spec.model.reflectionIntensity || 0;
    /** old material -> upgraded one, so a shared material upgrades ONCE. */
    this._paintUpgrades = new Map();

    /**
     * Shadow-casting is filtered by size. The Lamborghini's model is 829
     * separate meshes, and every caster is drawn a second time into the shadow
     * map - so letting a 2 cm interior switch cast a shadow doubles the cost of
     * the whole car for something no one can see through the glass. Anything
     * below `shadowMinRadius` still RECEIVES shadows, it just does not cast.
     *
     * Defaults to 0, so a vehicle that does not set it (the BMW) is unchanged.
     */
    const minShadow = this.spec.model.shadowMinRadius || 0;

    this.bodyModel.traverse((o) => {
      if (!o.isMesh) return;

      let casts = true;
      if (minShadow > 0 && o.geometry) {
        if (!o.geometry.boundingSphere) o.geometry.computeBoundingSphere();
        const bs = o.geometry.boundingSphere;
        // Geometry units are not always metres (this model is in millimetres),
        // so scale the radius by the node's own world scale.
        o.updateWorldMatrix(true, false);
        const sx = this._v.setFromMatrixScale(o.matrixWorld).x;
        casts = bs ? bs.radius * sx >= minShadow : true;
      }
      o.castShadow = casts;
      o.receiveShadow = true;
      o.frustumCulled = false;      // the car is always the subject

      const materials = Array.isArray(o.material) ? o.material : [o.material];
      for (let mi = 0; mi < materials.length; mi++) {
        let m = materials[mi];
        if (!m) continue;
        const name = (m.name || '').toUpperCase();

        /**
         * Paint gets a CLEAR COAT, on the cars that ask for one.
         *
         * Car paint is two surfaces, not one: a coloured metallic base and a
         * thin lacquer over it. A single roughness value can be one or the
         * other, never both, which is why one-layer paint always looks like
         * plastic - it has a single blurry highlight instead of a sharp
         * reflection floating over a soft one. This upgrade is what makes the
         * reflection probe's capture read as a reflection IN something rather
         * than a texture ON something.
         *
         * Opt-in per vehicle, so the BMW's appearance is untouched.
         */
        if (paintClearcoat && (name.includes('CARSKIN') || name.includes('CARPAINT'))) {
          m = this._withClearcoat(m, paintClearcoat);
          materials[mi] = m;
        }

        m.envMapIntensity = 1.15;

        if (name.includes('WINDSHIELD') || name.includes('WINDOW')
          || name.includes('VETRO')) {
          m.transparent = true;
          m.opacity = name.includes('MASK') ? 1.0 : 0.42;
          m.roughness = 0.06;
          m.metalness = 0.0;
          m.envMapIntensity = 1.6;
          m.depthWrite = false;
        } else if (name.includes('HEADLIGHTGLASS')) {
          m.transparent = true;
          m.opacity = 0.55;
          m.roughness = 0.08;
          m.envMapIntensity = 2.0;
        } else if (name.includes('CHROME') || name.includes('ALUMINUM')
          || name.includes('ALLUMINIUM')) {
          m.metalness = 0.95;
          m.roughness = Math.min(m.roughness ?? 0.3, 0.22);
          m.envMapIntensity = 1.5;
        } else if (name.includes('CARBONFIBER')) {
          m.metalness = 0.35;
          m.roughness = 0.42;
          m.envMapIntensity = 1.0;
        } else if (name.includes('ALCANTARA') || name.includes('PELLE')
          || name.includes('CUCITURE')) {
          m.metalness = 0.0;
          m.roughness = 0.92;
          m.envMapIntensity = 0.35;
        } else if (name.includes('RUBBER') || name.includes('DULLPLASTIC')
          || name.includes('WHEEL_001') || name.includes('NEROLOWGLOSS')
          || name.includes('GRIGLIA')) {
          m.roughness = 0.9;
          m.metalness = 0.0;
          m.envMapIntensity = 0.35;
        } else if (name.includes('CARSKIN') || name.includes('CARPAINT')) {
          m.metalness = 0.55;
          m.roughness = 0.22;
          m.envMapIntensity = 1.35;
        }

        /**
         * Which surfaces sample the real-time probe. Everything else keeps the
         * scene's sky IBL: a dashboard reflecting the barriers it cannot see
         * is worse than no reflection at all, and it is fill rate spent on
         * something nobody looks at.
         */
        if (probeIntensity > 0) {
          let gain = 0;
          if (name.includes('CARSKIN') || name.includes('CARPAINT')) gain = 1.0;
          else if (name.includes('CHROME') || name.includes('ALUMINUM')
            || name.includes('ALLUMINIUM')) gain = 1.15;
          else if (name.includes('WINDSHIELD') || name.includes('WINDOW')
            || name.includes('VETRO')) gain = 0.85;
          else if (name.includes('CARBONFIBER')) gain = 0.5;
          if (gain > 0 && !this.reflectiveMaterials.some((r) => r.material === m)) {
            this.reflectiveMaterials.push({
              material: m,
              intensity: m.envMapIntensity * probeIntensity * gain,
              skyIntensity: m.envMapIntensity,
            });
          }
        }

        /* --- brake lights ------------------------------------------------ */
        if (bl && name.includes(bl.material.toUpperCase())) {
          if (!bl.rearOnly) {
            // One material used only by the brake lights: light it directly.
            m.emissive = new THREE.Color(bl.colour || 0xff1408);
            m.emissiveIntensity = 0.0;
            if (!this.brakeLightMaterials.includes(m)) {
              this.brakeLightMaterials.push(m);
            }
          } else if (!Array.isArray(o.material)) {
            /**
             * This model uses ONE "Light" material for every lamp on the car,
             * headlights included. Making it emissive would light the front of
             * the car up under braking, so the rear lamps get their own clone
             * of it instead. One shared clone, not one per mesh - so the
             * merger below can still batch them into a single draw call.
             */
            o.updateWorldMatrix(true, false);
            this._v.setFromMatrixPosition(o.matrixWorld);
            this.object3D.worldToLocal(this._v);
            if (this._v.x < 0) {
              if (!this._brakeLightMaterial) {
                const clone = m.clone();
                clone.name = `${m.name || 'light'}_brake`;
                clone.emissive = new THREE.Color(bl.colour || 0xff1408);
                clone.emissiveIntensity = 0.0;
                this._brakeLightMaterial = clone;
                this.brakeLightMaterials.push(clone);
              }
              o.material = this._brakeLightMaterial;
            }
          }
        }
      }

      /**
       * Write back, in case a paint material was upgraded to a clear-coated
       * one. The brake-light branch above may have already replaced
       * `o.material` outright, so only touch it when this loop changed
       * something.
       */
      if (Array.isArray(o.material)) {
        for (let mi = 0; mi < materials.length; mi++) o.material[mi] = materials[mi];
      } else if (o.material !== this._brakeLightMaterial) {
        o.material = materials[0];
      }
    });
  }

  /**
   * Re-issue a material as a clear-coated one.
   *
   * three.js cannot turn a MeshStandardMaterial into a MeshPhysicalMaterial in
   * place, and `copy()` in that direction leaves every physical field
   * undefined, which breaks the shader. So the fields are carried across by
   * name. Shared materials upgrade ONCE and the result is reused - both
   * because it is wasteful otherwise and because `_mergeBody` batches by
   * material identity, and one paint material per mesh would defeat it.
   */
  _withClearcoat(material, amount) {
    if (material.isMeshPhysicalMaterial) {
      material.clearcoat = amount;
      return material;
    }
    const cached = this._paintUpgrades.get(material);
    if (cached) return cached;

    /**
     * Undefined values are not simply ignored by three.js - `setValues` warns
     * about every one of them - so anything optional is filtered out below.
     */
    const params = {
      name: material.name,
      color: material.color ? material.color.clone() : undefined,
      map: material.map || null,
      roughness: material.roughness ?? 0.5,
      roughnessMap: material.roughnessMap || null,
      metalness: material.metalness ?? 0.0,
      metalnessMap: material.metalnessMap || null,
      normalMap: material.normalMap || null,
      aoMap: material.aoMap || null,
      aoMapIntensity: material.aoMapIntensity ?? 1,
      emissive: material.emissive ? material.emissive.clone() : undefined,
      emissiveMap: material.emissiveMap || null,
      emissiveIntensity: material.emissiveIntensity ?? 1,
      alphaMap: material.alphaMap || null,
      envMapIntensity: material.envMapIntensity ?? 1,
      transparent: material.transparent,
      opacity: material.opacity,
      alphaTest: material.alphaTest,
      side: material.side,
      flatShading: material.flatShading,
      vertexColors: material.vertexColors,
      depthWrite: material.depthWrite,
      depthTest: material.depthTest,
      toneMapped: material.toneMapped,
      fog: material.fog,
      /** The lacquer itself: a thin, near-smooth layer over the base coat. */
      clearcoat: amount,
      clearcoatRoughness: 0.055,
    };
    for (const k of Object.keys(params)) {
      if (params[k] === undefined) delete params[k];
    }

    const p = new THREE.MeshPhysicalMaterial(params);
    if (material.normalScale) p.normalScale.copy(material.normalScale);
    this._paintUpgrades.set(material, p);
    return p;
  }

  /**
   * Point this car's glossy surfaces at a real-time reflection probe, or back
   * at the scene's sky IBL when handed `null`.
   *
   * @param {THREE.Texture|null} texture
   */
  applyReflectionProbe(texture) {
    for (const entry of this.reflectiveMaterials) {
      entry.material.envMap = texture;
      entry.material.envMapIntensity = texture ? entry.intensity : entry.skyIntensity;
      entry.material.needsUpdate = true;
    }
  }

  /**
   * Batch the remaining body meshes by material.
   *
   * The Lamborghini's GLB is 829 separate meshes. Drawn one by one - twice
   * over, once into the shadow map - that is 2,600 draw calls for a single car
   * and it halves the frame rate on its own. They almost all share one of 31
   * materials, so merging by material collapses that to a few dozen calls with
   * no visible change at all.
   *
   * This runs AFTER rigging, so by the time it sees `bodyModel` the four tyres
   * and the interior steering wheel have already been moved out into their own
   * pivots. Those are never merged - their hierarchy has to survive intact and
   * they each need an independent transform. Only static bodywork is touched.
   *
   * Opt-in per vehicle: the BMW's 74 meshes are not worth batching, and
   * leaving it off keeps that car byte-for-byte as it was.
   */
  _mergeBody() {
    if (!this.spec.model.mergeBody) return;

    this.bodyModel.updateMatrixWorld(true);
    const toBody = new THREE.Matrix4().copy(this.bodyModel.matrixWorld).invert();
    const local = new THREE.Matrix4();

    /** material + shadow flag -> the meshes that can share one draw call. */
    const groups = new Map();
    this.bodyModel.traverse((o) => {
      if (!o.isMesh || Array.isArray(o.material) || !o.material) return;
      if (!o.geometry || !o.geometry.attributes || !o.geometry.attributes.position) return;
      const key = `${o.material.uuid}|${o.castShadow ? 1 : 0}`;
      let g = groups.get(key);
      if (!g) {
        g = { material: o.material, castShadow: o.castShadow, meshes: [] };
        groups.set(key, g);
      }
      g.meshes.push(o);
    });

    let mergedCount = 0;
    let removed = 0;

    for (const group of groups.values()) {
      if (group.meshes.length < 2) continue;

      const geos = [];
      let ok = true;
      for (const mesh of group.meshes) {
        const g = mesh.geometry.clone();

        /**
         * mergeGeometries insists every input has an identical attribute set,
         * so reduce them all to the three attributes this renderer uses and
         * synthesise anything missing.
         */
        for (const attr of Object.keys(g.attributes)) {
          if (attr !== 'position' && attr !== 'normal' && attr !== 'uv') {
            g.deleteAttribute(attr);
          }
        }
        if (!g.attributes.normal) g.computeVertexNormals();
        if (!g.attributes.uv) {
          g.setAttribute('uv', new THREE.BufferAttribute(
            new Float32Array(g.attributes.position.count * 2), 2,
          ));
        }
        if (!g.index) {
          const n = g.attributes.position.count;
          const idx = n > 65535 ? new Uint32Array(n) : new Uint16Array(n);
          for (let i = 0; i < n; i++) idx[i] = i;
          g.setIndex(new THREE.BufferAttribute(idx, 1));
        }

        mesh.updateWorldMatrix(true, false);
        g.applyMatrix4(local.multiplyMatrices(toBody, mesh.matrixWorld));
        geos.push(g);
      }

      let merged = null;
      try {
        merged = mergeGeometries(geos, false);
      } catch (err) {
        // A group that will not merge is simply left alone.
        ok = false;
      }
      for (const g of geos) g.dispose();
      if (!ok || !merged) continue;

      const mesh = new THREE.Mesh(merged, group.material);
      mesh.name = `merged_${group.material.name || 'material'}`;
      mesh.castShadow = group.castShadow;
      mesh.receiveShadow = true;
      mesh.frustumCulled = false;
      this.bodyModel.add(mesh);

      for (const old of group.meshes) {
        old.removeFromParent();
        if (old.geometry) old.geometry.dispose();
        removed++;
      }
      mergedCount++;
    }

    this._mergeStats = { batches: mergedCount, meshesRemoved: removed };
    if (mergedCount) {
      console.info(
        `  Batched ${removed} body meshes into ${mergedCount} draw calls `
        + '(rigged wheels and steering wheel excluded).',
      );
    }
  }

  _logRigSummary() {
    const wheels = 4;
    const extra = this.riggedObjects.length - wheels;
    console.info(
      `${this.spec.displayName} rig: ${this.riggedObjects.length} objects under `
      + `physics control (${wheels} tyres + ${extra} other).`,
    );
    for (const id of ['FL', 'FR', 'RL', 'RR']) {
      const r = this.wheelRigs[id];
      console.info(
        `  ${id}: ${r.tyre.name} keeps ${r.childCount} child object(s) `
        + '(rim / bolts / brake disc) - hierarchy preserved.',
      );
    }
    if (this.steeringWheelPivot) {
      console.info('  Interior steering wheel IS animated, from the physical '
        + 'steering angle only.');
    } else {
      console.info('  The interior steering wheel is intentionally NOT animated.');
    }
  }

  /* ===================================================================== */
  /* Simulation                                                             */
  /* ===================================================================== */

  spawn(groundPosition, heading) {
    this.physics.reset(groundPosition, heading);
    this._captureTransform(true);
    this.syncVisuals(1, 0);
  }

  respawn() {
    this.physics.respawn();
    this._captureTransform(true);
    for (const rig of Object.values(this.wheelRigs)) {
      rig.lean = 0;
      rig.discHeat = 0;
    }
    this._swAngle = 0;
    this.syncVisuals(1, 0);
  }

  /** One fixed physics step. Call from the accumulator loop. */
  step(dt, input) {
    this._captureTransform(false);
    this.physics.step(dt, input);
    const b = this.physics.body;
    this._curPos.set(b.position.x, b.position.y, b.position.z);
    this._curQuat.set(b.quat.x, b.quat.y, b.quat.z, b.quat.w);
  }

  _captureTransform(snap) {
    const b = this.physics.body;
    if (snap) {
      this._prevPos.set(b.position.x, b.position.y, b.position.z);
      this._prevQuat.set(b.quat.x, b.quat.y, b.quat.z, b.quat.w);
      this._curPos.copy(this._prevPos);
      this._curQuat.copy(this._prevQuat);
    } else {
      this._prevPos.copy(this._curPos);
      this._prevQuat.copy(this._curQuat);
    }
  }

  /**
   * Push the physics state onto the graphics. This direction ONLY - nothing
   * here ever writes back into the simulation (requirement #10, #36).
   *
   * @param {number} alpha 0..1 interpolation between the last two physics steps
   * @param {number} dt    real frame time, for purely cosmetic easing
   */
  syncVisuals(alpha, dt) {
    if (!this.loaded) return;
    const p = this.physics;

    /* --- chassis ------------------------------------------------------- */
    this.object3D.position.lerpVectors(this._prevPos, this._curPos, alpha);
    this.object3D.quaternion.copy(this._prevQuat).slerp(this._curQuat, alpha);

    /* --- wheels -------------------------------------------------------- */
    for (const id of ['FL', 'FR', 'RL', 'RR']) {
      const rig = this.wheelRigs[id];
      if (!rig) continue;
      const w = rig.wheel;

      /* 1. suspension travel + tyre squash, along the body's up axis. */
      rig.hub.position.y = rig.restHubY
        + w.suspension.displacement
        - w.tire.deflection;

      /* 2. steering. The rear wheels are never given a steer angle: the
       *    physics holds theirs at zero and this simply follows. */
      rig.steer.rotation.y = -(w.steerAngle + w.staticToe);

      /* 3. camber + sidewall flex under lateral load. */
      const loadRef = Math.max(600, w.normalForce);
      const lateralRatio = clamp(w.latForce / loadRef, -1.6, 1.6);
      const leanTarget = lateralRatio * 0.055 * w.tire.sidewallCompliance;
      rig.lean = dt > 0 ? damp(rig.lean, leanTarget, 0.06, dt) : leanTarget;
      rig.camber.rotation.x = rig.staticCamber * rig.camberSign + rig.lean;

      /* 4. rolling rotation, straight from the integrated wheel angle.
       *    Never a constant animation rate (requirement #6). */
      rig.spin.rotation.z = -w.spinAngle;

      /* 5. tyre deformation uniforms, converted into local units. */
      const u = rig.uniforms;
      const defl = Math.min(w.tire.deflection, this.spec.tyre.maxVisualDeflection);
      u.uDeflection.value = defl * u._perMetre;
      u.uArcCos.value = lerp(0.94, 0.80,
        clamp01(w.tire.deflection / this.spec.tyre.maxVisualDeflection));

      if (w.hasContact && defl > 1e-5) {
        /**
         * Direction from the hub to the contact patch, in the TYRE's own local
         * frame - which is what keeps the flat spot at the bottom while the
         * wheel spins. The axial component is removed so the direction stays
         * in the wheel plane.
         */
        this._contactWorld.set(
          -w.contactNormal.x, -w.contactNormal.y, -w.contactNormal.z,
        );
        this._worldDirToLocal(rig.tyre, this._contactWorld, this._contactLocal);
        const axis = u.uAxis.value;
        const axial = this._contactLocal.dot(axis);
        this._contactLocal.addScaledVector(axis, -axial);
        if (this._contactLocal.lengthSq() > 1e-8) {
          u.uContactDir.value.copy(this._contactLocal.normalize());
        }
      }

      /* 6. brake discs heat up under braking and glow. */
      if (rig.discMaterials.length) {
        const power = Math.abs(w.brakeTorque * w.spinVelocity);
        const target = clamp01(power / 260000);
        rig.discHeat = dt > 0
          ? damp(rig.discHeat, target, target > rig.discHeat ? 0.55 : 3.2, dt)
          : target;
        const glow = rig.discHeat * rig.discHeat;
        for (const m of rig.discMaterials) {
          m.emissive.setRGB(glow * 0.85, glow * 0.16, glow * 0.02);
          m.emissiveIntensity = glow * 2.2;
        }
      }
    }

    /* --- interior steering wheel ---------------------------------------
     * Requirement #10: the PHYSICS drives this, never the other way round.
     * The angle is the real road-wheel angle multiplied by the model's own
     * steering ratio, so the rim and the front tyres can never disagree.
     *
     * SIGN: the pivot axis points rearward-and-up, i.e. toward the driver, so
     * a positive rotation about it reads as counter-clockwise from the driving
     * seat. Steering LEFT is a negative road-wheel angle, so negating it turns
     * the rim counter-clockwise for A and clockwise for D, as required.
     */
    if (this.steeringWheelPivot) {
      const cfg = this.spec.model.steeringWheel;
      const target = -p.steering.angle * cfg.ratio;
      // Smoothed, so it never snaps from lock to lock.
      this._swAngle = dt > 0 ? damp(this._swAngle, target, 0.035, dt) : target;
      this.steeringWheelPivot.quaternion.setFromAxisAngle(this._swAxis, this._swAngle);
    }

    /* --- brake lights --------------------------------------------------- */
    const braking = Math.max(p.brakes.pedal, p.brakes.handbrake * 0.85);
    for (const m of this.brakeLightMaterials) {
      m.emissiveIntensity = braking * 2.6;
    }
  }

  /* ===================================================================== */

  get body() { return this.physics.body; }
  get speedKmh() { return this.physics.speedKmh; }
  get position() { return this.object3D.position; }
  /** Interior steering wheel rotation in degrees (0 when a car has none). */
  get steeringWheelDegrees() { return (this._swAngle * 180) / Math.PI; }

  dispose() {
    this.object3D.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      const mats = o.material
        ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
      for (const m of mats) m.dispose();
    });
    this.object3D.removeFromParent();
  }
}
