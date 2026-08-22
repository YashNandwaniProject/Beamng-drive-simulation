# Physics Driving Simulation — BMW M3 GTR & Lamborghini Revuelto

A physics-driven driving simulation in the spirit of BeamNG.drive's
*behaviour* — raycast suspension, transient-slip tyres, real drivetrains, ABS,
weight transfer and a camera that responds to measured G-force.

**Two cars, one physics engine, two completely separate specifications.** They
are not variations on a theme: one is a rear-drive turbocharged saloon with a
torque converter, the other a mid-engined all-wheel-drive V12 hybrid with a
dual clutch and an electric front axle. 0–100 km/h takes 6.6 s in one and
3.5 s in the other, and the difference is entirely in the parameters.

It is deliberately **not** an arcade controller. No line anywhere in the project
writes to `car.position` in response to a key. The chain is:

```
key  →  throttle / brake / steering demand
     →  engine torque  →  clutch / torque converter  →  gearbox  →  diff
     →  wheel torque   →  wheel angular acceleration
     →  slip ratio / slip angle
     →  tyre forces at four contact patches
     →  forces and torques on a 6-DOF rigid body
     →  the car moves, dives, squats, rolls and yaws
```

Everything else — body roll, brake dive, wheelspin, understeer, a handbrake
turn — is a *consequence* of that chain, not a scripted effect.

---

## Running it

```bash
npm install     # only needed if node_modules/ is missing; three + gsap are
                # already vendored under vendor/, so this is optional
npm start       # serves the folder on http://localhost:5173
```

Then open <http://localhost:5173>.

A static server is required: ES modules and the `.glb` fetch are both blocked
under the `file://` origin. Any server works — `python -m http.server`,
`npx serve`, VS Code Live Server. `npm start` runs the dependency-free
`serve.mjs` included here.

Requires a desktop browser with WebGL2 (Chrome, Edge, Firefox, Safari 16+).

---

## Controls

| Key | Action |
| --- | --- |
| `W` / `↑` | Throttle |
| `S` / `↓` | Brake — **hold at a standstill to select reverse** |
| `A` / `D` | Steer left / right |
| `Space` | Handbrake (rear axle only) |
| `C` | Change camera — chase → side chase → bonnet → **interior** → orbit |
| `R` | Reset the car to the start line |
| `Esc` | Pause (shows the control list) |
| `H` / `F1` | Telemetry overlay |
| `B` / `T` | Toggle ABS / traction control |
| `Q` / `E` | Manual downshift / upshift (switches the box to manual; `R` restores auto) |
| `[` / `]` | Tyre pressure down / up, 2 psi at a time |
| `G` | Open the garage and change car |
| `M` | Mute / unmute |
| `V` | Reflection quality — off / low / high |

A gamepad is picked up automatically if one is connected: left stick steers,
the triggers are analogue throttle and brake, `A` is the handbrake.

**Reverse** works the way an automatic does. Come to a stop, keep the brake
held for a third of a second, and the box selects `R`; `S` then drives you
backwards and `W` brakes. Hold `W` at a stop to go back to `D`.

**Multiplayer** needs no setup: open <http://localhost:5173> in a second tab and
the two cars appear in each other's world, name-plated and correctly animated.
Both tabs can pick different cars.

---

## The BMW: model measurements

Nothing about the car's geometry is guessed. The `.glb` was parsed directly and
every wheel node's bounding box transformed into model space; those numbers are
in `js/config.js` and are the single source of truth for the physics *and* the
rig.

| | Value |
| --- | --- |
| Forward axis | **+X** (headlights at +X, licence plate at −X) |
| Up axis | +Y |
| Right axis | +Z |
| Tyre radius | 0.3290 m |
| Tyre width | 0.2920 m |
| Wheelbase | 2.7315 m |
| Track | 1.6178 m |
| Ground plane in model space | y = −0.0925 |
| Ground clearance (lowest bodywork) | 0.0575 m |

The four wheel objects the brief specifies:

| Object | Position (model space) | Corner |
| --- | --- | --- |
| `Object_221` | x = +1.617, z = −0.954 | **Front left** (driver / steering side) |
| `Object_95` | x = +1.617, z = +0.956 | **Front right** |
| `Object_236` | x = −1.114, z = −0.954 | **Rear left** |
| `Object_206` | x = −1.114, z = +0.956 | **Rear right** |

Two details worth knowing, both discovered from the file rather than assumed:

* **The names use a capital `O`.** The GLB stores `Object_221`, the brief writes
  `object_221`. The loader matches case-insensitively so both spellings work,
  and it still fails loudly if the object genuinely is not there.
* **Each tyre's pivot is on its outer face, not at the wheel centre**, and its
  local **+Y is the spin axis** pointing inboard. Half the tyre width (0.146 m)
  therefore separates the authored pivot from the hub. The rig accounts for
  this, which is why the wheels sit exactly where they were drawn while still
  steering about the correct point.

All four tyres share identical geometry, and each already has exactly two
children — the rim and the brake disc — as the brief describes. **That
hierarchy is never disturbed.**

### The wheel rig

```
chassis                          physics transform (at the centre of mass)
 ├── BMW_Body                    the loaded GLB, offset by −CoM
 └── Hub_FL     ×4               suspension travel + tyre squash   (translate)
     └── Steer_FL                steering angle                    (rotate about up)
         └── Camber_FL           static camber + sidewall flex     (rotate about forward)
             ├── caliper meshes  follow steering, never spin
             └── Spin_FL         rolling rotation                  (rotate about the axle)
                 └── Object_221  THE AUTHORED RUBBER TYRE — untouched
                     ├── rim         still its child
                     └── brake disc  still its child
```

Steering and rolling get **separate nodes**, which is what prevents the two
combining into one wrong axis. The brake calipers are reparented onto the
*steer* node — a caliper turns with the steering and travels with the
suspension, but it must not spin with the wheel; left on the body they would
have stayed behind while the wheel moved.

### The BMW's interior steering wheel is not animated

This is absolute in the brief, so the rig only ever touches objects it has
explicitly identified: the four named tyres and the eight caliper meshes.
Twelve objects in total, listed in `vehicle.riggedObjects` and printed to the
console at start-up. Everything else in the GLB — the entire interior included
— is left exactly as it loaded.

`tests/browser.mjs` verifies this the hard way: it snapshots the transform of
every interior mesh, drives the car hard for several seconds with full steering
lock, and asserts that not one of them moved.

Steering still works, of course — through the front road wheels, where it
belongs.

---

## The physics (shared by every vehicle)

### Rigid body

A plain 6-DOF Newton–Euler integrator (`vehiclePhysics.js`). It knows nothing
about cars: it accumulates forces and torques and integrates. Roll, pitch,
squat and dive all emerge from *where* forces are applied — at four contact
patches, half a metre below the centre of mass — never from a scripted
rotation.

Mass 1720 kg; inertia 560 / 2650 / 2480 kg·m² about the roll / yaw / pitch
axes; 52 % static front weight distribution, taken from the measured wheelbase.

### Suspension (`suspension.js`)

Four fully independent struts, each `F = staticLoad + k·x + c·ẋ` with
asymmetric bump/rebound damping, progressive bump and droop stops, and an
anti-roll bar per axle that resists a *difference* in travel (so it adds roll
stiffness without adding heave stiffness).

Travel is expressed as a strut *length*, which maps directly onto the ground
raycast. The compression rate comes from the change in that length, so the
damper feels a bump in the **road**, not just motion of the body.

Front 34 kN/m ≈ 1.39 Hz ride frequency; 90 mm bump, 100 mm droop.

### Ground contact (`wheelPhysics.js`)

For each wheel, solve for the strut length at which a tyre of radius *R* touches
the surface. For a locally planar patch with point **Q** and unit normal **N**:

```
dot(T + L·D − Q, N) = R      ⟹     L = (R − dot(T − Q, N)) / dot(D, N)
```

Two iterations — the first picks the surface patch using the previous hub
position, the second re-solves at the corrected one. This is what keeps the
tyre *on* slopes, banking and kerbs rather than floating above or sinking
through.

The strut force acts along the body's up axis but the ground can only push
along its own normal, so the normal force is `strutForce / dot(up, N)`. On the
16° banked corner that is a 4 % difference, and it is the reason the banking
actually helps you.

### Tyres (`tirePhysics.js`)

Transient slip with a relaxation length — the standard first-order lag that
keeps the model finite at standstill (where the steady-state slip definition
divides by zero) and gives the tyre the small delay that makes weight transfer
and countersteer feel right:

```
σ · dκ/dt + |vₓ|·κ = ω·Rₑ − vₓ
```

Both relaxation equations are integrated **implicitly**. Explicitly, the decay
rate is `|vₓ|/σ` — 125 s⁻¹ at 40 m/s — which blows straight past the stability
limit at 60 Hz. This is the single genuinely stiff term in the model and it is
why a naive implementation looks fine at 240 Hz and falls apart when the
physics rate drops.

Force comes from one **normalised Magic Formula**. Both slips are divided by
their own peak, combined on a circle, run through a single curve, then split
back along the slip direction:

```
sx = κ / κpeak
sy = tan α / tan αpeak
s  = √(sx² + sy²)
F  = μ·Fz · MF(s)         Fx = (sx/s)·F      Fy = (sy/s)·F
```

That one step gives peak force at the peak slip, progressive break-away past
it, and a proper friction ellipse where longitudinal force steals from lateral
and vice versa. `B` is solved so the peak lands exactly at `s = 1`; `E = 0.5`
sets the tail so a fully locked tyre makes 68 % of peak — if the tail is too
flat, locking the wheels costs nothing and ABS becomes pointless.

Also modelled: **load sensitivity** (μ falls as load rises, which is why
lateral weight transfer costs an axle grip), carcass temperature, pneumatic
trail collapsing at the limit (steering going light), and rolling resistance.

### Tyre pressure is not cosmetic

Every tyre carries its own PSI, and it feeds five separate things:

| Pressure effect | Mechanism |
| --- | --- |
| Peak slip ratio and slip angle | `∝ (P₀/P)^0.35` — a soft carcass winds up before the patch does, which is *why* an under-inflated car steers vaguely |
| Grip | peaks near 34 psi, falls away both sides |
| Carcass stiffness | 195 kN/m at nominal, ±4.6 kN/m per psi |
| Deformation | load ÷ stiffness — soft tyres visibly squash further under the same load |
| Rolling resistance | rises as pressure drops |

Press `[` and `]` and you can feel all of it. The test suite asserts each
relationship independently.

### Tyre deformation

The squash is `normal load ÷ carcass stiffness`, so it responds to load
transfer and to pressure. It is applied in a **vertex shader** on the tyre
material, not by scaling the object, for two reasons: the flat spot has to stay
at the bottom of a *spinning* wheel, and scaling the tyre would squash the rim
and brake disc with it. The sidewall bulges outward by the same amount, because
rubber is near enough incompressible.

### Drivetrain (`drivetrain.js`)

Turbocharged inline-six, 500 N·m from 1800–5000 rpm, into an **8-speed
automatic**:

* **Torque converter**, modelled with the classic capacity-factor tables
  (`Tp = (rpm/K(sr))²`, `Tt = Tp·TR(sr)`). Stall speed works out at ~2650 rpm.
  This is why the revs flare on a standing start, why the car creeps at idle
  (about 6 km/h, then stops gaining — exactly as the real thing does), and why
  the driveline takes up without a shock. A friction-clutch model cannot do any
  of this: at partial engagement it transmits its full clamp load, which in
  first gear is several thousand N·m at the wheel.
* **Lock-up clutch** with hysteresis, closing only once the turbine has nearly
  caught the pump.
* **Reflected inertia.** Locked up in first, the crank is geared 14.8 : 1 to the
  road, so its 0.235 kg·m² appears at the wheel as 51 kg·m² — a real ~20 %
  effective mass penalty, and a large part of why the car cannot simply
  accelerate at the friction limit.
* **Turbo lag** — a first-order lag on the throttle plate.
* **Clutch-pack LSD**, capacity = preload + a share of the torque through it.
* **Idle governor** with an integrator, because a proportional-only governor
  physically cannot hold idle (the steady-state error *is* the throttle needed
  to balance friction).

Both the lock-up clutch and the LSD are solved **semi-implicitly**: each
computes the torque that would exactly equalise the two speeds over the step,
then clamps it to what the hardware can transmit. Below the limit it sticks,
above it, it slips at exactly the Coulomb bound. That is unconditionally
stable — an explicit stiffness diverges the moment the physics rate drops.

### Brakes, ABS and traction control (`brakes.js`)

Nothing subtracts velocity from the car. The system produces a **brake torque**
per wheel; the wheel decelerates, the tyre develops negative slip, and the
resulting longitudinal tyre force is what slows the vehicle — which is also
what generates the forward weight transfer and the nose dive.

ABS closes a proportional loop on slip, **per corner**, targeting −0.10 (just
inside the tyre's −0.115 peak) with a fast release and a slower re-apply. A
wheel on a low-grip patch releases while the other three keep braking. The
controller runs on its own fixed 100 Hz pump clock and simply *holds* pressure
in between, which is both what the real hardware does and what keeps the
braking model free of any dependence on the physics rate.

It earns its keep: **35.0 m from 100 km/h with ABS, against 45.1 m with the
wheels locked** — the 22 % that sliding past the peak of the tyre curve costs
you.

The handbrake is rear-axle only and deliberately **not** ABS-regulated —
locking the rears is the entire point.

### Steering (`steering.js`)

Rate-limited rack (so a digital key press is never a step input),
speed-sensitive lock (33° at a standstill falling to ~10° at 200 km/h),
self-centring driven by the tyres' own aligning moment, and **Ackermann
geometry**:

```
R      = wheelbase / tan δ
δinner = atan(wheelbase / (R − track/2))
δouter = atan(wheelbase / (R + track/2))
```

blended 78 % toward geometric, as a real rack is. At full lock the inner wheel
turns 37.6° and the outer 29.5°.

### The circuit (`terrain.js`)

A 1.37 km closed circuit defined as a Catmull–Rom spline with per-station
elevation, banking, width, surface type and roughness. It includes a long
straight, a 16° banked right-hand sweeper, a crest, an ess, a deliberately
broken washboard section, a tight hairpin, a wet-asphalt patch, kerbs, gravel
run-off and grass.

The important property: **the road mesh is tessellated by calling exactly the
same `heightAt()` the tyres raycast against.** There is no separate collision
road that could drift out of alignment — the banking you can see *is* the
banking the suspension feels.

### Camera (`camera.js`)

Five modes, all on one rig. The follow yaw is a blend of where the car points
and where it is actually travelling — a camera welded to body yaw swings wildly
in a drift, one welded to velocity ignores the slide entirely. Eye and target
are then displaced by the car's **measured acceleration**, so hard braking
pushes the view forward and cornering leans it. FOV widens with speed; vertical
impacts shake it; it never drops below the terrain.

Two of the five — the bonnet and the **interior** cameras — are marked `rigid`.
They are bolted to the shell and inherit its full attitude, pitch and roll
included, which is what makes them feel like being in the car rather than
watching it. The other three ride a flat yaw frame, which is what keeps a chase
camera readable through a slide.

**The interior camera** puts the eye in the driver's seat. Its position is not
a guessed number: on the Lamborghini it is anchored to the measured steering
wheel — `MODEL.steeringWheel.pivot` lands at body `(0.674, 0.253, -0.380)`, so
the eye sits 0.55 m behind that rim and 0.32 m above it, which is 1.02 m above
the road in a car that is 1.16 m tall, and 0.38 m left of centre because the
car is left-hand drive. It also runs a narrower 55° field of view and damps the
speed-with-FOV widening to a quarter, because from a fixed cockpit that effect
reads as the cabin breathing.

GSAP handles only the cross-fade between modes, the loading screen, the pause
overlay and HUD flourishes. It is never allowed near the vehicle state.

---

---

# Lamborghini Revuelto

A second car, added without touching the first. The physics engine is shared;
everything that differs between the two lives in a specification object.

```
         COMMON PHYSICS ENGINE
   vehiclePhysics · wheelPhysics · tirePhysics
   suspension · steering · drivetrain · brakes
                    |
                    v
        VEHICLE-SPECIFIC SPECIFICATION
         js/vehicles/{bmw,lamborghini}.js
                    |
        +-----------+-----------+
        v                       v
   BMW M3 GTR          Lamborghini Revuelto
```

## The BMW is provably unchanged

Its tuning was moved out of `config.js` into `js/vehicles/bmw.js` **verbatim** —
not one number was retuned. The 127-assertion suite that pins its behaviour
still passes, and the headline figures are identical to the digit:

| | before | after |
| --- | --- | --- |
| 0–100 km/h | 6.646 s | 6.646 s |
| Braking 100–0 | 35.032 m | 35.032 m |
| Peak braking G | 1.241 G | 1.241 G |
| Lateral G | 1.224 G | 1.224 G |
| Body roll | 5.026° | 5.026° |
| Front static load | 4391.391 N | 4391.391 N |

`tests/vehicles.mjs` additionally asserts that the BMW is still rear-drive,
still has a torque converter, still has no electric axle, and still does not
animate its interior steering wheel.

## Model measurements

Measured out of `assets/lamborghini-revuelto.glb` the same way as the BMW —
walking the glTF node tree and transforming each object's bounding box into
model space. The agreement with the real car is a good sign the model is to
scale:

| | measured | real Revuelto |
| --- | --- | --- |
| Length | 4.948 m | 4.947 m |
| Wheelbase | 2.781 m | 2.779 m |
| Front track | 1.712 m | 1.700 m |
| Rear track | 1.680 m | 1.660 m |
| Rear tyre width | 0.337 m | 0.345 m (345 section) |
| Ground clearance | 0.075 m | very low, as it should be |

The four wheel objects, exactly as specified:

| Object | Position (model space) | Corner |
| --- | --- | --- |
| `Wheel_001-material.001` | x = 0.000, z = +0.857 | **Front left** |
| `Wheel_001-material` | x = 0.000, z = −0.855 | **Front right** |
| `Wheel_001-material.003` | x = +2.781, z = +0.842 | **Rear left** |
| `Wheel_001-material.002` | x = +2.781, z = −0.838 | **Rear right** |

and the interior steering wheel, `MAT_CarbonFiberMatte-material.006`.

### Four things about this model that had to be handled

This GLB is authored nothing like the BMW's, and each difference is a way the
rig could have silently produced something wrong:

**1. It faces backwards.** Its own front wheels are at x = 0 and its rear
wheels at x = +2.781, so the nose points along **−X** — the opposite of the
BMW. `modelYaw: PI` carries it into the shared body frame. Without it the car
drives backwards and steers the wrong way. (The model's own `_3DWheel_front_L`
naming, the steering wheel sitting *behind* the front axle on the left, and a
single light forward of the front axle with the rest at the tail all agree on
which end is the nose.)

**2. Its wheel pivots are at the model origin, not at the wheels.** Rotating
those nodes in place would swing each wheel in a two-metre arc around the car.
The rig therefore never assumes where a wheel's origin is — it **measures** the
geometric centre from the bounding box and builds the pivot there. That one
decision is what makes both models work from the same code.

**3. three.js renames the objects on import.** `GLTFLoader` runs every node
name through `PropertyBinding.sanitizeNodeName`, which strips the characters
its animation paths reserve — `[ ] . : /`. So

```
Wheel_001-material.001      (what the .glb contains)
Wheel_001-material001       (what three.js hands back)
```

Looking up the authored name alone finds **only the wheel without a dot in its
name** and misses the other three — a near-miss that is very easy to mistake
for a broken model. The loader resolves both spellings. Nothing is renamed, and
it still refuses to substitute another mesh if neither matches.

**4. Its rear-right wheel is mirrored.** `Wheel_001-material.002` hangs off
`_3DWheel_back_R`, whose scale in the file is `(-0.001, -0.001, -0.001)` — a
point **reflection**, not a rotation. The rig preserves the authored
orientation for this car rather than rebuilding it; rebuilding would turn that
wheel inside out. The BMW takes the opposite treatment, because `Object_221`
carries a baked ~6° rotation that would show as a wobble.

That mirror had a second consequence, and it took a reported symptom to find:
**on that one wheel the flat spot at the contact patch rode round with the
tyre** instead of staying at the bottom. A matrix with a negative determinant
cannot be described by a quaternion, so `getWorldQuaternion` — which
`Matrix4.decompose` implements by flipping the sign of a single scale axis —
reproduced the *matrix* correctly (the wheel drew in the right place) but
handed back an orientation that was not the node's. The deformation shader was
being told the road lay somewhere else, by as much as 180°, and the error moved
with the spin angle.

The fix is to stop going through a quaternion at all. `Vehicle._worldDirToLocal`
inverts the **linear part of the world matrix**, which is exact for a
reflection, a non-uniform scale, or any combination of them — and every corner
now uses it, so there is no special case to forget. `tests/vehicles.mjs`
section 17 measures the round trip at 24 spin angles on both rear wheels
(worst error 0.000°) and separately asserts that the quaternion route would
still be up to 180° out, so the test records what the bug *was* rather than
only that it is gone.

### Hierarchy

The authored hierarchy is preserved exactly. Each tyre keeps its children:

```
Wheel_001-material.001   (rubber tyre — the parent)
├── Wheel-material.001       rim
├── CarPaintBlack-material.011 / .013   bolts, brake disc
└── ext_logo_dark-material.002
```

Nothing is detached or reparented below the tyre, so the rim, bolts and disc
follow its rotation because they are its children — which is the point.

> One note: the rear-left wheel has **three** children in the file, not four —
> it is the one wheel with no separate rim mesh. That is how the model ships.
> The rig reports each wheel's child count at start-up rather than assuming a
> number, and does not treat it as an error.

### Interior steering wheel

This car's interior wheel **is** animated (the BMW's must not be). Its node
origin is also at the model origin, so it gets the same treatment as the
wheels: a pivot group at the measured centre of the rim.

The rotation axis was recovered by fitting a plane to the assembly's 12,707
vertices. It comes out at **22.5° from horizontal with essentially zero lateral
component** — which is exactly what a real steering column is, and a good sign
the fit found the right plane rather than some arbitrary one.

The data flow is one-way and enforced by the architecture:

```
Player input
   -> steering physics
      -> actual road-wheel angle
         ├── front left wheel
         ├── front right wheel
         └── interior steering wheel  (angle x 13.0 steering ratio)
```

The animation never feeds back. `A` turns the rim anticlockwise from the
driving seat, `D` clockwise, smoothed so it cannot snap from lock to lock.
`tests/browser.mjs` drives the car and asserts the rim turns the correct way
for each key, turns further than the road wheels, and that the *pivot
quaternion actually changes*.

## What makes it drive differently

Nothing is copied from the BMW. The differences are structural, not cosmetic:

| | BMW M3 GTR | Lamborghini Revuelto |
| --- | --- | --- |
| Engine | 3.0 turbo I6, 505 Nm, 6,750 rpm | 6.5 NA V12, 725 Nm, **9,500 rpm** |
| Induction | turbocharged — 0.28 s of lag | **naturally aspirated — 0.045 s** |
| Electric | none | **two 110 kW front motors** |
| System output | 362 PS | **1,125 PS** |
| Power / weight | 210 PS/t | **597 PS/t** |
| Drive | rear | **all-wheel** |
| Coupling | torque converter (×2.0 at stall) | **dual clutch (×1.0)** |
| Shift time | 160 ms | **90 ms** |
| Mass / balance | 1720 kg, 52 % front | 1885 kg, **44 % front** |
| Yaw inertia | 2650 kg·m² | **2200 kg·m²** (mid-engined) |
| Spring rate F/R | 34 / 39 kN/m | **68 / 96 kN/m** |
| Suspension travel | 190 mm front | **132 mm front** |
| Tyres | 0.329 m all round | **0.319 front / 0.344 rear (staggered)** |
| Peak friction | 1.25 | **1.42** |
| Downforce @200 km/h | 0.55 kN | **2.83 kN** |
| Steering lock / rack | 33° at 2.05 rad/s | **30° at 2.60 rad/s** |

Three of those needed genuinely new physics rather than new numbers.

### The electric front axle (`ElectricAxle` in `drivetrain.js`)

The Revuelto's front axle has **no mechanical connection to the engine at
all** — two independent motors, one per wheel. That gives it three things a
geared drive cannot:

1. **Full torque at zero rpm.** No clutch to slip, no gear to be in. This is
   most of why it reaches 100 km/h in 3.5 s where the rear-drive BMW needs 6.6.
2. **Constant-power fade.** Each motor is power limited, so above ~130 km/h its
   torque falls as 1/ω and the car quietly becomes rear-drive — as the real one
   does. The suite asserts the fade happens.
3. **Per-wheel torque vectoring**, shuffling torque to the outside wheel in
   proportion to steering demand.

Plus regenerative braking on a closed throttle. Torque is written to a separate
`wheel.electricTorque` so the two power paths stay legible in the telemetry.

### The dual clutch

A DCT, not an automatic, selected by `transmission.coupling: 'clutch'`. Two
things follow. There is **no torque multiplication** off the line — the suite
asserts the Lamborghini's torque ratio is exactly 1.0 at launch while the BMW's
converter is above 1.4 — and launching becomes an active control problem: the
actuator slips the clutch against a target engine speed, then squeezes shut.

Two failure modes had to be designed out, and both are the sort that look like
"the car feels wrong" rather than an error:

* **Clutch dump.** The launch controller reads "engine above target" and keeps
  biting — into wheels that have already let go. A slip guard stops it
  squeezing once the driven tyres pass their peak, which is what a real launch
  control does.
* **A 0.34 G lurch on every upshift.** A V12 at 8,400 rpm must lose nearly
  3,000 rpm to match the next gear, and cannot do that on engine friction
  inside a 90 ms shift. What absorbs it in a real DCT is the incoming clutch
  *slipping* as it takes up, so the re-engagement rate is scaled by the current
  slip — with a floor, or a large mismatch deadlocks it shut.

### Staggered tyres

20-inch front, 21-inch rear, on genuinely different radii — so the front wheels
visibly turn faster than the rears at the same road speed, which the suite
checks. This meant making the tyre radius, width and rotational inertia
per-axle rather than per-car. The BMW declares one radius for both ends and is
unaffected.

The model's four wheels do not quite share one contact plane (the rears sit
12 mm lower), which is normal for a downloaded asset. Nothing needed
correcting: each corner has its own suspension and simply finds its own ground.

## Measured results

All from `npm run test:vehicles`, on the flat skidpad:

| | BMW M3 GTR | Lamborghini Revuelto |
| --- | --- | --- |
| 0–100 km/h | 6.65 s | **3.54 s** |
| Peak longitudinal G | 0.74 G | **1.39 G** |
| Speed after 18 s | 190 km/h | **334 km/h** |
| Braking 100–0 | 35.0 m | **31.6 m** |
| Lateral G | 1.22 G | **1.40 G** |
| Body roll | 5.03° | **2.53°** |
| Roll gradient | 4.11 °/G | **1.81 °/G** |

The Revuelto's braking figure (31.6 m) is close to the real car's ~31 m, and its
roll gradient is less than half the BMW's — which is what 2× the spring rate, a
lower centre of gravity and stiffer anti-roll bars buy you.

**On the quoted 0–100:** the real Revuelto claims 2.5 s with launch control and
warm tyres. This model does 3.5 s, and **the garage says 3.5**, because every
figure it displays is read from the spec the physics actually uses or measured
by the test suite. It cannot promise something the car will not do.

---

# Garage, audio and multiplayer

**These three systems did not exist before this change.** The brief asked to
extend an existing garage, not break existing multiplayer, and to give the new
car its own engine sound — but the project was single-player with no garage UI
and no audio at all. They are new, built to match the existing architecture and
styling rather than bolted on beside it.

## Garage (`js/garage.js`)

Opens on start, and on `G` at any time. Cards for each car, a normalised
comparison across ten metrics, and a full handling profile for the selected
vehicle.

**Every number is derived from the vehicle spec the physics uses.** Power is
integrated from the engine's own torque curve, grip is the tyre model's
friction coefficient, weight distribution comes from the measured centre of
mass, and the acceleration and braking figures are the ones the tests record.
`tests/vehicles.mjs` asserts each of those correspondences, so the garage
cannot drift away from the simulation.

Switching cars mid-session respawns at the same point on the circuit, so the
two can be compared back to back from the same corner.

## Audio (`js/audio.js`)

Procedural, synthesised from the physics state — there are no audio assets to
ship and nothing to license. The engine note is a harmonic stack whose
fundamental is the engine's actual firing frequency:

```
f = rpm / 60 × cylinders / 2
```

A V12 fires six times per revolution against the BMW six's three, so at the
same engine speed its fundamental is an octave higher — and it revs to 9,500
rather than 6,750. On top of that the Revuelto gets bright high harmonics, a
much wider open filter, **no turbo content at all**, and a faint electric whine
from the front axle that no other car has. The BMW keeps its turbo whistle,
which rises with boost.

Both also carry a load-dependent noise bed, tyre scrub tied to real slip, and a
shift cut. `M` mutes. Browsers require a gesture before audio starts, so the
graph is built lazily and resumed on the first key press.

## Multiplayer (`js/network.js`, `js/multiplayer.js`)

A transport interface, a wire protocol, an authoritative local car and
interpolated remote ones. Two transports ship:

* **`BroadcastChannelTransport`** — peers are other **tabs** of the same
  browser. No server needed, and genuinely testable: open the page twice and
  the two cars see each other. This is the default, and what the automated
  test uses.
* **`WebSocketTransport`** — a real server, speaking the same JSON protocol.
  Point it at a URL; nothing else changes.

**Vehicle identity is the ID**, never the display name. A peer announces
`"lamborghini-revuelto"`, and the receiving client looks that up in the
registry to decide which GLB to load and which profile applies — so renaming a
car in the garage cannot break a session.

**Each client simulates only its own car.** Remote cars are never stepped; they
replay replicated state, interpolated 120 ms behind so jitter does not show.
That keeps local input fully responsive.

A remote car is a **full `Vehicle`**, not a simplified stand-in — same GLB, same
wheel rig, same tyre shader, same interior steering wheel, drawn by the same
`syncVisuals()`. Only its state source differs. That is deliberate: a separate,
lesser renderer for other people's cars is how you end up with wheels that do
not turn on everyone else's screen.

The snapshot carries position, orientation, both velocities, the steering rack
angle, all four wheel *angles*, suspension travel, tyre deflection, braking and
drivetrain state — **323 bytes at 20 Hz, about 6 KB/s per player**. Wheel
angles rather than angular velocities, so a dropped packet cannot leave a
remote wheel spinning at the wrong rate. Ackermann is recomputed on the
receiving end from the one rack angle rather than sending four numbers that can
be derived from one.

---

## Reflections (`js/reflections.js`)

**There is no ray tracing in WebGL.** There is no API to call, and nothing here
traces a ray. What this does instead is the technique ray tracing is usually
*wanted* for on a car — reflections that contain the actual world rather than a
static sky:

* a **cube camera rides with the car** and renders the real scene around it —
  road, kerbs, barriers, trees, the gantry, other players — into a cube map;
* three.js pre-filters that capture into a roughness pyramid, so a mirror
  finish gets a sharp reflection and rough asphalt a broad soft one, from the
  same capture;
* the bodywork and the road sample it as their environment map.

Before this, every reflective surface sampled a **static sky** IBL: the panels
picked up a gradient and nothing else, and the road could not reflect at all.
Now the Revuelto reflects the road it is on and the scenery it is passing, and
the asphalt picks up the sky, the barriers and the low sun.

Two supporting changes matter as much as the probe:

**The asphalt gained a clear coat.** Real asphalt is not a mirror, but it is
not matte either — the bitumen between the aggregate is smooth, and that is why
a road goes silver-grey at a glancing angle. One roughness value cannot express
both at once, so the stone stays rough in the base layer and the binder became
a weak clear coat (`clearcoat: 0.35`) over the top. That coat is what the world
reflects in.

**The Revuelto's paint became a two-layer material.** Car paint is a coloured
metallic base under a thin lacquer, and the lacquer is most of what you see. A
single-layer material has one blurry highlight and always reads as plastic;
`MeshPhysicalMaterial` with `clearcoat: 0.85` puts a sharp reflection over the
softer colour underneath. It is opt-in per vehicle (`paintClearcoat` in the
spec), so the BMW's appearance is untouched — it takes the probe on the road
under it, and nothing else.

### What it costs, and how to turn it down

One probe update renders the scene **six more times**. That is why the capture
is small, why it runs at a fraction of the frame rate, and why there is an off
switch:

| Level | Face size | Rate | |
| --- | --- | --- | --- |
| Off | — | — | exactly the previous renderer |
| Low | 96 px | 4 Hz | |
| High | 160 px | 10 Hz | default |

`V` cycles them and the telemetry overlay (`H`) reports the level and the
measured cost per capture. `?reflections=off\|low\|high` sets it from the URL —
the same escape hatch as `?audio=off`, and what the browser suite uses so six
extra renders a capture do not have to be paid under software WebGL.

### The honest limits

* Reflections are captured from **one point** above the car, so a reflection of
  something very close to it is not perfectly placed.
* The local car is **excluded from its own capture** — a probe sitting inside
  the shell would otherwise smear the inside of it over the paint. The
  consequence is that the car cannot see itself in a reflection.
* The capture rate is low. At 10 Hz, a barrier flashing past is a few frames
  behind in the reflection. This is invisible on paint at speed and would not
  be on a flat mirror.

---

## Architecture

```
Car-Simulation/
├── index.html
├── serve.mjs                 zero-dependency static server
├── css/style.css
├── js/
│   ├── config.js             WORLD settings: sim rate, surfaces, camera, render
│   ├── vehicles/
│   │   ├── registry.js       id -> spec; what multiplayer resolves against
│   │   ├── spec.js           shared derivation: body frame, axles, static loads
│   │   ├── bmw.js            BMW M3 GTR — tuning moved here verbatim
│   │   └── lamborghini.js    Lamborghini Revuelto — its own complete profile
│   ├── mathUtils.js          Vec3 / Quat / Mat3, noise — no three.js dependency
│   ├── terrain.js            circuit spline + analytic surface (physics + mesh)
│   ├── tirePhysics.js        pressure, temperature, slip, Magic Formula
│   ├── suspension.js         spring / damper / bump stops / anti-roll bar
│   ├── wheelPhysics.js       contact solve, patch frame, spin dynamics
│   ├── steering.js           rack, speed sensitivity, Ackermann
│   ├── drivetrain.js         engine, converter OR dual clutch, gearbox, LSD,
│   │                         and the electric drive axle
│   ├── brakes.js             hydraulics, ABS, traction control
│   ├── collision.js          chassis hull vs barriers, poles and terrain
│   ├── vehiclePhysics.js     rigid body + the fixed-step pipeline
│   ├── vehicle.js            GLB load, validation, wheel + steering rig, sync
│   ├── environment.js        sky, IBL, road/terrain meshes, scenery
│   ├── reflections.js        real-time cube probe: the world in the paint
│   ├── camera.js             chase / side / bonnet / interior / orbit,
│   │                         per-vehicle framing
│   ├── input.js              keyboard + gamepad
│   ├── hud.js                cluster, G-meter, tyre telemetry, overlays
│   ├── debug.js              telemetry panel + 3D gizmos
│   ├── garage.js             vehicle selection and comparison
│   ├── audio.js              procedural engine audio, per-vehicle voice
│   ├── network.js            transport, protocol, vehicle state codec
│   ├── multiplayer.js        remote players' cars
│   └── main.js               bootstrap and the fixed-timestep loop
├── vendor/                   three.js and gsap, vendored (see Licensing)
├── assets/
│   ├── BMW.glb
│   └── lamborghini-revuelto.glb
└── tests/
    ├── run.mjs               BMW physics regression suite (127 assertions)
    ├── vehicles.mjs          multi-vehicle suite (155 assertions)
    ├── browser.mjs           end-to-end check in headless Chrome
    └── perf.mjs              per-vehicle render cost
```

### Adding a vehicle

Three steps, and nothing outside them needs to change:

1. Measure the model and write `js/vehicles/yourcar.js` — a spec object with
   its geometry, chassis, suspension, tyres, engine, gearbox, brakes, steering,
   garage card, camera framing and audio voice.
2. Add one import and one line to `js/vehicles/registry.js`.
3. Run `npm run test:vehicles`.

The garage, the comparison bars, multiplayer identity, audio and the camera all
pick it up from the registry.

**The physics layer has no three.js dependency.** `mathUtils.js` provides its
own `Vec3`/`Quat`/`Mat3`, which means (a) the entire simulation can be driven
from Node for testing, and (b) no renderer state can leak into the physics
state. The graphics read the physics; the reverse never happens.

### Fixed timestep

Physics runs at a fixed **240 Hz** behind an accumulator, completely decoupled
from the render rate. The leftover time becomes the interpolation factor for
drawing, so the car looks smooth at 60, 120 or 144 fps and behaves identically
on all of them.

The brief suggests ~1/60 s. 240 Hz is used instead because the wheel rotational
dynamics are stiff: first gear puts ~6800 N·m through a 1.28 kg·m² wheel, which
is 5400 rad/s². At 60 Hz that is a 90 rad/s jump per step; at 240 Hz it is a
well-behaved 22 rad/s. The rate is a single constant in `config.js` and the
model is stable across it — the suite measures a **3.1 % speed difference, a
3.8 % 0–100 difference and a 4.2 % braking-distance difference between 60 Hz
and 240 Hz**, with 120/240/480 Hz agreeing to within 1 %.

That robustness is not free. Five separate terms had to be made
timestep-independent, and each one was found by watching a specific number
diverge at 60 Hz:

| Term | Treatment |
| --- | --- |
| Tyre relaxation | implicit — its decay rate is 125 s⁻¹ at 40 m/s, past the explicit limit |
| Limited-slip differential | semi-implicit; explicitly it diverges below ~150 Hz |
| Lock-up clutch | semi-implicit |
| Wheel spin ↔ tyre force | linearised about the tyre's own slope, so it damps instead of exploding |
| Brake torque | shares that implicit denominator; explicitly, one 60 Hz step took 40 % off the wheel's speed and lengthened the stop by a fifth |
| ABS controller | runs on its own fixed 100 Hz pump clock, integrating over elapsed time |

---

## Testing

### Physics — `npm test` and `npm run test:vehicles`

```bash
npm test               # tests/run.mjs      — 127 assertions, pins the BMW
npm run test:vehicles  # tests/vehicles.mjs — 155 assertions, both cars
npm run test:browser   # tests/browser.mjs  — end-to-end in headless Chrome
npm run test:all       # all three
```

`tests/run.mjs` is 20 scenarios and **127 assertions** covering every item on
the original brief's closing checklist. It is also the guarantee that adding a
second car changed nothing about the first: it still passes, with figures
identical to the digit.

`tests/vehicles.mjs` is **155 assertions** across 17 sections covering
everything the second car brought with it — the registry and vehicle identity,
the Lamborghini's measured geometry against the actual `.glb`, its physics at
rest and at the limit, the electric axle, the dual clutch, staggered tyres,
timestep independence for the new drivetrain paths, the network state codec,
and a section that asserts the two cars genuinely **do not** drive alike.
Section 17 is the mirrored-wheel guard described above: it calls the real
`Vehicle._worldDirToLocal` on a rebuild of the actual node chain from the .glb,
so it fails if that method is ever changed back to a quaternion. Two fixtures are used: a flat infinite skidpad for vehicle dynamics
(so the numbers are reproducible and not contaminated by track geometry), and
the real circuit for anything about the world.

Representative measured results:

| | |
| --- | --- |
| Static load split | 52.0 % front / 48.0 % rear |
| 0–100 km/h | 6.6 s, 91 m |
| Peak longitudinal G | 0.74 G (traction limited) |
| Braking 100–0 km/h | **35.0 m**, peak 1.24 G, mean 10.9 m/s² |
| ABS vs locked wheels | 35.0 m vs 45.1 m |
| Peak lateral G | 1.22 G |
| Body roll | 5.0° at 1.22 G ≈ 4.1 deg/G |
| Idle creep | 5.7 km/h |
| Wet vs dry grip | 0.85 G vs 1.21 G |
| Tyre pressure vs grip | 18 psi 1.09 G · 32 psi 1.21 G · 44 psi 1.16 G |
| Physics cost | 0.023 ms/step → 5.5 ms of CPU per simulated second |

The suite asserts *relationships*, not just magnitudes: that ABS stops shorter
than locked wheels, that the inner wheel steers more than the outer, that soft
tyres deform more and need more slip angle, that the outer wheels turn faster
than the inner ones in a corner, that locking the rears destroys their lateral
grip, that a flat surface produces far less suspension movement than the broken
one.

### End-to-end — `npm run test:browser`

```bash
npm run test:browser                   # headless Chrome
npm run test:browser -- --screenshots   # also writes tests/screenshots/
```

It runs the whole flow for **both** cars: garage, model load, wheel validation,
rig hierarchy, driving, steering, braking and the interior steering wheel —
then opens a **second tab** and checks the two see each other.

For each car it verifies that the GLB loads, that all four named objects are
found and assigned to the right corners, that each keeps its children, that the
rig hierarchy is `tyre < Spin < Camber < Steer < Hub`, that the car
accelerates, steers and brakes on real key presses, that the front wheels steer
and the rear ones do not, and that ABS modulates under braking.

Several checks are worth calling out:

* **The interior never moves.** It snapshots the transform of every interior
  mesh, drives hard with full steering lock, and asserts none of them changed.
  Run for **both** cars — the Lamborghini's animated steering wheel lives in
  its own pivot outside the body model, so its interior meshes must be just as
  still as the BMW's.
* **The interior steering wheel turns the right way.** For the Revuelto it
  drives right, then left, and asserts the rim rotates in opposite directions,
  turns further than the road wheels do, and that the pivot quaternion actually
  changes. For the BMW it asserts there is no steering-wheel animation at all.
* **The visual wheel exactly follows the physics.** It samples the whole
  braking event and asserts `hub.y === restHubY + suspensionTravel − tyreSquash`
  every time, to the last bit. Measured error: **0.00e+0 m**.
* **The flat spot stays at the road on every wheel.** Sampled from inside
  `syncVisuals`, because that is the only instant at which the shader's contact
  direction and the wheel's transform describe the same moment — between frames
  at 90 km/h the wheel has already turned more than a full revolution, which
  makes a naive sample look catastrophically wrong. The comparison is made in
  the wheel plane, since the shader removes the axial component deliberately
  and measuring against the raw road normal would only report camber. Measured:
  **0.000°**. This is the live guard on the mirrored rear right.
* **The cockpit camera is really in the cockpit.** It asserts the eye is within
  1.2 m of the centre of mass, at 0.75–1.5 m above the road, and using the
  narrower cockpit field of view — a camera that drifted outside the shell
  would still *look* plausible in a screenshot.
* **Reflections are wired, not just enabled.** It asserts the probe is
  capturing, that the road and the Revuelto's panels sample *the probe's*
  texture rather than the sky, that the paint really is clear-coated, and that
  switching reflections off puts the sky environment back rather than leaving
  the panels with no environment at all.
* **Multiplayer actually works.** Two tabs, different cars. It asserts each
  side loaded the *other* car — chosen by vehicle ID — that the remote car
  moves, that its **wheels roll** and its **front wheels steer** on the other
  screen, and that closing a tab cleans the peer up.

---

## Performance

* Physics: ~5 ms of CPU per simulated second at 240 Hz per car, allocation-free
  in the hot path (every vector operation has an in-place variant and all
  scratch objects are pre-allocated). Only the local car is simulated; remote
  cars cost a state write and a visual sync.
* **Draw calls.** The Lamborghini's GLB is 829 separate meshes — 2,628 draw
  calls a frame once shadows are counted, which halved the frame rate on its
  own. Two fixes, both opt-in per vehicle so the BMW is untouched:

  | | before | after |
  | --- | --- | --- |
  | Draw calls | 2,628 | **369** |
  | Meshes | 829 | 93 |
  | Shadow casters | 829 | 43 |

  Static bodywork is batched by material *after* rigging, so the four tyres and
  the steering wheel — which need their own transforms and whose hierarchy has
  to survive — are never merged. And only parts at least 10 cm across cast
  shadows; smaller ones still receive them, they just are not drawn a second
  time into the shadow map for a result nobody can see through the glass.
* Both GLBs are parsed **once** and cloned per instance, so a multiplayer
  session with four Revueltos does not re-download 20 MB four times.
* **Reflections cost six extra scene renders per capture.** That is why the
  probe captures at 96–160 px rather than a useful texture size, and at 4–10 Hz
  rather than every frame. Measured on the CI machine's *software* renderer
  (which is the worst case by a wide margin): 4.6 ms a capture at Low, and
  under 6 ms at High once the batched body is in view. `V` and
  `?reflections=off` both take it out of the picture entirely.
* Terrain queries are analytic with a spatial hash on the centreline —
  ~3.5 µs per full surface sample including the finite-difference normal.
* Scenery is instanced (barriers, posts, trees), keeping draw calls low.
* One tightly-wrapped directional shadow cascade follows the car rather than
  trying to cover the whole 1.4 km circuit.
* Textures are generated procedurally at start-up, so there is nothing to
  download.

If you need more headroom: drop `SIM.physicsHz` to 120 (still well within
tolerance), lower `RENDER.shadowMapSize`, or reduce `RENDER.maxPixelRatio`.

---

## Licensing and third-party assets

| Component | Licence |
| --- | --- |
| three.js r185 (`vendor/three/`) | MIT |
| GSAP 3.13 core + CSSPlugin (`vendor/gsap/`) | GreenSock standard "no charge" licence |
| `assets/BMW.glb` | supplied by the project owner — not authored here |
| Sky, IBL and every texture | generated procedurally by this project |

**No HDRI is bundled, and that is deliberate.** The brief rightly warns not to
assume a random HDRI found online may be redistributed. Rather than ship
something with an uncertain licence, the sky — and the image-based lighting
derived from it — is generated at run time by an analytic sky shader and
pre-filtered with three.js's own `PMREMGenerator`. The project therefore ships
with **zero third-party image assets**.

If you would rather use a real capture, the code path exists:

```js
// after the environment is built
await sim.environment.loadHdri('assets/hdri/your_sky_2k.hdr');
```

Good sources with clear licences are Poly Haven (CC0) and HDRI Skies. Drop the
file into `assets/hdri/` and record its source and licence there.

Every texture in the scene — asphalt with lane markings, kerbs, gravel, grass —
is drawn into a canvas at start-up. Noise is made seamless by blending four
offset copies of the field, so nothing shows a tiling seam, and the road
texture's tile length is chosen to divide the lap exactly so the markings meet
where the circuit closes.

---

## Tuning

World settings live in `js/config.js`; per-vehicle tuning lives in
`js/vehicles/bmw.js` and `js/vehicles/lamborghini.js`, grouped and commented.
A few starting points:

| Want | Change |
| --- | --- |
| More grip | `TYRE.muNominal` |
| Sharper turn-in | `TYRE.peakSlipAngle` down, or `TYRE.relaxLateral` down |
| Less body roll | `SUSPENSION.*.antiRollRate` up |
| More oversteer | rear `antiRollRate` up, or front down |
| Softer ride | `SUSPENSION.*.springRate` down (damping scales as √k) |
| Snappier gearbox | `TRANSMISSION.shiftTime` down |
| Easier to drive | leave ABS and TC on; `STEERING.highSpeedScale` up |
| More dramatic camera | `CAMERA.modes.*.gForceGain` |

After a change, run `npm test && npm run test:vehicles` — the suites are
written to catch a tuning change that breaks a physical relationship, not just
one that breaks the code. `tests/vehicles.mjs` will also tell you if a change
has quietly made the two cars drive alike.

---

## Known limitations

Stated plainly rather than hidden:

* **The garage, the audio and the multiplayer are new, not extensions.** The
  brief asked to extend an existing garage, keep an existing multiplayer
  working, and give the new car a different engine sound from the old one — but
  the project was single-player with no garage UI and no audio at all. All
  three are built here, matching the existing architecture and styling. There
  was no existing implementation to preserve or break.
* **Multiplayer is peer-to-peer between browser tabs by default.** That is a
  real, testable session layer — and the automated test drives one tab and
  watches the other — but it is not a hosted server. `WebSocketTransport` is
  implemented and speaks the same protocol; it needs a server process to talk
  to, which is outside what was specified. There is no authority arbitration or
  anti-cheat, because with no server there is nothing to arbitrate.
* **The Lamborghini reaches 100 km/h in 3.5 s, not the factory's 2.5 s.** The
  model is grip-limited at about 1.4 G; the real car uses launch control and
  warmed track rubber. The garage quotes the measured figure, not the brochure.
* **Remote cars are visually replicated, not simulated.** They replay position,
  orientation, wheel angles, steering and suspension. They do not collide with
  your car — collision is local-only, because resolving it without a server
  authority would let two clients disagree about what happened.
* **The reflections are not ray tracing.** WebGL has no hardware ray tracing to
  call. What is here is a real-time cube probe that captures the actual scene
  and pre-filters it — the look ray tracing is wanted for on a car, by a
  different route. Its limits are listed under
  [Reflections](#reflections-jsreflectionsjs): one capture point, ten captures
  a second, and the car excluded from its own reflection.

* **The chassis is rigid.** BeamNG's defining feature is a soft-body node/beam
  chassis that deforms on impact. That is out of reach in a browser at
  interactive rates, so this is a rigid body with an impulse-based hull for
  collisions. The *driving* behaviour is the target here, not the crash
  deformation.
* **Suspension geometry is a vertical strut.** Camber and caster are static
  parameters; there is no camber gain through travel and no roll centre
  migration. A double-wishbone linkage solve would add both.
* **Camber generates no thrust.** It is a visual and geometric parameter only —
  including a camber thrust term with the wrong sign would be worse than
  omitting it.
* **The road mesh is tessellated at 1.25 m**, so the sub-centimetre micro-bumps
  in the height field are sampled analytically by the tyres but not fully
  resolved visually. Macro geometry — elevation, banking, kerbs — is exact.
* **No post-processing stack.** Tone mapping (ACES), MSAA and IBL are on, but
  there is no bloom or SSAO; the frame budget was spent on physics rate and
  shadow quality instead.
* **Aerodynamics are a simple drag + downforce model** with no yaw sensitivity.
