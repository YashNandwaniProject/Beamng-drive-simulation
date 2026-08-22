# Road data

**This folder is intentionally empty.**

The circuit is not a mesh asset. It is defined as *data* at the top of
`js/terrain.js` — a list of control points carrying position, elevation,
banking, width, surface type and roughness, interpolated with a closed
Catmull–Rom spline and resampled to a uniform arc length.

That matters for more than tidiness. Because the road is an analytic height
field, **`environment.js` tessellates the visible mesh by calling exactly the
same `Terrain.heightAt()` the tyres raycast against.** There is no separate
collision mesh that could drift out of alignment: the banking you can see *is*
the banking the suspension feels, and the kerb the wheel climbs is the kerb
that is drawn.

## Editing the circuit

Edit `CONTROL_POINTS` in `js/terrain.js`:

```js
{ x: 262, z: -36, y: 4.0, bank: -15, halfWidth: 6.5, surface: 'ASPHALT' },
```

| Field | Meaning |
| --- | --- |
| `x`, `z` | world position of the centreline |
| `y` | centreline elevation |
| `bank` | cross-slope in degrees. **Negative banks a right-hand turn** (it raises the left, outside edge) |
| `halfWidth` | half the asphalt width |
| `surface` | a key from `SURFACES` in `config.js` — `ASPHALT`, `WET`, `GRAVEL`, … |
| `bumpAmp`, `bumpWave` | optional washboard ripples, for a deliberately broken section |

Everything downstream regenerates from those points: the road mesh, kerbs,
run-off, barriers and their colliders, light poles, the start gantry, the
terrain blend around the corridor, and the physics surface itself.

The current layout is a 1.37 km loop with a long straight, a 16° banked
right-hand sweeper, a crest, an ess, a washboard section, a tight hairpin and
a wet-asphalt patch.

## After editing

Run `npm test`. Several tests locate features by fraction of lap distance (the
banked corner at 40 %, the broken section at 78 %, the wet patch at 50 %) and
will tell you plainly if the layout has moved out from under them. `SPAWN`
in `config.js` sets where the car starts, as a distance along the centreline.
