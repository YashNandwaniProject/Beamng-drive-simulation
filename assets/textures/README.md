# Textures

**This folder is intentionally empty.**

Every texture in the scene is generated procedurally into a canvas at start-up
by `js/environment.js`:

| Texture | Function | Contents |
| --- | --- | --- |
| Road surface | `makeRoadTexture()` | asphalt aggregate, polished wheel tracks, a tar seam, solid edge lines and a dashed centre line — all in one map |
| Road roughness | `makeRoadRoughness()` | paint and worn tracks read smoother than raw grain |
| Kerbs | `makeKerbTexture()` | red/white blocks with grime |
| Gravel run-off, grass | `makeNoiseTexture()` | seamless fBm between two colours |

Two details make them usable rather than merely present:

* **Seamless tiling.** Value noise is not periodic, so a naive texture shows a
  hard seam wherever it wraps — on a road that repeats every 16 m that is a
  line straight across the carriageway. `seamlessFbm()` blends four copies of
  the field, each offset by exactly one tile, which makes opposite edges
  identical by construction.
* **The road texture's U axis spans the road width exactly once** (clamped
  rather than repeated), so the lane markings follow the carriageway as it
  narrows through the hairpin and widens on the banking. Its tile length along
  the road is chosen so a whole number of tiles fits the lap, which is what
  makes the markings meet where the circuit closes.

## Using bitmaps instead

Drop images here and swap the `make*Texture()` calls for `THREE.TextureLoader`.
Set `colorSpace = THREE.SRGBColorSpace` on colour maps (but *not* on roughness
or normal maps), keep `wrapS`/`wrapT` as they are, and set a sensible
`anisotropy`.

Record the source and licence of anything you add:

| File | Source | Licence | Added |
| --- | --- | --- | --- |
| _(none)_ | | | |
