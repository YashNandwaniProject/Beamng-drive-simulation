# HDRI environments

**This folder is intentionally empty.**

The simulation does not ship an HDRI. A random environment map found online is
very unlikely to be licensed for redistribution, so rather than bundle
something with an uncertain licence, `js/environment.js` generates the sky
analytically at run time and pre-filters it with three.js's own
`PMREMGenerator` to produce the image-based lighting. The project therefore
contains no third-party image assets at all.

## Using a real capture instead

Drop a `.hdr` (equirectangular; 2k is plenty) in this folder and call:

```js
await sim.environment.loadHdri('assets/hdri/your_sky_2k.hdr');
```

`Environment.loadHdri()` swaps both the background and the IBL and hides the
procedural sky. It is already wired up and uses the vendored `HDRLoader`.

## Sources with clear licences

| Source | Licence |
| --- | --- |
| <https://polyhaven.com/hdris> | CC0 — public domain, no attribution required |
| <https://ambientcg.com> | CC0 |
| <https://hdri-skies.com> | free tier; check each file's terms |

**Record whatever you add here**, with its source URL and licence, so the
project's asset provenance stays auditable:

| File | Source | Licence | Added |
| --- | --- | --- | --- |
| _(none)_ | | | |
