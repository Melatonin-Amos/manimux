# ManiMux landing page

English, buildless HTML/CSS/JavaScript. No application dependencies, external fonts,
analytics, runtime services, GPU rendering, or robot connection.

## Preview

From the repository root:

```bash
python3 -m http.server 18087 --bind 127.0.0.1 --directory website
```

Open http://127.0.0.1:18087/. The existing documentation and GitHub Pages workflow
are not modified by this landing page.

## Static hosting

```bash
node --check website/app.js
python3 website/build.py
```

Deploy `website/dist/` to any static host. `.openai/hosting.json` identifies the
private Sites preview. The publication helper operates on an isolated source copy
at `.local/sites/manimux-landing/`, not on the parent repository's dirty checkout.
Copy the latest website sources there before a later Sites publication; preserve
that copy's `.git` and use its existing project identity. Never create a second Site
for the same page.

The source build is dependency-free. A host with HTTP range support is recommended
for native video seeking. Python's simple server is sufficient for local preview.

## Page sequence

1. Three-axis headline and archived physical-robot RoboGUI excerpt.
2. Scroll-driven pullback into a 4 × 4 wall, using one video and repeated stills.
3. Embodiment / policy / inference chips with an entrance animation and factual
   per-integration descriptions. These are independent selectors, not a combination
   compatibility checker. Current compatibility is linked to the support catalog.
4. Deploy / Run / Review / Score & save tabs. Deployment text types on first view.
   Example scoring is held in page memory until reload; it never writes an experiment.
5. Recording gallery, full film player, research title, code and documentation links.

## Media truth and replacement

`assets/media.json` records source paths, trims and provenance boundaries.

- `execution.mp4` is a 12-second, original-speed excerpt of the repository's archived
  real-robot demo, transcoded from 1080p to 720p / 30 fps. It is **not** verified as
  a new capture satisfying the contributor skill's full inference/provenance contract.
- The wall repeats four frames from that same archive. Its caption explicitly says
  so; it is not evidence of 16 distinct runs, robots or policies.
- `product-demo.mp4` is the existing 92-second, 1440p / 30 fps product film. Its
  synthetic GUI and illustrative human-evaluation sequence remain disclosed.
- Paper authors, paper URL, benchmark results and unverified real deployments are
  deliberately not invented. Add verified publication information when available.

For final recordings, use `.agents/skills/manimux-wall-capture/SKILL.md`. Require
continuous footage showing the physical robot, actual model identity and inference
activity, with model-generated trajectories; retain capture manifests. Replace the
hero excerpt, gallery metadata and wall stills together. Remove the repeated-frame
caption only when it is no longer true. Keep recorded footage labeled as recorded.

## Performance and motion

- The 720p hero is about 435 KB. The 9.6 MB full film has no source until clicked.
- At most one video plays at a time. Hero pauses off-screen, on page hide, and while
  the dialog is open. Dialog media unloads on close. No 16-stream video decoding.
- Scroll work uses one queued animation frame per scroll event; there is no
  perpetual rendering loop. Chip animations run once.
- Reduced-motion preferences disable scroll choreography, smooth scrolling, typing
  and chip animation, and prevent initial autoplay. Data Saver prevents autoplay.
- The explicit pause control stops video playback; static content remains usable.
- Fonts are local Manrope / JetBrains Mono; OFL notices are included.

## Browser checks performed

Desktop at the default browser viewport and mobile at 390 × 844 were visually
checked. Verified architecture selection, off-screen video pause, score selection
and session-only saving, state retention between workflow tabs, arrow-key tab
navigation, film playback, Escape dismissal, source release on close, and no
horizontal overflow on mobile. Browser console reported no errors in these checks.
Reduced-motion behavior is implemented with a media query; OS preference switching
was not exercised in this session.
