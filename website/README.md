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

1. Immediate RoboGUI background with a persistent “Real robot experiments. One inference harness.” headline.
2. A 23.2-second synthetic UI orbit loops with a 0.8-second seam dissolve. No wall
   pullback, title takeover, or timed headline fade in the hero; those remain in the film.
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
- `hero-robogui.mp4` is a 720p / 30 fps background loop from the bundled synthetic
  RoboGUI recording. It shows preview motion, not physical execution or model inference.
- `product-demo-hook.mp4` is the reordered 92-second, 1440p / 30 fps product film.
  Its synthetic GUI and illustrative evaluation sequence remain disclosed. Original
  source intervals are [74,85), [0,74), [85,92); the white closing card stays last.
  The original film is preserved in recordings/agent-demo-v5-1440p/.
- Paper authors, paper URL, benchmark results and unverified real deployments are
  deliberately not invented. Add verified publication information when available.

For final recordings, use `.agents/skills/manimux-wall-capture/SKILL.md`. Require
continuous footage showing the physical robot, actual model identity and inference
activity, with model-generated trajectories; retain capture manifests. Replace the
hero loop, gallery metadata and wall composition together. Remove the repeated-frame
caption only when it is no longer true. Keep recorded footage labeled as recorded.

## Performance and motion

- The homepage decodes a single 720p / 30 fps clip. The full 2K film has no source
  until clicked. The separate archived real-robot excerpt remains in the gallery.
- At most one video plays at a time. Hero pauses off-screen, on page hide, and while
  the dialog is open. Dialog media unloads on close. No 36-stream video decoding.
- The headline stays visible throughout playback; no per-frame JavaScript runs.
  Chip animations run once. Deployment text types at 20 ms per character.
- Reduced-motion preferences disable smooth scrolling, typing and chip animation,
  and prevent initial autoplay. Data Saver prevents autoplay.
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

## Recutting the opening

Source: `scripts/media/recut-product-hook.py` (Python with Pillow, FFmpeg). Run under
`.agents/skills/manimux-demo-video/scripts/throttle.py` with the same one-core,
60% duty cycle, 1536 MiB / zero-swap systemd scope described by the video skill.
The script encodes sequential segments, concatenates them, checks full decoding,
and verifies 2560 × 1440, 30 fps, 2760 frames / 92 seconds. `--hook-only` reuses the
existing story and closing segments when refining only the opening title.

## Continuous hero background

`scripts/media/render-hero-loop.py` produces the background and poster from the
bundled 24-second RoboGUI clip. Run under the same video throttle/systemd limits.
Only the wrap seam is dissolved; the orbit retains its original speed. The full
film is unchanged by this hero update and still contains its earlier wording.
