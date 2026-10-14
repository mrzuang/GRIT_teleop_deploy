# GRIT interactive MuJoCo demo

Static browser demo for [GRIT](https://github.com/mrzuang/GRIT_teleop_deploy). Open `index.html` through HTTP(S); `demo.html` provides the same interface.

- MuJoCo 3.12 WASM and the original GRIT model_99000 ONNX run entirely in the browser.
- AI fighting: different warmups, then approach; 30% guard / 35% kick / 35% punch selection, opponent-facing ground motions and calibrated kick windup/spacing, physical hit feedback, knockdown or 60-second points decision.
- Single-robot stage: 39 curated actions with previous/next and direct selection.
- Enter starts/pauses, Space resets positions, R resets the round, P selects AI fighting, V selects the showcase, and M toggles audio.

The default site address after Pages is enabled is https://mrzuang.github.io/GRIT_teleop_deploy/ .

## Enable GitHub Pages

In the repository's **Settings → Pages**, select **Deploy from a branch**, choose **gh-pages**, choose **/(root)**, and save. This branch contains only the static website and its runtime assets.

All JavaScript, WASM, ONNX and model assets are served locally from this branch. First loading the fighting arena transfers about 73 MB; the single-robot stage loads additional assets when selected. Use Chrome/Edge with graphics acceleration. Quality settings include auto, smooth and high.

See [credits.html](credits.html) and `vendor/*LICENSE*` for provenance and runtime licenses. Original motion sources retain their own terms. `deployment.json` records the content hashes of this release.
