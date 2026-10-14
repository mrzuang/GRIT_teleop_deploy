# GRIT interactive MuJoCo demo

Static browser demo for [GRIT](https://github.com/mrzuang/GRIT_teleop_deploy). Open `index.html` through HTTP(S); `demo.html` provides the same interface.

- MuJoCo 3.12 WASM and selectable GRIT model_99000, SONIC v1.1 G1, and HEFT G1_PMG controllers run entirely in the browser. Choose each fighter’s model in the left-side pre-match panel; all nine pairings are supported. Choices lock and the panel hides when the match starts, including during pauses and highlights. R opens a new round for changing models. The showcase keeps its own model selector.
- AI fighting: different warmups, then approach; 20% guard / 40% kick / 40% punch selection, opponent-facing ground motions and calibrated kick windup/spacing, physical hit feedback, knockdown or 60-second points decision.
- Manual fighting: C takes over red while blue remains AI. WASD moves, Q/E turns, F faces the opponent; J/K/L punches, hold H to guard, U/I/O kicks, Y taunts, B warms up. Complete keyboard guide is at the bottom right. Manual kicks allow misses; buffered attacks follow the landing.
- Anonymous global page-view and completed-match totals appear below the title, starting with the 2026-10-14 counter release. Replays, pauses and position resets do not count as new matches. No login, player profile or ranking is required.
- Follow camera only. T / Highlights replays recorded hit and KO poses at 0.25x, 0.5x or 1x, pausing live physics and scoring until playback ends.
- Per-model competition action eligibility is checked in the actual browser simulator; the full showcase library remains available for every controller.
- Single-robot stage: 37 curated actions (turning back kick and spinning kick removed) with previous/next and direct selection.
- Enter starts/pauses, Space resets positions, R resets the round, C selects manual fighting, P selects AI fighting, V selects the showcase, and M toggles audio.

The default site address after Pages is enabled is https://mrzuang.github.io/GRIT_teleop_deploy/ .

## Enable GitHub Pages

In the repository's **Settings → Pages**, select **Deploy from a branch**, choose **gh-pages**, choose **/(root)**, and save. This branch contains only the static website and its runtime assets.

All JavaScript, WASM, ONNX and model assets are served locally from this branch. First loading the fighting arena transfers about 73 MB; the single-robot stage loads additional assets when selected. SONIC loads an additional 167 MB on first selection; HEFT adds about 24 MB. Model sessions are reused after loading. Original FP32 weights are retained; SONIC uses an output-equivalent G1 encoder subgraph and a batched decoder. HEFT uses its matching nopos observation configuration. Use Chrome/Edge with graphics acceleration. Quality settings include auto, smooth and high.

See [credits.html](credits.html) and `vendor/*LICENSE*` for provenance and runtime licenses. Original motion sources retain their own terms. `deployment.json` records the content hashes of this release.
