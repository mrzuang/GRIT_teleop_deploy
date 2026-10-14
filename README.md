# GRIT interactive MuJoCo demo

Static browser demo for [GRIT](https://github.com/mrzuang/GRIT_teleop_deploy). Both index.html and demo.html open the same interactive arena.

- Both fighters and the single-robot showcase use the original GRIT model_99000. MuJoCo 3.12 WASM and ONNX Runtime run in the browser.
- Manual fighting, AI fighting and the complete 37-action showcase are available. R restarts, Space resets positions, C selects manual fighting, P selects AI fighting, V selects the showcase, and Enter starts or pauses.
- Default third-person camera stays behind the red robot. Follow view stays perpendicular to the line between the fighters. Use the wheel or pinch to zoom. Camera buttons use larger text.
- T / Highlights replays recorded hit and KO poses at 0.25x, 0.5x or 1x while pausing live physics and scoring.
- Matches start with warmups, then use 20% guard / 40% kick / 40% punch selection. Physical head, body and thigh hits score; knockdown wins immediately, otherwise points decide after 60 seconds.
- Shared anonymous view and completed-match counters start from the 2026-10-14 counter release. No login or player details are requested.

Large resources load in parallel and are cached by SHA256 in browser Cache Storage. The unchanged binary physics scene loads through MEMFS, avoiding the old per-byte conversion. Local Chrome measurements improved from about 28.6 seconds to 3.1 seconds on first load and 1.9 seconds with the asset cache; first internet downloads still depend on network speed. No simulation geometry or policy weights were changed by this optimization.

Public demo: https://mrzuang.github.io/GRIT_teleop_deploy/ . This gh-pages branch contains the static website and runtime assets. GitHub Pages serves its root directory.

See [credits.html](credits.html) and vendor licenses for provenance. deployment.json records this release's content hashes.
