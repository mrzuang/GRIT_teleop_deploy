# Third-party software and assets

The native G1 bridge vendors source snapshots so it can be built without
fetching dependencies at configure time. Version pins are listed in
`g1_sim2real/third_party/VERSIONS.md`.

| Component | Upstream | License in this repository |
| --- | --- | --- |
| Unitree SDK2 | https://github.com/unitreerobotics/unitree_sdk2 | `g1_sim2real/third_party/unitree_sdk2/LICENSE` |
| yaml-cpp | https://github.com/jbeder/yaml-cpp | `g1_sim2real/third_party/yaml-cpp/LICENSE` |
| zlib | https://github.com/madler/zlib | `g1_sim2real/third_party/zlib/LICENSE` |

The G1 MuJoCo description and meshes under `sim2real/config/g1/assets/` are
derived from Unitree G1 robot descriptions. Review the upstream Unitree terms
before redistributing those assets independently.

PICO teleoperation uses XRoboToolkit. Its PC service and Python binding are not
vendored; `sim2real/install_xrobottoolkit_sdk.sh` obtains them from:

- https://github.com/XR-Robotics/XRoboToolkit-PC-Service
- https://github.com/Axellwppr/XRoboToolkit-PC-Service-Pybind

Those projects remain subject to their own licenses and distribution terms.

