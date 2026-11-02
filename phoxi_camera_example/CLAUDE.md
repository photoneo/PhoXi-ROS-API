<!-- claude-docs-generated-from: ce298b2d6bc0ae46b8fe8e0cb9de5eeb6f08e5c7 @ 2026-10-14T11:50:04Z -->
_Last updated: 2026-10-14 (ce298b2)_

# phoxi_camera_example

Client code showing how to drive the `phoxi_camera` driver. How to run each example is in [README.md](README.md).

| Area | Sources | Launch file | What it shows |
|------|---------|-------------|---------------|
| Simple node | `src/simple_node/simple_node_service_caller.cpp`, `simple_node_topic_listener.cpp` | `launch/simple_node/run_simple_node_example.launch.py` | Lifecycle transitions over services, `trigger_frame`, receiving `points`; runs with RViz and a static TF. |
| Combined point cloud | `src/simple_node/combined_point_cloud_*.cpp` | `launch/simple_node/run_combined_point_cloud_example.launch.py` | The same flow with `publish_combined` and the single `point_cloud` topic. |
| Settings | `src/settings/settings_example_caller.cpp` | `launch/settings/run_settings_example.launch.py` | `frame_settings.*` / `device_settings.*` overrides before configure and live parameter changes. |
| Composition | `include/phoxi_camera_example/composition/composition_example.h`, `src/composition/` | `launch/composition/run_composition_example.launch.py` | `CompositionExample`, a component loaded with the driver into one `component_container_mt` for intra-process transfer; it re-triggers after each frame and logs rate and timing. Also built as a standalone executable. |
| Profiles | `src/profiles/list_profiles.cpp`, `manage_profiles.cpp`, `export_import_profiles.cpp` | none (run with `ros2 run`) | Calls to the `~/profiles/*` services. |

The simple-node, combined and settings launch files default `sensor_sn` to a file camera bundled with PhoXi Control, so they run without hardware; the composition example is brought up and connected by hand, as described in its launch file. RViz configurations for the examples live in `rviz/`.
