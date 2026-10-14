<!-- claude-docs-generated-from: ce298b2d6bc0ae46b8fe8e0cb9de5eeb6f08e5c7 @ 2026-10-14T11:50:04Z -->
_Last updated: 2026-10-14 (ce298b2)_

# PhoXi-ROS-API

ROS 2 driver for Photoneo PhoXi / MotionCam / Alpha 3D sensors, a colcon workspace of six ament packages. The driver is a lifecycle node that talks to the device through PhoXi Control (the PhoXi API) and publishes point clouds, images and per-frame metadata. Device settings are exposed as ROS 2 parameters. User-facing documentation (installation, topics, services, parameters, Docker setup) is in [README.md](README.md); this file covers how the code is organised.

## Packages

| Package | Role | Depends on |
|---------|------|------------|
| `phoxi_camera_vendor` | CMake interface target `phoxi_camera_vendor::phoxi_camera_vendor` wrapping the PhoXi API (`phoxi::phoxi`) found under `$PHOXI_CONTROL_PATH`; an env hook prepends the API library directory to `LD_LIBRARY_PATH`. | PhoXi Control installation |
| `phoxi_camera_msgs` | Messages (`FrameInfo`, `FrameError`, `FrameMessage`) and services (trigger, connect, log download, profile management). | — |
| `phoxi_camera` | The driver: API abstraction, lifecycle node, conversions to ROS messages, tests. | vendor, msgs |
| `phoxi_camera_example` | Example clients, a composable example node and launch files. | phoxi_camera, msgs |
| `phoxi_camera_description` | URDF/xacro models and STL meshes for every camera model; `view_camera.launch.py`. | — |
| `phoxi_camera_rviz` | RViz configuration and a launch file running the driver with RViz. | phoxi_camera |

Data flows one way: PhoXi Control → `PhoXiInterface` (in `phoxi_camera`) → `PhoXiCamera` lifecycle node → ROS topics/services, with `phoxi_camera_msgs` defining the custom interfaces.

## Build, test, CI

- Requires an installed PhoXi Control with `PHOXI_CONTROL_PATH` set; build with `colcon build`, see [README.md](README.md#installation).
- Supported distros: Humble, Jazzy, Kilted, Lyrical, Rolling (the CI matrix in `.github/workflows/build.yml`).
- CI builds the image from `.github/docker/Dockerfile` (ROS desktop image plus PhoXi Control), runs the `phoxi_camera` linters (`ament_clang_format`, `ament_cppcheck`, `ament_cpplint` over `src/ include/ test/ test_mock/`), starts PhoXi Control headless, builds the workspace and runs only the mock tests (`colcon test --ctest-args -R "_mock_"`). Hardware tests need a real device.
- C++17. Style configs live in `phoxi_camera/` and `phoxi_camera_example/` (`.clang-format`, `CPPLINT.cfg`, `ament_code_style.cfg`).

## Index

- [`phoxi_camera/CLAUDE.md`](phoxi_camera/CLAUDE.md): driver architecture, the API abstraction, the parameter bridge, frame publishing and the tests.
- [`phoxi_camera_example/CLAUDE.md`](phoxi_camera_example/CLAUDE.md): example executables and launch files.
- [`phoxi_camera/test/README.md`](phoxi_camera/test/README.md) and [`phoxi_camera/test_mock/README.md`](phoxi_camera/test_mock/README.md): test suites.
- [`phoxi_camera_example/README.md`](phoxi_camera_example/README.md): how to run the examples.
