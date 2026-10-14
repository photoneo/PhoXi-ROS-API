<!-- claude-docs-generated-from: ce298b2d6bc0ae46b8fe8e0cb9de5eeb6f08e5c7 @ 2026-10-14T11:50:04Z -->
_Last updated: 2026-10-14 (ce298b2)_

# phoxi_camera

The driver package. The topics, services, parameters and lifecycle it exposes are documented in the root [README.md](../README.md#phoxi_camera-node); this file covers the internals.

## Build targets (`CMakeLists.txt`)

- `phoxi_camera_PhoXi_Interface` (shared library): `src/PhoXiInterface.cpp`, linked against `phoxi_camera_vendor`. The only code that touches the PhoXi API.
- `phoxi_camera_Ros_Interface` (shared library): `src/PhoXiCamera.cpp`, `src/RosConversions.cpp`. Registered as the component `phoxi_camera::PhoXiCamera`, so it can run in a component container.
- `phoxi_camera_node` (executable): `src/phoxi_camera_node.cpp`, which spins one `PhoXiCamera` with intra-process communication on a single-threaded executor.
- `launch/phoxi_camera.launch.py` starts the node with `config/phoxi_camera.yaml`; launch arguments `camera_name` (node name, which scopes topics, services and the default TF frame) and `device_id`.

## Layers

**`PhoXiInterface`** (`include/phoxi_camera/PhoXiInterface.h`): the API abstraction. All methods are virtual so tests can replace it.
- Connection and acquisition: device discovery, `connectCamera(deviceId, frameCallback)`, start/stop acquisition, trigger, trigger mode.
- Device maintenance: reboot, shutdown, factory reset, log download.
- Profiles: list, get/set active and startup, create, update, delete, export/import, reset.
- Settings: the device schema is loaded on connect into `SettingInfo` records (slash-separated keys such as `CapturingSettings/LaserPower`), read and written as `SettingValue`, a `std::variant` tagged by `SettingValueType` (scalars, arrays and PhoXi API geometry types such as ROIs and sizes). Frame output components are toggled separately (`get/setFrameOutputSettings`).
- It mixes the C++ API and the C API. `PhoXiFactory` handles the device list, connecting and maintenance; the connected device handle handles acquisition, triggering and profiles. Frames arrive asynchronously through the C API's `phoxi_aframe_enable` into the static `frameAcceptor`. The device schema, settings and frame output components go through `phoxi_command_execute`, with JSON responses parsed by `jsoncons`.
- Errors are thrown as the `PhoXiInterfaceException` hierarchy in `PhoXiException.h` (PhoXi Control not running, device not found, unable to connect / start / stop / trigger, corrupted frame, not connected).

**`PhoXiFrame`** (`PhoXiFrame.h`): a non-owning view over the C API's frame records, handed to the frame callback.

**`PhoXiCamera`** (`PhoXiCamera.h`, `src/PhoXiCamera.cpp`): the `rclcpp_lifecycle::LifecycleNode`.
- Lifecycle: `on_configure` connects and declares the device-derived parameters, `on_activate` starts acquisition and activates publishers, `on_deactivate` / `on_cleanup` / `on_shutdown` undo that. `mPhoXiInterface` is a protected `unique_ptr<PhoXiInterface>`, which is how tests inject a mock.
- Parameter bridge: node parameters (`device_id`, `frame_id`, `publish_combined`, `trigger_mode`, `logout_on_exit`, `stop_acquisition_on_exit`) are declared at construction. During configure, the device schema becomes `device_settings.*` (dots instead of slashes; object-typed settings are split into one parameter per field), `frame_settings.*` and read-only `device_info.*`. Values supplied before configure are applied to the device as overrides. Later writes go through `onParametersChanged` to `setSettings` / `setFrameOutputSettings`; a partial update of an object setting is merged into the full value first.
- Frames: `onFrameCallback` runs on the API's frame-delivery thread and publishes either the combined `point_cloud` or the individual topics, plus `frame_info`, camera infos and `frame_error`. A topic is published only if its publisher is activated and has at least one subscriber; the log line reports how many topics were published. Lifecycle transitions call `drainFrameCallback()` (via `mFrameMutex`) so no callback is in flight while publishers change.

**Conversions**: `RosConversions.{h,cpp}` turn frame records into `PointCloud2` (full or XYZ-only) and `Image` messages; `PhoXiConversions.h` maps API device information to `PhoXiDeviceInformation`.

## Configuration notes

- `config/phoxi_camera.yaml` is keyed by the node name `phoxi_camera`, so its values (including the explicit `frame_id: phoxi_camera_sensor` and the `frame_settings` overrides) apply only when the node keeps that name. With another `camera_name`, the code defaults apply, for example `frame_id = <node name>_sensor`.
- The URDF in `phoxi_camera_description` names its sensor link `phoxi_camera_sensor`, which matches the default frame of a node named `phoxi_camera`.

## Tests

- `test_mock/` (GMock, no device): `mock_phoxi_interface.h` and `camera_test_fixture.h` drive the node against a mocked `PhoXiInterface`. Suites cover lifecycle and services, conversions, frame/device settings parameters, device info parameters, setting types and multiple cameras. Target names contain `_mock_`, which is what CI selects. See [test_mock/README.md](test_mock/README.md).
- `test/` (GTest, real device via `PHO_TEST_DEVICE_ID`): lifecycle, frame acquisition in combined and individual-topic modes, and settings against a live device. Because topics without subscribers are skipped, frame tests call `waitForSubscribers` (`hardware_test_fixture.h`) before triggering. See [test/README.md](test/README.md).
