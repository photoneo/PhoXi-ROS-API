# PhoXi Camera ROS 2 Driver

![image](https://photoneo.com/files/dw/dw/github/Personal_Linkedin_banner_v4.png)

A ROS 2 driver for [Photoneo](https://www.photoneo.com) PhoXi 3D sensors. Provides a lifecycle-managed node that publishes point clouds, depth maps, normal maps, and camera metadata. Device settings are exposed as ROS 2 parameters and can be changed at runtime without restarting the node.

## Packages

| Package | Description |
|---------|-------------|
| [`phoxi_camera`](#phoxi_camera-node) | Core driver — lifecycle node, publishers, services, parameter bridge |
| [`phoxi_camera_msgs`](#phoxi_camera_msgs) | Custom message and service definitions |
| [`phoxi_camera_example`](#examples) | Example client nodes and launch files |
| [`phoxi_camera_description`](#phoxi_camera_description) | URDF and mesh models for RViz |
| [`phoxi_camera_rviz`](#phoxi_camera_rviz) | RViz launch files and configurations |
| [`phoxi_camera_vendor`](#phoxi_camera_vendor) | CMake vendor wrapper for the Photoneo PhoXi API |

---

## Prerequisites

- **ROS 2** Jazzy or Rolling, installed and sourced
- **PhoXi Control** installed and running — download from [photoneo.com/downloads/phoxi-control](https://www.photoneo.com/downloads/phoxi-control)

```bash
# Install PhoXi Control
sudo chmod +x PhotoneoPhoXiControlInstaller-<version>.run
sudo ./PhotoneoPhoXiControlInstaller-<version>.run

# Point the build system at the installation
export PHOXI_CONTROL_PATH=/opt/Photoneo/PhoXiControl-<version>
```

---

## Installation

```bash
# 1. Clone into your workspace
cd /path/to/ros2_ws/src
git clone <repository-url>

# 2. Install ROS dependencies
cd /path/to/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build

# 4. Source
source install/setup.bash

# 5. Expose the PhoXi API libraries at runtime
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PHOXI_CONTROL_PATH/API/lib:/opt/ros/<distro>/lib
```

---

## Quick Start

### 1. Launch the driver

```bash
ros2 launch phoxi_camera phoxi_camera.launch.py
```

Or pass your device serial number directly:

```bash
ros2 run phoxi_camera phoxi_camera_node --ros-args -p device_id:='<your-serial-number>'
```

### 2. Bring the node online

The node starts in the `unconfigured` state. Drive it through the lifecycle:

```bash
ros2 lifecycle set /phoxi_camera configure   # connects to device, declares parameters
ros2 lifecycle set /phoxi_camera activate    # starts acquisition, activates publishers
```

### 3. Trigger a scan and receive data

```bash
# Trigger a single frame
ros2 service call /phoxi_camera/trigger_frame phoxi_camera_msgs/srv/TriggerFrame '{}'

# Inspect the point cloud
ros2 topic echo /phoxi_camera/points

# Watch per-frame metadata
ros2 topic echo /phoxi_camera/frameInfo
```

### 4. Adjust device settings at runtime

```bash
# Read a setting
ros2 param get /phoxi_camera deviceSettings.CapturingSettings.LaserPower

# Write a setting — applied to the device immediately
ros2 param set /phoxi_camera deviceSettings.CapturingSettings.LaserPower 1024
```

---

## `phoxi_camera` Node

`phoxi_camera` is a [ROS 2 lifecycle node](https://docs.ros.org/en/jazzy/Concepts/About-Lifecycle-Nodes.html). The device connection and all publishers are tied to lifecycle transitions — nothing is active until the node is explicitly configured and activated.

### Lifecycle

```
[unconfigured]
      │  configure  → connects to device, declares deviceSettings.* / deviceInfo.* parameters
      ▼
[inactive]
      │  activate   → starts acquisition, activates all publishers
      ▼
[active]
      │  deactivate → stops acquisition, deactivates publishers
      ▼
[inactive]
      │  cleanup    → disconnects from device, releases all resources
      ▼
[unconfigured]
```

Lifecycle transitions can be triggered from the command line:

```bash
ros2 lifecycle set /phoxi_camera configure
ros2 lifecycle set /phoxi_camera activate
ros2 lifecycle set /phoxi_camera deactivate
ros2 lifecycle set /phoxi_camera cleanup
```

Or via the `~/change_state` service:

```bash
ros2 service call /phoxi_camera/change_state lifecycle_msgs/srv/ChangeState \
    '{transition: {id: 1}}'  # 1=configure, 3=activate, 4=deactivate, 6=cleanup
```

### Parameters

**Node parameters** (declared on node creation):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_id` | string | `""` | Hardware serial number or network ID of the target device. Set before `configure`. |
| `frame_id` | string | `"phoxi_camera_sensor"` | TF frame ID stamped on all published messages. |
| `publish_combined` | bool | `false` | `true`: publish a single `point_cloud` with all fields. `false`: publish individual topics. |

**Frame output settings** (declared during `configure` from the device schema; applied on every write thereafter):

Available components are discovered from the connected device — typical components across PhoXi models are listed below. Default values are read from the device at configure time.

| Parameter | Type | Description |
|-----------|------|-------------|
| `frameSettings.PointCloud` | bool | Enable the point cloud component. |
| `frameSettings.NormalMap` | bool | Enable per-pixel surface normals. |
| `frameSettings.DepthMap` | bool | Enable the depth map. |
| `frameSettings.Texture` | bool | Enable the grayscale texture. |
| `frameSettings.ConfidenceMap` | bool | Enable the confidence map. |
| `frameSettings.ColorCameraImage` | bool | Enable the color camera image (color cameras only). |
| `frameSettings.EventMap` | bool | Enable the event map (MotionCam only). |

Values supplied via launch file or parameter overrides before `configure` are compared against the device defaults and applied as overrides during configure. Post-configure writes are forwarded immediately.

**Device settings** (declared during `configure` from the device schema):

| Parameter | Type | Description |
|-----------|------|-------------|
| `deviceSettings.*` | various | Device settings. Applied to the device on every write. Parameters marked read-only by the device reject changes. |
| `deviceInfo.*` | various | Device identity information (name, type, IP address, firmware version, etc.). Read-only. |

To list all declared device settings after configure:

```bash
ros2 param list /phoxi_camera | grep deviceSettings
```

### Published Topics

Topics are only active in the **active** state.

**Always published:**

| Topic | Type | Description |
|-------|------|-------------|
| `frameError` | `phoxi_camera_msgs/FrameError` | Published when a scan fails; contains device error codes. |
| `frameInfo` | `phoxi_camera_msgs/FrameInfo` | Per-frame metadata: timing, sensor pose, PTP sync, temperatures. |
| `frameInfo/currentCamera` | `sensor_msgs/CameraInfo` | Primary camera intrinsics. |
| `frameInfo/currentColorCamera` | `sensor_msgs/CameraInfo` | Color camera intrinsics. |

**When `publish_combined` is `false` (default):**

| Topic | Type | Encoding | Description |
|-------|------|----------|-------------|
| `points` | `sensor_msgs/PointCloud2` | XYZ float32 | Lean XYZ-only point cloud. |
| `normals` | `sensor_msgs/Image` | 32FC3 | Per-pixel surface normal vectors. |
| `depth` | `sensor_msgs/Image` | 32FC1 | Orthogonal distance in mm. |
| `confidence` | `sensor_msgs/Image` | 32FC1 | Measurement confidence score. |
| `intensity` | `sensor_msgs/Image` | 32FC1 | Grayscale texture. |
| `texture` | `sensor_msgs/Image` | rgb8 | RGB texture. |
| `color_camera_image` | `sensor_msgs/Image` | rgb8 | Color camera image (color cameras only). |
| `event` | `sensor_msgs/Image` | 32FC1 | Time-of-measurement map (MotionCam only). |

**When `publish_combined` is `true`:**

| Topic | Type | Description |
|-------|------|-------------|
| `point_cloud` | `sensor_msgs/PointCloud2` | XYZ + normals + color/intensity + confidence + depth + event in a single message. |

### Services

**Available in all lifecycle states:**

| Service | Type | Description |
|---------|------|-------------|
| `~/reboot` | `std_srvs/Trigger` | Reboot the device. |
| `~/shutdown` | `std_srvs/Trigger` | Shut down the device. |
| `~/factory_reset` | `std_srvs/Trigger` | Restore factory defaults. |
| `~/log_download` | `phoxi_camera_msgs/LogDownload` | Download device logs to a file. |

**Available after `configure`:**

| Service | Type | Description |
|---------|------|-------------|
| `~/trigger_frame` | `phoxi_camera_msgs/TriggerFrame` | Manually trigger a scan (active state only). |
| `~/profiles/list` | `phoxi_camera_msgs/GetProfileList` | List saved profiles. |
| `~/profiles/get_active` | `phoxi_camera_msgs/GetActiveProfile` | Get the name of the active profile. |
| `~/profiles/set_active` | `phoxi_camera_msgs/SetActiveProfile` | Switch to a profile by name. |
| `~/profiles/create` | `phoxi_camera_msgs/CreateProfile` | Save current settings as a new profile. |
| `~/profiles/delete` | `phoxi_camera_msgs/DeleteProfile` | Delete a profile by name. |
| `~/profiles/update` | `phoxi_camera_msgs/UpdateProfile` | Overwrite a profile with current settings. |
| `~/profiles/get_startup` | `phoxi_camera_msgs/GetStartupProfile` | Get the startup profile name. |
| `~/profiles/set_startup` | `phoxi_camera_msgs/SetStartupProfile` | Set the startup profile. |
| `~/profiles/export` | `phoxi_camera_msgs/ExportProfile` | Export the active profile as binary. |
| `~/profiles/import` | `phoxi_camera_msgs/ImportProfile` | Import a profile from binary. |
| `~/profiles/reset` | `std_srvs/Trigger` | Reset the active profile to factory defaults. |

> **Deprecated:** `~/connect` and `~/disconnect` are kept for backwards compatibility. Use the lifecycle `configure` and `cleanup` transitions instead.

### Trigger a Frame

```bash
# Trigger and return immediately
ros2 service call /phoxi_camera/trigger_frame \
    phoxi_camera_msgs/srv/TriggerFrame '{wait_grabbing_end: false}'

# Trigger and wait until grabbing completes on the device
ros2 service call /phoxi_camera/trigger_frame \
    phoxi_camera_msgs/srv/TriggerFrame '{wait_grabbing_end: true}'
```

### Profile Management

```bash
# List all saved profiles
ros2 service call /phoxi_camera/profiles/list phoxi_camera_msgs/srv/GetProfileList

# Save current settings as a new profile
ros2 service call /phoxi_camera/profiles/create \
    phoxi_camera_msgs/srv/CreateProfile '{name: "my_profile"}'

# Switch to a profile
ros2 service call /phoxi_camera/profiles/set_active \
    phoxi_camera_msgs/srv/SetActiveProfile '{name: "my_profile"}'
```

---

## `PhoXiInterface` Class

`PhoXiInterface` is the C++ abstraction layer between `PhoXiCamera` and the Photoneo PhoXi API. It owns the API device handle and exposes a virtual API so the driver can be tested with mock implementations.

### Device Settings API

After `connectCamera()` succeeds, the device schema is available for reading and writing settings:

```cpp
// Enumerate all available settings
for (const auto& info : interface.getSettingInfos()) {
    // info.key       — slash-separated path, e.g. "CapturingSettings/LaserPower"
    // info.type      — SettingValueType discriminator
    // info.isSettable — false for read-only settings
}

// Read a single setting
SettingValue val = interface.getSetting("CapturingSettings/LaserPower");
int64_t power = std::get<int64_t>(val);

// Write a setting
interface.setSetting("CapturingSettings/LaserPower", SettingValue{int64_t(1024)});

// Batch read / write
auto vals = interface.getSettings({"CapturingSettings/LaserPower", "ScanMultiplier"});
interface.setSettings({
    {"CapturingSettings/LaserPower", SettingValue{int64_t(512)}},
    {"ScanMultiplier",               SettingValue{int64_t(2)}},
});
```

`SettingValue` is a `std::variant` over `bool`, `int64_t`, `double`, `std::string`, `std::vector<double>`, and several PhoXi API geometry types. The active alternative is identified by the `SettingValueType` enum.

---

## `phoxi_camera_msgs`

Custom interfaces used by the driver. The package is separate so other packages can depend on the message definitions without pulling in the full driver.

**Messages:**

| Message | Description |
|---------|-------------|
| `FrameError` | Published when a scan fails; contains device error codes. |
| `FrameInfo` | Per-frame metadata: timing, sensor pose, temperatures, PTP sync info. |
| `FrameMessage` | A single diagnostic entry used in both `FrameError` and `FrameInfo`. |

**Services:**

| Service | Description |
|---------|-------------|
| `Connect` | Connect to a device by ID (deprecated — prefer lifecycle `configure`). |
| `TriggerFrame` | Trigger a scan, optionally waiting for grabbing to complete on the device. |
| `LogDownload` | Download device logs to a specified path. |
| `GetProfileList` / `GetActiveProfile` / `SetActiveProfile` | Profile enumeration and selection. |
| `GetStartupProfile` / `SetStartupProfile` | Startup profile management. |
| `CreateProfile` / `DeleteProfile` / `UpdateProfile` | Profile lifecycle. |
| `ExportProfile` / `ImportProfile` | Binary profile transfer. |

---

## Examples

The `phoxi_camera_example` package contains three self-contained examples with ready-to-run launch files.

### Simple Node Example

Demonstrates the basic driver workflow: configure → activate → trigger → receive → cleanup.

Two executables run alongside the driver:

- **`simple_node_service_caller`** — drives the lifecycle, calls `trigger_frame`, and exits when done.
- **`simple_node_topic_listener`** — subscribes to `point_cloud` and logs the first received message.

```bash
ros2 launch phoxi_camera_example run_simple_node_example.launch.py \
    sensor_sn:='<your-serial-number>'
```

The launch file opens RViz with a pre-configured view and shuts everything down automatically when the service caller exits.

> With the default `sensor_sn` (`InstalledExamples-basic-example`) the example runs against a file camera bundled with PhoXi Control — no hardware required.

### Settings Example

Demonstrates configuring `frameSettings.*` and `deviceSettings.*` before and after `configure`, and changing a live setting while the camera is running.

```bash
ros2 launch phoxi_camera_example run_settings_example.launch.py \
    sensor_sn:='<your-serial-number>'
```

Pass device and frame setting overrides directly in the launch file — they are applied during `on_configure`:

```python
parameters=[{
    'device_id':                                   LaunchConfiguration('sensor_sn'),
    'frameSettings.PointCloud':                    True,
    'frameSettings.NormalMap':                     False,
    'deviceSettings.CapturingSettings.LaserPower': 2,
    'deviceSettings.ScanMultiplier':               1,
}]
```

The `settings_example_caller` node then:
1. Sets additional `frameSettings.*` via the parameter service before configure.
2. Triggers configure — the driver connects to the device and applies all overrides.
3. Lists all declared `deviceSettings.*` and `frameSettings.*` parameters.
4. Activates the camera.
5. Changes `LaserPower` live via the parameter service.

### Composition Example

Runs `phoxi_camera` and a client node as composable components inside a shared multi-threaded container, enabling zero-copy intra-process communication.

```bash
# Start the container with both components
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch phoxi_camera_example run_composition_example.launch.py
```

Then in separate terminals:

```bash
# Drive the lifecycle
ros2 lifecycle set /phoxi_camera configure
ros2 lifecycle set /phoxi_camera activate

# Start the back-to-back trigger loop
ros2 service call /phoxi_camera_composition_example/start std_srvs/srv/Trigger '{}'

# Stop the loop
ros2 service call /phoxi_camera_composition_example/stop std_srvs/srv/Trigger '{}'
```

The `CompositionExample` node drives a tight acquire-publish-retrigger loop and logs FPS and frame-to-frame timing on each received point cloud.

---

## Other Packages

### `phoxi_camera_description`

URDF models and meshes for all PhoXi camera families. Use with `robot_state_publisher` to visualize the sensor in RViz:

```bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(xacro /path/to/phoxi.urdf.xacro)"
```

### `phoxi_camera_rviz`

RViz configuration files and launch helpers for standard point cloud and image visualization. These configurations are also used by the example launch files.

### `phoxi_camera_vendor`

A CMake vendor wrapper that makes the Photoneo PhoXi API libraries and headers available to other packages in the colcon workspace via `ament_cmake` targets:

```cmake
find_package(phoxi_camera_vendor REQUIRED)
target_link_libraries(my_target phoxi_camera_vendor::phoxi_api)
```

---

## Setup with Docker

### PhoXi Control inside the container

```bash
xhost +local:docker
docker run \
    --name phoxi_camera_container \
    --network host \
    --device /dev/dri \
    --shm-size 2g \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)":/workspace/src/PhoXi-ROS-API \
    -e DISPLAY=$DISPLAY \
    -it \
    osrf/ros:jazzy-desktop \
    /bin/bash
```

Install PhoXi Control into the container, start avahi, and launch PhoXi Control:

```bash
export PHOXI_CONTROL_PATH=/opt/Photoneo/PhoXiControl-1.16.5
export PHOXI_CONTROL_INSTALLER=PhotoneoPhoXiControlInstaller-1.16.5-Ubuntu24-STABLE.run
export PHOXI_CONTROL_URL=https://photoneo.com/files/dw/dw/pxc/1.16/PhotoneoPhoXiControlInstaller-1.16.5-Ubuntu24-STABLE.tar.gz

docker exec phoxi_camera_container /bin/bash -c "
    curl -o /tmp/pxc.tar.gz ${PHOXI_CONTROL_URL} \
        && tar -C /tmp -xf /tmp/pxc.tar.gz \
        && chmod a+x /tmp/${PHOXI_CONTROL_INSTALLER} \
        && yes | /tmp/${PHOXI_CONTROL_INSTALLER} --accept \
        && rm -rf /tmp/pxc.tar.gz /tmp/${PHOXI_CONTROL_INSTALLER} \
        && apt update && apt install -y --no-install-recommends avahi-daemon
"

docker exec phoxi_camera_container \
    /bin/bash -c "service dbus start && /usr/sbin/avahi-daemon --daemonize"
docker exec \
    -e PHOXI_CONTROL_PATH=${PHOXI_CONTROL_PATH} \
    phoxi_camera_container \
    ${PHOXI_CONTROL_PATH}/bin/PhoXiControl
```

Build and run the workspace inside the container:

```bash
docker exec \
    -e PHOXI_CONTROL_PATH=${PHOXI_CONTROL_PATH} \
    phoxi_camera_container \
    /bin/bash -c "
        source /opt/ros/jazzy/setup.bash \
            && cd /workspace \
            && rosdep update --rosdistro jazzy \
            && rosdep install --from-paths src --ignore-src -r -y \
            && colcon build
    "

docker exec \
    -it \
    -e PHOXI_CONTROL_PATH=${PHOXI_CONTROL_PATH} \
    -e LD_LIBRARY_PATH=${PHOXI_CONTROL_PATH}/API/lib \
    phoxi_camera_container \
    /bin/bash

# inside the container:
source /opt/ros/jazzy/setup.bash
cd /workspace && source install/local_setup.bash
```

### PhoXi Control on the host

When PhoXi Control runs on the host, mount the shared memory device and the installation directory into the container:

```bash
docker run \
    --name phoxi_camera_container \
    -v /dev/shm/:/dev/shm/:rw \
    -v "$(pwd)":/workspace/src/PhoXi-ROS-API \
    -v "${PHOXI_CONTROL_PATH}":"${PHOXI_CONTROL_PATH}":rw \
    -e PHOXI_CONTROL_PATH="${PHOXI_CONTROL_PATH}" \
    -it \
    osrf/ros:jazzy-desktop \
    /bin/bash
```

Build and enter the container the same way as above.

---

## Testing

The `phoxi_camera` package includes a layered test suite covering data conversions, ROS interface behaviour (mock-based), and hardware integration.

For full instructions see the **[Testing README](./phoxi_camera/test/README.md)**.

```bash
# Build and run all tests
colcon build --packages-select phoxi_camera
colcon test  --packages-select phoxi_camera
colcon test-result --verbose

# Run only the fast mock-based tests
colcon test --packages-select phoxi_camera \
    --ctest-args "-R 'ros_interface_test|conversion_test'"

# Run only the hardware integration test (requires a connected device)
colcon test --packages-select phoxi_camera \
    --ctest-args "-R hardware_integration_test"
```

---

## Support

- Help Center: [photoneo.com/Help-Center](https://www.photoneo.com/Help-Center)
- Support pages: [photoneo.com/support](https://www.photoneo.com/support)
- Issues: file a report in this repository's issue tracker
