# PhoXi Camera ROS 2 Driver — Examples

The `phoxi_camera_example` package contains three self-contained examples with ready-to-run launch files demonstrating how to interact with the `phoxi_camera` driver.

## Prerequisites

- The `phoxi_camera` package and all its dependencies must be built in your workspace.
- **PhoXi Control** must be installed and running.
- Set `LD_LIBRARY_PATH` before running any example:
  ```bash
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PHOXI_CONTROL_PATH/API/lib:/opt/ros/<distro>/lib
  ```

---

## Simple Node Example

Demonstrates the basic driver workflow: configure → activate → trigger → receive → cleanup.

Two executables run alongside the driver:

- **`simple_node_service_caller`** — drives the lifecycle and calls `trigger_frame`. It performs the following sequence:
  1. Waits for the `phoxi_camera` services to become available.
  2. Configures and activates the node (lifecycle transitions).
  3. Calls `~/trigger_frame` to request a scan.
  4. Waits for a point cloud to be received.
  5. Deactivates and cleans up the node.
- **`simple_node_topic_listener`** — subscribes to `/phoxi_camera/points` and logs the first received message.

```bash
ros2 launch phoxi_camera_example run_simple_node_example.launch.py \
    sensor_sn:='<your-serial-number>'
```

The launch file opens RViz with a pre-configured view and shuts everything down automatically when the service caller exits.

> With the default `sensor_sn` (`InstalledExamples-basic-example`) the example runs against a file camera bundled with PhoXi Control — no hardware required.

### What to Expect

- **Terminal output:** Log messages from both the `phoxi_camera` node and `simple_node_service_caller`, showing lifecycle transitions, the trigger call, and confirmation of receiving a point cloud.
- **RViz:** The triggered point cloud appears in the pre-configured view.
- **Shutdown:** Once the service caller exits, the launch file automatically terminates the driver and RViz.

---

## Settings Example

Demonstrates configuring `frame_settings.*` and `device_settings.*` before and after `configure`, and changing a live setting while the camera is active.

```bash
ros2 launch phoxi_camera_example run_settings_example.launch.py \
    sensor_sn:='<your-serial-number>'
```

Pass device and frame setting overrides directly in the launch file — they are applied to the device during `on_configure`:

```python
parameters=[{
    'device_id':                                   LaunchConfiguration('sensor_sn'),
    'frame_settings.PointCloud':                    True,
    'frame_settings.NormalMap':                     False,
    'device_settings.CapturingSettings.LaserPower': 2,
    'device_settings.ScanMultiplier':               1,
}]
```

The `settings_example_caller` node then:
1. Triggers configure — the driver connects to the device and applies all overrides.
2. Lists all declared `device_settings.*` and `frame_settings.*` parameters.
3. Activates the camera.
4. Changes `LaserPower` live via the parameter service while the camera is running.

### What to Expect

- **Terminal output:** The caller logs all declared `device_settings.*` and `frame_settings.*` parameters after configure, then confirms the live `LaserPower` change.
- **Shutdown:** Once the caller's workflow is complete it shuts down, and the launch file automatically terminates the camera node.

---

## Composition Example

Runs `phoxi_camera` and a client node as composable components inside a shared multi-threaded container, enabling zero-copy intra-process communication. This allows for more efficient resource usage and easier integration with other nodes in a larger system.

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

# Stop the loop at any time
ros2 service call /phoxi_camera_composition_example/stop std_srvs/srv/Trigger '{}'
```

### What to Expect

The `CompositionExample` node drives a tight acquire-publish-retrigger loop: each arriving `PointCloud2` message immediately re-triggers the next scan, achieving the highest sustainable frame rate. It logs FPS and frame-to-frame timing on every received point cloud.

For background on composable nodes see the [ROS 2 Composition tutorial][composable] and [dynamic discovery][dynamic-discovery].

[composable]: https://docs.ros.org/en/kilted/Tutorials/Intermediate/Composition.html
[dynamic-discovery]: https://docs.ros.org/en/kilted/Tutorials/Advanced/Improved-Dynamic-Discovery.html
