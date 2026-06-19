# Hardware Tests — `phoxi_camera`

Tests in this directory require a physical PhoXi device connected to the network with PhoXi Control running. They exercise the full driver stack end-to-end: lifecycle management, frame acquisition, topic publishing, and device settings.

For mock-based (no-hardware) tests see **[test_mock/README.md](../test_mock/README.md)**.

## Requirements

- **PhoXi Control** installed and running with a device accessible on the network.
- Device serial number exposed via the `PHO_TEST_DEVICE_ID` environment variable:
  ```bash
  export PHO_TEST_DEVICE_ID='<your-serial-number>'
  ```
- **GTest** installed (`sudo apt-get install libgtest-dev`).
- ROS 2 workspace built and sourced.

---

## Test Suites

| Test binary | Suite | What it covers |
|-------------|-------|----------------|
| `phoxi_camera_conversions_test` | `ConversionTest` | Pure unit tests for data-type conversions (no device required). |
| `phoxi_camera_hardware_integration_test` | `HardwareIntegrationTest` | Full lifecycle transitions, frame triggering, and combined `point_cloud` topic in `publish_combined` mode. |
| `phoxi_camera_get_frame_test` | `FrameTest` | Frame acquisition in combined mode; verifies timestamp and frame ID synchronisation across all published topics. |
| `phoxi_camera_get_frame_individual_test` | `IndividualTopicsFrameTest` | Frame acquisition in individual-topics mode; verifies each image and point-cloud topic is published correctly. |
| `phoxi_camera_settings_test` | `DeviceRequiredTest`, `DirectInterfaceTest` | Device setting declaration, live parameter updates, and `PhoXiInterface::getSettings` / `getFrameOutputSettings` called directly on the device. |

---

## Building and Running

```bash
# Build
colcon build --packages-select phoxi_camera
source install/setup.bash

# Run all tests
export PHO_TEST_DEVICE_ID='<your-serial-number>'
colcon test --packages-select phoxi_camera
colcon test-result --verbose
```

### Running a single test

```bash
source install/setup.bash
colcon test --packages-select phoxi_camera --ctest-args "-R <test_name>"
```

### Accessing detailed logs

```bash
ls -l log/latest_test/phoxi_camera/
```
