# Mock Tests — `phoxi_camera`

Tests in this directory use GoogleMock to replace `PhoXiInterface` with a mock implementation. No physical device or PhoXi Control installation is required — they run entirely in-process and are fast enough for CI.

For hardware tests that exercise a real device see **[test/README.md](../test/README.md)**.

## Requirements

- **GTest & GMock** installed:
  ```bash
  sudo apt-get install libgtest-dev libgmock-dev
  ```
- ROS 2 workspace built and sourced.

---

## Test Suites

| Test binary | Suite | What it covers |
|-------------|-------|----------------|
| `phoxi_camera_mock_conversions_test` | `ConversionTest` | Pure unit tests for ROS ↔ PhoXi data-type conversions. |
| `phoxi_camera_mock_ros_interface_test` | `RosInterfaceTest` | Core ROS lifecycle behaviour: configure, activate, deactivate, cleanup, and service availability at each state. |
| `phoxi_camera_mock_frame_settings_test` | `FrameSettingsTest` | `frame_settings.*` parameter declaration, override detection, and `setFrameOutputSettings` call semantics. |
| `phoxi_camera_mock_device_settings_params_test` | `DeviceSettingsParamsTest` | `device_settings.*` parameter declaration from the device schema, override application, and live `setSettings` call semantics. |
| `phoxi_camera_mock_device_info_params_test` | `DeviceInfoParamsTest` | `device_info.*` read-only parameter declaration after configure. |
| `phoxi_camera_mock_setting_types_test` | `SettingTypesTest` | Typed `SettingValue` variant conversion for all supported setting types. |

---

## Building and Running

```bash
# Build
colcon build --packages-select phoxi_camera
source install/setup.bash

# Run all mock tests
colcon test --packages-select phoxi_camera --ctest-args "-R mock"
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
