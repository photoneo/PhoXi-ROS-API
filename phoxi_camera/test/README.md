# Testing the `phoxi_camera` ROS 2 Node

All tests are built using the C++ GoogleTest (GTest) and GoogleMock (GMock) frameworks.

## Requirements

### General Requirements
*   **GoogleTest & GoogleMock:** These are required to compile and run the tests.
    ```bash
    sudo apt-get update
    sudo apt-get install libgtest-dev libgmock-dev
    ```
*   **ROS 2 Workspace:** All commands should be run from the root of your `colcon` workspace.

* To run the `hardware_integration_test`, you **must** also have PhoXiControl application running and accessible on the network.

---

## How to Compile and Run Tests

### 1. Build the Workspace
First, build your workspace with testing enabled. `colcon` will automatically build the test executables.

```bash
cd /path/to/your/ros2_ws
colcon build --packages-select phoxi_camera
```

### 2. Run the Tests
After building, source the workspace and run the tests using `colcon test`.

```bash
source install/setup.bash
colcon test --packages-select phoxi_camera
```

### 3. View the Results
The primary command to see a summary of the test results is:

```bash
colcon test-result --verbose
```
This will show which tests passed, failed, or were skipped.

### Running Specific Tests
Running the hardware test can be slow. If you only want to run the fast unit/integration tests, you can do so with `ctest-args`.

*   **Run only the mock-based tests:**
    ```bash
    colcon test --packages-select phoxi_camera --ctest-args "-R 'ros_interface_test|conversion_test'"
    ```

*   **Run only the integration test:**
    ```bash
    colcon test --packages-select phoxi_camera --ctest-args "-R hardware_integration_test"
    ```

### Accessing Detailed Logs
For more detailed output from a failed test, you can look in the `log` directory of your workspace:

```bash
ls -l log/latest_test/phoxi_camera/
```
This will contain the full `stdout` and `stderr` from the test executables.
