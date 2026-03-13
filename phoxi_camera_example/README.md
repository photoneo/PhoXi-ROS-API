# PhoXi Camera ROS 2 Driver - Examples

## Prerequisites

- You must have the `phoxi_camera` package and all its dependencies built in your workspace.
- **Hardware:** To run this example, a real PhoXi device must be powered on and connected to the network, and the PhoXi Control software must be running.

## Simple node example

This package provides a simple C++ node that demonstrates the correct workflow for interacting with the main `phoxi_camera` driver node.

The example node performs the following sequence of actions:
1.  Waits for the `phoxi_camera` services to become available.
2.  Calls the `~/connect` service to connect to the device.
3.  Calls the `~/trigger_frame` service to request a new scan.
4.  Subscribes to the `/point_cloud` topic and waits to receive one message.
5.  Calls the `~/disconnect` service.

### How to Run the Example

This example is best run using the provided launch file, which starts the main camera driver, RViz, and the example client node all at once.

1.  **Source your ROS 2 workspace:**
    ```bash
    source /path/to/your/ros2_ws/install/setup.bash
    ```

2. **Update `LD_LIBRARY_PATH`**
    ```bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/Photoneo/PhoXiControl-x.xx.x/API/lib/:/opt/ros/{your-ros-distro}/lib/
    ```

3.  **Run the launch file:** You can provide the serial number of your device as an argument, otherwise the example will connect to `basic-example` file camera.
    ```bash
    ros2 launch phoxi_camera_example run_simple_node_example.launch.py sensor_sn:='your-sensor-sn-goes-here'
    ```

### What to Expect

- **Terminal Output:** You will see log messages from both the `phoxi_camera` node and the `simple_node_service_caller` node, showing the sequence of service calls and the confirmation of receiving a point cloud message.
- **RViz:** The RViz window will open, and you should see the triggered point cloud appear.
- **Shutdown:** Once the example node's workflow is complete, it will shut down, and the launch file will automatically terminate all other nodes, including the driver and RViz.

## Composition example

The `composition` example demonstrates how to use the `phoxi_camera` driver node as a component within a
[composable node architecture]. This allows for more efficient resource usage and easier integration with other nodes
in a larger system.

### How to Run the Composition Example

1. Launch the composition example, which starts a container and loads the `phoxi_camera` node as a component.
   Set the `ROS_AUTOMATIC_DISCOVERY_RANGE` environment variable to a value that suits your needs, according to
   the rules of [dynamic discovery] in ROS 2.
    ```bash
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 launch phoxi_camera_example run_composition_example.launch.py
    ```
2. In another terminal, configure and activate the `phoxi_camera` node and then connect to the device
    ```bash
    ros2 lifecycle set /phoxi_camera configure
    ros2 lifecycle set /phoxi_camera activate
    ros2 service call /phoxi_camera/connect phoxi_camera_msgs/srv/Connect \
        "{sn: '<device-serial-number>'}"
    ```
3. Start the acquisition loop
    ```bash
    ros2 service call /phoxi_camera_composition_example/start std_srvs/srv/Trigger
    ```
4. Stop the loop at any time
   ```bash
   ros2 service call /phoxi_camera_composition_example/stop std_srvs/srv/Trigger
   ```

[composable node architecture]: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html
[dynamic discovery]: https://docs.ros.org/en/rolling/Tutorials/Advanced/Improved-Dynamic-Discovery.html
