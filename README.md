# `phoxi_camera` ROS 2 Driver

This package provides a robust ROS 2 interface for the Photoneo PhoXi 3D Scanner/Camera. It leverages ROS 2 lifecycle management for graceful startup and shutdown, and publishes point cloud data in a standard `sensor_msgs/msg/PointCloud2` format.

## Packages in this Repository

*   **`phoxi_camera`**: The core ROS 2 driver node.
*   **`phoxi_camera_description`**: Contains the URDF and mesh files for the PhoXi camera models. This allows for visualizing the camera model itself in tools like RViz.
*   **`phoxi_camera_example`**: A demonstration package showing how to interact with the driver.
*   **`phoxi_camera_msgs`**: ROS 2 service and message definitions custom to `phoxi_camera`.
*   **`phoxi_camera_rviz`**: Simple RViz visualization.
*   **`phoxi_camera_vendor`**: A vendor package that wraps the official Photoneo PhoXi API. It makes the libraries and headers available to other packages within the ROS 2 build system.

## Installation

### Prerequisites

*   **ROS 2:** (e.g., Jazzy, Rolling) installed and sourced.
*   **Photoneo PhoXiControl:** The official driver software must be installed and running in the background. You can download it from the [Photoneo website](https://www.photoneo.com/3d-scanning-software/).
```bash
sudo chmod +x PhotoneoPhoXiControlInstaller-xxxxx.run
sudo ./PhotoneoPhoXiControlInstaller-xxxxx.run
# It is important to create environment variable pointing to the installed PhoXiControl
export PHOXI_CONTROL_PATH=/opt/Photoneo/PhoXiControl-x.xx.x/
```

### Installation Steps

1.  **Clone the repository** into your ROS 2 workspace `src` folder.
2.  **Install dependencies:**
    ```bash
    cd /path/to/your/ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  **Build the workspace:**
    ```bash
    colcon build
    ```
4.  **Source your workspace:**
    ```bash
    source install/setup.bash
    ```
5.  **Export PhoXi API path and ROS path to `LD_LIBRARY_PATH`**
    ```bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/Photoneo/PhoXiControl-x.xx.x/API/lib/:/opt/ros/{your-ros-distro}/lib/
    ```

---

## Setup with Docker (PhoXi Control in Docker)

Docker provides a straightforward way how to interact with the `phoxi_camera` package.

Start with a base ROS 2 image, e.g. `osrf/ros:jazzy-desktop`:

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

In a separate terminal:

1. install PhoXi Control into the container,
2. prepare runtime environment for PhoXi Control,
3. run PhoXi Control:

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
        && apt update && apt install -y --no-install-recommends \
            avahi-daemon
"

docker exec phoxi_camera_container \
        /bin/bash -c "service dbus start && /usr/sbin/avahi-daemon --daemonize"
docker exec \
        -e PHOXI_CONTROL_PATH=${PHOXI_CONTROL_PATH} \
        phoxi_camera_container \
        ${PHOXI_CONTROL_PATH}/bin/PhoXiControl
```

In a separate terminal, build the `phoxi_camera` workspace:

```bash
export PHOXI_CONTROL_PATH=/opt/Photoneo/PhoXiControl-1.16.5
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
```

Once the workspace is built you can start using it:

```bash
docker exec \
        -it \
        -e PHOXI_CONTROL_PATH=${PHOXI_CONTROL_PATH} \
        -e LD_LIBRARY_PATH=${PHOXI_CONTROL_PATH}/API/lib \
        phoxi_camera_container \
        /bin/bash

 # in the container, source the global underlay and the workspace overlay:
source /opt/ros/jazzy/setup.bash
cd /workspace
source install/local_setup.bash

colcon info
```

## Setup with Docker (PhoXiControl runs on host)

When running with PhoXiControl on the host you have to have PhoXiControl installed
and the environment variable `PHOXI_CONTROL_PATH` set.

Run PhoXiControl on the host, e.g.

```bash
PhoXiControl
```

Start with a base ROS 2 image, e.g. `osrf/ros:jazzy-desktop`:

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

In a separate terminal, build the `phoxi_camera` workspace:

```bash
docker exec \
        -e PHOXI_CONTROL_PATH="${PHOXI_CONTROL_PATH}" \
        phoxi_camera_container \
        /bin/bash -c "
            source /opt/ros/jazzy/setup.bash \
                && cd /workspace \
                && apt update \
                && rosdep update --rosdistro jazzy \
                && rosdep install --from-paths src --ignore-src -r -y \
                && colcon build
        "
```

Once the workspace is built you can start using it:

```bash
docker exec \
        -it \
        -e PHOXI_CONTROL_PATH="${PHOXI_CONTROL_PATH}" \
        -e LD_LIBRARY_PATH="${PHOXI_CONTROL_PATH}/API/lib" \
        phoxi_camera_container \
        /bin/bash

 # in the container, source the global underlay and the workspace overlay:
source /opt/ros/jazzy/setup.bash
cd /workspace
source install/local_setup.bash

colcon info
```


## `phoxi_camera` Node: ROS 2 Interfaces

The `phoxi_camera` node is a [ROS 2 Lifecycle Node](https://docs.ros.org/en/jazzy/Concepts/About-Lifecycle-Nodes.html). This means its services and topics are only available when it's in the `active` state.

### Parameters

*   `~/sensor_sn` (string): The serial number of the PhoXi device to connect to. Default: `""`.
*   `~/frame_id` (string): The coordinate frame ID for published data. Default: `"phoxi_camera_sensor"`.

### Available ROS 2 Services

*   `~/connect`: `phoxi_camera_msgs/srv/Connect`
*   `~/disconnect`: `std_srvs/srv/Trigger`
*   `~/trigger_frame`: `std_srvs/srv/Trigger`
*   `~/change_state`: `lifecycle_msgs/srv/ChangeState`
*   `~/get_state`: `lifecycle_msgs/srv/GetState`

### Available ROS 2 Topics

*   `/point_cloud`: `sensor_msgs/msg/PointCloud2`

---

## Testing

This package includes a comprehensive, multi-layered test suite using GTest/GMock to ensure correctness from data conversion up to hardware integration.

For detailed instructions on the testing philosophy and how to build and run the different test suites, please see the **[Testing README](./phoxi_camera/test/README.md)**.

## Example Usage

A separate package, `phoxi_camera_example`, is provided to demonstrate how to write a client application that interacts with this driver. It includes nodes for calling services, listening to topics, and a launch file to run a complete visual demonstration with RViz.

For complete instructions, please see the **[Example Package README](./phoxi_camera_example/README.md)**.

