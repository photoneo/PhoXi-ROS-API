ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG PHOXI_CONTROL_URL
ARG PHOXI_CONTROL_INSTALLER
ARG PHOXI_CONTROL_PATH

RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache \
    dbus \
    avahi-daemon \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ament-clang-format \
    && rm -rf /var/lib/apt/lists/*

# Install PhoXi Control
RUN curl -o /tmp/pxc.tar.gz ${PHOXI_CONTROL_URL} \
    && tar -C /tmp -xf /tmp/pxc.tar.gz \
    && chmod a+x /tmp/${PHOXI_CONTROL_INSTALLER} \
    && yes | /tmp/${PHOXI_CONTROL_INSTALLER} --accept \
    && rm -rf /tmp/pxc.tar.gz /tmp/${PHOXI_CONTROL_INSTALLER}

ENV PHOXI_CONTROL_PATH=${PHOXI_CONTROL_PATH}
ENV PhoXi_DIR=${PHOXI_CONTROL_PATH}

