FROM ghcr.io/irobot-algorithm/sentry_navigation/environment:latest
ADD src /basic_dev/src/
ADD setup.bash /

RUN chmod +x /setup.bash

USER root

RUN rm -f /etc/apt/sources.list.d/realsense.list && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor --batch --yes -o /usr/share/keyrings/ros1-latest-archive-keyring.gpg && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get -o Acquire::http::No-Cache=True -o Acquire::http::Pipeline-Depth=0 update && \
    apt-get install -y python3-catkin-tools ros-noetic-geographic-msgs \
      ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
      ros-noetic-roslint net-tools ros-noetic-mavros liborocos-bfl-dev

ENV ROS_DISTRO=noetic

WORKDIR /basic_dev/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && ./src/livox_ros_driver2/build.sh ROS1 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver2" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_ros" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="path_sender" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j14

ENTRYPOINT [ "/setup.bash" ]
