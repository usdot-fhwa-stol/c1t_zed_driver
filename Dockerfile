# Copyright 2021 U.S. Department of Transportation, Federal Highway Administration
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM dustynv/ros:noetic-ros-base-l4t-r32.4.4 as setup

# tf2_geometry_msgs ROS package depends on libeigen3-dev and liborocos-kdl-dev
# robot_state_publisher ROS package depends on liburdfdom-headers-dev and liburdfdom-dev
RUN apt update && apt install -y --no-install-recommends \
    libeigen3-dev \ 
    liborocos-kdl-dev \
    liburdfdom-headers-dev \
    liburdfdom-dev \
    && rm -rf /var/lib/apt/lists/*

RUN echo "# R32 (release), REVISION: 4.4" > /etc/nv_tegra_release && \
    wget -q --no-check-certificate -O ZED_SDK_Tegra_JP44_v3.5.0.run https://download.stereolabs.com/zedsdk/3.5/jp44/jetsons && \
    chmod +x ZED_SDK_Tegra_JP44_v3.5.0.run && \
    ./ZED_SDK_Tegra_JP44_v3.5.0.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Tegra_JP44_v3.5.0.run && \
    rm -rf /var/lib/apt/lists/*

# Base image already has ros_base packages in this workspace
WORKDIR /workspace/ros_catkin_ws/
COPY . src/
RUN src/docker/checkout.bash
RUN src/docker/install.sh

FROM dustynv/ros:noetic-ros-base-l4t-r32.4.4

# Reinstalling the same packages from setup stage
# zed_wrapper ROS package depends on libturbojpeg
RUN apt update && apt install -y --no-install-recommends \
    libeigen3-dev \ 
    liborocos-kdl-dev \
    liburdfdom-headers-dev \
    liburdfdom-dev \
    libturbojpeg \
    && rm -rf /var/lib/apt/lists/*

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

COPY --from=setup /workspace/ros_catkin_ws/install_isolated /opt/ros/$ROS_DISTRO
COPY --from=setup /usr/local/zed /usr/local/zed
COPY --from=setup /usr/local/lib/python3.6/dist-packages/pyzed /usr/local/lib/python3.6/dist-packages/pyzed
COPY --from=setup /usr/local/lib/python3.6/dist-packages/pyzed-3.5.dist-info /usr/local/lib/python3.6/dist-packages/pyzed-3.5.dist-info
COPY --from=setup /etc/ld.so.conf.d/zed.conf /etc/ld.so.conf.d/zed.conf

# Allows streaming from within the Docker container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

CMD ["roslaunch", "c1t_zed_driver", "c1t_zed_driver.launch"]
