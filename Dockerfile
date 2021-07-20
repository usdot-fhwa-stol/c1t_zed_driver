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

FROM usdotfhwastoldev/carma-base:noetic-f1tenth-develop as setup

# robot_state_publisher ROS package depends on liburdfdom-headers-dev and liburdfdom-dev
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    liburdfdom-headers-dev \
    liburdfdom-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# The ZED SDK installation script expects the root user only when installing in a Docker container. The script also
# requires root access to install successfully. Changing to root seems to be the better practice in Dockerfiles
# compared to using sudo.
USER root
RUN echo '# R32 (release), REVISION: 4.4' | tee /etc/nv_tegra_release > /dev/null && \
    cd / && \
    wget -q --no-check-certificate -O ZED_SDK_Tegra_JP44_v3.5.0.run https://download.stereolabs.com/zedsdk/3.5/jp44/jetsons && \
    chmod +x ZED_SDK_Tegra_JP44_v3.5.0.run && \
    ./ZED_SDK_Tegra_JP44_v3.5.0.run silent skip_tools skip_python && \
    rm -rf /usr/local/zed/resources/* && \
    rm -rf ZED_SDK_Tegra_JP44_v3.5.0.run && \
    rm -rf /var/lib/apt/lists/* && \
    chmod -R 755 /usr/local/zed
USER carma

WORKDIR /home/carma
COPY --chown=carma . src/
RUN src/docker/checkout.bash
RUN src/docker/install.sh

FROM usdotfhwastoldev/carma-base:noetic-f1tenth-develop

# Reinstalling the same packages from setup stage
# zed_wrapper ROS package depends on libturbojpeg
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    liburdfdom-headers-dev \
    liburdfdom-dev \
    libturbojpeg \
    && sudo rm -rf /var/lib/apt/lists/*

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="CARMA"
LABEL org.label-schema.description="Binary application for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/c1t_zed_driver"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=setup /home/carma/install_isolated /opt/carma/install

USER root
# Built ZED SDK libraries need to be copied over so they can be used at runtime
COPY --from=setup /usr/local/zed /usr/local/zed
COPY --from=setup /etc/ld.so.conf.d/zed.conf /etc/ld.so.conf.d/zed.conf

# ZED SDK installation script does not properly set permissions since it's assumed evertying is run as root
RUN chmod -R 755 /usr/local/zed

# Allows streaming from within the Docker container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# Needed so carma user can access ZED SDK as non root
RUN usermod -aG video carma
USER carma

CMD ["roslaunch", "c1t_zed_driver", "c1t_zed_driver.launch"]
