# C1T ZED Driver

The `c1t_zed_driver` package contains a CARMA Hardware Interface wrapper for the `zed_wrapper` package, which is used
for interfacing with the ZED camera (used in the F1tenth cars). This package is meant for use in the CARMA 1Tenth
platform.

# Getting started

There are two ways to use the `c1t_zed_driver` package: natively or in Docker.

## Running natively

1. Create a ROS workspace:
   ```shell
   $ mkdir -p ros_ws/src
   $ cd ros_ws/src
   ```

2. Clone the `c1t_zed_driver` and `zed-ros-wrapper` repositories into the `src` directory:
   ```shell
   $ git clone https://github.com/usdot-fhwa-stol/c1t_zed_driver.git --branch=develop
   $ git clone https://github.com/stereolabs/zed-ros-wrapper.git
   $ cd ..
   ```
   Omit `--branch=develop` to use the repository's default branch.

3. Clone the repositories containing the unreleased package dependencies:
   ```shell
   $ vcs import --input src/c1t_zed_driver/carma-msgs.repos ./src
   $ vcs import --input src/c1t_zed_driver/carma-utils.repos ./src
   ```

4. Build the `c1t_zed_driver` package:
   ```shell
   $ catkin_make --only-pkg-with-deps c1t_zed_driver
   ```

5. Launch the `c1t_zed_driver` nodes
   ```shell
   $ source devel/setup.bash
   $ roslaunch c1t_zed_driver c1t_zed_driver.launch

## Running in Docker

---

**Note:** The Docker container is currently targeted towards the [NVIDIA Jetson](
https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/) platforms and is running ROS 1 Noetic. The 
container *should* work with different ROS versions, but ROS is not designed to be backwards compatible. The container
has been successfully tested in a ROS 1 Melodic network.

---

1. Clone the repository:
   ```shell
   $ git clone https://github.com/usdot-fhwa-stol/c1t_zed_driver.git --branch=develop
   $ cd c1t_zed_driver
   ```
   Omit `--branch=develop` to use the repository's default branch.

2. Build the container image:
   ```shell
   $ sudo docker build -t c1t_zed_driver .
   ```

3. Run the container:
   ```shell
   $ sudo docker run \
         --network host \
         --privileged \
         --env ROS_IP=127.0.0.1 \
         c1t_zed_driver:latest
   ```
   `--network host` means the container will use the host's networking stack (see the [Docker docs](
   https://docs.docker.com/network/host/)). This will allow non-Docker ROS nodes to communicate with the nodes running 
   in the container. 
   
   `--privileged` gives the container access all devices on the host.
   
   `--env ROS_IP=127.0.0.1` sets the `ROS_IP` environment variable inside the container to `127.0.0.1` (also known
   as `localhost`). Using the `ROS_IP` variable allows the ROS nodes inside the container to communicate with ROS nodes
   outside of the container. For more information, see the [ROS documentation](
   http://wiki.ros.org/ROS/EnvironmentVariables#ROS_IP.2FROS_HOSTNAME) along with an [explanation](
   https://answers.ros.org/question/225276/what-the-ros_ip-for/) about why `ROS_IP` is needed.

# Implementation notes

The Docker container image in this repository targets the [NVIDIA Jetson](
https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/) platform, which currently does not support
Ubuntu 20.04 (the targeted operating system for ROS 1 Noetic); the latest supported version is Ubuntu 18.04. Therefore,
the ROS 1 Noetic binaries are unavailable for the Jetson computers. ROS 1 Noetic can be built from source for
the Jetsons (which is what the Docker container does when it is built), but there are several complications when doing
this for an unsupported platform:

* ROS package binaries (ones that are traditionally installed through `apt`) are unavailable. This is more subtle than
  it appears at first. What this means is that packages **and their dependencies** must be built from source.
* ROS tools (_e.g.,_ `rosdep` and `rosinstall_generator`) will not be able to find packages' source repositories.

The solution (more accurately, workaround) is to list all of a package's dependencies in a `.repos` file and import
the necessary repositories into the workspace using `vcs`.
