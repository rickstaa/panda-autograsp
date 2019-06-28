# Panda_autograsp
Panda_autograsp is an autonomous ROS based grasping solution that works with the [Panda Emika Franka robot](https://www.franka.de/panda/). In this grasping solution a number of opensource grasping solutions are implemented on the [Panda Emika Franka robots](https://www.franka.de/panda/) robot. The grasping solutions present in this package both work with the physical panda robot as well as a simulated verison of the panda robot. A simulated version of the panda robot is shiped with the panda_autograsp package. The panda_autograsp package currently contains the following grasping algorithms:

- [BerkleyAutomation/gqcnn](https://github.com/BerkeleyAutomation/gqcnn)

## Instalation instructions
There are two ways to use this package. You can build the package from source or use the supplied singularity image.

### Build from source

#### Prerequisites

The [panda_autograsp](https://github.com/rickstaa/panda_autograsp) contained in this branch was built for ROS kinetic running under Ubuntu 16.04. First, you need to make sure you installed `ros-kinetic-desktop-full` and that you have all the necessary dependencies installed.

#### Dependencies

- A number of ROS packages
- [Latest stable tensorflow release](https://www.tensorflow.org)
- [ROS kinetic](https://wiki.ros.org/kinetic)
- [libfreenect2](https://github.com/OpenKinect/libfreenect2)
- [Miniconda3](https://docs.conda.io/en/latest/miniconda.html)
- [conda tensorflow-gpu](https://anaconda.org/anaconda/tensorflow-gpu)
- [libfranka](https://github.com/frankaemika/libfranka)
- [panda_autograsp package](https://github.com/rickstaa/panda_autograsp_ws)


#### ROS Packages
For the package to work you will need the following dependencies can be installed by running the following command:

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

#### Libfranka
After these packages are installed, you also need to build the *libfranka* library from source. This needs the be done since you have to pass the *libfranka* library to *catkin_make* or *catkin build* while building the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package. A guide on how to build the *libfranka* library from source can be found [here](https://frankaemika.github.io/docs/installation.html#building-from-source).

#### Libfreenect2
In order to use the package together with the Kinect camera, we need to install the [libfreenect2](https://github.com/OpenKinect/libfreenect2.git) library. The library [libfreenect2](https://github.com/OpenKinect/libfreenect2.git) library can be installed by running the following commands:

    apt-get update
    apt-get install -q -y \
        build-essential \
        cmake \
        pkg-config \
        libusb-1.0-0-dev \
        libturbojpeg \
        libjpeg-turbo8-dev \
        libglfw3-dev \
        libopenni2-dev
    bash -c "git clone https://github.com/OpenKinect/libfreenect2.git \
        && cd libfreenect2 \
        && pwd \
        && mkdir build \
        && cd build \
        && cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 \
        && make \
        && make install \
        && mkdir -p /etc/udev/rules.d/ \
        && cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/"

#### Install python 3.6 and tensorflow-gpu
You are advised to use the conda *tensorflow-gpu* package as this package already contains the necessary drivers for the GPU computing to work. After you [installed conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/) you can install this package by running the following command:

    conda install tensorflow-gpu

See the [tensorflow documentation](https://www.tensorflow.org/install/) if you want to use another package manger.

#### Panda-autograsp package

After all the dependencies are installed you can build the `panda_autograsp` package as follows:


**Clone and build the package using the https protocol**

If you did not add an ssh key to your GitHub account you can use the following bash command:

```
bash -c "mkdir -p /panda_autograsp_ws \
        && cd /panda_autograsp_ws \
        && source /opt/ros/kinetic/setup.sh \
        && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
        && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
        && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"
```

**Clone and build the package using the ssh protocol**

If you added an ssh key to your GitHub account you can also use the following bash command.

```
bash -c "mkdir -p /panda_autograsp_ws \
        && cd /panda_autograsp_ws \
        && source /opt/ros/kinetic/setup.sh \
        && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
        && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
        && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"
```

### Use the supplied singularity image

[![https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg](https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg)](https://singularity-hub.org/collections/3134)


Alternatively, a *singularity recipe* (panda-autograsp-tensorflow-gpu-miniconda3-ros_kinetic-xenial.def.def) for this package containing all the necessary packages and dependencies can be found on in the [deep_robotics_singularity_recipes repository](https://github.com/rickstaa/deep_robotics_singularity_recipes) or on the [singularity-hub.org website](https://www.singularity-hub.org/collections/3134).

## How to use

### Panda simulation

After you set up the catkin workspace and sourced its `devel/setup.bash` file you can launch the panda simulation with the following command:

```
roslaunch panda_simulation simulation.launch
```

### Panda auto grasp solution

After you set up the catkin workspace and sourced its `devel/setup.bash` file you can launch the `panda autograsp solution` with the following command:

```
roslaunch panda_autograsp simulation.launch
```
