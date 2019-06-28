# Panda_autograsp (v0.0.1)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/d7bc3ceaf48a4f878d0fc8dbda5b3002)](https://www.codacy.com?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=rickstaa/panda_autograsp&amp;utm_campaign=Badge_Grade)
[![Build Status](https://travis-ci.com/rickstaa/panda_autograsp.svg?token=khBpAzuAEsiEMbLE5tHM&branch=kinetic-devel)](https://travis-ci.com/rickstaa/panda_autograsp)
[![Ros kinetic](https://img.shields.io/badge/ROS%20Kinetic-recommended-brightgreen.svg)](https://wiki.ros.org/kinetic)
[![Ros melodic](https://img.shields.io/badge/ROS%20Melodic-not%20tested-yellow.svg)](https://wiki.ros.org/melodic)

Panda_autograsp is an autonomous ROS based grasping solution that works with the [Panda Emika Franka robot](https://www.franka.de/panda/). In this grasping solution a number of opensource grasping solutions are implemented on the [Panda Emika Franka robots](https://www.franka.de/panda/) robot. The grasping solutions present in this package both work with the physical panda robot as well as a simulated verison of the panda robot. A simulated version of the panda robot is shiped with the panda_autograsp package. The panda_autograsp package currently contains the following grasping algorithms:

-   [BerkleyAutomation/gqcnn](https://github.com/BerkeleyAutomation/gqcnn)

## Submodules contained in this package

The panda_autograsp package contains the following submodules:

-   [deep_robotics_singularity_recipes](https://github.com/rickstaa/deep_robotics_singularity_recipes)
-   [franka_ros](https://github.com/rickstaa/franka_ros)
-   [gqcnn](https://github.com/BerkeleyAutomation/gqcnn)
-   [movit_tutorials](https://github.com/ros-planning/moveit_tutorials)
-   [panda_movit_config](https://github.com/rickstaa/panda_moveit_config)
-   [panda_simulation](https://github.com/rickstaa/panda_simulation)

## Instalation instructions

There are two ways to use this package. You can build the package from source or use the supplied singularity image.

### Build from source

#### Prerequisites

The [panda_autograsp](https://github.com/rickstaa/panda_autograsp) contained in this branch was built for ROS kinetic running under Ubuntu 16.04. First, you need to make sure you installed `ros-kinetic-desktop-full` and that you have all the necessary dependencies installed.

#### Dependencies

##### Essential

The following packages are essential for using the panda_autograsp package.

-   A number of ROS packages
-   [Latest stable tensorflow release](https://www.tensorflow.org)
-   [ROS kinetic](https://wiki.ros.org/kinetic)
-   [libfreenect2](https://github.com/OpenKinect/libfreenect2)
-   [Miniconda3](https://docs.conda.io/en/latest/miniconda.html)
-   [conda tensorflow-gpu](https://anaconda.org/anaconda/tensorflow-gpu)
-   [libfranka](https://github.com/frankaemika/libfranka)

##### Recommended

For optimal performance you are advices to install the following packages.

-   [CUDA Toolkit 10.0](https://developer.nvidia.com/cuda-10.0-download-archive)
-   [CuDNN 7.6.1](https://developer.nvidia.com/cudnn)
-   [Cuda Samples](https://docs.nvidia.com/cuda/cuda-samples/index.html)

##### Optional

The documentation of this package was created using [the sphinx framework](http://www.sphinx-doc.org/en/stable/) in order to build the documentation you will need to installt he following packages:

-   [sphinx package](http://www.sphinx-doc.org/en/stable/)
-   [sphinx_rtd_theme](https://sphinx-rtd-theme.readthedocs.io/en/stable/)

#### ROS Packages

For the package to work you will need the following dependencies can be installed by running the following command:

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

#### Libfranka

After these packages are installed, you also need to build the _libfranka_ library from source. This needs the be done since you have to pass the _libfranka_ library to _catkin_make_ or _catkin build_ while building the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package. A guide on how to build the _libfranka_ library from source can be found [here](https://frankaemika.github.io/docs/installation.html#building-from-source).

#### Libfreenect2

In order to use the package together with the Kinect camera, we need to install the [libfreenect2](https://github.com/OpenKinect/libfreenect2.git) library. The documentation for installing the [libfreenect2](https://github.com/OpenKinect/libfreenect2.git) can be found [in the readme of the repository](https://github.com/OpenKinect/libfreenect2).

#### Pylibfreenect2

In order to communicate with the [libfreenect2](https://github.com/OpenKinect/libfreenect2.git) library through the python programming language we need to install the [Pylibfreenect2 package](https://github.com/r9y9/pylibfreenect2). The documentation for installing the [pylibfreenect2](https://github.com/r9y9/pylibfreenect2) can be found [in the readme of the repository](https://github.com/r9y9/pylibfreenect2).

#### Install python 3.6 and tensorflow-gpu

You are advised to use the conda _tensorflow-gpu_ package as this package already contains the necessary drivers for the GPU computing to work. After you [installed conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/) you can install this package by running the following command:

    conda install tensorflow-gpu

See the [tensorflow documentation](https://www.tensorflow.org/install/) if you want to use another package manger.

#### Build the Panda-autograsp package

After all the dependencies are installed you can build the `panda_autograsp` package as follows:

##### Clone and build the package using the https protocol

If you did not add an ssh key to your GitHub account you can use the following bash command:

    bash -c "mkdir -p /panda_autograsp_ws \
            && cd /panda_autograsp_ws \
            && source /opt/ros/kinetic/setup.sh \
            && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
            && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
            && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"

##### Clone and build the package using the ssh protocol

If you added an ssh key to your GitHub account you can also use the following bash command.

    bash -c "mkdir -p /panda_autograsp_ws \
            && cd /panda_autograsp_ws \
            && source /opt/ros/kinetic/setup.sh \
            && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
            && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
            && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"

#### Build the panda_autograsp documentation

Building panda_autograsp’s documentation requires a few extra dependencies (see dependencies above). To install the documentation dependencies, simply change directories into the panda_autograsp source and run

`pip install .[docs]`

Then go into the `docs/` directory and make the documentation using the `make html` command. After the make command has finished the documentation files generated in this manner can be found in docs/build.

### Use the supplied singularity image

[![https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg](https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg)](https://singularity-hub.org/collections/3134)

Alternatively, a _singularity recipe_ (panda-autograsp-tensorflow-gpu-miniconda3-ros_kinetic-xenial.def.def) for this package containing all the necessary packages and dependencies can be found on in the [deep_robotics_singularity_recipes repository](https://github.com/rickstaa/deep_robotics_singularity_recipes) or on the [singularity-hub.org website](https://www.singularity-hub.org/collections/3134).

## How to use

### Panda simulation

After you set up the catkin workspace and sourced its `devel/setup.bash` file you can launch the panda simulation with the following command:

    roslaunch panda_simulation simulation.launch

### Panda auto grasp solution

After you set up the catkin workspace and sourced its `devel/setup.bash` file you can launch the `panda autograsp solution` with the following command:

    roslaunch panda_autograsp simulation.launch
