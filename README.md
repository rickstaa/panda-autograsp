# Panda_autograsp_ws
This repository contains all the ROS packages that are needed to use the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package. 

## Prerequisites

The [panda_autograsp](https://github.com/rickstaa/panda_autograsp) contained in this branch was built for ROS kinetic running under Ubuntu 16.04. First, you need to make sure you installed `ros-kinetic-desktop-full` and that you have all the necessary dependencies installed. These dependencies can be installed by running the following command:

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

After these packages are installed, you then need to build the *libfranka* library from source. This needs the be done since you have to pass the *libfranka* library to *catkin_make* or *catkin build* while building the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package. A guide on how to build the *libfranka* library from source can be found [here](https://frankaemika.github.io/docs/installation.html#building-from-source). 

## Build instructions

After the *libfranka* library has been built the `panda_grasp` package together with its dependencies can be built. This can be done using the panda_autograsp catkin workspace repository [(panda_autograsp_ws)](https://github.com/rickstaa/panda_autograsp_ws) is available for the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package. However, since the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package is still under development you need to be added as a contributor before you can clone it. For access to the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package please email [rick.staa@outlook.com](mailto:rick.staa@outlook.com). After obtaining the right permissions the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package can be cloned using the `ssh` or `https` protocol:

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

### Singularity image

[![https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg](https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg)](https://singularity-hub.org/collections/2876)

Alternatively, a *singularity recipe* containing all the necessary packages and dependencies can be found on the [panda_autograsp_singularity_recipes repository](https://github.com/rickstaa/panda_autograsp_singularity_recipes) or on the [singularity-hub.org website](https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg)](https://singularity-hub.org/collections/2739). This repository currently contains the following recipe file:

- **panda_autograsp_kinetic.def:** Recipe file that allows running the panda_autograsp ROS kinetic package under Ubuntu 16.04.

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
