# Panda_autograsp_ws
This repository contains all the ROS packages that are needed to use the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package. 

## Build instrucutions

### Catkin Build instructions

The [panda_autograsp](https://github.com/rickstaa/panda_autograsp) contained in this branch was built for ROS kinetic running under Ubuntu 16.04. First you therefore need to run the following command to make sure that all additional packages are installed:

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

After these packages are installed, you first need to build the *libfranka* library from source. Following you have to pass the *libfranka* library to *catkin_make* or *catkin build* while building the packages. A guide on how to build the *libfranka* library from source can be found [here](https://frankaemika.github.io/docs/installation.html#building-from-source). After the *libfranka* library has been built the `panda_grasp` package together with its dependencies can be build by running the following code in your terminal:

    bash -c "mkdir -p /panda_autograsp_ws \
        && cd /panda_autograsp_ws \
        && source /opt/ros/kinetic/setup.sh \
        && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
        && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
        && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build"

You can then use the package by sourcing the `~/panda_grasp_solutions_ws/devel/setup.bash` file.

### Singularity image

[![https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg](https://www.singularity-hub.org/static/img/hosted-singularity--hub-%23e32929.svg)](https://singularity-hub.org/collections/2739)

Alternatively a *singularity recipe* can be found in the [panda_autograsp_singularity_recipes repository](https://github.com/rickstaa/panda_autograsp_singularity_recipes). This repository contains the singularity recipes which can be used to run the panda_autograsp ROS package. These recipe files can be used to create a singularity container in which all the packages and libraries that are needed to run the panda_autograsp package are set up correctly. This repository currently contains the following recipe file:

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
