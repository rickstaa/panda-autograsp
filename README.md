# Panda_autograsp_ws
This repository contains all the ros packages that are needed to use the [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package. This repository can be cloned in your `catkin workspace` after which the *catkin_make* or *catkin build* modules can be used to build the ROS packages it contains.

## Build instructions

The [panda_autograsp](https://github.com/rickstaa/panda_autograsp) package was built for ROS kinetic running under Ubuntu 16.04. First you therefore need to run the following command to make sure that all additional packages are installed:

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

After these packages are installed you then need to build the *libfranka* library from source and pass its directory to *catkin_make* or *catkin build* when building. How to build the *libfranka* library from sourece is described in [this tutorial](https://frankaemika.github.io/docs/installation.html#building-from-source). After the *libfranka* library is build the `panda_grasp` package together with its dependencies can be build by running the following code in your terminal:

    ## Build panda_auto_grasp project packages
    bash -c "mkdir -p /panda_grasp_solutions_ws \
        && cd /panda_grasp_solutions_ws \
        && source /opt/ros/${ROS_DISTRO}/setup.sh \
        && git clone git@github.com:rickstaa/panda_autograsp_ws.git src \
        && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
        && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build"

You can then use the package by sourcing the `~/panda_grasp_solutions_ws/devel/setup.bash` file.

## Singularity image

Alternatively a *singularity recipe* is also available on [this repository](https://github.com/rickstaa/panda_autograsp_singularity_recipes). This recipe creates a container in which all the needed packages are already set up correctly. This container also contains a [.singularity_bashrc](https://github.com/rickstaa/panda_autograsp_singularity_recipes/blob/master/.singularity_bashrc) file to automatically source the `panda_autograsp` workspace when you `run` the container.

### NOTE
As the `panda_autograsp` ROS package is still under development the *singurity recipe* can not be uploaded yet to [singularity-hub.org](https://www.singularity-hub.org). You therefore currently have to download the `panda_grasp_solutions.def` singularity recipe file from [this PRIVATE repository](https://github.com/rickstaa/panda_autograsp_singularity_recipes). As a result you also need to be added to the `panda_autograsp_ws` an the `panda_autograsp` repositories before your able to build the current container.

### How to use
You can build the singularity container using one of the following `build` commands:

- Normal singularity container: `$sudo singularity build <YOUR_IMAGE_NAME> <SINGULARITY_RECIPE_FILE>'
- Development container: `$sudo singularity build --sandbox <YOUR_IMAGE_NAME> <SINGULARITY_RECIPE_FILE>'

Following you can run the container by using one of the following `run` commands:

- With Nvidia GPU: `$ singularity run --nv <YOUR_IMAGE_NAME>`
- Without Nvidia GPU: `$ singularity run <YOUR_IMAGE_NAME>`

Additionally you can also add the `--writable` parameter to the `run command` to receive write premissions. For in order of this to work you need to have the right system permissions.

For more documentation on how to install and use singularity one is reffered to the [the singularity documentation](https://www.sylabs.io/docs/).
