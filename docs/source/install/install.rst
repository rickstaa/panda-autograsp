.. _install:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Prerequisites
=========================

Hardware
---------------

.. figure:: https://user-images.githubusercontent.com/17570430/69431164-7aa1ae00-0d37-11ea-9fd1-28851089a7ca.jpg
    :alt: setup
    :target: https://user-images.githubusercontent.com/17570430/69431164-7aa1ae00-0d37-11ea-9fd1-28851089a7ca.jpg

    Overview of an example setup.

- 1x Microsoft kinect v2.
- 1x Panda Emika Franka Robot.
- A laptop or PC with at least 12gb RAM and a decent Graphics card.

Software
-------------

ROS
^^^^^^^^^^^^^^^^^^^

The `panda_autograsp`_ package has only been tested with ROS ``Melodic``
and Kinect. The ROS melodic installation instructions can be found
`here <https://wiki.ros.org/melodic>`__. The ROS kinetic installation
instructions can be found `here <https://wiki.ros.org/kinetic>`__.

You further also need the following ROS packages:

.. code-block:: bash

    sudo apt-get install ros-melodic-moveit-ros-move-group ros-melodic-controller-manager* ros-melodic-moveit* ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller ros-melodic-gazebo-ros* ros-melodic-rviz* libboost-filesystem-dev libjsoncpp-dev

.. warning::

    As ROS melodic is the recommended version, the instructions below are for ROS Melodic.
    Change melodic to kinetic in each command if you want to use ROS kinetic.

Python
^^^^^^^^^^^^^^^^^^^

As ROS melodic does not yet fully support python 3, the `panda_autograsp`_
package has only been tested with ``Python 2.7``.


Ubuntu
^^^^^^^^^^^^^^^^^^^

The `panda_autograsp`_ package has only been tested with ubuntu ``18.04``.

Libfreenect2 library
^^^^^^^^^^^^^^^^^^^^^^^^^

To use the package together with the Kinect camera, we need to install the
`libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ library. The documentation
for installing the `libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ can be
found in the `readme of the repository <https://github.com/OpenKinect/libfreenect2>`_.

Libfranka library
^^^^^^^^^^^^^^^^^^^^^

In order to use this package with the panda emika franka robots
you need to build the Libfranka library from source. The steps
for doing this can be found in the
`libfranka documentation <https://frankaemika.github.io/docs/installation_linux.html>`_.

.. note::

    If you want to use the `panda_autograsp`_ package with the real robots you
    also have to install the real-time kernel.

Virtualenv
^^^^^^^^^^^^^^^^^^^

Although the package can be installed on the system python you are advised
to use a python environment management system like `Virtualenv <https://virtualenv.pypa.io/en/stable/>`_
or `conda <https://conda.io/en/latest/>`_.

.. warning::

    As ROS doesn't play nicely with anaconda I wrote a small
    `ROS Conda_wrapper <https://github.com/rickstaa/.ros_conda_wrapper>`_.
    Unfortunately, I wasn't yet able to solve all the problems caused
    case by this CONDA ROS incompatibility. You are therefore currently
    advised to use a `Virtualenv <https://virtualenv.pypa.io/en/stable/>`_
    instead.

CUDA & CUDNN (NVIDIA GPUs only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Since Tensorflow needs GPU computing capabilities of your NVIDIA
graphics card, the CUDA v10.0 and CDNN v7.6.5 toolkits
need to be installed. For installation instructions, you are referred to the
`CUDA <https://docs.nvidia.com/cuda/archive/10.0/>`_,
`CUDNN <https://docs.nvidia.com/deeplearning/sdk/cudnn-install/index.html>`_
or `tensorflow <https://www.tensorflow.org/install/gpu>`_ documentation.
Additionally, these toolkits are included in the Conda version
of the python opencv package. You can install this package by running the
``conda install -c conda-forge opencv``
command in your Conda environment.

AMD build of tensorflow (AMD GPUs only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A guide explaining how to use GPU computing with a AMD graphics card be
found `here <https://blog.codeinside.eu/2018/12/04/howto-use-tensorflow-with-amd-gpus/>`__.

Package build from source instructions
========================================

Clone the repository and build the package
--------------------------------------------------------

Clone or download the `panda_autograsp`_ catkin package from Github:

.. code-block:: bash

    bash -c "mkdir -p ~/panda_autograsp_ws \
    && cd ~/panda_autograsp_ws \
    && source /opt/ros/melodic/setup.sh \
    && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src

Install the ROS system and python dependencies
------------------------------------------------------

Install the ROS package dependencies using the following command:

.. code-block:: bash

    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka


Install python package using pip
----------------------------------------

As ``rosdep`` does not yet support specifying specific versions for
python packages, we need to install some additional packages using
the `pip install command`. To ease this process a ``setup.py`` file
was created. This file can be invoked using the following commands:

Build the package
-------------------------

The catkin package can be build by executing one of the following commands:

.. code-block:: bash

    catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=<PATH_TO_LIBFRANKA>/libfranka/build -Dfreenect2_DIR=<PATH_TO_FREENECT2>/freenect2/lib/cmake/freenect2"
    cd ~/panda_autograsp
    pip install .

Singularity Container installation instructions
==================================================

Nvidia container
---------------------------------------------
A ready to run Nividia compatible singularity
container is provided. This container can be build using the
recipe files found in the ``panda_autograsp/containers/singularity``
folder or by pulling directly from the `singularity-hub.org <https://www.singularity-hub.org>`_
container registry.

.. note::

    Due to the fact that I wasn't able to solve the ros_conda_wrapper problem explained above the
    container, which uses anaconda is not fully ready. I will update the container with the stable
    `ROS Conda_wrapper <https://github.com/rickstaa/.ros_conda_wrapper>`_ when it is stable.

.. warning::

    As the Franka real-time kernel does not yet support NVIDIA drivers
    (`see the Libfranka docs <https://frankaemika.github.io/docs/installation_linux.html>`_)
    the NVIDIA container can currently only be used with the simulated robot.
    An AMD container will be created when singularity starts to support AMD graphics cards.
    Currently, if you want to use this package on the real robot, you, therefore, have to install it and its dependencies manually. Taking a look at the bash code in the
    ``./containers/singularity/Singularity.ros_melodic-cuda10-bionic``
    might ease this process.

1. Build the container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The containers in this repository can be pulled directly from
the `singularity-hub <https://www.singularity-hub.org>`_ container
registry as follows:

.. code-block:: bash

    build <CONTAINER_NAME>.simg shub://rickstaa/panda_autograsp:ros-melodic-cuda10-bionic

Go to the ``panda_autograsp/containers/singularity`` folder and
built the container using the recipe file. This is done by running the
following command:

.. code-block:: bash

    sudo singularity <CONTAINER_NAME>.simg shub://rickstaa/panda_autograsp:ros-melodic-cuda10-bionic

You can also add the ``--sandbox`` argument to build the container as
a writeable folder.

.. warning:: You need root access to build from a recipe file.

2. Run the container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After te container has been build run it using the
``singularity run --writable <YOUR_CONTAINER_NAME>`` command.

3. Clone the repository and build the package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As the panda_autograsp package is still private it can not be
automatically build during the container generation. You, therefore,
have to clone and build the package manually after the docker
container is build. This is done by running the following commands:

.. code-block:: bash

    bash -c "mkdir -p ~/panda_autograsp_ws \
    && cd ~/panda_autograsp_ws \
    && source /opt/ros/melodic/setup.sh \
    && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
    && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build -Dfreenect2_DIR=/opt/freenect2/lib/cmake/freenect2"

.. warning::

    As all of the system dependencies for the `panda_autograsp`_ package
    have already been installed during the container creation, contrary
    to normal build instructions, for the singularity container, you
    don't need to run ``rosdep install`` command. If you want to install
    new system dependencies or run the ``rosdep install`` command you have
    to make sure you start the container as the root user. This is necessary
    since, in a singularity container, you are the same user inside and outside
    the container. When developing inside the singularity container, you are
    therefore advised to place the `panda_autograsp`_ workspace on a path
    which can be both accessed by you and the root user (``/opt/`` or the
    container main path ``/`` for example).

4. Add additional permissions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you did build the singularity container as a writeable folder,
you could give your user write and read access from outside the singularity
container by:

#. Changing the group owner to your user group.

.. code-block:: bash

    sudo chgrp -R <YOUR_USER_NAME> ./<YOUR_CONTAINER_NAME>

#. Giving your user group read and write access to the ``<YOUR_CONTAINER_NAME`` folder.

.. code-block:: bash

    sudo chmod -R g+rwx ./<YOUR_CONTAINER_NAME>

AMD compatible container
----------------------------

Currently, singularity does not yet support AMD graphics. This option is planned to be included in the next
release `see this announcement <https://sylabs.io/2019/06/towards-generalized-gpu-support-in-the-singularity-container-runtime-an-isc-preview-involving-amd-radeon-instinct-accelerators-and-the-rocm-open-software-platform/>`_.

Docker container installation instructions
===========================================

We do not yet provide a docker container for this package.
