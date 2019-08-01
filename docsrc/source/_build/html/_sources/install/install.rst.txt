.. pre_prerequisites:

.. _panda_autograsp: https://github.com/BerkeleyAutomation/gqcnn

Prerequisites
=========================

Python
-----------

The `panda_autograsp`_ package has only been tested with ``Python 3.5``
, ``Python 3.6``, and ``Python 3.7``.


Ubuntu
-----------------

The `panda_autograsp`_ package has only been tested with `Ubuntu 12.04`, `Ubuntu 14.04` and `Ubuntu 16.04`.

Virtualenv
-------------------

We highly recommend using a Python environment management system like `Virtualenv <https://virtualenv.pypa.io/en/stable/>`_ or `conda <https://conda.io/en/latest/>`_ with the Pip and ROS installations.

Essential dependencies
------------------------------

The following packages are essential for using the `panda_autograsp`_ package.

- `ROS kinetic <https://wiki.ros.org/kinetic>`_
- `A number of additional ROS packages <#ROS-packages>`_
- `libfreenect2 <https://github.com/OpenKinect/libfreenect2>`_

ROS Packages
^^^^^^^^^^^^^^^^^^^^^^^

For the package to work you will need the following dependencies can be
installed by running the following command:

.. code-block:: bash

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

Libfreenect2
^^^^^^^^^^^^^^^^^^^^^^^^

In order to use the package together with the Kinect camera, we need to install the
`libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ library. The documentation
for installing the `libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ can be
found in the `readme of the repository <https://github.com/OpenKinect/libfreenect2>`_.

Build the Panda-autograsp package
========================================

After all the dependencies are installed you can build the `panda_autograsp`_
package as follows:

Clone and build the package using the https protocol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you did not add an ssh key to your GitHub account you can use the following
bash command:

code-block:: bash

    bash -c "mkdir -p /panda_autograsp_ws \
            && cd /panda_autograsp_ws \
            && source /opt/ros/kinetic/setup.sh \
            && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
            && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
            && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"

##### Clone and build the package using the ssh protocol

If you added an ssh key to your GitHub account you can also use the following
bash command.

.. code-block:: bash

    bash -c "mkdir -p /panda_autograsp_ws \
            && cd /panda_autograsp_ws \
            && source /opt/ros/kinetic/setup.sh \
            && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
            && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
            && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"


Singularity Installation
============================

We currently do provide a pre-built Docker images, but you can build them yourself. This will require you to have installed `Docker`_ or `Nvidia-Docker`_ if you plan on using GPUs. Note that our provided build for GPUs uses CUDA 10.0 and cuDNN 7.0, so make sure that this is compatible with your GPU hardware. If you wish to use a different CUDA/cuDNN version, change the base image in `docker/gpu/Dockerfile` to the desired `CUDA/cuDNN image distribution`_. **Note that other images have not yet been tested.**

.. _Docker: https://www.docker.com/
.. _Nvidia-Docker: https://github.com/NVIDIA/nvidia-docker
.. _CUDA/cuDNN image distribution: https://hub.docker.com/r/nvidia/cuda/

1. Clone the repository
"""""""""""""""""""""""
Clone or download the `project`_ from Github. ::

    $ git clone https://github.com/BerkeleyAutomation/gqcnn.git

.. _project: https://github.com/BerkeleyAutomation/gqcnn

2. Build Docker images
""""""""""""""""""""""
Change directories into the `gqcnn` repository and run the build script. ::

    $ ./scripts/docker/build-docker.sh

This will build the images `gqcnn/cpu` and `gqcnn/gpu`.

3. Run singularity image
""""""""""""""""""""
To run `gqcnn/cpu`: ::

    $ docker run --rm -it gqcnn/cpu

To run `gqcnn/gpu`: ::
    
    $ nvidia-docker run --rm -it gqcnn/gpu

Note the use of `nvidia-docker` in the latter to enable the Nvidia runtime.

You will then see an interactive shell like this: ::

    $ root@a96488604093:~/Workspace/gqcnn#

Now you can proceed to run the examples and tutorial!
















ROS packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For the package to work you will need the following dependencies can be installed by running the following command:

.. code-block:: bash:

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev


Recommended dependencies
------------------------------

For optimal performance you are advices to install the following packages.

- `CUDA Toolkit 10.0 <https://developer.nvidia.com/cuda-10.0-download-archive>`_
- `CuDNN 7.6.1 <https://developer.nvidia.com/cudnn>`_
- `Cuda Samples <https://docs.nvidia.com/cuda/cuda-samples/index.html>`_
